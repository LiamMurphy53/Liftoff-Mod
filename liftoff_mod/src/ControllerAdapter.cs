using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Reflection.Emit;
using HarmonyLib;
using Liftoff.FlightControllers.ZetaFlight;
using UnityEngine;

namespace GoProGeometry
{
    // Version-guarded interoperability with the owner's game. No game code is redistributed.
    internal sealed class ControllerAdapter
    {
        static readonly Module Game=typeof(FlightManager).Module;
        static FieldInfo Field(int token)=>Game.ResolveField(token);
        static readonly FieldInfo Pids=Field(GameTokens.Pids), Settings=Field(GameTokens.Settings),
            Outputs=Field(GameTokens.Outputs), Forces=Field(GameTokens.Forces), Rpm=Field(GameTokens.Rpm),
            MaxRpm=Field(GameTokens.MaxRpm), Sign=Field(GameTokens.Sign), Storage=Field(GameTokens.Storage),
            Gains=Field(GameTokens.PidGains), Integral=Field(GameTokens.PidIntegral);
        static readonly ConstructorInfo PidConstructor=(ConstructorInfo)Game.ResolveMethod(GameTokens.PidConstructor);
        internal static readonly MethodInfo Calculate=(MethodInfo)Game.ResolveMethod(GameTokens.PidCalculate);
        static readonly FieldInfo PropulsionOwner=Field(GameTokens.PropulsionOwner);
        static readonly MethodInfo PropulsionGetter=(MethodInfo)Game.ResolveMethod(GameTokens.PropulsionGetter),
            MaxThrust=(MethodInfo)Game.ResolveMethod(GameTokens.MaxThrust);
        readonly ThrustCurve curve;
        readonly object[] thrustArgs=new object[2];
        object propulsion;
        readonly Component controller;
        readonly MotorAllocator allocator;
        readonly IDictionary original, adapted;
        readonly object pitchPid, rollPid;
        readonly double[] forces=new double[4];
        float pitchIntegral,rollIntegral;
        bool mixedPreviousStep;
        internal int MixCalls { get; private set; }
        internal float LastThrottle { get; private set; }
        internal float[] LastForces { get; private set; }=new float[4];
        internal string Report { get; }

        internal ControllerAdapter(Component controller,GeometryProfile profile,Vector3 originalBodyInertia)
        {
            this.controller=controller; allocator=MotorAllocator.FromProfile(profile);
            curve=profile.HasThrustCurve?ThrustCurve.FromProfile(profile):null;
            original=(IDictionary)Pids.GetValue(controller);
            adapted=(IDictionary)Activator.CreateInstance(original.GetType());
            var tensor=GeometryMath.UnityTensor(profile.inertiaKgM2);
            var newInertia=new Vector3((float)tensor[0,0],(float)tensor[1,1],(float)tensor[2,2]);
            bool auto=((ZetaFlightControllerSettings)Settings.GetValue(controller)).isAutotuneOn;
            foreach(TaitBryanAngle axis in new[]{TaitBryanAngle.PITCH,TaitBryanAngle.ROLL,TaitBryanAngle.YAW}) {
                var old=(PIDGains)Gains.GetValue(original[axis]);
                int index=axis==TaitBryanAngle.PITCH?0:axis==TaitBryanAngle.YAW?1:2;
                float oldScale=originalBodyInertia[index],newScale=newInertia[index];
                if(!auto) { oldScale+=(originalBodyInertia.x+originalBodyInertia.y+originalBodyInertia.z)*.1f; newScale+=(newInertia.x+newInertia.y+newInertia.z)*.1f; }
                float ratio=newScale/oldScale;
                var gains=new PIDGains { P=old.P*ratio,I=old.I*ratio,D=old.D*ratio };
                // Auto mode is P-only in this game. Add a bounded 0.5 s integral term
                // to reject residual load/aerodynamic bias; native stick rates are unchanged.
                if(auto && axis!=TaitBryanAngle.YAW) gains.I=2*gains.P;
                if(gains.P<=0 || float.IsNaN(gains.P)) throw new InvalidOperationException("Unsupported controller gains.");
                float maxBias=(float)(profile.massKg*9.80665*.02); // at most a 20 mm equivalent load offset
                float integralLimit=gains.I>0?maxBias/gains.I:0;
                adapted[axis]=PidConstructor.Invoke(new object[]{axis,gains,integralLimit});
            }
            pitchPid=adapted[TaitBryanAngle.PITCH]; rollPid=adapted[TaitBryanAngle.ROLL];
            Report="geometry mixer; hover shares FL/FR/RL/RR="+string.Join("/",allocator.HoverShares.Select(x=>(100*x).ToString("F2")))+
                "%; inertia-scaled native "+(auto?"auto PI":"manual PID")+"; stock rates retained; "+
                (curve!=null?"thrust curve "+profile.thrustN.Length+" points, "+(curve.Max/9.80665*1000).ToString("F0")+" g per motor at full throttle":"Liftoff propulsion");
        }
        internal void Install(Propeller[] orderedProps) {
            var nativeProps=controller.GetType().BaseType.GetFields(Plugin.Members).Single(f=>f.FieldType==typeof(List<Propeller>));
            if(!((List<Propeller>)nativeProps.GetValue(controller)).SequenceEqual(orderedProps))
                throw new InvalidOperationException("Native motor order does not match the verified geometry labels.");
            Pids.SetValue(controller,adapted); Verify();
        }
        internal void Verify() {
            if(!ReferenceEquals(Pids.GetValue(controller),adapted)) throw new InvalidOperationException("Controller settings changed. Stop and reapply the GoPro profile.");
        }
        internal void Restore() { if(controller) Pids.SetValue(controller,original); }
        internal void BeforeStep() {
            // Do not carry stored correction through disarming / a controller branch that did not mix motors.
            if(!mixedPreviousStep) { Integral.SetValue(pitchPid,0f); Integral.SetValue(rollPid,0f); }
            pitchIntegral=(float)Integral.GetValue(pitchPid); rollIntegral=(float)Integral.GetValue(rollPid);
            mixedPreviousStep=false;
        }
        static float[] Values(FieldInfo field,object target) {
            var result=(float[])Storage.GetValue(field.GetValue(target));
            if(result.Length!=4) throw new InvalidOperationException("Unexpected controller output dimensions.");
            return result;
        }
        // Liftoff's propeller model: static or in-flight thrust (N) of one motor at an RPM and axial inflow (m/s).
        float Thrust(float rpm,float inflow) { thrustArgs[0]=rpm; thrustArgs[1]=inflow; return (float)MaxThrust.Invoke(propulsion,thrustArgs); }
        internal void Mix(float maximum,float inflow) {
            Verify();
            if(!(maximum>0) || float.IsInfinity(maximum) || float.IsNaN(inflow)) throw new InvalidOperationException("Invalid motor thrust limit.");
            var demand=Values(Outputs,controller); var motor=Values(Forces,controller); var rpm=Values(Rpm,controller);
            float collective=motor.Sum();
            float idle=((ZetaFlightControllerSettings)Settings.GetValue(controller)).idleMotorThrottlePercentage;
            float maxRpm=(float)MaxRpm.GetValue(controller);
            bool reversed=(float)Sign.GetValue(controller)<0;
            double low=idle*maximum, high=maximum, airspeed=1;
            if(curve!=null) {
                // Liftoff sets each motor to its propeller's thrust at throttle × max RPM. Read the throttle back
                // out of that, then give the measured thrust at that throttle, keeping Liftoff's airspeed losses.
                propulsion=PropulsionGetter.Invoke(PropulsionOwner.GetValue(controller),null);
                float perMotor=Math.Abs(collective)/4, rpmLo=0, rpmHi=maxRpm;
                if(perMotor>=maximum) rpmLo=maxRpm;
                else for(int k=0;k<20;k++) { float mid=.5f*(rpmLo+rpmHi); if(Thrust(mid,inflow)<perMotor) rpmLo=mid; else rpmHi=mid; }
                double throttle=rpmLo/maxRpm;
                float still=Thrust(maxRpm,0);
                airspeed=still>1e-4f?Math.Max(0,Math.Min(2,maximum/still)):1;
                float atThrottle=Thrust((float)throttle*maxRpm,0);
                double here=atThrottle>1e-3f?Math.Max(0,Math.Min(2,Thrust((float)throttle*maxRpm,inflow)/atThrottle)):airspeed;
                low=curve.Thrust(idle)*airspeed; high=curve.Max*airspeed;
                collective=(float)(4*curve.Thrust(throttle)*here*Math.Sign(collective==0?1:collective));
                LastThrottle=(float)throttle;
            }
            // Native pitch/roll outputs have the opposite sign to Unity physical moments.
            double authority=allocator.Allocate(collective,-demand[0],-demand[2],reversed?-high:low,reversed?-low:high,forces);
            int[] slots={0,1,3,2}; // game's output vector swaps the rear motors
            for(int i=0;i<4;i++) {
                motor[slots[i]]=(float)forces[i];
                double command=curve!=null?curve.Throttle(Math.Abs(forces[i])/airspeed)*Math.Sign(forces[i]):forces[i]/maximum;
                rpm[slots[i]]=(float)(command*maxRpm);
            }
            LastForces=forces.Select(f=>(float)f).ToArray();
            if(authority<.999999) {
                StopWinding(pitchPid,pitchIntegral,demand[0]); StopWinding(rollPid,rollIntegral,demand[2]);
            }
            mixedPreviousStep=true; MixCalls++;
        }
        static void StopWinding(object pid,float before,float output) {
            float after=(float)Integral.GetValue(pid);
            if((after-before)*output>0) Integral.SetValue(pid,before);
        }
        internal static object TestPid(TaitBryanAngle axis,float inertia) {
            float p=inertia/Time.fixedDeltaTime;
            return PidConstructor.Invoke(new object[]{axis,new PIDGains {P=p,I=axis==TaitBryanAngle.YAW?0:2*p,D=0},.0793f/p});
        }
        internal static void InstallPatch(Harmony harmony) {
            harmony.Patch(Game.ResolveMethod(GameTokens.ControlStep),
                prefix:new HarmonyMethod(typeof(Plugin).GetMethod("BeforeControlStep",BindingFlags.Static|BindingFlags.NonPublic)),
                transpiler:new HarmonyMethod(typeof(ControllerAdapter).GetMethod(nameof(Transpile),BindingFlags.Static|BindingFlags.NonPublic)));
        }
        static IEnumerable<CodeInstruction> Transpile(IEnumerable<CodeInstruction> input,MethodBase __originalMethod) {
            var code=input.ToList();
            // Match the first powered motor assignment, following both collective branches.
            var setter=typeof(Propeller).GetProperty("TargetRPM").GetSetMethod();
            var assignments=code.Select((instruction,index)=>(instruction,index)).Where(x=>x.instruction.Calls(setter)).ToArray();
            var locals=__originalMethod.GetMethodBody().LocalVariables;
            if(assignments.Length!=5 || locals[3].LocalType!=typeof(float) || locals[4].LocalType!=typeof(float))
                throw new InvalidOperationException("Unsupported motor-output control path.");
            int end=assignments[1].index;
            int start=end;
            while(start>0 && !(code[start].opcode==OpCodes.Ldarg_0 && code[start+1].opcode==OpCodes.Ldfld &&
                ((FieldInfo)code[start+1].operand).FieldType==typeof(List<Propeller>))) start--;
            if(start==0 || code[start+2].opcode!=OpCodes.Ldc_I4_0) throw new InvalidOperationException("Cannot locate powered motor boundary.");
            var injected=new[]{new CodeInstruction(OpCodes.Ldarg_0),new CodeInstruction(OpCodes.Ldloc_S,(byte)4),new CodeInstruction(OpCodes.Ldloc_3),
                new CodeInstruction(OpCodes.Call,typeof(Plugin).GetMethod("MixControlStep",BindingFlags.Static|BindingFlags.NonPublic))};
            // BepInEx ships a framework Harmony; access label collections through reflection
            // to avoid mscorlib/netstandard Label type forwarding incompatibilities at build time.
            var labels=typeof(CodeInstruction).GetField("labels");
            var from=(IList)labels.GetValue(code[start]); var to=(IList)labels.GetValue(injected[0]);
            foreach(var label in from) to.Add(label); from.Clear();
            code.InsertRange(start,injected);
            return code;
        }
    }
}
