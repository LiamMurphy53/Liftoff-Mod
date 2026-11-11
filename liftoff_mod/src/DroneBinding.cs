using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using UnityEngine;

namespace GoProGeometry
{
    internal sealed class DroneBinding
    {
        internal const string SupportedModule = GameTokens.Module;
        internal readonly Rigidbody Body;
        internal readonly Propeller[] Props;
        internal readonly Component Controller;
        internal readonly FlightManager Manager;
        internal bool Applied { get; private set; }
        internal int ForceCalls;
        internal string ProfileName;
        internal readonly Vector3[] StockPoints;
        readonly float mass;
        readonly Vector3 cg, inertia;
        readonly Quaternion inertiaRotation;
        readonly Dictionary<Transform, Vector3> originalPositions = new Dictionary<Transform, Vector3>();
        readonly Transform[] motorRoots;
        Vector3[] targetPoints;
        Vector3 targetCg, targetInertia;
        Quaternion targetRotation;
        float targetMass;
        internal ControllerAdapter Control { get; private set; }
        DragAdapter drag;
        // Liftoff's own drag table for this drone (profile axes, Cd·A m²), read before any change.
        internal readonly float[] StockDragDirections=new float[0], StockDragArea=new float[0];
        internal readonly float StockDragCd;
        readonly FramePart frame;
        readonly float stockCameraAngle;
        GameObject bodyRoot;
        readonly List<Renderer> hidden = new List<Renderer>();
        string cameraReport = "Liftoff camera";

        internal static DroneBinding TryFind()
        {
            var fm = UnityEngine.Object.FindObjectOfType<FlightManager>();
            if (!fm || fm.IsResettingDrone) return null;
            object state = typeof(FlightManager).GetProperty("CurrentDrone").GetValue(fm);
            if (state == null) return null;
            if (typeof(FlightManager).Module.ModuleVersionId.ToString() != SupportedModule)
                throw new InvalidOperationException("This Liftoff build needs a compatibility check. No geometry changed.");
            var properties = state.GetType().GetProperties(Plugin.Members);
            var body = (Rigidbody)properties.Single(p => p.PropertyType == typeof(Rigidbody)).GetValue(state);
            if (!body) return null;
            var controller = body.GetComponents<Component>().SingleOrDefault(c => c && c.GetType().MetadataToken == GameTokens.Controller && c.GetType().Assembly == typeof(FlightManager).Assembly);
            if (!controller) throw new InvalidOperationException("This prototype supports the inspected ZetaFlight controller only.");
            return new DroneBinding(fm, body, controller);
        }
        DroneBinding(FlightManager fm, Rigidbody body, Component controller)
        {
            Manager=fm; Body=body; Controller=controller;
            if ((body.transform.lossyScale-Vector3.one).sqrMagnitude > 1e-6f)
                throw new InvalidOperationException("Scaled drone roots are not supported.");
            var props=body.GetComponentsInChildren<Propeller>();
            if (props.Length != 4) throw new InvalidOperationException("Expected four propellers.");
            // Resolve labels from the actual flight body's right/up/forward axes.
            var frontToBack=props.OrderByDescending(p => Point(p).z).ToArray();
            Props=frontToBack.Take(2).OrderBy(p => Point(p).x).Concat(frontToBack.Skip(2).OrderBy(p => Point(p).x)).ToArray();
            StockPoints=Props.Select(Point).ToArray();
            var center=StockPoints.Aggregate(Vector3.zero,(s,p)=>s+p)/4f;
            if (!(StockPoints[0].x<center.x && StockPoints[1].x>center.x && StockPoints[2].x<center.x && StockPoints[3].x>center.x &&
                  StockPoints[0].z>center.z && StockPoints[1].z>center.z && StockPoints[2].z<center.z && StockPoints[3].z<center.z))
                throw new InvalidOperationException("Cannot verify motor ordering and body axes.");
            motorRoots=new Transform[4];
            var motors=body.GetComponentsInChildren<MotorPart>();
            for(int i=0;i<4;i++)
            {
                var ancestor=Props[i].GetComponentInParent<MotorPart>();
                motorRoots[i]=ancestor ? ancestor.transform : motors.OrderBy(m=>(m.transform.position-Props[i].transform.position).sqrMagnitude).FirstOrDefault()?.transform;
                Remember(Props[i].transform);
                Remember(Props[i].GetComponentInParent<PropellerPart>()?.transform);
                Remember(motorRoots[i]);
            }
            if (motorRoots.Any(t=>!t) || motorRoots.Distinct().Count()!=4)
                throw new InvalidOperationException("Cannot resolve four independent motor assemblies.");
            mass=Body.mass; cg=Body.centerOfMass; inertia=Body.inertiaTensor; inertiaRotation=Body.inertiaTensorRotation;
            frame=body.GetComponentInChildren<FramePart>();
            if (frame) { stockCameraAngle=frame.CameraAngle; if (frame.cameraPart) Remember(frame.cameraPart.transform); }
            if (frame)
                try { StockDragCd=frame.DragCoefficient; (StockDragDirections,StockDragArea)=DragAdapter.Read(controller,StockDragCd); }
                catch (Exception e) { Plugin.Instance?.LogWarning("Drag table unavailable: "+e.Message); }
        }
        void Remember(Transform t) { if(t && !originalPositions.ContainsKey(t)) originalPositions.Add(t,t.localPosition); }
        Vector3 Point(Propeller p) => Body.transform.InverseTransformPoint(p.transform.position);
        internal float StockMass => mass;
        internal bool IsAlive => Body && Controller && Props.All(p=>p);
        internal bool CanEdit => IsAlive && !Manager.IsResettingDrone && Body.velocity.sqrMagnitude<0.0625f && Body.angularVelocity.sqrMagnitude<0.25f;
        internal static bool IsOnline()
        {
            var type=Type.GetType("Photon.Pun.PhotonNetwork, PhotonUnityNetworking");
            if(type==null) throw new InvalidOperationException("Cannot verify the network mode.");
            bool inRoom=(bool)type.GetProperty("InRoom",BindingFlags.Public|BindingFlags.Static).GetValue(null);
            bool offline=(bool)type.GetProperty("OfflineMode",BindingFlags.Public|BindingFlags.Static).GetValue(null);
            return inRoom && !offline;
        }
        internal void Apply(GeometryProfile p, string folder)
        {
            p.Validate();
            if(!CanEdit) throw new InvalidOperationException("Reset and stop the drone before changing geometry.");
            if(IsOnline()) throw new InvalidOperationException("Geometry Lab is for single-player Free Flight.");
            var targets=Enumerable.Range(0,4).Select(i=>Vector(GeometryMath.UnityPoint(p.motorsM,3*i))).ToArray();
            // Preserve the stock motor-plane location relative to its collision mesh.
            var offset=StockPoints.Aggregate(Vector3.zero,(s,v)=>s+v)/4f-targets.Aggregate(Vector3.zero,(s,v)=>s+v)/4f;
            var eig=GeometryMath.PrincipalAxes(GeometryMath.UnityTensor(p.inertiaKgM2));
            Restore();
            targetPoints=targets.Select(v=>v+offset).ToArray(); targetCg=Vector(GeometryMath.UnityPoint(p.centerOfMassM))+offset;
            targetInertia=Vector(eig.values); targetMass=p.massKg;
            targetRotation=Quaternion.LookRotation(Column(eig.rotation,2),Column(eig.rotation,1));
            // Read the CAD body before changing anything, so a bad file leaves the stock drone untouched.
            var parts=string.IsNullOrEmpty(p.bodyModel)?null:BodyModel.Read(System.IO.Path.Combine(folder,p.bodyModel));
            if((parts!=null || p.cameraEnabled) && !frame) throw new InvalidOperationException("This drone has no frame part for the camera or CAD body.");
            try
            {
                for(int i=0;i<4;i++)
                {
                    var desired=Body.transform.TransformPoint(targetPoints[i]);
                    motorRoots[i].position+=desired-Props[i].transform.position;
                    var propPart=Props[i].GetComponentInParent<PropellerPart>();
                    var move=propPart ? propPart.transform : Props[i].transform;
                    move.position+=desired-Props[i].transform.position;
                }
                Body.mass=targetMass; Body.centerOfMass=targetCg;
                Body.inertiaTensor=targetInertia; Body.inertiaTensorRotation=targetRotation;
                var r=Matrix4x4.Rotate(inertiaRotation);
                var stockBodyInertia=new Vector3(
                    r.m00*r.m00*inertia.x+r.m01*r.m01*inertia.y+r.m02*r.m02*inertia.z,
                    r.m10*r.m10*inertia.x+r.m11*r.m11*inertia.y+r.m12*r.m12*inertia.z,
                    r.m20*r.m20*inertia.x+r.m21*r.m21*inertia.y+r.m22*r.m22*inertia.z);
                if(p.cameraEnabled) PlaceCamera(Vector(GeometryMath.UnityPoint(p.cameraM))+offset,p.cameraUptiltDeg);
                if(parts!=null)
                {
                    var stock=frame.rendererRoot?frame.rendererRoot.GetComponentsInChildren<Renderer>():new Renderer[0];
                    bodyRoot=BodyModel.Attach(Body.transform,offset,parts,stock.Select(x=>x.sharedMaterial).FirstOrDefault(m=>m));
                    if(p.hideStockFrame)
                        foreach(var piece in stock)
                            if(piece.enabled && !piece.GetComponentInParent<MotorPart>() && !piece.GetComponentInParent<PropellerPart>() && !piece.GetComponentInParent<CameraPart>())
                            { piece.enabled=false; hidden.Add(piece); }
                }
                Control=new ControllerAdapter(Controller,p,stockBodyInertia); Control.Install(Props);
                if(p.HasDragTable)
                {
                    if(!frame) throw new InvalidOperationException("This drone has no frame part to read its drag coefficient from.");
                    drag=new DragAdapter(Controller,p,frame.DragCoefficient); drag.Install();
                }
                Applied=true; ForceCalls=0; ProfileName=p.name;
                Verify();
            }
            catch { Restore(); throw; }
        }
        internal void Verify()
        {
            if(!Applied) return;
            Control.Verify(); drag?.Verify();
            if(!IsAlive || Math.Abs(Body.mass-targetMass)>1e-5f || (Body.centerOfMass-targetCg).magnitude>1e-5f ||
               (Body.inertiaTensor-targetInertia).magnitude>1e-6f || Quaternion.Angle(Body.inertiaTensorRotation,targetRotation)>0.1f ||
               // Large Liftoff maps lose sub-millimeter precision in world/local conversions.
               Enumerable.Range(0,4).Any(i=>(Point(Props[i])-targetPoints[i]).magnitude>0.001f))
                throw new InvalidOperationException("Liftoff changed geometry or mass properties; custom geometry has been disabled.");
        }
        // Tilt first (the camera mount pivots), then move the mount so the FPV lens lands on the CAD point.
        void PlaceCamera(Vector3 lens,float uptilt)
        {
            frame.CameraAngle=uptilt;
            var mount=frame.cameraPart?frame.cameraPart.transform:null;
            if(!mount) { cameraReport="uptilt "+uptilt.ToString("F1")+"°; no camera mount to move"; return; }
            var view=Body.GetComponentsInChildren<Camera>(true).FirstOrDefault(c=>c.transform.IsChildOf(mount));
            var from=view?view.transform.position:mount.position;
            mount.position+=Body.transform.TransformPoint(lens)-from;
            cameraReport="uptilt "+uptilt.ToString("F1")+"°; "+(view?"FPV lens":"camera mount")+" at "+Body.transform.InverseTransformPoint(view?view.transform.position:mount.position).ToString("F4");
        }
        internal void Restore()
        {
            Applied=false; ForceCalls=0;
            Control?.Restore(); Control=null;
            drag?.Restore(); drag=null;
            BodyModel.Destroy(bodyRoot); bodyRoot=null;
            foreach(var r in hidden) if(r) r.enabled=true;
            hidden.Clear(); cameraReport="Liftoff camera";
            if(!Body) return;
            if(frame) frame.CameraAngle=stockCameraAngle;
            foreach(var pair in originalPositions) if(pair.Key) pair.Key.localPosition=pair.Value;
            Body.mass=mass; Body.centerOfMass=cg; Body.inertiaTensor=inertia; Body.inertiaTensorRotation=inertiaRotation;
        }
        internal string Report() => "mass="+Body.mass.ToString("F6")+" kg; CG="+Body.centerOfMass.ToString("F6")+"; principal inertia="+Body.inertiaTensor.ToString("F6")+
            "; rotation="+Body.inertiaTensorRotation+"; controller="+(Control?.Report??"stock")+"; camera: "+cameraReport+"; body: "+(bodyRoot?"CAD model":"stock frame")+"; "+(drag?.Report??"Liftoff drag")+"\n"+string.Join("\n",Props.Select((p,i)=>new[]{"FL","FR","RL","RR"}[i]+"="+Point(p).ToString("F6")));
        static Vector3 Vector(double[] a)=>new Vector3((float)a[0],(float)a[1],(float)a[2]);
        static Vector3 Column(double[,] a,int j)=>new Vector3((float)a[0,j],(float)a[1,j],(float)a[2,j]);
    }
}
