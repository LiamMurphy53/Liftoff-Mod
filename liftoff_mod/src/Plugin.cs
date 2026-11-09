using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using BepInEx;
using HarmonyLib;
using UnityEngine;

namespace GoProGeometry
{
    [BepInPlugin("com.goprodrone.geometry", "GoPro Geometry Lab", "0.1.0")]
    public class Plugin : BaseUnityPlugin
    {
        internal static readonly BindingFlags Members=BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance;
        internal static Plugin Instance;
        DroneBinding drone;
        GeometryProfile profile;
        Harmony patches;
        FlightManager subscribedManager;
        string lastError;
        string profilePath, status="Open single-player Free Flight, then reset the drone.";
        string[] motorText=new string[12], cgText=new string[3];
        string massText;
        bool visible=true;
        float nextScan;
        int lastReportedCalls;
        void Awake()
        {
            Instance=this;
            profilePath=Config.Bind("Geometry","ProfilePath",Path.Combine(Paths.ConfigPath,"GoProGeometry","active.json"),"Editable profile; meters and kg. Changes apply only with the Apply button.").Value;
            try
            {
                var shipped=Path.Combine(Path.GetDirectoryName(Info.Location),"profiles","gopro-drone.json");
                if(!File.Exists(profilePath)) { Directory.CreateDirectory(Path.GetDirectoryName(profilePath)); File.Copy(shipped,profilePath); }
                LoadProfile();
                if(typeof(FlightManager).Module.ModuleVersionId.ToString()!=DroneBinding.SupportedModule)
                    throw new InvalidOperationException("Unverified Liftoff build. Geometry changes are disabled.");
                PhysicsCheck.Run(Logger,JsonUtility.FromJson<GeometryProfile>(File.ReadAllText(shipped)));
                patches=new Harmony("com.goprodrone.geometry");
                ControllerAdapter.InstallPatch(patches);
                patches.Patch(typeof(Propeller).GetMethod("ApplyControllerForceAtPropeller"),prefix:new HarmonyMethod(typeof(Plugin).GetMethod(nameof(ObserveForce),BindingFlags.NonPublic|BindingFlags.Static)));
                Logger.LogInfo("Geometry Lab ready; "+GameTokens.Game+". Stock physics selected until Apply.");
            }
            catch(Exception e) { patches?.UnpatchSelf(); patches=null; Fail(e); }
        }
        void Update()
        {
            if(Input.GetKeyDown(KeyCode.F8)) visible=!visible;
            if(visible && Input.GetKeyDown(KeyCode.F9) && profile!=null && drone!=null) Act(Apply);
            if(visible && Input.GetKeyDown(KeyCode.F10) && drone!=null && drone.CanEdit) Act(()=>{drone.Restore();status="Stock geometry, mass properties and controller restored.";Logger.LogInfo("Restored stock drone:\n"+drone.Report());});
            if(Time.realtimeSinceStartup<nextScan) return;
            nextScan=Time.realtimeSinceStartup+0.5f;
            try
            {
                if(drone!=null && (!drone.IsAlive || drone.Manager.IsResettingDrone))
                { drone.Restore(); drone=null; status="Drone reset. Apply your profile again when stopped."; }
                if(drone==null && patches!=null)
                {
                    drone=DroneBinding.TryFind();
                    if(drone!=null)
                    {
                        if(subscribedManager) subscribedManager.onDroneResetStart-=BeforeReset;
                        subscribedManager=drone.Manager; subscribedManager.onDroneResetStart+=BeforeReset;
                        Logger.LogInfo("Stock drone bound:\n"+drone.Report()); status="Stock drone ready. Reset/stop, then choose GoPro Drone.";
                    }
                }
                if(drone!=null && drone.Applied)
                {
                    if(DroneBinding.IsOnline()) throw new InvalidOperationException("Online session detected; custom geometry restored to stock.");
                    drone.Verify();
                    if(drone.ForceCalls>0 && lastReportedCalls==0) Logger.LogInfo("Verified live Liftoff propeller-force calls using the custom motor positions; geometry mixer steps="+drone.Control.MixCalls+".");
                    lastReportedCalls=drone.ForceCalls;
                }
            }
            catch(Exception e) { if(drone!=null && drone.Applied) drone.Restore(); Fail(e); }
        }
        static void BeforeControlStep(Component __instance)
        {
            var d=Instance?.drone;
            if(d!=null && d.Applied && d.Controller==__instance) {
                try { d.Control.BeforeStep(); } catch(Exception e) { d.Restore(); Instance.Fail(e); }
            }
        }
        static void MixControlStep(Component controller,float maximum)
        {
            var d=Instance?.drone;
            if(d==null || !d.Applied || d.Controller!=controller) return;
            try { d.Control.Mix(maximum); }
            catch(Exception e) { d.Restore(); Instance.Fail(e); }
        }
        static void ObserveForce(Propeller __instance)
        {
            var d=Instance?.drone;
            if(d!=null && d.Applied && __instance.ShouldApplyForce && d.Props.Contains(__instance)) d.ForceCalls++;
        }
        void BeforeReset()
        {
            drone?.Restore(); drone=null; status="Drone reset. Apply your profile again when stopped.";
        }
        void LoadProfile()
        {
            var p=JsonUtility.FromJson<GeometryProfile>(File.ReadAllText(profilePath)); p.Validate(); profile=p;
            for(int i=0;i<12;i++) motorText[i]=(p.motorsM[i]*1000f).ToString("F3",CultureInfo.InvariantCulture);
            for(int i=0;i<3;i++) cgText[i]=(p.centerOfMassM[i]*1000f).ToString("F3",CultureInfo.InvariantCulture);
            massText=(p.massKg*1000f).ToString("F3",CultureInfo.InvariantCulture);
        }
        GeometryProfile Edited()
        {
            var p=JsonUtility.FromJson<GeometryProfile>(JsonUtility.ToJson(profile));
            p.motorsM=motorText.Select(ParseMm).ToArray(); p.centerOfMassM=cgText.Select(ParseMm).ToArray(); p.massKg=ParseMm(massText); p.Validate(); return p;
        }
        static float ParseMm(string s)=>float.Parse(s,NumberStyles.Float,CultureInfo.InvariantCulture)/1000f;
        void Apply()
        {
            var p=Edited(); drone.Apply(p); profile=p; lastReportedCalls=0;
            status="GoPro geometry and controller applied. Ready to fly.";
            Logger.LogInfo("Applied "+p.name+":\n"+drone.Report());
            File.WriteAllText(Path.Combine(Path.GetDirectoryName(profilePath),"last-applied.txt"),DateTime.UtcNow.ToString("O")+"\n"+drone.Report());
        }
        void Act(Action action) { try { action(); } catch(Exception e) { Fail(e); } }
        void Fail(Exception e) { status=e.Message; if(lastError!=e.Message) { Logger.LogError(e.ToString()); lastError=e.Message; } }
        void OnGUI()
        {
            var matrix=GUI.matrix; bool enabled=GUI.enabled;
            try { DrawGui(); }
            finally { GUI.matrix=matrix; GUI.enabled=enabled; }
        }
        void DrawGui()
        {
            float scale=Mathf.Max(1f,Screen.height/950f);
            GUI.matrix=Matrix4x4.Scale(new Vector3(scale,scale,1));
            if(GUI.Button(new Rect(16,16,220,32),"GoPro Geometry Lab  •  F8")) visible=!visible;
            if(!visible) return;
            GUI.Box(new Rect(16,56,620,626),"GoPro Drone • experimental geometry lab");
            GUI.Label(new Rect(32,88,590,38),"Free Flight only. Stop first. F9: apply. F10: restore stock.");
            var applied=drone!=null && drone.Applied;
            GUI.Label(new Rect(32,116,590,28),"Flying: "+(applied?drone.ProfileName+" geometry":"Stock Liftoff drone"));
            GUI.enabled=profile!=null && drone!=null && drone.CanEdit;
            if(GUI.Button(new Rect(32,149,274,34),"Apply GoPro Drone / edited geometry")) Act(Apply);
            if(GUI.Button(new Rect(318,149,298,34),"Restore stock drone")) Act(()=>{drone.Restore();status="Stock geometry, mass properties and controller restored.";Logger.LogInfo("Restored stock drone:\n"+drone.Report());});
            GUI.enabled=profile!=null;
            GUI.Label(new Rect(32,195,590,24),"Motor centers (mm)      Left +X        Back +Y           Up +Z");
            for(int row=0;row<4;row++)
            {
                GUI.Label(new Rect(32,225+34*row,130,28),new[]{"Front left","Front right","Rear left","Rear right"}[row]);
                for(int col=0;col<3;col++) { int i=3*row+col; motorText[i]=GUI.TextField(new Rect(176+142*col,225+34*row,130,28),motorText[i]??""); }
            }
            GUI.Label(new Rect(32,372,130,28),"Mass (grams)"); massText=GUI.TextField(new Rect(176,372,130,28),massText??"");
            GUI.Label(new Rect(32,409,130,28),"CG (mm X/Y/Z)");
            for(int i=0;i<3;i++) cgText[i]=GUI.TextField(new Rect(176+142*i,409,130,28),cgText[i]??"");
            GUI.Label(new Rect(32,448,586,46),"Inertia comes from the profile JSON and stays fixed when motors move.\nPropulsion, drag and frame collisions remain Liftoff's.");
            if(GUI.Button(new Rect(32,504,178,30),"Save edits")) Act(()=>{var p=Edited();File.WriteAllText(profilePath,JsonUtility.ToJson(p,true));status="Profile saved. Press Apply to fly these values.";});
            if(GUI.Button(new Rect(220,504,190,30),"Reload profile JSON")) Act(()=>{LoadProfile();status="Profile loaded. Press Apply to use it.";});
            if(GUI.Button(new Rect(420,504,196,30),"Open profile folder")) Application.OpenURL(new Uri(Path.GetDirectoryName(profilePath)).AbsoluteUri);
            GUI.enabled=true;
            GUI.Label(new Rect(32,546,584,54),applied && drone.ForceCalls>0 ? "LIVE: "+drone.ForceCalls+" Liftoff motor-force calls observed at custom positions." : status);
            GUI.Label(new Rect(32,612,584,52),"Geometry experiment, not a calibrated GoPro Drone model.\n"+(applied?"Geometry-aware mixer + controller active. Yaw remains simplified.":"Apply includes geometry-aware mixing and controller compensation."));
        }
        void OnDestroy() { if(subscribedManager) subscribedManager.onDroneResetStart-=BeforeReset; drone?.Restore(); patches?.UnpatchSelf(); if(Instance==this) Instance=null; }
    }
}
