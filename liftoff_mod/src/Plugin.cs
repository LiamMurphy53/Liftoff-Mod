using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using BepInEx;
using BepInEx.Configuration;
using HarmonyLib;
using UnityEngine;

namespace GoProGeometry
{
    [BepInPlugin("com.goprodrone.geometry", "GoPro Geometry Lab", "0.3.0")]
    public class Plugin : BaseUnityPlugin
    {
        internal static readonly BindingFlags Members=BindingFlags.Public|BindingFlags.NonPublic|BindingFlags.Instance;
        internal static Plugin Instance;
        DroneBinding drone;
        GeometryProfile profile;
        Harmony patches;
        FlightManager subscribedManager;
        EditorServer server;
        ConfigEntry<bool> autoApply;
        readonly ConcurrentQueue<Action> mainThread=new ConcurrentQueue<Action>();
        string lastError, profilePath, folder, lastReport="";
        string status="Open single-player Free Flight to bind the drone.";
        bool visible=true, wantApplied, failed;
        float nextScan;
        int lastReportedCalls;

        void Awake()
        {
            Instance=this;
            profilePath=Config.Bind("Geometry","ProfilePath",Path.Combine(Paths.ConfigPath,"GoProGeometry","active.json"),"Editable profile; meters and kg. Changes apply only with Apply.").Value;
            autoApply=Config.Bind("Geometry","ReapplyAfterReset",true,"Apply the profile again on its own after a drone reset, once the drone is still.");
            int port=Config.Bind("Editor","Port",8737,"Local port for the drone editor (127.0.0.1 only; the next free port is used if taken).").Value;
            folder=Path.GetDirectoryName(profilePath);
            var pluginDir=Path.GetDirectoryName(Info.Location);
            try
            {
                var shipped=Path.Combine(pluginDir,"profiles","gopro-drone.json");
                if(!File.Exists(profilePath)) { Directory.CreateDirectory(folder); File.Copy(shipped,profilePath); }
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
            try
            {
                server=new EditorServer(Path.Combine(pluginDir,"editor"),port,Api);
                Logger.LogInfo("Drone editor at "+server.Url);
            }
            catch(Exception e) { Logger.LogError("Drone editor could not start: "+e); }
        }

        void Update()
        {
            while(mainThread.TryDequeue(out var action)) action();
            if(Input.GetKeyDown(KeyCode.F8)) visible=!visible;
            if(visible && Input.GetKeyDown(KeyCode.F9) && profile!=null && drone!=null) Act(Apply);
            if(visible && Input.GetKeyDown(KeyCode.F10) && drone!=null && drone.CanEdit) Act(Restore);
            if(Time.realtimeSinceStartup<nextScan) return;
            nextScan=Time.realtimeSinceStartup+0.5f;
            try
            {
                if(drone!=null && (!drone.IsAlive || drone.Manager.IsResettingDrone))
                { drone.Restore(); drone=null; status="Drone reset."; }
                if(drone==null && patches!=null)
                {
                    drone=DroneBinding.TryFind();
                    if(drone!=null)
                    {
                        if(subscribedManager) subscribedManager.onDroneResetStart-=BeforeReset;
                        subscribedManager=drone.Manager; subscribedManager.onDroneResetStart+=BeforeReset;
                        Logger.LogInfo("Stock drone bound:\n"+drone.Report()); status=wantApplied&&autoApply.Value?"Drone reset. Re-applying when it's still…":"Stock drone ready. Apply when it's still.";
                    }
                }
                // After a reset, put the profile back once the drone has settled.
                if(drone!=null && !drone.Applied && wantApplied && autoApply.Value && drone.CanEdit && !DroneBinding.IsOnline()) Apply();
                if(drone!=null && drone.Applied)
                {
                    if(DroneBinding.IsOnline()) throw new InvalidOperationException("Online session detected; custom geometry restored to stock.");
                    drone.Verify();
                    if(drone.ForceCalls>0 && lastReportedCalls==0) Logger.LogInfo("Verified live Liftoff propeller-force calls using the custom motor positions; geometry mixer steps="+drone.Control.MixCalls+".");
                    lastReportedCalls=drone.ForceCalls;
                }
            }
            catch(Exception e) { if(drone!=null && drone.Applied) drone.Restore(); wantApplied=false; Fail(e); }
        }

        static void BeforeControlStep(Component __instance)
        {
            var d=Instance?.drone;
            if(d!=null && d.Applied && d.Controller==__instance) {
                try { d.Control.BeforeStep(); } catch(Exception e) { d.Restore(); Instance.Fail(e); }
            }
        }
        static void MixControlStep(Component controller,float maximum,float inflow)
        {
            var d=Instance?.drone;
            if(d==null || !d.Applied || d.Controller!=controller) return;
            try { d.Control.Mix(maximum,inflow); }
            catch(Exception e) { d.Restore(); Instance.Fail(e); }
        }
        static void ObserveForce(Propeller __instance)
        {
            var d=Instance?.drone;
            if(d!=null && d.Applied && __instance.ShouldApplyForce && d.Props.Contains(__instance)) d.ForceCalls++;
        }
        void BeforeReset() { drone?.Restore(); drone=null; status="Drone reset."; }

        void LoadProfile() { var p=JsonUtility.FromJson<GeometryProfile>(File.ReadAllText(profilePath)); p.Validate(); profile=p; }
        void Apply()
        {
            if(drone==null) throw new InvalidOperationException("No drone yet. Open single-player Free Flight first.");
            LoadProfile(); drone.Apply(profile,folder); lastReportedCalls=0; wantApplied=true; failed=false;
            status=profile.name+" applied. Ready to fly.";
            lastReport=drone.Report();
            Logger.LogInfo("Applied "+profile.name+":\n"+lastReport);
            File.WriteAllText(Path.Combine(folder,"last-applied.txt"),DateTime.UtcNow.ToString("O")+"\n"+lastReport);
        }
        void Restore()
        {
            wantApplied=false;
            if(drone==null) return;
            drone.Restore(); status="Stock Liftoff drone restored."; Logger.LogInfo("Restored stock drone:\n"+drone.Report());
        }
        internal void LogWarning(string message) => Logger.LogWarning(message);
        void Act(Action action) { try { action(); } catch(Exception e) { Fail(e); } }
        void Fail(Exception e) { status=e.Message; failed=true; if(lastError!=e.Message) { Logger.LogError(e.ToString()); lastError=e.Message; } }

        // ---------------------------------------------------------------- editor API

        [Serializable] class State
        {
            public string game, status, profileName, report, editorUrl;
            public bool bound, canEdit, applied, live, failed, autoApply, wantApplied, compatible;
            public int forceCalls, mixCalls;
            public float throttle, stockMassKg, stockDragCd;
            public float[] forces=new float[4], stockDragDirections=new float[0], stockDragArea=new float[0];
        }
        [Serializable] class Result { public bool ok; public string message, report; }

        // Runs on the server thread; anything that touches the game runs on Unity's main thread.
        Reply Api(string method,string path,Dictionary<string,string> query,byte[] body)
        {
            switch(method+" "+path)
            {
                case "GET /api/state": return OnMain(()=>Reply.Json(JsonUtility.ToJson(StateNow())));
                case "GET /api/profile": return OnMain(()=>Reply.Json(File.ReadAllText(profilePath)));
                case "PUT /api/profile": return OnMain(()=>{
                    var p=JsonUtility.FromJson<GeometryProfile>(Encoding.UTF8.GetString(body));
                    p.schema=2; p.Validate();
                    if(!string.IsNullOrEmpty(p.bodyModel) && !File.Exists(Path.Combine(folder,p.bodyModel))) throw new ArgumentException("Body model "+p.bodyModel+" hasn't been uploaded.");
                    File.WriteAllText(profilePath,JsonUtility.ToJson(p,true)); profile=p;
                    return Ok("Saved "+p.name+".");
                });
                case "PUT /api/model": {
                    var name=ModelName(query);
                    if(body.Length>400_000_000) throw new ArgumentException("Body model is over 400 MB.");
                    var temp=Path.Combine(folder,name+".upload");
                    File.WriteAllBytes(temp,body);
                    try { BodyModel.Read(temp); } catch { File.Delete(temp); throw; }
                    var target=Path.Combine(folder,name);
                    if(File.Exists(target)) File.Delete(target);
                    File.Move(temp,target);
                    return Ok("Uploaded "+name+" ("+(body.Length/1e6).ToString("F1")+" MB).");
                }
                case "GET /api/model": {
                    var file=Path.Combine(folder,ModelName(query));
                    return File.Exists(file)?Reply.Bytes(File.ReadAllBytes(file),"application/octet-stream"):Reply.Json("{\"ok\":false,\"message\":\"No such model.\"}",404);
                }
                case "POST /api/apply": return OnMain(()=>{ Apply(); return Ok(status); });
                case "POST /api/restore": return OnMain(()=>{ Restore(); return Ok("Stock drone restored."); });
                case "POST /api/autoapply": return OnMain(()=>{ autoApply.Value=query.TryGetValue("on",out var on) && on=="1"; return Ok(autoApply.Value?"Re-applies after resets.":"Won't re-apply after resets."); });
            }
            return Reply.Json("{\"ok\":false,\"message\":\"Unknown request.\"}",404);
        }
        static string ModelName(Dictionary<string,string> query)
        {
            if(!query.TryGetValue("name",out var name) || !Regex.IsMatch(name??"",@"^[A-Za-z0-9][A-Za-z0-9._-]{0,80}\.glmb$")) throw new ArgumentException("Model name must be letters, digits, ._- and end in .glmb.");
            return name;
        }
        Reply Ok(string message) => Reply.Json(JsonUtility.ToJson(new Result{ok=true,message=message,report=lastReport}));
        Reply OnMain(Func<Reply> work)
        {
            Reply reply=null; Exception error=null;
            using(var done=new ManualResetEventSlim())
            {
                mainThread.Enqueue(()=>{ try { reply=work(); } catch(Exception e) { error=e; } finally { done.Set(); } });
                if(!done.Wait(8000)) return Reply.Json("{\"ok\":false,\"message\":\"Liftoff didn't respond. Is the game paused in the background?\"}",503);
            }
            if(error!=null) { status=error.Message; failed=true; return Reply.Json(JsonUtility.ToJson(new Result{ok=false,message=error.Message,report=lastReport}),400); }
            return reply;
        }
        State StateNow()
        {
            var s=new State{game=GameTokens.Game,status=status,profileName=profile?.name??"",report=drone!=null?drone.Report():"",editorUrl=server?.Url??"",
                bound=drone!=null,canEdit=drone!=null&&drone.CanEdit,applied=drone!=null&&drone.Applied,failed=failed,autoApply=autoApply.Value,
                wantApplied=wantApplied,compatible=patches!=null};
            if(s.applied) { s.live=drone.ForceCalls>0; s.forceCalls=drone.ForceCalls; s.mixCalls=drone.Control.MixCalls; s.throttle=drone.Control.LastThrottle; s.forces=drone.Control.LastForces; }
            if(drone!=null) { s.stockMassKg=drone.StockMass; s.stockDragCd=drone.StockDragCd; s.stockDragDirections=drone.StockDragDirections; s.stockDragArea=drone.StockDragArea; }
            return s;
        }

        // ---------------------------------------------------------------- in-game panel

        GUIStyle card, title, heading, body, small, button, accent, pill, toggle;
        Texture2D Fill(Color c) { var t=new Texture2D(1,1){hideFlags=HideFlags.HideAndDontSave}; t.SetPixel(0,0,c); t.Apply(); return t; }
        void Styles()
        {
            if(card!=null) return;
            card=new GUIStyle{normal={background=Fill(new Color(.07f,.08f,.10f,.88f))},padding=new RectOffset(18,18,14,16)};
            title=new GUIStyle(GUI.skin.label){fontSize=11,fontStyle=FontStyle.Bold,normal={textColor=new Color(.55f,.62f,.70f)}};
            heading=new GUIStyle(GUI.skin.label){fontSize=20,fontStyle=FontStyle.Bold,normal={textColor=Color.white},wordWrap=true};
            body=new GUIStyle(GUI.skin.label){fontSize=13,richText=true,wordWrap=true,normal={textColor=new Color(.86f,.89f,.93f)}};
            small=new GUIStyle(body){fontSize=11,normal={textColor=new Color(.55f,.62f,.70f)}};
            button=new GUIStyle(GUI.skin.button){fontSize=13,fixedHeight=32,normal={background=Fill(new Color(.18f,.20f,.24f)),textColor=Color.white},
                hover={background=Fill(new Color(.25f,.28f,.33f)),textColor=Color.white},active={background=Fill(new Color(.12f,.13f,.16f)),textColor=Color.white},
                border=new RectOffset(0,0,0,0)};
            accent=new GUIStyle(button){fontStyle=FontStyle.Bold,normal={background=Fill(new Color(.98f,.45f,.09f)),textColor=Color.white},
                hover={background=Fill(new Color(1f,.55f,.2f)),textColor=Color.white},active={background=Fill(new Color(.85f,.38f,.06f)),textColor=Color.white}};
            pill=new GUIStyle(button){fontSize=12,fixedHeight=28,alignment=TextAnchor.MiddleLeft,padding=new RectOffset(12,12,0,0),richText=true,
                normal={background=Fill(new Color(.07f,.08f,.10f,.85f)),textColor=Color.white}};
            toggle=new GUIStyle(GUI.skin.toggle){fontSize=12,normal={textColor=new Color(.86f,.89f,.93f)},onNormal={textColor=new Color(.86f,.89f,.93f)}};
        }
        string Dot(string colour) => "<color="+colour+">●</color>";
        (string text,string colour) StatusLine()
        {
            if(patches==null) return ("Disabled on this Liftoff build","#f87171");
            if(drone==null) return ("No drone. Open Free Flight","#94a3b8");
            if(drone.Applied && drone.ForceCalls>0) return ("LIVE: "+profile?.name,"#4ade80");
            if(drone.Applied) return ("Applied: arm to go live","#facc15");
            return ("Stock Liftoff drone","#94a3b8");
        }
        GeometryProfile factsFor; string factsLine;
        static string Facts(GeometryProfile p)
        {
            var facts=new List<string>{(p.massKg*1000).ToString("F0")+" g"};
            if(p.HasThrustCurve)
            {
                var c=ThrustCurve.FromProfile(p);
                facts.Add((c.Max/9.80665*1000).ToString("F0")+" g/motor");
                double hover=p.massKg*9.80665/4;
                facts.Add(hover<c.Max?"hover "+(c.Throttle(hover)*100).ToString("F0")+"%":"can't hover");
            }
            else facts.Add("Liftoff thrust");
            facts.Add(p.cameraEnabled?"cam "+p.cameraUptiltDeg.ToString("F0")+"°":"Liftoff cam");
            if(!string.IsNullOrEmpty(p.bodyModel)) facts.Add("CAD body");
            if(p.HasDragTable) facts.Add("CdA "+(DragTable.FromProfile(p).Area(0,-1,0)*1e4).ToString("F0")+" cm²");
            return string.Join("  ·  ",facts);
        }
        void OnGUI()
        {
            var matrix=GUI.matrix; bool enabled=GUI.enabled;
            try { Styles(); DrawGui(); }
            finally { GUI.matrix=matrix; GUI.enabled=enabled; }
        }
        void DrawGui()
        {
            float scale=Mathf.Max(1f,Screen.height/950f);
            GUI.matrix=Matrix4x4.Scale(new Vector3(scale,scale,1));
            var (line,colour)=StatusLine();
            if(!visible)
            {
                if(GUI.Button(new Rect(16,16,260,28),Dot(colour)+"  Geometry Lab · F8",pill)) visible=true;
                return;
            }
            GUILayout.BeginArea(new Rect(16,16,360,Screen.height/scale-32));
            GUILayout.BeginVertical(card);
            GUILayout.BeginHorizontal();
            GUILayout.Label("GEOMETRY LAB",title);
            GUILayout.FlexibleSpace();
            if(GUILayout.Button("Hide  F8",small)) visible=false;
            GUILayout.EndHorizontal();
            GUILayout.Label(profile?.name??"No profile",heading);
            GUILayout.Label(Dot(colour)+"  "+line,body);
            GUILayout.Space(6);
            if(profile!=null)
            {
                // The facts line validates curves and tables, so build it once per profile, not per GUI pass.
                if(!ReferenceEquals(factsFor,profile)) { factsLine=Facts(profile); factsFor=profile; }
                GUILayout.Label(factsLine,small);
            }
            if(drone!=null && drone.Applied && profile!=null && profile.HasThrustCurve && drone.Control.MixCalls>0)
                GUILayout.Label("Throttle "+(drone.Control.LastThrottle*100).ToString("F0")+"%   motors "+string.Join(" ",drone.Control.LastForces.Select(f=>(f/9.80665*1000).ToString("F0")))+" g",small);
            GUILayout.Space(10);
            GUILayout.BeginHorizontal();
            GUI.enabled=profile!=null && drone!=null && drone.CanEdit;
            if(GUILayout.Button(drone!=null&&drone.Applied?"Re-apply  F9":"Apply  F9",accent)) Act(Apply);
            GUI.enabled=drone!=null && drone.CanEdit;
            if(GUILayout.Button("Stock  F10",button)) Act(Restore);
            GUI.enabled=true;
            GUILayout.EndHorizontal();
            GUILayout.Space(6);
            GUI.enabled=server!=null;
            if(GUILayout.Button("Open drone editor ↗",button)) Application.OpenURL(server.Url);
            GUI.enabled=true;
            GUILayout.Space(8);
            bool auto=GUILayout.Toggle(autoApply.Value," Re-apply after resets",toggle);
            if(auto!=autoApply.Value) autoApply.Value=auto;
            GUILayout.Space(6);
            GUILayout.Label(failed?"<color=#f87171>"+status+"</color>":status,small);
            GUILayout.EndVertical();
            GUILayout.EndArea();
        }

        void OnDestroy()
        {
            server?.Dispose();
            if(subscribedManager) subscribedManager.onDroneResetStart-=BeforeReset;
            drone?.Restore(); patches?.UnpatchSelf(); if(Instance==this) Instance=null;
        }
    }
}
