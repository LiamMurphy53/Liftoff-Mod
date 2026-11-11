using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using GoProGeometry;

var path=args.Length>0?args[0]:"liftoff_mod/profiles/gopro-drone.json";
GeometryProfile Load()=>JsonSerializer.Deserialize<GeometryProfile>(File.ReadAllText(path),new JsonSerializerOptions {IncludeFields=true});
void Check(bool value,string message) { if(!value) throw new Exception(message); Console.WriteLine("PASS "+message); }
void Reject(Action<GeometryProfile> change,string label) { var p=Load(); change(p); bool failed=false; try{p.Validate();}catch(ArgumentException){failed=true;} Check(failed,label); }
var profile=Load();profile.Validate();
var tensor=GeometryMath.UnityTensor(profile.inertiaKgM2);
var eig=GeometryMath.PrincipalAxes(tensor);
double error=0;
for(int i=0;i<3;i++)for(int j=0;j<3;j++) {
    double reconstructed=Enumerable.Range(0,3).Sum(k=>eig.rotation[i,k]*eig.values[k]*eig.rotation[j,k]);
    error=Math.Max(error,Math.Abs(tensor[i,j]-reconstructed));
}
Check(error<1e-12,"full inertia tensor reconstructed from principal axes");
var v=eig.rotation;
double determinant=v[0,0]*(v[1,1]*v[2,2]-v[1,2]*v[2,1])-v[0,1]*(v[1,0]*v[2,2]-v[1,2]*v[2,0])+v[0,2]*(v[1,0]*v[2,1]-v[1,1]*v[2,0]);
Check(Math.Abs(determinant-1)<1e-12,"principal axes form a proper rotation");
var fl=GeometryMath.UnityPoint(profile.motorsM);
Check(fl[0]<0 && fl[2]>0,"front-left stays front-left in Unity");
var cg=GeometryMath.UnityPoint(profile.centerOfMassM);
double[] moment=new double[3];
for(int i=0;i<4;i++){var leverMoment=GeometryMath.Moment(GeometryMath.UnityPoint(profile.motorsM,i*3),cg,1);for(int k=0;k<3;k++)moment[k]+=leverMoment[k];}
Check(Math.Abs(moment[0]-.073504)<1e-7,"deadcat equal-thrust pitch moment preserves CG offset");
Check(Math.Abs(moment[2]-.000084)<1e-7,"small lateral CG offset is preserved");
var a=GeometryMath.Moment(new double[]{.1,0,0},new double[3],1);
var b=GeometryMath.Moment(new double[]{.2,0,0},new double[3],1);
Check(Math.Abs(b[2]-2*a[2])<1e-12,"doubling motor lever arm doubles moment");
Reject(p=>p.massKg=float.NaN,"reject non-finite mass");
Reject(p=>p.motorsM[0]=float.PositiveInfinity,"reject non-finite motor position");
Reject(p=>p.inertiaKgM2[0]=-1,"reject negative inertia");
Reject(p=>p.inertiaKgM2[1]=1,"reject nonsymmetric inertia");
Reject(p=>p.inertiaKgM2=new float[]{.01f,0,0,0,.001f,0,0,0,.001f},"reject impossible principal moments");
Reject(p=>p.axes="right_forward_up","reject ambiguous coordinate convention");
Reject(p=>p.motorsM[0]=-p.motorsM[0],"reject incorrect motor order");
var mixer=MotorAllocator.FromProfile(profile);
var points=Enumerable.Range(0,4).Select(i=>GeometryMath.UnityPoint(profile.motorsM,3*i)).ToArray();
var f=new double[4];
(double pitch,double roll) Moments(double[] output) => (
    Enumerable.Range(0,4).Sum(i=>GeometryMath.Moment(points[i],cg,output[i])[0]),
    Enumerable.Range(0,4).Sum(i=>GeometryMath.Moment(points[i],cg,output[i])[2]));
double hover=profile.massKg*9.80665;
mixer.Allocate(hover,0,0,0,6,f);
var m=Moments(f);
Check(Math.Abs(f.Sum()-hover)<1e-10 && Math.Abs(m.pitch)+Math.Abs(m.roll)<1e-12,"hover balances the actual CG without moving it");
Check(Math.Abs((f[0]+f[1])/hover-.63768)<.0001,"front/rear hover split is approximately 64/36");
mixer.Allocate(hover,.08,-.12,0,6,f);m=Moments(f);
Check(Math.Abs(m.pitch-.08)+Math.Abs(m.roll+.12)<1e-10,"physical pitch/roll demands retain their sign and magnitude");
mixer.Allocate(24,.08,-.12,.24,6,f);m=Moments(f);
Check(f.All(x=>x>=.24 && x<=6) && Math.Abs(m.pitch-.08)+Math.Abs(m.roll+.12)<1e-9 && f.Sum()<24,"full throttle preserves attitude authority by reducing collective");
double scale=mixer.Allocate(hover,20,-10,.24,6,f);m=Moments(f);
Check(scale<1 && f.All(x=>x>=.24 && x<=6) && Math.Abs(m.pitch-20*scale)+Math.Abs(m.roll+10*scale)<1e-8,"impossible moments are reduced together within motor limits");
mixer.Allocate(-hover,.03,.04,-6,-.24,f);m=Moments(f);
Check(f.All(x=>x<=-.24 && x>=-6) && Math.Abs(m.pitch-.03)+Math.Abs(m.roll-.04)<1e-10,"reverse thrust respects bounds and moment signs");
var symmetric=new MotorAllocator(new[]{new[]{-.1,0,.1},new[]{.1,0,.1},new[]{-.1,0,-.1},new[]{.1,0,-.1}},new double[3]);
symmetric.Allocate(8,0,0,0,6,f);
Check(f.All(x=>Math.Abs(x-2)<1e-12),"symmetric geometry gives equal hover thrust");
bool invalid=false;try{new MotorAllocator(points,new[]{1.0,0,0});}catch(ArgumentException){invalid=true;}
Check(invalid,"reject CG outside controllable motor footprint");
var random=new Random(1487);
for(int i=0;i<1000;i++) {
    double pitch=random.NextDouble()*4-2,roll=random.NextDouble()*4-2;
    scale=mixer.Allocate(random.NextDouble()*30,pitch,roll,.24,6,f);m=Moments(f);
    if(!f.All(x=>x>=.24 && x<=6) || Math.Abs(m.pitch-pitch*scale)+Math.Abs(m.roll-roll*scale)>1e-8) throw new Exception("allocation sweep failed");
}
Check(true,"1000 mixed throttle/attitude requests preserve bounded physical moments");

// Thrust curve (schema 2): a thrust-stand sweep of one motor, in newtons.
float[] sweepThrottle={.1f,.2f,.3f,.4f,.5f,.6f,.7f,.8f,.9f,1f};
float[] sweepThrust={.18f,.55f,1.1f,1.9f,2.9f,4.0f,5.3f,6.6f,8.0f,9.4f};
var curve=new ThrustCurve(sweepThrottle,sweepThrust);
Check(Enumerable.Range(0,10).All(i=>Math.Abs(curve.Thrust(sweepThrottle[i])-sweepThrust[i])<1e-6),"thrust curve passes through every measured point");
Check(Math.Abs(curve.Thrust(0))<1e-9 && Math.Abs(curve.Max-9.4)<1e-5,"thrust curve starts from zero and tops out at the 100% measurement");
bool monotone=true; double last=-1;
for(int i=0;i<=1000;i++) { double t=curve.Thrust(i/1000.0); monotone&=t>=last-1e-12; last=t; }
Check(monotone,"interpolated thrust never dips between measured points");
double mid=curve.Thrust(.45);
Check(mid>1.9 && mid<2.9,"interpolates between neighbouring points ("+mid.ToString("F3")+" N at 45%)");
Check(Enumerable.Range(1,99).All(i=>Math.Abs(curve.Thrust(curve.Throttle(curve.Thrust(i/100.0)))-curve.Thrust(i/100.0))<1e-5),"inverse curve recovers throttle for any thrust");
var flat=new ThrustCurve(new[]{0f,.5f,.6f,1f},new[]{0f,3f,3f,5f});
Check(Math.Abs(flat.Thrust(.55)-3)<1e-9,"flat stretches stay flat (no overshoot)");
Reject(p=>{p.thrustThrottle=new[]{0f,.5f};p.thrustN=new[]{0f,3f};},"reject a thrust curve that stops before 100% throttle");
Reject(p=>{p.thrustThrottle=new[]{0f,.6f,.5f,1f};p.thrustN=new[]{0f,2f,3f,5f};},"reject throttle points out of order");
Reject(p=>{p.thrustThrottle=new[]{0f,.5f,1f};p.thrustN=new[]{0f,4f,3f};},"reject thrust that falls as throttle rises");
Reject(p=>{p.cameraEnabled=true;p.cameraUptiltDeg=95;},"reject an impossible camera uptilt");
Reject(p=>{p.bodyModel="../../outside.bin";},"reject a body model path outside the profile folder");
var v1=Load(); v1.schema=1; v1.Validate();
Check(!v1.HasThrustCurve && !v1.cameraEnabled,"schema 1 profiles still load and keep Liftoff's propulsion and camera");

// Drag table: Cd·A against direction of travel, in profile axes (left, back, up).
var cube=new System.Collections.Generic.List<float>(); var cda=new System.Collections.Generic.List<float>();
double Box(double x,double y,double z)=>.004*Math.Abs(x)+.006*Math.Abs(y)+.012*Math.Abs(z); // side, front/back, top/bottom
for(int i=-1;i<=1;i++)for(int j=-1;j<=1;j++)for(int k=-1;k<=1;k++) {
    if(i==0&&j==0&&k==0) continue; double n=Math.Sqrt(i*i+j*j+k*k);
    cube.AddRange(new[]{(float)(i/n),(float)(j/n),(float)(k/n)}); cda.Add((float)Box(i/n,j/n,k/n));
}
var drag=new DragTable(cube.ToArray(),cda.ToArray());
Check(Math.Abs(drag.Area(0,-1,0)-.006)<1e-9 && Math.Abs(drag.Area(0,0,1)-.012)<1e-9,"drag table returns exact entries for listed directions");
double between=drag.Area(0,-Math.Cos(.3),Math.Sin(.3));
Check(between>.006 && between<.012,"drag blends between neighbouring directions ("+(between*1e4).ToString("F0")+" cm²)");
Check(Math.Abs(drag.Area(0,-5,0)-.006)<1e-9,"drag lookup ignores the speed, only the direction");
var unity=GeometryMath.UnityPoint(new[]{.1f,-.2f,.3f}); var back=GeometryMath.ProfileVector(unity[0],unity[1],unity[2]);
Check(Math.Abs(back[0]-.1)+Math.Abs(back[1]+.2)+Math.Abs(back[2]-.3)<1e-6,"Liftoff drag directions convert back to profile axes");
Reject(p=>{p.dragDirections=cube.ToArray();p.dragAreaM2=cda.Select(x=>-x).ToArray();},"reject negative drag areas");
Reject(p=>{p.dragDirections=cube.Select(x=>x*2).ToArray();p.dragAreaM2=cda.ToArray();},"reject drag directions that aren't unit vectors");
Reject(p=>{p.dragDirections=cube.ToArray();p.dragAreaM2=cda.Take(10).ToArray();},"reject a drag table with mismatched lengths");
var front=Enumerable.Range(0,26).Where(i=>cube[3*i+1]<-.1f).ToArray();
Reject(p=>{p.dragDirections=front.SelectMany(i=>new[]{cube[3*i],cube[3*i+1],cube[3*i+2]}).ToArray();p.dragAreaM2=front.Select(i=>cda[i]).ToArray();},"reject a drag table that only covers the front");
var withDrag=Load(); withDrag.dragDirections=cube.ToArray(); withDrag.dragAreaM2=cda.ToArray(); withDrag.Validate();
Check(withDrag.HasDragTable && !v1.HasDragTable,"profiles carry an optional drag table");
