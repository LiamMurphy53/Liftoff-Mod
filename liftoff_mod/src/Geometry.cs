using System;
using System.Linq;

namespace GoProGeometry
{
    // Source coordinates: left, backward, up. Motor order: FL, FR, RL, RR.
    [Serializable]
    public sealed class GeometryProfile
    {
        public int schema = 2;
        public string name = "GoPro Drone";
        public string axes = "left_backward_up";
        public float massKg;
        public float[] centerOfMassM;
        public float[] motorsM;
        public float[] inertiaKgM2;
        public string notes;
        // Schema 2. Static thrust of one motor (N) against throttle (0–1). Empty keeps Liftoff's propulsion.
        public float[] thrustThrottle = new float[0];
        public float[] thrustN = new float[0];
        // FPV camera lens position and uptilt above the flat motor plane. Off keeps Liftoff's camera.
        public bool cameraEnabled;
        public float[] cameraM = new float[3];
        public float cameraUptiltDeg = 25f;
        // CAD body shown in place of the stock frame: a mesh file beside the profile, already in profile axes and meters.
        public string bodyModel = "";
        public bool hideStockFrame = true;
        // Editor-only state (CAD import units/axes, file names). The plugin stores it untouched.
        public string editor = "";

        public bool HasThrustCurve => thrustThrottle != null && thrustThrottle.Length > 0;

        public void Validate()
        {
            if ((schema != 1 && schema != 2) || axes != "left_backward_up") throw new ArgumentException("Unsupported profile schema or axes.");
            if (string.IsNullOrWhiteSpace(name)) throw new ArgumentException("Profile needs a name.");
            if (!Finite(massKg) || massKg < 0.05f || massKg > 10f) throw new ArgumentException("Mass must be 0.05–10 kg.");
            Check(centerOfMassM, 3, "Center of gravity");
            Check(motorsM, 12, "Motor positions");
            Check(inertiaKgM2, 9, "Inertia tensor");
            if (motorsM.Any(x => Math.Abs(x) > 2f) || centerOfMassM.Any(x => Math.Abs(x) > 2f))
                throw new ArgumentException("Positions are in meters, with a 2 m limit.");
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
                if (Math.Abs(inertiaKgM2[3*i+j] - inertiaKgM2[3*j+i]) > 1e-8)
                    throw new ArgumentException("Inertia tensor must be symmetric.");
            var e = GeometryMath.PrincipalAxes(GeometryMath.UnityTensor(inertiaKgM2));
            if (e.values.Any(x => x < 1e-7 || x > 10)) throw new ArgumentException("Inertia must be positive definite.");
            for (int i = 0; i < 3; i++)
                if (e.values[i] > e.values[(i+1)%3] + e.values[(i+2)%3] + 1e-8)
                    throw new ArgumentException("Inertia violates the physical triangle inequality.");
            // Explicit labels prevent accidental mirroring/reordering in imported profiles.
            if (!(motorsM[0] > 0 && motorsM[3] < 0 && motorsM[6] > 0 && motorsM[9] < 0 &&
                  motorsM[1] < motorsM[7] && motorsM[4] < motorsM[10]))
                throw new ArgumentException("Motor order must be FL, FR, RL, RR in left/back/up coordinates.");
            for (int i = 0; i < 4; i++) for (int j = i+1; j < 4; j++)
            {
                double d = 0;
                for (int k = 0; k < 3; k++) d += Math.Pow(motorsM[3*i+k] - motorsM[3*j+k], 2);
                if (d < 0.0004) throw new ArgumentException("Motor centers must be at least 20 mm apart.");
            }
            if (HasThrustCurve) ThrustCurve.FromProfile(this);
            if (cameraEnabled)
            {
                Check(cameraM, 3, "Camera position");
                if (cameraM.Any(x => Math.Abs(x) > 2f)) throw new ArgumentException("Camera position is in meters, with a 2 m limit.");
                if (!Finite(cameraUptiltDeg) || cameraUptiltDeg < -30f || cameraUptiltDeg > 80f) throw new ArgumentException("Camera uptilt must be −30° to 80°.");
            }
            if (!string.IsNullOrEmpty(bodyModel) && (bodyModel.Contains("/") || bodyModel.Contains("\\") || bodyModel.Contains("..")))
                throw new ArgumentException("Body model must be a file name beside the profile.");
        }
        static bool Finite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        static void Check(float[] a, int length, string label)
        {
            if (a == null || a.Length != length || a.Any(x => !Finite(x)))
                throw new ArgumentException(label + " has missing or invalid values.");
        }
    }

    // Monotone (PCHIP) interpolation through measured static thrust, so the curve never overshoots between
    // throttle points and can be inverted. Throttle 0–1 → thrust of one motor in newtons.
    public sealed class ThrustCurve
    {
        readonly double[] x, y, slope;
        public double Max => y[y.Length - 1];

        public ThrustCurve(float[] throttle, float[] thrust)
        {
            if (throttle == null || thrust == null || throttle.Length != thrust.Length || throttle.Length < 2)
                throw new ArgumentException("Thrust curve needs at least two throttle/thrust points.");
            if (throttle.Concat(thrust).Any(v => float.IsNaN(v) || float.IsInfinity(v)))
                throw new ArgumentException("Thrust curve has invalid values.");
            var px = throttle.Select(v => (double)v).ToList(); var py = thrust.Select(v => (double)v).ToList();
            if (px[0] > 0) { px.Insert(0, 0); py.Insert(0, 0); }
            for (int i = 0; i < px.Count; i++)
            {
                if (px[i] < 0 || px[i] > 1) throw new ArgumentException("Thrust curve throttle must be 0–100%.");
                if (py[i] < 0 || py[i] > 200) throw new ArgumentException("Thrust curve thrust must be 0–200 N per motor.");
                if (i > 0 && px[i] <= px[i - 1]) throw new ArgumentException("Thrust curve throttle points must increase.");
                if (i > 0 && py[i] < py[i - 1] - 1e-6) throw new ArgumentException("Thrust must not fall as throttle rises.");
            }
            if (px[px.Count - 1] < 0.999) throw new ArgumentException("Thrust curve must reach 100% throttle.");
            if (py[py.Count - 1] < 0.1) throw new ArgumentException("Thrust curve needs real thrust at full throttle.");
            x = px.ToArray(); y = py.ToArray();
            int n = x.Length; var d = new double[n - 1]; slope = new double[n];
            for (int i = 0; i < n - 1; i++) d[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
            slope[0] = d[0]; slope[n - 1] = d[n - 2];
            for (int i = 1; i < n - 1; i++)
            {
                if (d[i - 1] * d[i] <= 0) { slope[i] = 0; continue; }
                double h0 = x[i] - x[i - 1], h1 = x[i + 1] - x[i];
                slope[i] = 3 * (h0 + h1) / ((2 * h1 + h0) / d[i - 1] + (h1 + 2 * h0) / d[i]);
            }
        }

        public static ThrustCurve FromProfile(GeometryProfile p) => new ThrustCurve(p.thrustThrottle, p.thrustN);

        public double Thrust(double throttle)
        {
            throttle = Math.Max(0, Math.Min(1, throttle));
            int i = 0; while (i < x.Length - 2 && throttle > x[i + 1]) i++;
            double h = x[i + 1] - x[i], t = (throttle - x[i]) / h, t2 = t * t, t3 = t2 * t;
            return (2 * t3 - 3 * t2 + 1) * y[i] + (t3 - 2 * t2 + t) * h * slope[i] + (-2 * t3 + 3 * t2) * y[i + 1] + (t3 - t2) * h * slope[i + 1];
        }

        // Lowest throttle that gives this thrust (the curve is monotone, so bisection is exact to 1e-6).
        public double Throttle(double thrust)
        {
            if (thrust <= y[0]) return 0;
            if (thrust >= Max) return 1;
            double lo = 0, hi = 1;
            for (int k = 0; k < 24; k++) { double mid = 0.5 * (lo + hi); if (Thrust(mid) < thrust) lo = mid; else hi = mid; }
            return hi;
        }
    }

    public static class GeometryMath
    {
        // Unity body coordinates: right, up, forward. Tensor transform is A I A^T.
        public static double[] UnityPoint(float[] a, int offset = 0) => new double[] { -a[offset], a[offset+2], -a[offset+1] };
        public static double[,] UnityTensor(float[] i)
        {
            int[] order = { 0, 2, 1 }; int[] sign = { -1, 1, -1 };
            var r = new double[3,3];
            for (int a=0; a<3; a++) for (int b=0; b<3; b++) r[a,b] = sign[a]*sign[b]*i[3*order[a]+order[b]];
            return r;
        }
        // Jacobi diagonalization: columns of rotation are principal axes.
        public static (double[] values, double[,] rotation) PrincipalAxes(double[,] tensor)
        {
            var a = (double[,])tensor.Clone();
            var v = new double[,] {{1,0,0},{0,1,0},{0,0,1}};
            for (int step=0; step<40; step++)
            {
                int p=0, q=1;
                for (int x=0; x<3; x++) for (int y=x+1; y<3; y++)
                    if (Math.Abs(a[x,y]) > Math.Abs(a[p,q])) { p=x; q=y; }
                if (Math.Abs(a[p,q]) < 1e-14) break;
                double angle = 0.5*Math.Atan2(2*a[p,q], a[q,q]-a[p,p]);
                double c=Math.Cos(angle), s=Math.Sin(angle);
                double app=a[p,p], aqq=a[q,q], apq=a[p,q];
                a[p,p]=c*c*app-2*s*c*apq+s*s*aqq;
                a[q,q]=s*s*app+2*s*c*apq+c*c*aqq;
                a[p,q]=a[q,p]=0;
                for (int k=0; k<3; k++)
                {
                    if (k != p && k != q) {
                        double kp=a[k,p], kq=a[k,q];
                        a[k,p]=a[p,k]=c*kp-s*kq; a[k,q]=a[q,k]=s*kp+c*kq;
                    }
                    double vp=v[k,p], vq=v[k,q];
                    v[k,p]=c*vp-s*vq; v[k,q]=s*vp+c*vq;
                }
            }
            return (new[] {a[0,0], a[1,1], a[2,2]}, v);
        }
        public static double[] Moment(double[] point, double[] cg, double thrust)
            => new[] { -(point[2]-cg[2])*thrust, 0.0, (point[0]-cg[0])*thrust };
    }
}
