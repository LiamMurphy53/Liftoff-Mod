using Mono.Cecil;
using Mono.Cecil.Cil;

// Finds the obfuscated Liftoff members the plugin needs, by structure rather than by name, and writes
// src/GameTokens.cs. Every match must be unique; anything unexpected stops with an error instead of guessing.
// Usage: dotnet Inspector.dll <Assembly-CSharp.dll> locate <path/to/GameTokens.cs> "<version, build>"
static class Locator
{
    static T One<T>(IEnumerable<T> items, string what)
    {
        var list = items.ToList();
        if (list.Count != 1) throw new InvalidOperationException($"Expected exactly one {what}, found {list.Count}.");
        return list[0];
    }

    static IEnumerable<TypeDefinition> Walk(IEnumerable<TypeDefinition> types)
    {
        foreach (var t in types) { yield return t; foreach (var n in Walk(t.NestedTypes)) yield return n; }
    }

    static bool Calls(Instruction i, string name) => i.Operand is MethodReference r && r.Name == name;
    static int Tok(IMetadataTokenProvider p) => p.MetadataToken.ToInt32();
    static FieldDefinition FieldOf(Instruction i) => ((FieldReference)i.Operand).Resolve();

    public static void Run(AssemblyDefinition assembly, string output, string label)
    {
        var types = Walk(assembly.MainModule.Types).ToArray();

        // ZetaFlight's flight controller component: holds its settings, derives from the controller base
        // that owns the propeller list, and sets the four motors (plus one extra) in FixedUpdate.
        var controller = One(types.Where(t =>
            t.Fields.Any(f => f.FieldType.Name == "ZetaFlightControllerSettings") &&
            t.BaseType?.Resolve()?.Fields.Any(f => f.FieldType.FullName == "System.Collections.Generic.List`1<Propeller>") == true &&
            t.Methods.Any(m => m.Name == "FixedUpdate")), "ZetaFlight controller type");
        var step = One(controller.Methods.Where(m => m.Name == "FixedUpdate"), "control step");
        var code = step.Body.Instructions;
        var targetRpm = code.Where(i => Calls(i, "set_TargetRPM")).ToArray();
        if (targetRpm.Length != 5) throw new InvalidOperationException($"Control step sets TargetRPM {targetRpm.Length} times, expected 5.");
        if (step.Body.Variables.Count <= 4 || step.Body.Variables[4].VariableType.FullName != "System.Single" || step.Body.Variables[3].VariableType.FullName != "System.Single")
            throw new InvalidOperationException("Control step locals 3 (inflow) and 4 (thrust limit) are no longer floats.");

        var settings = One(controller.Fields.Where(f => f.FieldType.Name == "ZetaFlightControllerSettings"), "settings field");
        var pids = One(controller.Fields.Where(f => f.FieldType is GenericInstanceType g &&
            g.ElementType.FullName == "System.Collections.Generic.Dictionary`2" && g.GenericArguments[0].FullName == "TaitBryanAngle"), "PID dictionary");
        var pidInterface = ((GenericInstanceType)pids.FieldType).GenericArguments[1].Resolve();

        // The first TargetRPM call is the disarmed branch; the second is the first powered motor:
        // TargetRPM = Max(|rpm[0,0]|, idle * maxRpm). Its operands name the RPM vector and max RPM.
        int first = code.IndexOf(targetRpm[1]);
        var window = code.Skip(first - 12).Take(12).ToArray();
        var rpm = One(window.Where(i => i.OpCode == OpCodes.Ldfld && Tok(FieldOf(i).DeclaringType) == Tok(controller) &&
            FieldOf(i).FieldType.FullName != "System.Single" && FieldOf(i).FieldType.FullName != "System.Collections.Generic.List`1<Propeller>").Select(FieldOf), "RPM vector");
        var maxRpm = One(window.Where(i => i.OpCode == OpCodes.Ldfld && FieldOf(i).FieldType.FullName == "System.Single").Select(FieldOf), "max RPM field");
        // The motor force vector is what the controller hands to each propeller's force call after TargetRPM.
        var forces = code.Skip(first).Where(i => i.OpCode == OpCodes.Ldfld && FieldOf(i).FieldType == rpm.FieldType && FieldOf(i) != rpm)
            .Select(FieldOf).First();
        // Sign of thrust: a float that the collective is multiplied by before writing the force vector.
        var sign = One(code.Where((i, n) => i.OpCode == OpCodes.Ldfld && FieldOf(i).FieldType.FullName == "System.Single" &&
            n + 2 < code.Count && code[n + 1].OpCode == OpCodes.Ldc_R4 && (float)code[n + 1].Operand == 0f && code[n + 2].OpCode == OpCodes.Ble_Un_S)
            .Select(FieldOf).Distinct(), "thrust sign field");
        // Pitch/yaw/roll demand: the field declared next to the PIDs with the same vector type.
        var outputs = One(controller.Fields.Where(f => f.FieldType == rpm.FieldType && Tok(f) == Tok(pids) + 1), "PID output vector");
        if (Tok(forces) != Tok(outputs) + 1 || Tok(rpm) != Tok(outputs) + 2)
            throw new InvalidOperationException("Output, force and RPM vectors are no longer declared together.");
        var storage = One(rpm.FieldType.Resolve().Fields.Where(f => !f.IsStatic && f.FieldType.FullName == "System.Single[]"), "vector storage");

        // Thrust limit: V_4 = propulsion().MaxThrust(maxRpm, inflow V_3). Liftoff's propeller model, used to
        // read the throttle back out of its collective and to keep its airspeed losses under a custom thrust curve.
        int limit = code.IndexOf(One(code.Where(i => i.OpCode == OpCodes.Stloc_S && ((VariableDefinition)i.Operand).Index == 4), "thrust limit store"));
        if (!(code[limit - 1].OpCode == OpCodes.Callvirt && code[limit - 2].OpCode == OpCodes.Ldloc_3 &&
              code[limit - 3].OpCode == OpCodes.Ldfld && FieldOf(code[limit - 3]) == maxRpm && code[limit - 5].OpCode == OpCodes.Callvirt &&
              code[limit - 6].OpCode == OpCodes.Ldfld && code[limit - 7].OpCode == OpCodes.Ldarg_0))
            throw new InvalidOperationException("Thrust limit is no longer computed from the propulsion model.");
        var propulsionOwner = FieldOf(code[limit - 6]);
        var propulsionGetter = ((MethodReference)code[limit - 5].Operand).Resolve();
        var maxThrust = ((MethodReference)code[limit - 1].Operand).Resolve();
        if (maxThrust.ReturnType.FullName != "System.Single" || maxThrust.Parameters.Count != 2 || maxThrust.Parameters.Any(p => p.ParameterType.FullName != "System.Single") || propulsionGetter.Parameters.Count != 0)
            throw new InvalidOperationException("Propulsion thrust method changed shape.");

        // ZetaFlight's PID: implements the dictionary's interface with a (axis, gains, integral limit) constructor.
        var pid = One(types.Where(t => t.Interfaces.Any(i => i.InterfaceType.Resolve() == pidInterface) &&
            t.Methods.Any(m => m.IsConstructor && m.Parameters.Select(p => p.ParameterType.FullName)
                .SequenceEqual(new[] { "TaitBryanAngle", "PIDGains", "System.Single" }))), "PID implementation");
        var ctor = One(pid.Methods.Where(m => m.IsConstructor && m.Parameters.Count == 3), "PID constructor");
        var calculate = One(pid.Methods.Where(m => !m.IsConstructor && m.ReturnType.FullName == "System.Single" &&
            m.Parameters.Count == 2 && m.Parameters.All(p => p.ParameterType.FullName == "System.Single")), "PID calculate");
        var gains = One(pid.Fields.Where(f => f.FieldType.FullName == "PIDGains"), "PID gains field");
        var calc = calculate.Body.Instructions;
        var integral = One(calc.Where((i, n) => i.OpCode == OpCodes.Stfld && n > 0 && Calls(calc[n - 1], "Clamp")).Select(FieldOf), "PID integral");

        // The plugin injects its mixer before the first powered motor: `this.props[0]` after a branch target.
        int boundary = first;
        while (boundary > 0 && !(code[boundary].OpCode == OpCodes.Ldarg_0 && code[boundary + 1].OpCode == OpCodes.Ldfld &&
            FieldOf(code[boundary + 1]).FieldType.FullName == "System.Collections.Generic.List`1<Propeller>")) boundary--;
        if (boundary == 0 || code[boundary + 2].OpCode != OpCodes.Ldc_I4_0)
            throw new InvalidOperationException("Cannot locate the powered motor boundary.");

        string Hex(IMetadataTokenProvider p) => "0x" + Tok(p).ToString("x8");
        File.WriteAllText(output, $$"""
            // Generated by tools/Inspector locate for {{label}}. Do not edit by hand:
            // rerun the locator against the new game and review its checks when Liftoff updates.
            namespace GoProGeometry
            {
                internal static class GameTokens
                {
                    internal const string Game = "{{label}}";
                    internal const string Module = "{{assembly.MainModule.Mvid}}";
                    internal const int Controller = {{Hex(controller)}}, ControlStep = {{Hex(step)}};
                    internal const int Settings = {{Hex(settings)}}, Pids = {{Hex(pids)}}, Outputs = {{Hex(outputs)}}, Forces = {{Hex(forces)}}, Rpm = {{Hex(rpm)}};
                    internal const int Sign = {{Hex(sign)}}, MaxRpm = {{Hex(maxRpm)}}, Storage = {{Hex(storage)}};
                    internal const int PidGains = {{Hex(gains)}}, PidIntegral = {{Hex(integral)}}, PidConstructor = {{Hex(ctor)}}, PidCalculate = {{Hex(calculate)}};
                    internal const int PropulsionOwner = {{Hex(propulsionOwner)}}, PropulsionGetter = {{Hex(propulsionGetter)}}, MaxThrust = {{Hex(maxThrust)}};
                }
            }

            """);
        Console.WriteLine($"Wrote {output} for module {assembly.MainModule.Mvid}.");
    }
}
