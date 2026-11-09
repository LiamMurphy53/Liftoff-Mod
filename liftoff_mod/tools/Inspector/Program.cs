using Mono.Cecil;
using System.Text.Json;

// Read-only inspection. Never loads or executes game code.
if (args.Length < 1) throw new ArgumentException("Pass the installed Assembly-CSharp.dll path, optionally a metadata token.");
var resolver = new DefaultAssemblyResolver();
resolver.AddSearchDirectory(Path.GetDirectoryName(Path.GetFullPath(args[0])));
using var assembly = AssemblyDefinition.ReadAssembly(args[0], new ReaderParameters { AssemblyResolver = resolver });
if (args.Length > 1 && args[1] == "locate")
{
    Locator.Run(assembly, args.Length > 2 ? args[2] : "GameTokens.cs", args.Length > 3 ? args[3] : "unlabelled Liftoff build");
    return;
}
if (args.Length > 1)
{
    var member = args[1].StartsWith("hash:") ? (IMetadataTokenProvider)assembly.MainModule.Types.Single(t => Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(System.Text.Encoding.UTF8.GetBytes(t.Name))).ToLowerInvariant().StartsWith(args[1].Substring(5))) : assembly.MainModule.LookupToken(Convert.ToInt32(args[1], 16));
    if (member is MethodDefinition method)
    {
        Console.WriteLine(method.FullName);
        foreach (var variable in method.Body.Variables) Console.WriteLine($"LOCAL {variable.Index}: {variable.VariableType}");
        foreach (var instruction in method.Body.Instructions) Console.WriteLine(instruction);
    }
    if (member is TypeDefinition type)
    {
        Console.WriteLine(type.FullName);
        foreach (var f in type.Fields) Console.WriteLine($"FIELD {f.MetadataToken.ToInt32():x8}: {f.FullName}");
        foreach (var m in type.Methods) Console.WriteLine($"METHOD {m.MetadataToken.ToInt32():x8}: {m.FullName}");
    }
    return;
}
var needles = new[] { "ApplyControllerForceAtPropeller", "AddForceAtPosition", "inertiaTensor", "centerOfMass", "get_CurrentDrone" };
IEnumerable<TypeDefinition> Walk(IEnumerable<TypeDefinition> types) {
    foreach (var t in types) { yield return t; foreach (var n in Walk(t.NestedTypes)) yield return n; }
}
var types = Walk(assembly.MainModule.Types).ToArray();
var matches = types.SelectMany(t => t.Methods.Where(m => m.HasBody).SelectMany(m =>
    m.Body.Instructions.Where(i => i.Operand is MemberReference r && needles.Any(n => r.Name.Contains(n)))
    .Select(i => new { type = t.FullName, token = t.MetadataToken.ToInt32(), method = m.Name, methodToken = m.MetadataToken.ToInt32(), instruction = i.ToString() }))).ToArray();
var selected = matches.Select(m => m.type).Concat(new[] { "Propeller", "FlightManager", "FramePart", "MotorPart" }).ToHashSet();
var details = types.Where(t => selected.Contains(t.FullName)).Select(t => new {
    name = t.FullName, token = t.MetadataToken.ToInt32(), parent = t.BaseType?.FullName,
    fields = t.Fields.Select(f => new { name=f.Name, type=f.FieldType.FullName, token=f.MetadataToken.ToInt32() }),
    properties = t.Properties.Select(p => new { name=p.Name, type=p.PropertyType.FullName }),
    methods = t.Methods.Select(m => new { name=m.Name, token=m.MetadataToken.ToInt32(), signature=m.FullName })
});
Console.WriteLine(JsonSerializer.Serialize(new { moduleId=assembly.MainModule.Mvid, matches, types=details }, new JsonSerializerOptions { WriteIndented=true }));
