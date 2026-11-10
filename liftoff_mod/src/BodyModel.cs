using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering;

namespace GoProGeometry
{
    // The CAD body the editor exports: little-endian "GLMB", version 1, part count, then per part
    // RGB colour (3 float), vertex count, index count, positions and normals (float xyz, profile axes,
    // meters) and triangle indices (uint32). The editor has already applied units and axes.
    internal static class BodyModel
    {
        const uint Magic = 0x424d4c47; // "GLMB"
        const int MaxVertices = 3_000_000;

        internal sealed class Part { public Color color; public Vector3[] positions, normals; public int[] indices; }

        internal static List<Part> Read(string path)
        {
            using (var r = new BinaryReader(File.OpenRead(path)))
            {
                if (r.ReadUInt32() != Magic || r.ReadUInt32() != 1) throw new InvalidDataException("Body model is not a Geometry Lab mesh.");
                int count = r.ReadInt32(), total = 0;
                if (count < 1 || count > 4096) throw new InvalidDataException("Body model has an invalid part count.");
                var parts = new List<Part>();
                for (int p = 0; p < count; p++)
                {
                    var part = new Part { color = new Color(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()) };
                    int vertices = r.ReadInt32(), indices = r.ReadInt32();
                    total += vertices;
                    if (vertices < 3 || total > MaxVertices || indices < 3 || indices % 3 != 0 || indices > 3 * MaxVertices)
                        throw new InvalidDataException("Body model is empty or too large.");
                    // Profile axes (left, back, up; right-handed) → Unity (right, up, forward; left-handed).
                    Vector3 Next() { float x = r.ReadSingle(), y = r.ReadSingle(), z = r.ReadSingle(); return new Vector3(-x, z, -y); }
                    part.positions = new Vector3[vertices]; part.normals = new Vector3[vertices];
                    for (int i = 0; i < vertices; i++) part.positions[i] = Next();
                    for (int i = 0; i < vertices; i++) part.normals[i] = Next();
                    part.indices = new int[indices];
                    for (int i = 0; i < indices; i += 3)
                    {
                        // The axis change mirrors, so swap winding to keep faces pointing outward.
                        part.indices[i] = r.ReadInt32(); part.indices[i + 2] = r.ReadInt32(); part.indices[i + 1] = r.ReadInt32();
                    }
                    if (part.indices.Any(i => i < 0 || i >= vertices) || part.positions.Any(v => float.IsNaN(v.x + v.y + v.z) || v.magnitude > 2f))
                        throw new InvalidDataException("Body model has invalid vertices (positions must be within 2 m).");
                    parts.Add(part);
                }
                return parts;
            }
        }

        // Builds the body under the drone. Material: Unity's Standard shader if the game ships it,
        // otherwise a copy of the stock frame's own material.
        internal static GameObject Attach(Transform parent, Vector3 offset, List<Part> parts, Material fallback)
        {
            var root = new GameObject("GeometryLabBody");
            root.transform.SetParent(parent, false);
            root.transform.localPosition = offset;
            var shader = Shader.Find("Standard");
            foreach (var part in parts)
            {
                var mesh = new Mesh { indexFormat = part.positions.Length > 65000 ? IndexFormat.UInt32 : IndexFormat.UInt16 };
                mesh.vertices = part.positions; mesh.normals = part.normals; mesh.triangles = part.indices;
                mesh.RecalculateBounds();
                var go = new GameObject("Part");
                go.transform.SetParent(root.transform, false);
                go.AddComponent<MeshFilter>().sharedMesh = mesh;
                var renderer = go.AddComponent<MeshRenderer>();
                var material = shader != null ? new Material(shader) : fallback != null ? new Material(fallback) : null;
                if (material == null) throw new InvalidOperationException("No shader available to draw the CAD body.");
                material.color = part.color;
                if (material.HasProperty("_Glossiness")) material.SetFloat("_Glossiness", 0.35f);
                renderer.sharedMaterial = material;
                renderer.shadowCastingMode = ShadowCastingMode.On;
            }
            return root;
        }

        internal static void Destroy(GameObject root)
        {
            if (!root) return;
            foreach (var f in root.GetComponentsInChildren<MeshFilter>()) UnityEngine.Object.Destroy(f.sharedMesh);
            foreach (var r in root.GetComponentsInChildren<MeshRenderer>()) UnityEngine.Object.Destroy(r.sharedMaterial);
            UnityEngine.Object.Destroy(root);
        }
    }
}
