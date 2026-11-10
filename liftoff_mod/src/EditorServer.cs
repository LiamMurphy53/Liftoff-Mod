using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Text;
using System.Threading;

namespace GoProGeometry
{
    internal sealed class Reply
    {
        internal int Status = 200;
        internal string Type = "application/json; charset=utf-8";
        internal byte[] Body = new byte[0];
        internal static Reply Json(string json, int status = 200) => new Reply { Status = status, Body = Encoding.UTF8.GetBytes(json) };
        internal static Reply Bytes(byte[] data, string type) => new Reply { Body = data, Type = type };
    }

    // Serves the drone editor and its API on 127.0.0.1 only. Requests must name this host (no DNS rebinding),
    // and anything that changes state needs the X-Geometry-Lab header, which other web pages cannot send
    // without a CORS preflight this server never approves.
    internal sealed class EditorServer : IDisposable
    {
        static readonly Dictionary<string, string> Types = new Dictionary<string, string>
        {
            {".html", "text/html; charset=utf-8"}, {".js", "text/javascript; charset=utf-8"}, {".css", "text/css; charset=utf-8"},
            {".svg", "image/svg+xml"}, {".png", "image/png"}, {".json", "application/json"}, {".woff2", "font/woff2"},
        };
        HttpListener listener;
        readonly string root;
        readonly Func<string, string, Dictionary<string, string>, byte[], Reply> api;
        readonly Thread thread;
        volatile bool running = true;
        internal readonly int Port;
        internal string Url => "http://127.0.0.1:" + Port + "/";

        internal EditorServer(string root, int port, Func<string, string, Dictionary<string, string>, byte[], Reply> api)
        {
            this.root = Path.GetFullPath(root); this.api = api;
            for (int p = port; ; p++)
            {
                try { listener = new HttpListener(); listener.Prefixes.Add("http://127.0.0.1:" + p + "/"); listener.Start(); Port = p; break; }
                catch (HttpListenerException) when (p < port + 9) { }
            }
            thread = new Thread(Loop) { IsBackground = true, Name = "GeometryLabEditor" };
            thread.Start();
        }

        void Loop()
        {
            while (running)
            {
                HttpListenerContext context;
                try { context = listener.GetContext(); } catch { if (!running) return; continue; }
                ThreadPool.QueueUserWorkItem(_ => Handle(context));
            }
        }

        void Handle(HttpListenerContext context)
        {
            var request = context.Request; var response = context.Response;
            Reply reply;
            try
            {
                string host = request.Headers["Host"] ?? "";
                if (host != "127.0.0.1:" + Port && host != "localhost:" + Port) reply = Reply.Json("{\"ok\":false,\"message\":\"Wrong host.\"}", 403);
                else if (request.Url.AbsolutePath.StartsWith("/api/"))
                {
                    if (request.HttpMethod != "GET" && request.Headers["X-Geometry-Lab"] != "1") reply = Reply.Json("{\"ok\":false,\"message\":\"Missing editor header.\"}", 403);
                    else
                    {
                        byte[] body;
                        using (var buffer = new MemoryStream()) { request.InputStream.CopyTo(buffer); body = buffer.ToArray(); }
                        var query = new Dictionary<string, string>();
                        foreach (var key in request.QueryString.AllKeys) if (key != null) query[key] = request.QueryString[key];
                        reply = api(request.HttpMethod, request.Url.AbsolutePath, query, body);
                    }
                }
                else reply = Static(request.Url.AbsolutePath);
            }
            catch (Exception e) { reply = Reply.Json("{\"ok\":false,\"message\":" + Quote(e.Message) + "}", 500); }
            try
            {
                response.StatusCode = reply.Status; response.ContentType = reply.Type;
                response.Headers["Cache-Control"] = "no-store"; response.Headers["X-Content-Type-Options"] = "nosniff";
                response.ContentLength64 = reply.Body.Length;
                response.OutputStream.Write(reply.Body, 0, reply.Body.Length);
            }
            catch { }
            finally { try { response.Close(); } catch { } }
        }

        Reply Static(string path)
        {
            if (path == "/") path = "/index.html";
            var full = Path.GetFullPath(Path.Combine(root, Uri.UnescapeDataString(path).TrimStart('/')));
            if (!full.StartsWith(root + Path.DirectorySeparatorChar) || !File.Exists(full)) return new Reply { Status = 404, Type = "text/plain", Body = Encoding.UTF8.GetBytes("Not found") };
            return Reply.Bytes(File.ReadAllBytes(full), Types.TryGetValue(Path.GetExtension(full), out var t) ? t : "application/octet-stream");
        }

        internal static string Quote(string s)
        {
            var b = new StringBuilder("\"");
            foreach (char c in s ?? "")
                switch (c)
                {
                    case '"': b.Append("\\\""); break;
                    case '\\': b.Append("\\\\"); break;
                    case '\n': b.Append("\\n"); break;
                    case '\r': break;
                    case '\t': b.Append("\\t"); break;
                    default: if (c < 32) b.Append("\\u").Append(((int)c).ToString("x4")); else b.Append(c); break;
                }
            return b.Append('"').ToString();
        }

        public void Dispose() { running = false; try { listener.Stop(); listener.Close(); } catch { } }
    }
}
