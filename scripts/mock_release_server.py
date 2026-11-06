#!/usr/bin/env python3
"""Mock of GitHub's release API for testing rfly-update offline.

Usage: mock_release_server.py DIR PORT
  DIR/release.json   the release (GitHub's JSON shape; asset URLs may be bare file names)
  DIR/files/NAME     the asset files
Serves  /releases/latest  (asset URLs rewritten to this server) and  /files/NAME.
"""
import http.server, json, os, sys

root, port = sys.argv[1], int(sys.argv[2])


class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, code, body, ctype="application/octet-stream"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.rstrip("/") == "/releases/latest":
            try:
                rel = json.load(open(os.path.join(root, "release.json")))
            except FileNotFoundError:
                return self.send(404, b'{"message":"Not Found"}', "application/json")
            for a in rel.get("assets", []):
                a["browser_download_url"] = f"http://127.0.0.1:{port}/files/{a['name']}"
            return self.send(200, json.dumps(rel).encode(), "application/json")
        if self.path.startswith("/files/"):
            p = os.path.join(root, "files", os.path.basename(self.path))
            if os.path.isfile(p):
                return self.send(200, open(p, "rb").read())
        self.send(404, b"not found")


http.server.ThreadingHTTPServer(("127.0.0.1", port), H).serve_forever()
