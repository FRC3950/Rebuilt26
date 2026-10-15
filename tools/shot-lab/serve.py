#!/usr/bin/env python3
"""Serve the bundled offline app on loopback only; no Node runtime is needed."""
import argparse
import errno
import functools
import http.server
import pathlib
import urllib.request
import webbrowser

parser = argparse.ArgumentParser(description="Chuck Norris Shot Lab")
parser.add_argument("--port", type=int, default=5174)
parser.add_argument("--no-browser", action="store_true")
args = parser.parse_args()
directory = pathlib.Path(__file__).resolve().parent / "dist"
if not (directory / "index.html").is_file():
    raise SystemExit("Build the app first with npm run build.")
handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(directory))
url = f"http://127.0.0.1:{args.port}/"
try:
    server = http.server.ThreadingHTTPServer(("127.0.0.1", args.port), handler)
except OSError as error:
    if error.errno != errno.EADDRINUSE:
        raise
    try:
        with urllib.request.urlopen(url, timeout=2) as response:
            existing = response.read(20000)
        if b"Chuck Norris" not in existing or not (b"Shot Lab" in existing or b"Simple Shot Tuning" in existing):
            raise ValueError("another app is using this port")
    except Exception:
        raise SystemExit(f"Port {args.port} is in use. Run python3 serve.py --port 5175.")
    print(f"Shot Lab is already running: {url}")
    if not args.no_browser:
        webbrowser.open(url + "map.html")
    raise SystemExit(0)
with server:
    url = f"http://127.0.0.1:{server.server_port}/"
    print(f"Chuck Norris Shot Lab: {url}\nOffline workspace. Press Ctrl+C to stop.", flush=True)
    if not args.no_browser:
        webbrowser.open(url + "map.html")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
