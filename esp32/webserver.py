"""A small asyncio web server for driving a BrachioGraph from a browser.

The browser does the heavy lifting (decoding JPG/PNG/SVG and turning them into lines with a
JavaScript port of linedraw); the ESP32 stores the resulting ``.lines`` files on flash and plots
them. Plotting runs in a separate thread so the web interface stays responsive (and can stop it).

API (all responses are JSON unless noted):

    GET    /, /linedraw.js         the web interface (files in www/, a .gz version is preferred)
    GET    /api/status             plotter state and progress
    GET    /api/files              stored drawings
    GET    /api/file?name=x        download a drawing (text)
    POST   /api/upload?name=x      body = .lines text, streamed to flash
    POST   /api/delete?name=x
    POST   /api/plot?name=x[&wait=&resolution=&angular_step=]
    POST   /api/stop
    POST   /api/park | /api/box | /api/test_pattern | /api/quiet
    POST   /api/pen?state=up|down
    POST   /api/xy?x=..&y=..
"""

import asyncio
import gc
import json
import os
import sys

try:
    import _thread
except ImportError:
    _thread = None

from plotter import PlotStopped

WWW = "www"
DRAWINGS = "drawings"
CHUNK = 1024
MIME = {
    "html": "text/html; charset=utf-8",
    "js": "application/javascript",
    "css": "text/css",
    "svg": "image/svg+xml",
    "ico": "image/x-icon",
    "png": "image/png",
}


def _exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def _unquote(s):
    s = s.replace("+", " ")
    if "%" not in s:
        return s
    out = bytearray()
    parts = s.split("%")
    out.extend(parts[0].encode())
    for part in parts[1:]:
        try:
            out.append(int(part[:2], 16))
            out.extend(part[2:].encode())
        except ValueError:
            out.extend(("%" + part).encode())
    return out.decode()


def _parse_query(q):
    params = {}
    for pair in q.split("&"):
        if pair:
            k, _, v = pair.partition("=")
            params[_unquote(k)] = _unquote(v)
    return params


def _safe_name(name):
    """Keep drawing names to a single safe path component."""
    name = (name or "").strip().replace(" ", "-")
    keep = ""
    for ch in name:
        if ch.isalpha() or ch.isdigit() or ch in "-_.":
            keep += ch
    keep = keep.lstrip(".")[:48]
    if not keep:
        return None
    if not (keep.endswith(".lines") or keep.endswith(".json")):
        keep += ".lines"
    return keep


def _float(params, key):
    v = params.get(key)
    if v in (None, ""):
        return None
    return float(v)


class PlotterServer:
    def __init__(self, bg, root=""):
        self.bg = bg
        self.root = root
        self.www = root + WWW
        self.drawings = root + DRAWINGS
        self.job = None  # description of the running job
        self.last_error = None
        self.last_job = None
        if not _exists(self.drawings):
            os.mkdir(self.drawings)

    # ----------------- running jobs in a thread -----------------

    def start_job(self, label, fn, *args):
        if self.bg.busy:
            return False
        self.bg.busy = True
        self.bg._stop = False
        self.job = label
        self.last_error = None
        self.bg.progress_done = self.bg.progress_total = 0
        if _thread:
            if sys.platform == "esp32":
                _thread.stack_size(16 * 1024)
            _thread.start_new_thread(self._run, (label, fn, args))
        else:
            self._run(label, fn, args)
        return True

    def _run(self, label, fn, args):
        try:
            fn(*args)
            self.last_job = label + ": done"
        except PlotStopped:
            self.last_job = label + ": stopped"
            try:
                self.bg.park()
            except Exception as e:
                self.last_error = repr(e)
        except Exception as e:
            self.last_error = "{}: {}".format(type(e).__name__, e)
            self.last_job = label + ": failed"
            sys.print_exception(e) if hasattr(sys, "print_exception") else print(repr(e))
            try:
                self.bg.pen.up()
            except Exception:
                pass
        finally:
            self.job = None
            self.bg.busy = False
            gc.collect()

    # ----------------- HTTP plumbing -----------------

    async def handle(self, reader, writer):
        try:
            request = await reader.readline()
            if not request:
                return
            method, target, _ = request.decode().split(" ", 2)
            headers = {}
            while True:
                h = await reader.readline()
                if not h or h in (b"\r\n", b"\n"):
                    break
                k, _, v = h.decode().partition(":")
                headers[k.strip().lower()] = v.strip()

            path, _, query = target.partition("?")
            params = _parse_query(query)
            await self.route(method, path, params, headers, reader, writer)
        except Exception as e:
            try:
                await self.send_json(writer, {"error": repr(e)}, "500 Internal Server Error")
            except Exception:
                pass
        finally:
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass
            gc.collect()

    async def send(self, writer, body, status="200 OK", ctype="application/json", extra=""):
        if isinstance(body, str):
            body = body.encode()
        writer.write(
            "HTTP/1.0 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\n"
            "Cache-Control: no-store\r\nConnection: close\r\n{}\r\n".format(
                status, ctype, len(body), extra
            ).encode()
        )
        writer.write(body)
        await writer.drain()

    async def send_json(self, writer, obj, status="200 OK"):
        await self.send(writer, json.dumps(obj), status)

    async def send_file(self, writer, path, ctype, extra=""):
        size = os.stat(path)[6]
        writer.write(
            "HTTP/1.0 200 OK\r\nContent-Type: {}\r\nContent-Length: {}\r\n"
            "Connection: close\r\n{}\r\n".format(ctype, size, extra).encode()
        )
        buf = bytearray(CHUNK)
        mv = memoryview(buf)
        with open(path, "rb") as f:
            while True:
                n = f.readinto(buf)
                if not n:
                    break
                writer.write(bytes(mv[:n]))
                await writer.drain()

    # ----------------- routes -----------------

    async def route(self, method, path, params, headers, reader, writer):
        bg = self.bg

        if method == "GET" and not path.startswith("/api/"):
            return await self.static(writer, path)

        if path == "/api/status":
            return await self.send_json(writer, self.status())

        if path == "/api/files":
            return await self.send_json(writer, {"files": self.list_files(), "fs": self.fs_info()})

        if path == "/api/file":
            name = _safe_name(params.get("name"))
            p = self.drawings + "/" + (name or "")
            if not name or not _exists(p):
                return await self.send_json(writer, {"error": "no such file"}, "404 Not Found")
            return await self.send_file(writer, p, "text/plain; charset=utf-8")

        if method != "POST":
            return await self.send_json(writer, {"error": "not found"}, "404 Not Found")

        if path == "/api/upload":
            return await self.upload(params, headers, reader, writer)

        if path == "/api/stop":
            bg.request_stop()
            return await self.send_json(writer, {"ok": True})

        if path == "/api/delete":
            name = _safe_name(params.get("name"))
            try:
                os.remove(self.drawings + "/" + name)
            except (OSError, TypeError):
                return await self.send_json(writer, {"error": "no such file"}, "404 Not Found")
            return await self.send_json(writer, {"ok": True})

        # everything below moves the machine, one job at a time
        if bg.busy:
            return await self.send_json(writer, {"error": "busy: " + str(self.job)}, "409 Conflict")

        if path == "/api/plot":
            name = _safe_name(params.get("name"))
            p = self.drawings + "/" + (name or "")
            if not name or not _exists(p):
                return await self.send_json(writer, {"error": "no such file"}, "404 Not Found")
            args = (p, None, _float(params, "angular_step"), _float(params, "wait"), _float(params, "resolution"))
            ok = self.start_job("plot " + name, bg.plot_file, *args)
        elif path == "/api/park":
            ok = self.start_job("park", bg.park)
        elif path == "/api/box":
            ok = self.start_job("box", bg.box)
        elif path == "/api/test_pattern":
            ok = self.start_job("test pattern", bg.test_pattern)
        elif path == "/api/quiet":
            ok = self.start_job("quiet", bg.quiet)
        elif path == "/api/pen":
            ok = self.start_job("pen", bg.pen.down if params.get("state") == "down" else bg.pen.up)
        elif path == "/api/xy":
            ok = self.start_job("move", bg.xy, _float(params, "x"), _float(params, "y"))
        else:
            return await self.send_json(writer, {"error": "not found"}, "404 Not Found")

        await self.send_json(writer, {"ok": ok})

    async def static(self, writer, path):
        name = path.lstrip("/") or "index.html"
        ext = name.rpartition(".")[2]
        if "/" in name or ".." in name or ext not in MIME:
            return await self.send_json(writer, {"error": "not found"}, "404 Not Found")
        p = self.www + "/" + name
        if _exists(p + ".gz"):
            return await self.send_file(writer, p + ".gz", MIME[ext], "Content-Encoding: gzip\r\n")
        if _exists(p):
            return await self.send_file(writer, p, MIME[ext])
        await self.send_json(writer, {"error": "not found"}, "404 Not Found")

    async def upload(self, params, headers, reader, writer):
        name = _safe_name(params.get("name"))
        if not name:
            return await self.send_json(writer, {"error": "bad name"}, "400 Bad Request")
        length = int(headers.get("content-length", 0))
        if length <= 0:
            return await self.send_json(writer, {"error": "empty upload"}, "400 Bad Request")
        fs = self.fs_info()
        if length > fs["free"] - 8192:
            return await self.send_json(
                writer, {"error": "not enough space on flash ({} bytes free)".format(fs["free"])},
                "507 Insufficient Storage",
            )

        final = self.drawings + "/" + name
        tmp = final + ".part"
        remaining = length
        with open(tmp, "wb") as f:
            while remaining > 0:
                chunk = await reader.read(min(CHUNK, remaining))
                if not chunk:
                    break
                f.write(chunk)
                remaining -= len(chunk)
        if remaining:
            os.remove(tmp)
            return await self.send_json(writer, {"error": "upload incomplete"}, "400 Bad Request")
        if _exists(final):
            os.remove(final)
        os.rename(tmp, final)
        await self.send_json(writer, {"ok": True, "name": name, "size": length})

    # ----------------- information -----------------

    def list_files(self):
        files = []
        for name in sorted(os.listdir(self.drawings)):
            if name.endswith(".part"):
                continue
            files.append({"name": name, "size": os.stat(self.drawings + "/" + name)[6]})
        return files

    def fs_info(self):
        try:
            s = os.statvfs(self.drawings)
            return {"free": s[0] * s[3], "total": s[0] * s[2]}
        except (AttributeError, OSError):
            return {"free": 1 << 30, "total": 1 << 30}

    def status(self):
        bg = self.bg
        gc.collect()
        pw_1, pw_2 = bg.get_pulse_widths()
        return {
            "busy": bg.busy,
            "job": self.job,
            "last_job": self.last_job,
            "error": self.last_error,
            "done": bg.progress_done,
            "total": bg.progress_total,
            "pen": bg.pen.position,
            "x": round(bg.x, 2),
            "y": round(bg.y, 2),
            "angles": [round(bg.angle_1 or 0, 1), round(bg.angle_2 or 0, 1)],
            "pws": [int(pw_1 or 0), int(pw_2 or 0)],
            "bounds": list(bg.bounds),
            "virtual": bg.virtual,
            "wait": bg.wait,
            "resolution": bg.resolution,
            "angular_step": bg.angular_step,
            "mem_free": gc.mem_free() if hasattr(gc, "mem_free") else None,
        }


async def serve(bg, port=80, host="0.0.0.0", root=""):
    server = PlotterServer(bg, root)
    await asyncio.start_server(server.handle, host, port)
    print("BrachioGraph web interface listening on port", port)
    while True:
        await asyncio.sleep(3600)


def run(bg, port=80, root=""):
    try:
        asyncio.run(serve(bg, port, root=root))
    finally:
        asyncio.new_event_loop()
