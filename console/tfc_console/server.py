# SPDX-License-Identifier: MIT
"""The web server: static files for the page, a JSON API, and one Server-Sent-Events stream (`/api/stream`) that carries the state ten times a second.

Standard library only (`http.server`), like the rest of the tools that talk to the bus: nothing to install, nothing fetched from the network, so it runs on the bench with no internet and what it runs is
what is in the repository. The server listens on 127.0.0.1 unless told otherwise, and **every API request needs the session token** the console prints at start (a command is a command, even on a
desk rig): the token goes in the header `X-TFC-Token` (the page keeps it) or, for the event stream, which a browser cannot give a header, in `?token=`. A request whose `Host` is not the address the server
was started on is refused (DNS rebinding), and so is a POST whose `Origin` is another site.
"""
from __future__ import annotations

import json
import mimetypes
import queue
import secrets
import threading
import time
import traceback
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Callable
from urllib.parse import parse_qs, unquote, urlparse

from .hub import Hub, dumps

Route = Callable[["Request"], "tuple[int, object]"]


class Request:
    def __init__(self, method: str, path: str, query: dict[str, list[str]], body: dict | None, headers) -> None:
        self.method, self.path, self.query, self.body, self.headers = method, path, query, body or {}, headers

    def arg(self, name: str, default: str | None = None) -> str | None:
        v = self.query.get(name)
        return v[0] if v else default

    def num(self, name: str, default: float) -> float:
        try:
            return float(self.arg(name, str(default)))
        except (TypeError, ValueError):
            return default


class ApiError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status, self.message = status, message


class App:
    def __init__(self, hub: Hub, web_dir: Path, host: str = "127.0.0.1", port: int = 8765, token: str | None = None) -> None:
        self.hub, self.web_dir = hub, Path(web_dir)
        self.token = token or secrets.token_urlsafe(18)
        self.routes: dict[tuple[str, str], Route] = {}
        self.host = host
        self.httpd = ThreadingHTTPServer((host, port), self._handler())
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]
        self.hello_extra: Callable[[], dict] = lambda: {}
        self.on_close: list[Callable[[], None]] = []

    def route(self, method: str, path: str) -> Callable[[Route], Route]:
        def deco(fn: Route) -> Route:
            self.routes[(method, path)] = fn
            return fn
        return deco

    def url(self) -> str:
        shown = "127.0.0.1" if self.host in ("0.0.0.0", "") else self.host
        return f"http://{shown}:{self.port}/?token={self.token}"

    def serve_forever(self) -> None:
        self.httpd.serve_forever(poll_interval=0.2)

    def shutdown(self) -> None:
        for fn in self.on_close:
            try:
                fn()
            except Exception:  # noqa: BLE001 - shutting down: every cleanup gets its turn
                traceback.print_exc()
        self.httpd.shutdown()
        self.httpd.server_close()

    # ------------------------------------------------------------------ the handler
    def _allowed_hosts(self) -> set[str]:
        names = {"127.0.0.1", "localhost", "[::1]"}
        if self.host not in ("0.0.0.0", "", "::"):
            names.add(self.host)
            return {f"{n}:{self.port}" for n in names}
        return set()                                    # listening on every interface: any Host is accepted, and the token is the protection

    def _handler(self):
        app = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"
            server_version = "tfc-console"

            def log_message(self, fmt, *args):       # the console prints its own lines; a line per request would bury them
                pass

            # ---- plumbing
            def _send(self, status: int, body: bytes, ctype: str, extra: dict[str, str] | None = None) -> None:
                self.send_response(status)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Referrer-Policy", "no-referrer")
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(body)

            def _json(self, status: int, obj: object) -> None:
                self._send(status, dumps(obj).encode(), "application/json; charset=utf-8")

            def _host_ok(self) -> bool:
                allowed = app._allowed_hosts()
                host = self.headers.get("Host", "")
                return not allowed or host in allowed

            def _token_ok(self, query: dict[str, list[str]]) -> bool:
                given = self.headers.get("X-TFC-Token") or (query.get("token") or [""])[0]
                return secrets.compare_digest(given.encode(), app.token.encode())

            def _dispatch(self, method: str) -> None:
                url = urlparse(self.path)
                query = parse_qs(url.query)
                if not self._host_ok():
                    return self._json(HTTPStatus.FORBIDDEN, {"error": "bad Host header"})
                if not url.path.startswith("/api/"):
                    return self._static(url.path) if method == "GET" else self._json(HTTPStatus.METHOD_NOT_ALLOWED, {"error": "GET only"})
                if not self._token_ok(query):
                    return self._json(HTTPStatus.UNAUTHORIZED, {"error": "missing or wrong session token (open the URL the console printed)"})
                body: dict | None = None
                if method == "POST":
                    origin = self.headers.get("Origin")
                    if origin and urlparse(origin).netloc != self.headers.get("Host"):
                        return self._json(HTTPStatus.FORBIDDEN, {"error": "cross-origin POST refused"})
                    length = int(self.headers.get("Content-Length") or 0)
                    if length > 4 << 20:
                        return self._json(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, {"error": "body too large"})
                    raw = self.rfile.read(length) if length else b""
                    try:
                        body = json.loads(raw) if raw else {}
                    except ValueError:
                        return self._json(HTTPStatus.BAD_REQUEST, {"error": "the body is not JSON"})
                    if not isinstance(body, dict):
                        return self._json(HTTPStatus.BAD_REQUEST, {"error": "the body must be a JSON object"})
                if url.path == "/api/stream" and method == "GET":
                    return self._stream()
                fn = app.routes.get((method, url.path))
                if fn is None:
                    return self._json(HTTPStatus.NOT_FOUND, {"error": f"no such endpoint: {method} {url.path}"})
                try:
                    status, payload = fn(Request(method, url.path, query, body, self.headers))
                except ApiError as e:
                    return self._json(e.status, {"error": e.message})
                except Exception as e:  # noqa: BLE001 - a bug in one endpoint answers 500 with its message; it must not take the server with it
                    traceback.print_exc()
                    return self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": f"{type(e).__name__}: {e}"})
                self._json(status, payload)

            def do_GET(self) -> None:
                self._dispatch("GET")

            def do_POST(self) -> None:
                self._dispatch("POST")

            # ---- static files
            def _static(self, path: str) -> None:
                path = unquote(path)                    # %2e%2e is `..` too: the containment check below is on the decoded path
                rel = "index.html" if path in ("/", "") else path.lstrip("/")
                target = (app.web_dir / rel).resolve()
                if app.web_dir.resolve() not in target.parents and target != app.web_dir.resolve():
                    return self._json(HTTPStatus.FORBIDDEN, {"error": "outside the web directory"})
                if not target.is_file():
                    return self._json(HTTPStatus.NOT_FOUND, {"error": f"{rel} not found"})
                ctype = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
                if ctype.startswith("text/") or ctype in ("application/javascript", "application/json"):
                    ctype += "; charset=utf-8"
                self._send(HTTPStatus.OK, target.read_bytes(), ctype, {"Content-Security-Policy": "default-src 'self'; img-src 'self' data:; style-src 'self' 'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'"})

            # ---- the event stream
            def _stream(self) -> None:
                hub = app.hub
                q = hub.subscribe()
                try:
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/event-stream; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Connection", "keep-alive")
                    self.send_header("X-Accel-Buffering", "no")
                    self.end_headers()
                    hello = {"history": hub.history_columns(180.0), "events": hub.recent_events(400), "lines": hub.recent_lines(300), "snapshot": hub.snapshot(), **app.hello_extra()}
                    self.wfile.write(f"event: hello\ndata: {dumps(hello)}\n\n".encode())
                    self.wfile.flush()
                    while True:
                        try:
                            msg = q.get(timeout=10.0)
                        except queue.Empty:
                            msg = ": keepalive\n\n"
                        self.wfile.write(msg.encode())
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    hub.unsubscribe(q)

        return Handler


def start_in_thread(app: App) -> threading.Thread:
    t = threading.Thread(target=app.serve_forever, name="http", daemon=True)
    t.start()
    return t


def wait_for(cond: Callable[[], bool], timeout: float, step: float = 0.02) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(step)
    return cond()
