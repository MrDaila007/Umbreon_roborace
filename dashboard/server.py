#!/usr/bin/env python3
"""
server.py — Umbreon Dashboard Web Server

Serves the web UI and bridges WebSocket ↔ TCP car connection.

    cd dashboard && python server.py
    # Open http://localhost:8080 in browser

    python server.py 9090          # custom port
"""

import asyncio
import json
import sys
import os
from pathlib import Path

from aiohttp import web

# Add dashboard dir to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from protocol import parse_telemetry, parse_response
from car_config import DEFAULTS, KEY_NAMES, PARAM_GROUPS, FLOAT_KEYS, READONLY_KEYS


class CarBridge:
    """Manages TCP connection to car and bridges to WebSocket clients."""

    def __init__(self):
        self.reader = None
        self.writer = None
        self.connected = False
        self.ws_clients: set[web.WebSocketResponse] = set()
        self._read_task = None
        self.frame_count = 0

    async def connect(self, host: str, port: int):
        try:
            self.reader, self.writer = await asyncio.wait_for(
                asyncio.open_connection(host, int(port)),
                timeout=3.0,
            )
            self.connected = True
            self.frame_count = 0
            self._read_task = asyncio.create_task(self._read_loop())
            await self._broadcast_status()
            return True
        except Exception as e:
            self.connected = False
            return str(e)

    async def disconnect(self):
        self.connected = False
        if self._read_task and not self._read_task.done():
            self._read_task.cancel()
        if self.writer:
            self.writer.close()
            try:
                await self.writer.wait_closed()
            except Exception:
                pass
        self.reader = None
        self.writer = None
        await self._broadcast_status()

    async def send_to_car(self, text: str):
        if self.connected and self.writer:
            try:
                self.writer.write((text.strip() + "\n").encode("ascii"))
                await self.writer.drain()
            except Exception:
                await self.disconnect()

    async def _read_loop(self):
        buf = b""
        try:
            while self.connected:
                data = await self.reader.read(4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line_bytes, buf = buf.split(b"\n", 1)
                    line = line_bytes.decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    await self._process_line(line)
        except asyncio.CancelledError:
            return
        except Exception:
            pass
        finally:
            self.connected = False
            await self._broadcast_status()

    async def _process_line(self, line: str):
        if line[0].isdigit():
            frame = parse_telemetry(line)
            if frame:
                self.frame_count += 1
                msg = {
                    "type": "telemetry",
                    "data": {
                        "ms": frame.ms,
                        "s0": frame.s0,
                        "s1": frame.s1,
                        "s2": frame.s2,
                        "s3": frame.s3,
                        "steer": frame.steer,
                        "speed": round(frame.speed, 3),
                        "target": round(frame.target, 2),
                        "yaw": round(frame.yaw, 2) if frame.yaw is not None else 0,
                        "heading": round(frame.heading, 2) if frame.heading is not None else 0,
                        "has_imu": frame.has_imu,
                    },
                    "n": self.frame_count,
                }
                await self._broadcast(msg)
        elif line[0] == "$":
            resp = parse_response(line)
            await self._broadcast({"type": "response", "data": resp})
        # Header lines (#) — ignore

    async def _broadcast(self, msg: dict):
        text = json.dumps(msg)
        for ws in list(self.ws_clients):
            try:
                await ws.send_str(text)
            except Exception:
                self.ws_clients.discard(ws)

    async def _broadcast_status(self):
        await self._broadcast({
            "type": "status",
            "connected": self.connected,
            "frames": self.frame_count,
        })


bridge = CarBridge()


async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    bridge.ws_clients.add(ws)

    # Send current state to new client
    await ws.send_str(json.dumps({
        "type": "status",
        "connected": bridge.connected,
        "frames": bridge.frame_count,
    }))
    await ws.send_str(json.dumps({
        "type": "config",
        "defaults": {k: (float(v) if isinstance(v, float) else v)
                     for k, v in DEFAULTS.items()},
        "key_names": KEY_NAMES,
        "param_groups": PARAM_GROUPS,
        "float_keys": list(FLOAT_KEYS),
        "readonly_keys": list(READONLY_KEYS),
    }))

    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                data = json.loads(msg.data)
                action = data.get("action")

                if action == "connect":
                    host = data.get("host", "192.168.4.1")
                    port = data.get("port", 23)
                    result = await bridge.connect(host, port)
                    if result is not True:
                        await ws.send_str(json.dumps({
                            "type": "error",
                            "message": str(result),
                        }))

                elif action == "disconnect":
                    await bridge.disconnect()

                elif action == "command":
                    cmd = data.get("cmd", "")
                    await bridge.send_to_car(cmd)

            elif msg.type in (web.WSMsgType.ERROR, web.WSMsgType.CLOSE):
                break
    except Exception:
        pass
    finally:
        bridge.ws_clients.discard(ws)

    return ws


async def index_handler(request):
    return web.FileResponse(
        Path(__file__).parent / "static" / "index.html"
    )


def create_app():
    app = web.Application()
    app.router.add_get("/", index_handler)
    app.router.add_get("/ws", websocket_handler)
    static_dir = Path(__file__).parent / "static"
    if static_dir.exists():
        app.router.add_static("/static", static_dir)
    return app


def main():
    port = 8080
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass
    app = create_app()
    print(f"  Umbreon Dashboard -> http://localhost:{port}")
    web.run_app(app, host="0.0.0.0", port=port, print=None)


if __name__ == "__main__":
    main()
