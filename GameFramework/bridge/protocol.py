"""Wire format: newline-delimited JSON, one object per line, UTF-8.

Chosen because Godot speaks it with no addons: StreamPeerTCP for the socket,
JSON.parse_string / JSON.stringify for the payload.

Client -> server
  {"type":"hello"}                              ask for the config handshake
  {"type":"observe","now":12.5,
   "tags":[[0,84.0,271.0], ...],                (tag_id, x, y) full-frame px
   "players":{"2a":true,"2b":false,"3":false}}
  {"type":"reset"}                              start a fresh round

Server -> client
  {"type":"config", ...client_config()}         sent on hello and on connect
  {"type":"state", ...Engine.tick() snapshot}
  {"type":"error","message":"..."}
"""

from __future__ import annotations

import json

HOST = "127.0.0.1"
PORT = 8777
ENCODING = "utf-8"


def encode(obj) -> bytes:
    """One message, newline-terminated. separators keep frames small."""
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode(ENCODING)


class LineReader:
    """Reassembles newline-delimited JSON from arbitrary chunk boundaries.

    TCP does not preserve message framing, so a naive recv-and-parse breaks
    the moment a frame straddles two packets -- which it will, once the state
    snapshot grows past the MTU.
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, chunk: bytes):
        """Yields each complete message in `chunk`, holding any partial tail."""
        self._buf.extend(chunk)
        while True:
            nl = self._buf.find(b"\n")
            if nl < 0:
                return
            line = bytes(self._buf[:nl])
            del self._buf[:nl + 1]
            if not line.strip():
                continue
            try:
                yield json.loads(line.decode(ENCODING))
            except (ValueError, UnicodeDecodeError) as e:
                yield {"type": "error", "message": f"bad frame: {e}"}
