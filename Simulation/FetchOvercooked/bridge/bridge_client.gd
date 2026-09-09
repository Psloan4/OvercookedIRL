class_name BridgeClient
extends Node

## TCP link to the Python rules engine (GameFramework/bridge/server.py).
## Transport only -- it knows nothing about the game.

signal config_received(config: Dictionary)
signal state_received(state: Dictionary)

@export var host: String = "127.0.0.1"
@export var port: int = 8777
@export var autoconnect: bool = true

var connected := false
var config: Dictionary = {}
var state: Dictionary = {}

var _peer := StreamPeerTCP.new()
var _buf := ""


func _ready() -> void:
	if autoconnect:
		open()


func open() -> void:
	var err := _peer.connect_to_host(host, port)
	if err != OK:
		push_error("bridge: connect_to_host(%s:%d) failed -> %d" % [host, port, err])


func _process(_dt: float) -> void:
	if not _pump_link():
		return
	var avail := _peer.get_available_bytes()
	if avail <= 0:
		return
	var res: Array = _peer.get_data(avail)
	if res[0] != OK:
		return
	_buf += (res[1] as PackedByteArray).get_string_from_utf8()
	_drain()


func _pump_link() -> bool:
	_peer.poll()
	match _peer.get_status():
		StreamPeerTCP.STATUS_CONNECTED:
			if not connected:
				connected = true
				print("bridge: connected to %s:%d" % [host, port])
			return true
		StreamPeerTCP.STATUS_ERROR:
			if connected:
				push_warning("bridge: link lost")
			connected = false
	return false


# Messages are newline-delimited; TCP splits them anywhere, so buffer.
func _drain() -> void:
	while true:
		var nl := _buf.find("\n")
		if nl < 0:
			return
		var line := _buf.substr(0, nl)
		_buf = _buf.substr(nl + 1)
		if line.strip_edges().is_empty():
			continue
		var msg = JSON.parse_string(line)
		if typeof(msg) != TYPE_DICTIONARY:
			push_warning("bridge: unparseable frame")
			continue
		match msg.get("type", ""):
			"config":
				config = msg
				config_received.emit(msg)
			"state":
				state = msg
				state_received.emit(msg)
			"error":
				push_warning("bridge: %s" % msg.get("message", ""))


func send(msg: Dictionary) -> void:
	if connected:
		_peer.put_data((JSON.stringify(msg) + "\n").to_utf8_buffer())


func observe(now: float, tags: Array, players: Dictionary) -> void:
	send({"type": "observe", "now": now, "tags": tags, "players": players})


func reset_round() -> void:
	send({"type": "reset"})
