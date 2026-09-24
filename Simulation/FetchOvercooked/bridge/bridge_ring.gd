class_name BridgeRing
extends Node2D

## The destination ring painted over an item, mirroring _ItemImage in
## ui_components.py: a solid ring in the colour of the station the item is
## bound for, a split ring when it has two destinations, and a pulsing green
## ring -- which replaces the destination ring -- once it is ready to combine.

const READY := Color("#22c55e")
const SPLIT_GREEN := Color("#22c55e")
const SPLIT_YELLOW := Color("#facc15")

const WIDTH := 4.0
const READY_WIDTH := 5.0
const SEGMENTS := 48

const PULSE_SECONDS := 0.9
const PULSE_MIN := 0.3
const PULSE_MAX := 1.0

var radius := 14.0

var _color := ""
var _pulsing := false
var _clock := 0.0


func set_ring(color_hex: String, is_ready: bool) -> void:
	if color_hex == _color and is_ready == _pulsing:
		return
	_color = color_hex
	if is_ready and not _pulsing:
		_clock = 0.0
	_pulsing = is_ready
	queue_redraw()


func _process(delta: float) -> void:
	if not _pulsing:
		return
	_clock = fmod(_clock + delta, PULSE_SECONDS)
	queue_redraw()


# 0.3 -> 1.0 -> 0.3 over one period, matching the eased pulse in the live game.
func _alpha() -> float:
	var mid := (PULSE_MIN + PULSE_MAX) * 0.5
	var half := (PULSE_MAX - PULSE_MIN) * 0.5
	return mid - half * cos(TAU * _clock / PULSE_SECONDS)


func _draw() -> void:
	if _pulsing:
		var c := READY
		c.a = _alpha()
		draw_arc(Vector2.ZERO, radius, 0.0, TAU, SEGMENTS, c, READY_WIDTH, true)
		return
	if _color.is_empty():
		return
	# Two destinations: green over the top-left half, yellow over the rest.
	if _color == "green_yellow":
		draw_arc(Vector2.ZERO, radius, deg_to_rad(-225.0), deg_to_rad(-45.0),
			SEGMENTS, SPLIT_GREEN, WIDTH, true)
		draw_arc(Vector2.ZERO, radius, deg_to_rad(-45.0), deg_to_rad(135.0),
			SEGMENTS, SPLIT_YELLOW, WIDTH, true)
		return
	if not _color.begins_with("#"):
		return
	draw_arc(Vector2.ZERO, radius, 0.0, TAU, SEGMENTS, Color(_color), WIDTH, true)
