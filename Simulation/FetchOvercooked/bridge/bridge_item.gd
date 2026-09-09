class_name BridgeItem
extends Node2D

## A tagged item. Display only -- the engine owns its state, and its position
## is the only thing the engine is told. Deliberately not a physics body: an
## item sits ON the solid table, so it must not collide with it, and pickup is
## a distance check rather than an overlap.

const SIZE := Vector2(34, 34)

var tag: int = -1
var state: String = ""
var kind: String = ""
var held_by: Node2D = null

var _rect: ColorRect
var _label: Label


func setup(tag_id: int, item_kind: String) -> void:
	tag = tag_id
	kind = item_kind
	name = "Item%d" % tag_id

	_rect = ColorRect.new()
	_rect.size = SIZE
	_rect.position = -SIZE * 0.5
	_rect.color = Color(0.8, 0.8, 0.8)
	_rect.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_rect)

	_label = Label.new()
	_label.position = Vector2(-SIZE.x * 0.5, SIZE.y * 0.5 + 2.0)
	_label.add_theme_font_size_override("font_size", 10)
	_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_label)


func apply(new_state: String, tint: Color) -> void:
	state = new_state
	_label.text = "%d %s" % [tag, new_state]
	_rect.color = tint
