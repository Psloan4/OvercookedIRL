class_name BridgeItem
extends Node2D

## A tagged item. Display only -- the engine owns its state, and its position
## is the only thing the engine is told. Deliberately not a physics body: an
## item sits ON the solid table, so it must not collide with it, and pickup is
## a distance check rather than an overlap.
##
## Art is the live game's own PNGs, loaded from GameFramework/assets/ at
## runtime. res:// can't reach outside the Godot project, so importing them
## would mean a second copy that drifts; loading by absolute path keeps one.
## States with no picture fall back to the colour-coded rect.
##
## The scan bar under the item mirrors the live game's: same colours, same
## geometry, and a burn drains full -> empty instead of filling. The
## destination ring over it does too -- see bridge_ring.gd.

const SIZE := Vector2(34, 34)
const BAR_H := 8.0
const BAR_GAP := 3.0

# Same inset the live game uses: half the ring's pen width, plus one.
const RING_INSET := 3.0

# Matched to QProgressBar#ScanBar in style.py.
const BAR_TRACK := Color(0, 0, 0, 0.18)
const BAR_SCAN := Color("#2563eb")
const BAR_BURN := Color("#dc2626")
const BAR_COMBINE := Color("#22c55e")

# path -> Texture2D, or null when the file would not load. Shared by every
# item, so each PNG is read once per run.
static var _tex_cache: Dictionary = {}

var tag: int = -1
var state: String = ""
var kind: String = ""
var held_by: Node2D = null

var _assets: Dictionary = {}
var _dir: String = ""

var _rect: ColorRect
var _sprite: Sprite2D
var _ring: BridgeRing
var _label: Label
var _bar: Panel
var _fill: Panel
var _fill_style: StyleBoxFlat


static func _texture_for(path: String) -> Texture2D:
	if _tex_cache.has(path):
		return _tex_cache[path]
	var tex: Texture2D = null
	if FileAccess.file_exists(path):
		var img := Image.load_from_file(path)
		if img != null:
			tex = ImageTexture.create_from_image(img)
	else:
		push_warning("bridge: no art at %s" % path)
	_tex_cache[path] = tex
	return tex


func setup(tag_id: int, item_kind: String,
		assets: Dictionary = {}, assets_dir: String = "") -> void:
	tag = tag_id
	kind = item_kind
	name = "Item%d" % tag_id
	_assets = assets
	_dir = assets_dir

	_rect = ColorRect.new()
	_rect.size = SIZE
	_rect.position = -SIZE * 0.5
	_rect.color = Color(0.8, 0.8, 0.8)
	_rect.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_rect)

	_sprite = Sprite2D.new()
	_sprite.visible = false
	add_child(_sprite)

	# Added after the art so it paints on top of it, as the live game does.
	_ring = BridgeRing.new()
	_ring.radius = SIZE.x * 0.5 - RING_INSET
	add_child(_ring)

	_bar = Panel.new()
	_bar.size = Vector2(SIZE.x, BAR_H)
	_bar.position = Vector2(-SIZE.x * 0.5, SIZE.y * 0.5 + BAR_GAP)
	_bar.add_theme_stylebox_override("panel", _bar_style(BAR_TRACK))
	_bar.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_bar.visible = false
	add_child(_bar)

	_fill_style = _bar_style(BAR_SCAN)
	_fill = Panel.new()
	_fill.size = Vector2(0, BAR_H)
	_fill.add_theme_stylebox_override("panel", _fill_style)
	_fill.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_bar.add_child(_fill)

	_label = Label.new()
	_label.position = Vector2(-SIZE.x * 0.5, SIZE.y * 0.5 + BAR_GAP + BAR_H + 2.0)
	_label.add_theme_font_size_override("font_size", 10)
	_label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_label)


func _bar_style(c: Color) -> StyleBoxFlat:
	var sb := StyleBoxFlat.new()
	sb.bg_color = c
	sb.set_corner_radius_all(int(BAR_H * 0.5))
	return sb


# The picture for a state, or null when ASSET_MAP has none for it.
func _art_for(item_state: String) -> Texture2D:
	if _dir.is_empty() or not _assets.has(item_state):
		return null
	return _texture_for(_dir.path_join(str(_assets[item_state])))


func apply(new_state: String, tint: Color) -> void:
	state = new_state
	_label.text = "%d %s" % [tag, new_state]
	_rect.color = tint

	var tex := _art_for(new_state)
	_sprite.visible = tex != null
	_rect.visible = tex == null
	if tex == null:
		return
	_sprite.texture = tex
	# Fit the art to the block's real footprint, keeping its aspect.
	var px := tex.get_size()
	var longest: float = maxf(px.x, px.y)
	if longest > 0.0:
		_sprite.scale = Vector2.ONE * (SIZE.x / longest)


func set_ring(color_hex: String, is_ready: bool) -> void:
	_ring.set_ring(color_hex, is_ready)


func set_progress(progress: float, scanning: bool,
		burning: bool = false, combining: bool = false) -> void:
	_bar.visible = scanning
	if not scanning:
		return
	var p := clampf(progress, 0.0, 1.0)
	if burning:
		p = 1.0 - p
	_fill.size = Vector2(SIZE.x * p, BAR_H)
	if burning:
		_fill_style.bg_color = BAR_BURN
	elif combining:
		_fill_style.bg_color = BAR_COMBINE
	else:
		_fill_style.bg_color = BAR_SCAN
