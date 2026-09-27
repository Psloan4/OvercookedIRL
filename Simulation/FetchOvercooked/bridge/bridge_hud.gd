class_name BridgeHud
extends CanvasLayer

## The live game's top bar: order tickets on the left, POINTS in the middle,
## TIME LEFT on the right, each on a white rounded card. Mirrors GamePage /
## OrderRail / OrderTicket in ui_components.py and the #Card style in style.py,
## scaled down for the simulator window.
##
## Screen space (a CanvasLayer), so the camera's zoom never touches it;
## bridge_map.gd reserves HEIGHT at the top of the view for it.

const CARD := 48.0
const PAD := 8.0
const HEIGHT := CARD + PAD * 2.0
const MAX_VISIBLE := 6              # OrderRail.MAX_VISIBLE

# style.py: page fill, #Card, #HudLabel, body text.
const PAGE := Color("#e9edf5")
const CARD_BG := Color("#ffffff")
const CARD_RADIUS := 12
const LABEL := Color("#6b7280")
const VALUE := Color("#1f2937")

var _bar: PanelContainer
var _tickets: HBoxContainer
var _points: Label
var _time: Label
var _order_sig := ""
var _icons: Dictionary = {}
var _dir := ""
var _bold: SystemFont


func _ready() -> void:
	_bold = SystemFont.new()
	_bold.font_names = PackedStringArray(["Inter", "Segoe UI", "Arial"])
	_bold.font_weight = 900

	var bar := PanelContainer.new()
	_bar = bar
	bar.set_anchors_and_offsets_preset(Control.PRESET_TOP_WIDE)
	bar.custom_minimum_size = Vector2(0, HEIGHT)
	var fill := StyleBoxFlat.new()
	fill.bg_color = PAGE
	fill.set_content_margin_all(PAD)
	bar.add_theme_stylebox_override("panel", fill)
	bar.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(bar)

	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 12)
	bar.add_child(row)

	var header := _text("ORDERS", 16, LABEL)
	row.add_child(header)

	_tickets = HBoxContainer.new()
	_tickets.add_theme_constant_override("separation", 8)
	row.add_child(_tickets)

	row.add_child(_spacer())
	var points := _hud_block("POINTS", "0")
	_points = points[1]
	row.add_child(points[0])
	row.add_child(_spacer())
	var time := _hud_block("TIME LEFT", "0:00")
	_time = time[1]
	row.add_child(time[0])


## Pin the bar to a horizontal span of the screen (the arena's walkable
## width) instead of the full window.
func set_span(left: float, width: float) -> void:
	_bar.set_anchors_preset(Control.PRESET_TOP_LEFT)
	_bar.position = Vector2(left, 0.0)
	_bar.size = Vector2(width, HEIGHT)


## order type -> filename, and the folder they live in (client_config).
func set_icons(icons: Dictionary, assets_dir: String) -> void:
	_icons = icons
	_dir = assets_dir


func set_points(points: int) -> void:
	_points.text = str(points)


func set_time_left(seconds: float) -> void:
	var s: int = max(0, int(ceil(seconds)))
	_time.text = "%d:%02d" % [floori(s / 60.0), s % 60]


## orders: the engine's [{type, time}, ...], oldest first. Rebuilt only when
## the list actually changes, not every tick.
func set_orders(orders: Array) -> void:
	var shown := orders.slice(0, MAX_VISIBLE)
	var parts: PackedStringArray = []
	for o in shown:
		parts.append("%s@%s" % [o["type"], o["time"]])
	var sig := "|".join(parts)
	if sig == _order_sig:
		return
	_order_sig = sig
	for c in _tickets.get_children():
		c.queue_free()
	for o in shown:
		_tickets.add_child(_ticket(str(o["type"])))


func _ticket(order_type: String) -> Control:
	var card := _card(CARD * 0.2)
	card.custom_minimum_size = Vector2(CARD, CARD)
	var tex: Texture2D = null
	if _icons.has(order_type) and not _dir.is_empty():
		tex = BridgeItem._texture_for(_dir.path_join(str(_icons[order_type])))
	if tex != null:
		var pic := TextureRect.new()
		pic.texture = tex
		pic.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
		pic.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
		pic.mouse_filter = Control.MOUSE_FILTER_IGNORE
		card.add_child(pic)
	else:
		var name_label := _text(order_type.replace("complete_", ""), 11, VALUE)
		name_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
		name_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		card.add_child(name_label)
	return card


# A wide, short card: label beside the value, as _make_hud_block lays it out.
func _hud_block(label_text: String, value_text: String) -> Array:
	var card := _card(0.0)
	var style := card.get_theme_stylebox("panel") as StyleBoxFlat
	style.content_margin_left = 16.0
	style.content_margin_right = 16.0
	card.custom_minimum_size = Vector2(0, CARD)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 10)
	row.alignment = BoxContainer.ALIGNMENT_CENTER
	card.add_child(row)
	row.add_child(_text(label_text, 14, LABEL))
	var value := _text(value_text, 26, VALUE)
	# Wide enough for "0:00" / three-digit scores, so the card doesn't jitter.
	value.custom_minimum_size = Vector2(58, 0)
	row.add_child(value)
	return [card, value]


func _card(margin: float) -> PanelContainer:
	var card := PanelContainer.new()
	var sb := StyleBoxFlat.new()
	sb.bg_color = CARD_BG
	sb.set_corner_radius_all(CARD_RADIUS)
	sb.set_content_margin_all(margin)
	card.add_theme_stylebox_override("panel", sb)
	card.mouse_filter = Control.MOUSE_FILTER_IGNORE
	return card


func _text(t: String, size: int, color: Color) -> Label:
	var l := Label.new()
	l.text = t
	l.add_theme_font_override("font", _bold)
	l.add_theme_font_size_override("font_size", size)
	l.add_theme_color_override("font_color", color)
	l.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	l.mouse_filter = Control.MOUSE_FILTER_IGNORE
	return l


func _spacer() -> Control:
	var c := Control.new()
	c.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	c.mouse_filter = Control.MOUSE_FILTER_IGNORE
	return c
