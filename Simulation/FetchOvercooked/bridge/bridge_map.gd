extends Node2D

## Builds the whole scene from the engine's config handshake, then streams
## observations to it every physics frame and applies what comes back.
##
## Nothing here is hand-placed: the table, station rects, colours and the tag
## pool all come from config.py, so editing config.py moves the Godot world.
##
## Godot world units == config.py's full-frame pixel coords (identity), so a
## station rect is used verbatim. Change WORLD_SCALE to rescale.

const WORLD_SCALE := 1.0
const ORIGIN := Vector2(130, 30)

# Walking room around the table, and the wall thickness that pens it in.
const WALK_MARGIN := 180.0
const WALL := 14.0
const WALL_COLOR := Color(0.16, 0.18, 0.22)

# Body size: a third of the Cooking station's short side, then tripled in area.
const BODY_FRACTION := 1.0 / 3.0
const BODY_AREA_SCALE := 3.0
const FALLBACK_RADIUS := 16.0

# How close a body must be to a gated station's rect to attend it. A body
# attends at most ONE station -- the nearest -- because the gated rects sit
# side by side and any padding large enough to be reachable overlaps them.
# PLAYER_ZONES can't be reused: those rects live in each station's own camera
# frame, not table space.
const PRESENCE_REACH := 70.0

# Title bar and borders aren't part of the client size, so leave room for
# them when sizing the window to the arena.
const WINDOW_CHROME := 80.0

enum Round { IDLE, RUNNING, ENDED }

@export var client: BridgeClient

var _items_root: Node2D
var _players: Array[BridgePlayer] = []
var _gated: Dictionary = {}              # zone key -> Rect2 (world coords)
var _stage_colors: Dictionary = {}
var _home: Dictionary = {}               # node -> start position
var _play := Rect2()
var _view_size := Vector2.ZERO
var _now := 0.0
var _game_seconds := 150.0
var _round: int = Round.IDLE
var _built := false
var _sized := false
var _final_score := 0

var _hud: BridgeHud
var _banner: Label


func _ready() -> void:
	if client == null:
		client = BridgeClient.new()
		add_child(client)
	client.config_received.connect(_on_config)
	client.state_received.connect(_on_state)

	_items_root = Node2D.new()
	_items_root.name = "Items"

	_hud = BridgeHud.new()

	_banner = Label.new()
	_banner.add_theme_font_size_override("font_size", 26)
	_banner.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_banner.visible = false


func _to_world(x: float, y: float) -> Vector2:
	return ORIGIN + Vector2(x, y) * WORLD_SCALE


# ---------------- build ----------------

func _on_config(config: Dictionary) -> void:
	if _built:
		return
	_built = true
	_stage_colors = config.get("stage_colors", {})
	_game_seconds = float(config.get("game_seconds", 150.0))

	var t: Array = config.get("table_region", [7, 110, 604, 322])
	var table := Rect2(_to_world(float(t[0]), float(t[1])),
		Vector2(float(t[2]), float(t[3])) * WORLD_SCALE)

	_add_table(table)
	var body_radius := FALLBACK_RADIUS
	for s in config.get("stations", []):
		_add_region(s["x"], s["y"], s["w"], s["h"], s["color"],
			"%s %s" % [s["stype"], s["name"]])
		var zone = s.get("player_zone")
		if zone != null:
			_gated[zone] = Rect2(_to_world(s["x"], s["y"]),
				Vector2(s["w"], s["h"]) * WORLD_SCALE)
		if str(s["name"]) == "Cooking":
			var short_side: float = min(float(s["w"]), float(s["h"]))
			var third: float = short_side * WORLD_SCALE * BODY_FRACTION
			body_radius = third * 0.5 * sqrt(BODY_AREA_SCALE)

	# The delivery board is a real object, so config.py hands it over already
	# sized and placed in table space -- not the camera crop the live game uses.
	var f: Dictionary = config.get("final_station", {})
	var board := Rect2()
	var sections: Array = []
	if not f.is_empty():
		board = Rect2(_to_world(f["x"], f["y"]),
			Vector2(f["w"], f["h"]) * WORLD_SCALE)
		sections = f.get("sections", [])
		_add_board(board, str(f["color"]), sections)

	var play := table.grow(WALK_MARGIN)
	if board.size.x > 0.0:
		play = play.merge(board.grow(WALK_MARGIN))
	_add_walls(play)

	# Blocks rest in their type's section on the board, two side by side per
	# section, and return there when delivered or binned.
	var tags: Array = config.get("food_tags", [])
	var types: Dictionary = config.get("tag_types", {})
	# The live game's art, by item type, loaded from its own folder.
	var assets: Dictionary = config.get("assets", {})
	var assets_dir := str(config.get("assets_dir", ""))
	_hud.set_icons(config.get("order_icons", {}), assets_dir)
	add_child(_items_root)
	var slots: Dictionary = {}
	var rows: int = max(sections.size(), 1)
	var sec_h: float = board.size.y / float(rows)
	for i in tags.size():
		var tag := int(tags[i])
		var kind := str(types.get(str(tag), "?"))
		var it := BridgeItem.new()
		it.setup(tag, kind, assets.get(kind, {}), assets_dir)
		if sec_h > 0.0:
			var row: int = sections.find(kind)
			if row < 0:
				row = i % rows
			var slot: int = int(slots.get(kind, 0))
			slots[kind] = slot + 1
			it.position = board.position + Vector2(
				board.size.x * (0.30 + 0.40 * float(slot)),
				sec_h * (float(row) + 0.5))
		else:
			it.position = Vector2(
				table.position.x + 30.0 + float(i % 9) * 46.0,
				table.end.y + 28.0 + floor(float(i) / 9.0) * 44.0)
		_items_root.add_child(it)
		_home[it] = it.position

	# Bodies start in the lanes above and below the table -- the board now owns
	# the left lane, so spawning there would drop them on top of it.
	_players.append(_spawn_player("p1",
		Vector2(table.position.x + table.size.x * 0.25,
			table.position.y - WALK_MARGIN * 0.5),
		Color(0.2, 0.6, 1.0), body_radius))
	_players.append(_spawn_player("p2",
		Vector2(table.position.x + table.size.x * 0.75,
			table.end.y + WALK_MARGIN * 0.5),
		Color(1.0, 0.45, 0.2), body_radius))

	add_child(_hud)
	add_child(_banner)
	_play = play
	_fit_window()
	_fit_view()

	_enter_idle()
	print("bridge: built table %s, %d stations, %d items" % [
		str(table), config.get("stations", []).size(), tags.size()])


# Scale the arena to whatever the window is now -- in or out -- below the HUD
# bar, and undo that zoom on the banner so text keeps its authored size. Re-run
# on every resize.
# Wrap the window around the arena, once, so the walls meet its edges and the
# HUD bar sits straight on top of them -- no dead space on any side. _fit_view
# then finds the same zoom with nothing left over to letterbox.
func _fit_window() -> void:
	if _sized or _play.size.x <= 0.0:
		return
	_sized = true
	var frame := _play.grow(WALL)
	# The project stretches by canvas_items, so the base viewport -- not the
	# window -- is what the arena is laid out against. Match it to the arena
	# and there is nothing left over to letterbox.
	var base := Vector2i(int(round(frame.size.x)),
		int(round(frame.size.y + BridgeHud.HEIGHT)))
	get_window().content_scale_size = base

	var usable := Vector2(DisplayServer.screen_get_usable_rect().size)
	var fit := minf(1.0, minf(usable.x / float(base.x),
		(usable.y - WINDOW_CHROME) / float(base.y)))
	DisplayServer.window_set_size(Vector2i(Vector2(base) * fit))
	print("bridge: viewport %v, window %v" % [base, Vector2(base) * fit])


func _fit_view() -> void:
	if _play.size.x <= 0.0:
		return
	var frame := _play.grow(WALL)
	var view := get_viewport_rect().size
	var room := view - Vector2(0.0, BridgeHud.HEIGHT)
	if room.x <= 0.0 or room.y <= 0.0:
		return
	_view_size = view
	var fit: float = min(room.x / frame.size.x, room.y / frame.size.y)
	print("bridge: fit view=%v frame=%v play=%v zoom=%.3f" % [
		view, frame.size, _play.size, fit])

	var cam := get_node_or_null("Camera2D") as Camera2D
	if cam == null:
		push_warning("bridge: no Camera2D -- view not framed")
		return
	# Shift up by half the bar so the arena centres in the room below it.
	cam.position = _play.get_center() - Vector2(0.0, BridgeHud.HEIGHT * 0.5 / fit)
	cam.zoom = Vector2.ONE * fit
	# Full width, so the arena flows straight into the bar with no gap.
	_hud.set_span(0.0, view.x)
	print("bridge: camera current=%s zoom=%v pos=%v" % [
		str(cam.is_current()), cam.zoom, cam.position])

	_banner.size = Vector2(_play.size.x * fit, 40)
	_banner.position = Vector2(_play.position.x, _play.get_center().y - 20.0 / fit)
	_banner.scale = Vector2.ONE / fit


func _add_table(rect: Rect2) -> void:
	var body := StaticBody2D.new()
	body.name = "Table"
	body.position = rect.get_center()
	add_child(body)

	var shape := CollisionShape2D.new()
	var box := RectangleShape2D.new()
	box.size = rect.size
	shape.shape = box
	body.add_child(shape)

	var surface := ColorRect.new()
	surface.size = rect.size
	surface.position = -rect.size * 0.5
	surface.color = Color(0.05, 0.07, 0.10)
	surface.mouse_filter = Control.MOUSE_FILTER_IGNORE
	body.add_child(surface)


func _add_walls(play: Rect2) -> void:
	var body := StaticBody2D.new()
	body.name = "Walls"
	add_child(body)
	var spans: Array[Rect2] = [
		Rect2(play.position.x - WALL, play.position.y - WALL,
			play.size.x + WALL * 2.0, WALL),
		Rect2(play.position.x - WALL, play.end.y,
			play.size.x + WALL * 2.0, WALL),
		Rect2(play.position.x - WALL, play.position.y, WALL, play.size.y),
		Rect2(play.end.x, play.position.y, WALL, play.size.y),
	]
	for r in spans:
		var shape := CollisionShape2D.new()
		var box := RectangleShape2D.new()
		box.size = r.size
		shape.shape = box
		shape.position = r.get_center()
		body.add_child(shape)

		var face := ColorRect.new()
		face.position = r.position
		face.size = r.size
		face.color = WALL_COLOR
		face.mouse_filter = Control.MOUSE_FILTER_IGNORE
		body.add_child(face)


func _spawn_player(prefix: String, at: Vector2, tint: Color,
		body_radius: float) -> BridgePlayer:
	var p := BridgePlayer.new()
	add_child(p)
	p.setup(prefix, at, tint, _items_root, body_radius)
	_home[p] = at
	return p


func _add_region(x: float, y: float, w: float, h: float,
		color_hex: String, label_text: String) -> void:
	var holder := Node2D.new()
	holder.position = _to_world(x, y)
	add_child(holder)

	var rect := ColorRect.new()
	rect.size = Vector2(w, h) * WORLD_SCALE
	var c := Color(color_hex)
	c.a = 0.30
	rect.color = c
	rect.mouse_filter = Control.MOUSE_FILTER_IGNORE
	holder.add_child(rect)

	var label := Label.new()
	label.position = Vector2(6, 4)
	label.text = label_text
	label.add_theme_font_size_override("font_size", 13)
	label.mouse_filter = Control.MOUSE_FILTER_IGNORE
	holder.add_child(label)


func _add_board(rect: Rect2, color_hex: String, sections: Array) -> void:
	var holder := Node2D.new()
	holder.position = rect.position
	add_child(holder)

	var base := ColorRect.new()
	base.size = rect.size
	var c := Color(color_hex)
	c.a = 0.30
	base.color = c
	base.mouse_filter = Control.MOUSE_FILTER_IGNORE
	holder.add_child(base)

	# The board is narrow and tall, so the title sits above it.
	var title := Label.new()
	title.position = Vector2(0, -24)
	title.text = "4 Delivery"
	title.add_theme_font_size_override("font_size", 13)
	title.mouse_filter = Control.MOUSE_FILTER_IGNORE
	holder.add_child(title)

	var n := sections.size()
	if n == 0:
		return
	var sec_h := rect.size.y / float(n)
	for i in n:
		if i > 0:
			var divider := ColorRect.new()
			divider.size = Vector2(rect.size.x, 2.0)
			divider.position = Vector2(0.0, sec_h * float(i) - 1.0)
			divider.color = Color(1.0, 1.0, 1.0, 0.35)
			divider.mouse_filter = Control.MOUSE_FILTER_IGNORE
			holder.add_child(divider)
		var section_name := Label.new()
		section_name.text = str(sections[i])
		section_name.position = Vector2(6.0, sec_h * float(i) + 3.0)
		section_name.add_theme_font_size_override("font_size", 11)
		section_name.mouse_filter = Control.MOUSE_FILTER_IGNORE
		holder.add_child(section_name)

# ---------------- round lifecycle ----------------

func _enter_idle() -> void:
	_round = Round.IDLE
	_now = 0.0
	_set_players_active(false)
	_banner.text = "Press Enter to start"
	_banner.visible = true
	_hud.set_points(0)
	_hud.set_time_left(_game_seconds)
	_hud.set_orders([])


func _begin_round() -> void:
	for node in _home:
		(node as Node2D).position = _home[node]
	for p in _players:
		p.release()
		p.rotation = 0.0
	_now = 0.0
	_final_score = 0
	client.reset_round()
	_banner.visible = false
	_set_players_active(true)
	_round = Round.RUNNING
	print("bridge: round started")


func _end_round() -> void:
	_round = Round.ENDED
	_set_players_active(false)
	_hud.set_time_left(0.0)
	_banner.text = "Time! Final score %d\nPress Enter to play again" % _final_score
	_banner.visible = true
	print("bridge: round over, score %d" % _final_score)


func _set_players_active(on: bool) -> void:
	for p in _players:
		p.active = on


# ---------------- tick ----------------

func _physics_process(delta: float) -> void:
	if not _built:
		return
	# The window is usually still settling when the config lands, so refit off
	# the size actually in force rather than trusting one early reading.
	if get_viewport_rect().size != _view_size:
		_fit_view()
	if not client.connected:
		return

	if _round != Round.RUNNING:
		if Input.is_action_just_pressed("ui_accept"):
			_begin_round()
		return

	_now += delta
	if _now >= _game_seconds:
		_end_round()
		return

	var tags: Array = []
	for child in _items_root.get_children():
		var it := child as BridgeItem
		if it == null:
			continue
		var p := (it.global_position - ORIGIN) / WORLD_SCALE
		tags.append([it.tag, p.x, p.y])

	# Nearest gated station within reach, one per body.
	var players: Dictionary = {}
	for zone in _gated:
		players[zone] = false
	for p in _players:
		var best := ""
		var best_d := PRESENCE_REACH
		for zone in _gated:
			var d := _dist_to_rect(p.global_position, _gated[zone])
			if d < best_d:
				best_d = d
				best = zone
		if best != "":
			players[best] = true

	client.observe(_now, tags, players)


func _on_state(state: Dictionary) -> void:
	var items: Dictionary = state.get("items", {})
	var scans: Dictionary = state.get("scans", {})
	var burning: Dictionary = state.get("burning", {})
	var combining: Dictionary = state.get("combining", {})
	var delivery: Dictionary = state.get("delivery_scans", {})
	var ready: Array = state.get("combine_ready", [])

	for child in _items_root.get_children():
		var it := child as BridgeItem
		if it == null:
			continue
		var key := str(it.tag)
		if not items.has(key):
			continue
		var new_state := str(items[key]["state"])
		it.apply(new_state, _tint_for(new_state))
		# Ring = the station this item goes to next, or a green pulse once it
		# is ready to combine.
		it.set_ring(_ring_for(new_state), ready.has(key))
		# A station scan, or the hold over the delivery board -- the bar
		# carries progress now, so the stage colour stays true.
		if scans.has(key):
			it.set_progress(float(scans[key]), true,
				bool(burning.get(key, false)),
				bool(combining.get(key, false)))
		elif delivery.has(key):
			it.set_progress(float(delivery[key]), true)
		else:
			it.set_progress(0.0, false)

	_final_score = int(state.get("points", 0))
	if _round != Round.RUNNING:
		return
	_hud.set_points(_final_score)
	_hud.set_time_left(_game_seconds - _now)
	_hud.set_orders(state.get("orders", []))


func _dist_to_rect(p: Vector2, r: Rect2) -> float:
	var cx: float = clamp(p.x, r.position.x, r.end.x)
	var cy: float = clamp(p.y, r.position.y, r.end.y)
	return p.distance_to(Vector2(cx, cy))


# The raw STAGE_COLORS entry, keeping the "green_yellow" sentinel intact --
# the ring draws that as a split, where the flat tint can't.
func _ring_for(state_name: String) -> String:
	return str(_stage_colors.get(state_name, ""))


func _tint_for(state_name: String) -> Color:
	if _stage_colors.has(state_name):
		var v = _stage_colors[state_name]
		# STAGE_COLORS has one non-hex entry ("green_yellow").
		if typeof(v) == TYPE_STRING and str(v).begins_with("#"):
			return Color(str(v))
		return Color.YELLOW_GREEN
	return Color(0.75, 0.75, 0.75)
