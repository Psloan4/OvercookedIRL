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
const WALK_MARGIN := 110.0
const WALL := 40.0

# How close a body must be to a gated station's rect to attend it. A body
# attends at most ONE station -- the nearest -- because the gated rects sit
# side by side and any padding large enough to be reachable overlaps them.
# PLAYER_ZONES can't be reused: those rects live in each station's own camera
# frame, not table space.
const PRESENCE_REACH := 70.0

enum Round { IDLE, RUNNING, ENDED }

@export var client: BridgeClient

var _items_root: Node2D
var _players: Array[BridgePlayer] = []
var _gated: Dictionary = {}              # zone key -> Rect2 (world coords)
var _stage_colors: Dictionary = {}
var _home: Dictionary = {}               # node -> start position
var _now := 0.0
var _game_seconds := 150.0
var _round: int = Round.IDLE
var _built := false
var _final_score := 0

var _hud: Label
var _banner: Label


func _ready() -> void:
	if client == null:
		client = BridgeClient.new()
		add_child(client)
	client.config_received.connect(_on_config)
	client.state_received.connect(_on_state)

	_items_root = Node2D.new()
	_items_root.name = "Items"

	_hud = Label.new()
	_hud.position = Vector2(12, 8)
	_hud.add_theme_font_size_override("font_size", 18)
	_hud.text = "connecting to rules engine..."

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
	for s in config.get("stations", []):
		_add_region(s["x"], s["y"], s["w"], s["h"], s["color"],
			"%s %s" % [s["stype"], s["name"]])
		var zone = s.get("player_zone")
		if zone != null:
			_gated[zone] = Rect2(_to_world(s["x"], s["y"]),
				Vector2(s["w"], s["h"]) * WORLD_SCALE)

	var f: Dictionary = config.get("final_station", {})
	if not f.is_empty():
		_add_region(f["x"], f["y"], f["w"], f["h"], f["color"], "4 Delivery")

	var play := table.grow(WALK_MARGIN)
	_add_walls(play)

	# Items park along the strip below the table, like a prep counter.
	var tags: Array = config.get("food_tags", [])
	var types: Dictionary = config.get("tag_types", {})
	add_child(_items_root)
	for i in tags.size():
		var tag := int(tags[i])
		var it := BridgeItem.new()
		it.setup(tag, str(types.get(str(tag), "?")))
		it.position = Vector2(
			table.position.x + 30.0 + float(i % 9) * 46.0,
			table.end.y + 28.0 + floor(float(i) / 9.0) * 44.0)
		_items_root.add_child(it)
		_home[it] = it.position

	_players.append(_spawn_player("p1",
		Vector2(table.position.x - WALK_MARGIN * 0.5, table.position.y + 60.0),
		Color(0.2, 0.6, 1.0)))
	_players.append(_spawn_player("p2",
		Vector2(table.end.x + WALK_MARGIN * 0.5, table.end.y - 60.0),
		Color(1.0, 0.45, 0.2)))

	add_child(_hud)
	add_child(_banner)
	_banner.size = Vector2(play.size.x, 40)
	_banner.position = Vector2(play.position.x, play.get_center().y - 20)

	var cam := get_node_or_null("Camera2D") as Camera2D
	if cam != null:
		cam.position = play.get_center()

	_enter_idle()
	print("bridge: built table %s, %d stations, %d items" % [
		str(table), config.get("stations", []).size(), tags.size()])


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


func _spawn_player(prefix: String, at: Vector2, tint: Color) -> BridgePlayer:
	var p := BridgePlayer.new()
	add_child(p)
	p.setup(prefix, at, tint, _items_root)
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


# ---------------- round lifecycle ----------------

func _enter_idle() -> void:
	_round = Round.IDLE
	_now = 0.0
	_set_players_active(false)
	_banner.text = "Press Enter to start"
	_banner.visible = true
	_hud.text = "Score 0   %ds" % int(_game_seconds)


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
	_banner.text = "Time! Final score %d\nPress Enter to play again" % _final_score
	_banner.visible = true
	print("bridge: round over, score %d" % _final_score)


func _set_players_active(on: bool) -> void:
	for p in _players:
		p.active = on


# ---------------- tick ----------------

func _physics_process(delta: float) -> void:
	if not _built or not client.connected:
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

	for child in _items_root.get_children():
		var it := child as BridgeItem
		if it == null:
			continue
		var key := str(it.tag)
		if not items.has(key):
			continue
		var new_state := str(items[key]["state"])
		var tint := _tint_for(new_state)
		# Brighten while a scan is running, so progress is visible.
		if scans.has(key):
			tint = tint.lerp(Color.WHITE, float(scans[key]) * 0.6)
		it.apply(new_state, tint)

	_final_score = int(state.get("points", 0))
	if _round != Round.RUNNING:
		return
	var open: Array = []
	for o in state.get("orders", []):
		open.append(str(o["type"]).replace("complete_", ""))
	_hud.text = "Score %d   %ds left   orders: %s" % [
		_final_score, int(max(0.0, _game_seconds - _now)), ", ".join(open)]


func _dist_to_rect(p: Vector2, r: Rect2) -> float:
	var cx: float = clamp(p.x, r.position.x, r.end.x)
	var cy: float = clamp(p.y, r.position.y, r.end.y)
	return p.distance_to(Vector2(cx, cy))


func _tint_for(state_name: String) -> Color:
	if _stage_colors.has(state_name):
		var v = _stage_colors[state_name]
		# STAGE_COLORS has one non-hex entry ("green_yellow").
		if typeof(v) == TYPE_STRING and str(v).begins_with("#"):
			return Color(str(v))
		return Color.YELLOW_GREEN
	return Color(0.75, 0.75, 0.75)
