class_name BridgePlayer
extends CharacterBody2D

## Tank-drive body, carried over from player.gd. Under the bridge the only
## interaction is pick up / put down: the engine infers everything else from
## where the item ends up, so there is no station or drop-off special-casing.
##
## The table is solid, so a body walks around it and reaches across the edge --
## drop_distance is what makes an item land on the far side of that edge.

const DEFAULT_RADIUS := 16.0

# Clearance past the body edge, so the drop scales with body size.
const DROP_CLEARANCE := 28.0

# Nose geometry, shared by the drawing and the grab point so the two can't
# drift apart. Fractions of radius.
const NOSE_LEN := 0.9
const NOSE_WIDTH := 0.3
const NOSE_INSET := 0.85

# Catch radius around the arm tip. Fixed rather than scaled by body size,
# because it is sized against BridgeItem.SIZE, which doesn't scale either.
const GRAB_RADIUS := 22.0

@export var player_name: String = "p1"
@export var move_speed := 630.0
@export var turn_speed := 3.5
@export var radius := DEFAULT_RADIUS
@export var drop_distance := DEFAULT_RADIUS + DROP_CLEARANCE
@export var arm_length := DEFAULT_RADIUS * (1.0 + NOSE_LEN * NOSE_INSET)
@export var grab_radius := GRAB_RADIUS

var active := false
var carrying: BridgeItem = null
var items_root: Node2D = null


func setup(input_prefix: String, at: Vector2, tint: Color, items: Node2D,
		body_radius: float = DEFAULT_RADIUS) -> void:
	player_name = input_prefix
	position = at
	items_root = items
	name = "Player_" + input_prefix
	radius = body_radius
	drop_distance = radius + DROP_CLEARANCE
	arm_length = radius * (1.0 + NOSE_LEN * NOSE_INSET)

	var shape := CollisionShape2D.new()
	var circle := CircleShape2D.new()
	circle.radius = radius
	shape.shape = circle
	add_child(shape)

	var body := ColorRect.new()
	body.size = Vector2(radius * 2.0, radius * 2.0)
	body.position = Vector2(-radius, -radius)
	body.color = tint
	body.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(body)

	# Nose, so the facing direction is visible. Its tip is the grab point.
	var nose_len := radius * NOSE_LEN
	var nose_w := radius * NOSE_WIDTH
	var nose := ColorRect.new()
	nose.size = Vector2(nose_w, nose_len)
	nose.position = Vector2(-nose_w * 0.5, -radius - nose_len * NOSE_INSET)
	nose.color = Color.WHITE
	nose.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(nose)


func _physics_process(delta: float) -> void:
	if not active:
		return

	if Input.is_action_just_pressed(player_name + "_grab"):
		if carrying == null:
			_pick_up_nearest()
		else:
			release()

	var throttle := Input.get_action_strength(player_name + "_up") \
		- Input.get_action_strength(player_name + "_down")
	var turn := Input.get_action_strength(player_name + "_right") \
		- Input.get_action_strength(player_name + "_left")

	rotation += turn * turn_speed * delta
	velocity = Vector2.UP.rotated(rotation) * throttle * move_speed
	move_and_slide()

	if carrying != null:
		carrying.global_position = global_position \
			+ (drop_distance * Vector2.UP).rotated(rotation)


# Tip of the white arm, in world space.
func grab_point() -> Vector2:
	return global_position + (arm_length * Vector2.UP).rotated(rotation)


func _pick_up_nearest() -> void:
	if items_root == null:
		return
	var tip := grab_point()
	var best: BridgeItem = null
	var best_d := grab_radius * grab_radius
	for child in items_root.get_children():
		var it := child as BridgeItem
		if it == null or it.held_by != null:
			continue
		var d := tip.distance_squared_to(it.global_position)
		if d < best_d:
			best_d = d
			best = it
	if best != null:
		carrying = best
		best.held_by = self


func release() -> void:
	if carrying == null:
		return
	carrying.held_by = null
	carrying = null
