class_name BridgePlayer
extends CharacterBody2D

## Tank-drive body, carried over from player.gd. Under the bridge the only
## interaction is pick up / put down: the engine infers everything else from
## where the item ends up, so there is no station or drop-off special-casing.
##
## The table is solid, so a body walks around it and reaches across the edge --
## drop_distance is what makes an item land on the far side of that edge.

const RADIUS := 16.0

@export var player_name: String = "p1"
@export var move_speed := 210.0
@export var turn_speed := 3.5
@export var drop_distance := 44.0
@export var reach := 64.0

var active := false
var carrying: BridgeItem = null
var items_root: Node2D = null


func setup(input_prefix: String, at: Vector2, tint: Color, items: Node2D) -> void:
	player_name = input_prefix
	position = at
	items_root = items
	name = "Player_" + input_prefix

	var shape := CollisionShape2D.new()
	var circle := CircleShape2D.new()
	circle.radius = RADIUS
	shape.shape = circle
	add_child(shape)

	var body := ColorRect.new()
	body.size = Vector2(RADIUS * 2.0, RADIUS * 2.0)
	body.position = Vector2(-RADIUS, -RADIUS)
	body.color = tint
	body.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(body)

	# Nose, so the facing direction is visible.
	var nose := ColorRect.new()
	nose.size = Vector2(5, 14)
	nose.position = Vector2(-2.5, -RADIUS - 12.0)
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


func _pick_up_nearest() -> void:
	if items_root == null:
		return
	var best: BridgeItem = null
	var best_d := reach * reach
	for child in items_root.get_children():
		var it := child as BridgeItem
		if it == null or it.held_by != null:
			continue
		var d := global_position.distance_squared_to(it.global_position)
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
