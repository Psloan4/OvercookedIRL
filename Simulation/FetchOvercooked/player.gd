class_name Player
extends CharacterBody2D

var item: Item = null
var working_station: Station = null

@onready var item_scan: Area2D = $ItemScan
@onready var station_scan: Area2D = $StationScan
@onready var drop_off_scan: Area2D = $DropOffScan

@export var stats: Stats
@export var player_name: String
@export var move_speed := 210.0
@export var turn_speed := 3.5   # radians per second
@export var drop_distance := 55.0


func get_player_state() -> Dictionary:
	var ps = {
		"pos_x": position.x,
		"pos_y": position.y,
		"vel_x": velocity.x,
		"vel_y": velocity.y,
		"rot": rotation,
		"item": null,
		"station": null,
	}
	if item != null:
		ps["item"] = item.name
	if working_station != null:
		ps["station"] = working_station.name
	return ps

func _physics_process(delta):
	var closest_item := _get_closest_item()
	var closest_station := _get_closest_station()
	var closest_drop_off := _get_closest_drop_off()

	# --- Update item labels (closest visible, others hidden) ---
	_update_item_labels(closest_item)

	# --- Single-button grab: pick up, drop off, place into station, or drop in front ---
	if Input.is_action_just_pressed(player_name + "_grab"):
		if item == null:
			# Prefer taking from station first if it has an item
			if closest_station != null and closest_station.item != null:
				var taken := closest_station.take_item()
				if taken != null:
					_pick_up(taken)
			elif closest_item != null:
				_pick_up(closest_item)

		else:
			# HIGHEST PRIORITY: drop off (deletes item)
			if closest_drop_off != null:
				_drop_off()

			# Next: place into station if empty
			elif closest_station != null and closest_station.item == null:
				_place_into_station(closest_station)

			# Otherwise: drop in front
			else:
				_drop_in_front()

	# --- Hold interact to process manual stations (automatic == false) ---
	var interact_held := Input.is_action_pressed(player_name + "_interact")

	if closest_station != null and closest_station.automatic == false:
		if interact_held:
			working_station = closest_station
			working_station.start_process()   # uses the existing Timer-based process
		else:
			# Released early -> cut short
			if working_station == closest_station:
				working_station.cancel_process()
				working_station = null
	else:
		# Walked away / no manual station nearby -> cut short any ongoing manual process
		if working_station != null:
			working_station.cancel_process()
			working_station = null

	# --- Movement ---
	var throttle := Input.get_action_strength(player_name + "_up") - Input.get_action_strength(player_name + "_down")
	var turn := Input.get_action_strength(player_name + "_right") - Input.get_action_strength(player_name + "_left")

	rotation += turn * turn_speed * delta
	velocity = Vector2.UP.rotated(rotation) * throttle * move_speed
	move_and_slide()

	# --- Carry item ---
	if item:
		item.global_position = global_position
		item.body.rotation = rotation


# ---------------- Actions ----------------

func _pick_up(it: Item) -> void:
	item = it
	item.set_label_visible(false)

func _place_into_station(station: Station) -> void:
	station.place_item(item)
	item = null

func _drop_in_front() -> void:
	item.global_position = global_position + (drop_distance * Vector2.UP).rotated(rotation)
	item.set_label_visible(true)
	item = null

func _drop_off() -> void:
	if item.state < 4: return
	stats.inc_score()
	# Delete item from the scene tree
	item.queue_free()
	item = null


# ---------------- Queries + UI ----------------

func _get_closest_item() -> Item:
	if item != null:
		return null  # don't highlight items while holding one

	var bodies := item_scan.get_overlapping_bodies()
	var closest: Item = null
	var closest_d2 := INF

	for b in bodies:
		if not (b is Item):
			continue
		var it := b as Item
		var d2 := global_position.distance_squared_to(it.global_position)
		if d2 < closest_d2:
			closest_d2 = d2
			closest = it

	return closest

func _update_item_labels(closest: Item) -> void:
	var show_item := (item == null)
	for b in item_scan.get_overlapping_bodies():
		if b is Item:
			var it := b as Item
			it.set_label_visible(show_item and it == closest)

func _get_closest_station() -> Station:
	var bodies := station_scan.get_overlapping_bodies()
	var closest: Station = null
	var closest_d2 := INF

	for b in bodies:
		if not (b is Station):
			continue
		var st := b as Station
		var d2 := global_position.distance_squared_to(st.global_position)
		if d2 < closest_d2:
			closest_d2 = d2
			closest = st

	return closest

func _get_closest_drop_off() -> Node:
	# If you have class_name DropOff, change return type to DropOff and the `is` check below.
	var bodies := drop_off_scan.get_overlapping_bodies()
	var closest: Node = null
	var closest_d2 := INF

	for b in bodies:
		if not (b is Node2D):
			continue
		# Optional filtering:
		# if not (b is DropOff): continue
		var d2 := global_position.distance_squared_to(b.global_position)
		if d2 < closest_d2:
			closest_d2 = d2
			closest = b

	return closest
