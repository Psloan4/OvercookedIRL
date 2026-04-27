class_name Station
extends Node2D

var item: Item

@onready var timer = $Timer
@onready var id_label = $ColorRect/IDLabel
@onready var progress_bar = $ProgressBar

@export var id: int
@export var type: String = ""
@export var automatic: bool = true
@export var time: float = 10.0

func get_station_state() -> Dictionary:
	var state = {
		"pos_x": position.x,
		"pox_y": position.y,
		"type": type + str(id),
		"auto": automatic,
		"time": time,
		"item": null,
	}
	if item != null:
		state["item"] = item.name
	return state

func _ready():
	id_label.text = str(id) + type
	timer.timeout.connect(prog_complete)

func _physics_process(_delta):
	# Show progress whenever timer is running (automatic OR manual)
	if not timer.is_stopped():
		progress_bar.visible = true
		progress_bar.value = 100.0 * (1.0 - timer.time_left / time)
	else:
		progress_bar.visible = false

func place_item(inp: Item):
	if item: return
	item = inp
	item.global_position = global_position
	item.body.rotation = 0

	# If this station is automatic, start immediately
	if automatic:
		start_process()
	else:
		# manual stations should not keep old progress
		cancel_process()

func take_item() -> Item:
	if item == null:
		return null
	cancel_process()
	var out := item
	item = null
	return out

func start_process():
	# (I also fixed your type check; your original condition was almost certainly wrong)
	if item == null: return
	if item.state != id: return
	if type != "" and item.type != type: return
	if not timer.is_stopped(): return  # already running

	timer.start(time)

func cancel_process():
	# Cut short + reset progress
	if not timer.is_stopped():
		timer.stop()

func prog_complete():
	if item == null: return
	item.inc_state()
