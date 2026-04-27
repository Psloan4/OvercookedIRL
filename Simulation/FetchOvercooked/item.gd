class_name Item
extends CharacterBody2D

var state: int = 1
var type: String = ["a", "b"].pick_random()

@onready var body = $Body
@onready var pick_up_label = $PickUpLabel
@onready var state_label = $Body/StateLabel

func get_item_state() -> Dictionary:
	var item_state = {
		"pos_x": position.x,
		"pos_y": position.y,
		"state": str(state),
	}
	if state == 2:
		item_state["state"] += type
	return item_state

func set_label_visible(vis: bool):
	pick_up_label.visible = vis

func inc_state():
	state += 1
	state_label.text = str(state)
	if state == 2: state_label.text += type
