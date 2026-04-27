extends Node2D

const ITEM = preload("res://item.tscn")

func _ready():
	spawn_item.call_deferred()

func _on_area_2d_body_exited(_body):
	spawn_item()

func spawn_item():
	var item = ITEM.instantiate()
	item.global_position = global_position
	get_parent().get_node("Items").add_child.call_deferred(item)
