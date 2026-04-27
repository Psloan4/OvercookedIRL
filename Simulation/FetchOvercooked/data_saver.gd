class_name DataSaver
extends Node

@onready var map: Node2D = get_parent()

var game_state: Dictionary = {}


func _ready():
	set_physics_process(false)

func _physics_process(_delta):
	compile_game_state()

func compile_game_state():
	var ngs = {}
	ngs["player1"] = map.get_node("Player1").get_player_state()
	ngs["player2"] = map.get_node("Player2").get_player_state()
	
	for station_name in ["Station1", "Station2a", "Station2b", "Station3"]:
		ngs[station_name] = map.get_node(station_name).get_station_state()
		
	var item_data = {}
	for item in map.get_node("Items").get_children():
		item_data[item.name] = item.get_item_state()
	ngs["items"] = item_data
	
	ngs["time"] = map.get_node("Timer").time_left
