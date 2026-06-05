class_name Table
extends StaticBody2D

@onready var player_scan_2a: Area2D = $"Station_Scan_2a"
@onready var player_scan_2b: Area2D = $"Station_Scan_2b"


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	if _player_in_area_2a():
		$Station2a.start_process()
	else:
		$Station2a.cancel_process()
	if _player_in_area_2b():
		$Station2b.start_process()
	else:
		$Station2b.cancel_process()
	
func _player_in_area_2a() -> bool:
	var bodies := player_scan_2a.get_overlapping_bodies()
	for b in bodies:
		if b is Player:
			return true
	return false
	
func _player_in_area_2b() -> bool:
	var bodies := player_scan_2b.get_overlapping_bodies()
	for b in bodies:
		if b is Player:
			return true
	return false
