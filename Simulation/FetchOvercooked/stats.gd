class_name Stats
extends Control

@onready var score_board = $ScoreBoard
@onready var timer = $Timer
@onready var time_label = $TimeLabel
@onready var start_button = $StartButton
@onready var end_panel = $EndPanel
@onready var final_score_label = $EndPanel/FinalScoreLabel

var score = 0
var game_over = false

func _ready():
	start_button.pressed.connect(start_game)
	timer.timeout.connect(end_game)

func _physics_process(_delta):
	if timer.is_stopped():
		return

	var total_seconds := int(ceil(timer.time_left))
	var minutes := total_seconds / 60.0
	var seconds := total_seconds % 60

	time_label.text = "%02d:%02d" % [minutes, seconds]

func inc_score():
	if game_over: return
	score += 10
	score_board.text = "Score: " + str(score)

func start_game():
	start_button.visible = false
	timer.start(180)

func end_game():
	game_over = true
	end_panel.visible = true
	final_score_label.text = "Final score: " + str(score)
