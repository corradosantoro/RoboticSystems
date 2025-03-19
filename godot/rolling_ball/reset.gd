extends Button

@onready var ball: RigidBody2D = $"../../Ball"

var initial_position

func _ready() -> void:
	self.pressed.connect(on_pressed)
	initial_position = ball.global_position
	
func on_pressed():
	ball.apply_force(Vector2(0,0))
	PhysicsServer2D.body_set_state(
		ball,
		PhysicsServer2D.BODY_STATE_TRANSFORM,
		Transform2D.IDENTITY.translated(initial_position))
	
