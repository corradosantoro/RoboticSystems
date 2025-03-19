extends RigidBody2D

@onready var label: Label = $"../Control/Label"
@onready var dds: Node2D = $"../DDS"

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	dds.subscribe("force")

@export var force = 100
var t : float = 0
var p : float = 0

func _integrate_forces(state):
	if Input.is_action_pressed("ui_right"):
		state.apply_central_impulse(Vector2(force,0))
	if Input.is_action_pressed("ui_left"):
		state.apply_central_impulse(Vector2(-force,0))
	var v = dds.read("force")
	if v != null:
		#print(v)
		state.apply_force(Vector2(v,0))

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	label.text = "t = %.3f seconds\nv = %.3f pix/s\np = %.3f pix" % \
		[t, self.linear_velocity.x, self.global_position.x]
	dds.publish("speed", dds.DDS_TYPE_FLOAT, self.linear_velocity.x)
	dds.publish("position", dds.DDS_TYPE_FLOAT, self.global_position.x)
	t += delta
