extends RigidBody3D

var force = 0
var torque = 0

func set_f_t(f, t):
	force = f
	torque = t

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass

func _integrate_forces(state):
	var pos = Vector3(0,-0.15,0)
	state.apply_force(Vector3(force,0,0), pos)
	state.apply_torque(Vector3(0,torque,0))
	#var f1 = state.transform.basis * Vector3(force,0,0)
	#var f2 = state.transform.basis * Vector3(-force,0,0)
	#if Input.is_action_pressed("ui_up"):
		#state.apply_force(f1, pos)
	#elif Input.is_action_pressed("ui_down"):
		#state.apply_force(f2, pos)
	#elif Input.is_action_pressed("ui_left"):
		#state.apply_torque(Vector3(0,torque,0))
	#elif Input.is_action_pressed("ui_right"):
		#state.apply_torque(Vector3(0,-torque,0))
