extends RigidBody3D
class_name Arm
@export_enum("X", "Y", "Z") var rotation_axis: String = "X"

var vectorTorque = Vector3.ZERO
var nextTorque = Vector3.ZERO

# Called when the node enters the scene tree for the first time.
func _ready():
	_update_torque_vector()

func setTorque(value):
	_update_torque_vector()
	nextTorque = vectorTorque * value


func _integrate_forces(state):
	state.apply_torque(nextTorque)
	
func _update_torque_vector():
	match rotation_axis:
		"X" :
			vectorTorque = transform.basis.x
		"Y":
			vectorTorque = transform.basis.y
		"Z":
			vectorTorque = transform.basis.z
