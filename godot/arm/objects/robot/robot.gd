extends Node3D

@onready var waist:Arm = $Waist
@onready var arm1:Arm = $Arm1
@onready var arm2:Arm = $Arm2
@onready var wrist:Arm = $Wrist

var torque_waist = 0
var torque_arm_1 = 0
var torque_arm_2 = 0
var torque_wrist = 0
var theta1: float = 0
var theta2: float = 0

func _ready():
	#abilitazione motori
	$motorBase["motor/enable"] = true
	$motorArm1["motor/enable"] = false
	#settaggio motori
	$motorBase.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY,0)
	$motorArm1.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY,0)
	DDS.subscribe("torque")
	
func _physics_process(delta):
	var t = DDS.read("torque")
	if t != null:
		arm1.setTorque(t)
	DDS.publish("speed", DDS.DDS_TYPE_FLOAT, arm1.angular_velocity.x)
	DDS.publish("angle", DDS.DDS_TYPE_FLOAT, arm1.global_rotation.x + PI/2)
