extends Node3D

@export var udpPort: int = 4444

@onready var waist:Arm = $Waist
@onready var arm1:Arm = $Arm1
@onready var arm2:Arm = $Arm2
@onready var wrist:Arm = $Wrist

var server: UDPServer

var torque_waist = 0
var torque_arm_1 = 0
var torque_arm_2 = 0
var torque_wrist = 0
var theta1: float = 0
var theta2: float = 0

func _ready():
	#inizializzazione server
	server = UDPServer.new()
	server.listen(udpPort)

	#abilitazione motori
	$motorBase["motor/enable"] = true
	$motorArm1["motor/enable"] = true
	
	#settaggio motori
	$motorBase.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY,0)
	$motorArm1.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY,0)
	pass
	
func _physics_process(delta):
	server.poll()
	
	if server.is_connection_available():
		var peer: PacketPeerUDP = server.take_connection()
		var packet = peer.get_packet()
		torque_arm_1 = packet.decode_float(0)
		
		arm1.setTorque(torque_arm_1)
		
		var tosend =  PackedFloat32Array()
		tosend.append(delta)
		
		tosend.append(arm1.global_rotation.x)
		tosend.append(arm1.angular_velocity.x)
		
		peer.put_var(tosend)
