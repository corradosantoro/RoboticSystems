extends Node3D

@export var udpPort: int = 4444

var vl : float = 0
var vr : float = 0
var server: UDPServer
var theRobot
var left_motor : HingeJoint3D
var right_motor : HingeJoint3D

# Called when the node enters the scene tree for the first time.
func _ready():
	#inizializzazione server
	server = UDPServer.new()
	server.listen(udpPort)
	theRobot = $Body
	left_motor = $LeftMotor
	right_motor = $RightMotor


func _physics_process(delta):
	#print(theRobot.global_position.x, " ", -theRobot.global_position.z, " ", theRobot.global_rotation.y)
	server.poll()
	
	if server.is_connection_available():
		var peer: PacketPeerUDP = server.take_connection()
		var packet = peer.get_packet()
		vl = packet.decode_float(0)
		vr = packet.decode_float(4)
		
		left_motor["motor/enable"] = true
		right_motor["motor/enable"] = true
		left_motor["motor/target_velocity"] = vl
		right_motor["motor/target_velocity"] = vr
		#theRobot.set_f_t(force, torque)

		#left_motor.set_torque(Vector3(torque,torque,0))
		#right_motor.set_torque(Vector3(0,torque,torque))
		
		var tosend =  PackedFloat32Array()
		tosend.append(delta)
		
		tosend.append(theRobot.global_position.x) # x
		tosend.append(-theRobot.global_position.z) # y
		tosend.append(theRobot.global_rotation.y) # theta
		
		tosend.append(theRobot.linear_velocity.x)
		tosend.append(theRobot.angular_velocity.y)
		
		peer.put_var(tosend)
		
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	pass

func _integrate_forces(state):
	pass
