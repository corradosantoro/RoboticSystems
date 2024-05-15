extends Node3D

@export var udpPort: int = 4444
var drone

var server: UDPServer

# Called when the node enters the scene tree for the first time.
func _ready():
	#inizializzazione server
	drone = get_node("drone")
	server = UDPServer.new()
	server.listen(udpPort)

func _physics_process(delta):
	#print(theRobot.global_position.x, " ", -theRobot.global_position.z, " ", theRobot.global_rotation.y)
	server.poll()
	
	if server.is_connection_available():
		var peer: PacketPeerUDP = server.take_connection()
		var packet = peer.get_packet()
		var f1 = packet.decode_float(0)
		var f2 = packet.decode_float(4)
		var f3 = packet.decode_float(8)
		var f4 = packet.decode_float(12)
		drone.set_forces(f1,f2,f3,f4)
		var pose = drone.get_pose()
		var vel = drone.get_velocity()
		var pos = pose[0]
		var att = pose[1]
		var lin_vel = vel[0]
		var rot_vel = vel[1]
		
		var tosend =  PackedFloat32Array()
		tosend.append(delta)
		#
		tosend.append(pos.z) # x
		tosend.append(pos.x) # y
		tosend.append(pos.y) # z
		
		tosend.append(att.z) # x
		tosend.append(att.x) # y
		tosend.append(att.y) # z
		
		tosend.append(lin_vel.z) # x
		tosend.append(lin_vel.x) # y
		tosend.append(lin_vel.y) # z
		
		tosend.append(rot_vel.z) # x
		tosend.append(rot_vel.x) # y
		tosend.append(rot_vel.y) # z
		#
		peer.put_var(tosend)
		
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	#print(drone.get_velocity())
	pass

func _integrate_forces(state):
	pass

