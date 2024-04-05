extends RigidBody2D
@onready var slider_label = $"../InputForce"
@onready var reset_button = $"../ResetButton"
@export var udpPort: int = 4444

var server: UDPServer
var initial_position

# Called when the node enters the scene tree for the first time.
func _ready():
	reset_button.pressed.connect(self._reset_pressed)
	initial_position = self.global_position
	initial_position.y -= 10
	#inizializzazione server
	server = UDPServer.new()
	server.listen(udpPort)

func _reset_pressed():
	#print("pressed")
	PhysicsServer2D.body_set_state(
		self,
		PhysicsServer2D.BODY_STATE_TRANSFORM,
		Transform2D.IDENTITY.translated(initial_position))

func _physics_process(delta):
	print("P = ", self.global_position.x, ", V = ", self.linear_velocity.x)
	server.poll()
	
	if server.is_connection_available():
		var peer: PacketPeerUDP = server.take_connection()
		var packet = peer.get_packet()
		var new_force = packet.decode_float(0)

		self.apply_central_force(Vector2(new_force,0))

		var tosend =  PackedFloat32Array()
		tosend.append(delta)
		tosend.append(self.global_position.x)
		tosend.append(self.linear_velocity.x)
		peer.put_var(tosend)
		
		

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass


var force = 3000

func _integrate_forces(state):
	if Input.is_action_pressed("ui_right"):
		state.apply_central_force(Vector2(force,0))
	elif Input.is_action_pressed("ui_left"):
		state.apply_central_force(Vector2(-force,0))


func _on_v_slider_value_changed(value):
	slider_label.text = str(value) + " Kg px/s^2"
	force = value
