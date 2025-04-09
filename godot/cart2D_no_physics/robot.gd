extends Node3D

var theRobot
var left_motor : HingeJoint3D
var right_motor : HingeJoint3D

# Called when the node enters the scene tree for the first time.
func _ready():
	#inizializzazione server
	theRobot = $Body
	DDS.subscribe("X")
	DDS.subscribe("Y")
	DDS.subscribe("Theta")


func _physics_process(delta):
	pass


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	#print(theRobot.global_position.x, " ", -theRobot.global_position.z, " ", theRobot.global_rotation.y)
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, _delta)

	var x = DDS.read("X")
	var y = DDS.read("Y")
	var theta = DDS.read("Theta")

	if (x != null)and(y != null)and(theta != null):
		theRobot.global_position.x = x
		theRobot.global_position.z = -y
		theRobot.global_rotation.y = theta


func _integrate_forces(state):
	pass
