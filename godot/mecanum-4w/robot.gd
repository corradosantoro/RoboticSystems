extends StaticBody3D

@onready var pose: Label = $"../Pose"
@onready var wheels: Label = $"../Wheels"

@onready var w_1: MeshInstance3D = $W1
@onready var w_2: MeshInstance3D = $W2
@onready var w_3: MeshInstance3D = $W3
@onready var w_4: MeshInstance3D = $W4

func _ready() -> void:
	DDS.subscribe("X")
	DDS.subscribe("Y")
	DDS.subscribe("Theta")
	DDS.subscribe("w1")
	DDS.subscribe("w2")
	DDS.subscribe("w3")
	DDS.subscribe("w4")

func _process(delta: float) -> void:
	#print(theRobot.global_position.x, " ", -theRobot.global_position.z, " ", theRobot.global_rotation.y)
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, delta)

	var x = DDS.read("X")
	var y = DDS.read("Y")
	var theta = DDS.read("Theta")
	var w1 = DDS.read("w1")
	var w2 = DDS.read("w2")
	var w3 = DDS.read("w3")
	var w4 = DDS.read("w4")

	if (x != null)and(y != null)and(theta != null):
		self.global_position.x = -y
		self.global_position.z = -x
		self.global_rotation.y = theta
		pose.text = "X: %.3f, Y: %.3f, Theta: %.0f" % \
			[x, y, rad_to_deg(theta)]
		if (w1 != null)and(w2 != null)and(w3 != null)and(w4 != null):
			w_1.rotation.x += w1 * delta
			w_2.rotation.x += w2 * delta
			w_3.rotation.x += -w3 * delta
			w_4.rotation.x += -w4 * delta
			#wheels.text = "%.3f %.3f %.3f %.3f" % \
			#	[w1, w2, w3, w4]
		
