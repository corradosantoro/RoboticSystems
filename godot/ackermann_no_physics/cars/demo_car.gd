extends CharacterBody3D

func _ready() -> void:
	DDS.subscribe("X")
	DDS.subscribe("Y")
	DDS.subscribe("Theta")

func _process(delta: float) -> void:
	#print(theRobot.global_position.x, " ", -theRobot.global_position.z, " ", theRobot.global_rotation.y)
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, delta)

	var x = DDS.read("X")
	var y = DDS.read("Y")
	var theta = DDS.read("Theta")

	if (x != null)and(y != null)and(theta != null):
		self.global_position.x = -y
		self.global_position.z = -x
		self.global_rotation.y = theta
