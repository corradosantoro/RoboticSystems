extends StaticBody3D
@onready var joint_1: Node3D = $Joint_base/Joint_1
@onready var joint_3: Node3D = $Joint_base/Joint_1/arm_1/Joint_2/joint_3
@onready var joint_5: Node3D = $Joint_base/Joint_1/arm_1/Joint_2/joint_3/arm_2/joint_4/joint_5

func _ready() -> void:
	DDS.subscribe("theta1")
	DDS.subscribe("theta2")
	DDS.subscribe("theta3")
	
	

func _process(delta: float) -> void:
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, delta)
	#print(delta)
	var t1 = DDS.read("theta1")
	var t2 = DDS.read("theta2")
	var t3 = DDS.read("theta3")
	if t1 != null:
		joint_1.rotation.y = PI/2 + t1
	if t2 != null:
		joint_3.rotation.x = t2
	if t3 != null:
		joint_5.rotation.z = PI/2
		joint_5.rotation.x = -PI/2 - t3
