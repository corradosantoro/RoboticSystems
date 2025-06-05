extends StaticBody3D

@onready var joint_base: Node3D = $Joint_base
@onready var joint_1: Node3D = $Joint_base/Joint_1
@onready var joint_3: Node3D = $Joint_base/Joint_1/arm_1/Joint_2/joint_3
@onready var joint_5: Node3D = $Joint_base/Joint_1/arm_1/Joint_2/joint_3/arm_2/joint_4/joint_5
@onready var theta_1_box: TextEdit = $"../../theta1box"
@onready var theta_2_box: TextEdit = $"../../theta2box"
@onready var theta_3_box: TextEdit = $"../../theta3box"
@onready var current_pose: Label = $"../../CurrentPose"
@onready var camera_view: TextureRect = $"../../CameraView"

@onready var tick_count = 0

func _ready() -> void:
	DDS.subscribe("theta0")
	DDS.subscribe("theta1")
	DDS.subscribe("theta2")
	DDS.subscribe("theta3")
	DDS.subscribe("x")
	DDS.subscribe("y")
	DDS.subscribe("a")
	DDS.subscribe("read_image")

	

func _process(delta: float) -> void:
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, delta)
	#print(delta)
	var read_image = DDS.read("read_image")
	if read_image == 1:
		DDS.subscribed_vars['read_image'] = 0
		var image = camera_view.texture.get_image()
		print(image.get_width(), ",", image.get_height(), ",", image.get_format())
		var texture : PackedByteArray = image.get_data()
		print(texture.size())
		CameraSender.send_data(texture)
		
	var t0 = DDS.read("theta0")
	var t1 = DDS.read("theta1")
	var t2 = DDS.read("theta2")
	var t3 = DDS.read("theta3")
	var x = DDS.read("x")
	var y = DDS.read("y")
	var a = DDS.read("a")
	if t0 != null:
		joint_base.rotation.y = t0
	if t1 != null:
		joint_1.rotation.y = PI/2 + t1
		theta_1_box.text = "%.2f" % (rad_to_deg(t1))
	if t2 != null:
		joint_3.rotation.x = t2
		theta_2_box.text = "%.2f" % (rad_to_deg(t2))
	if t3 != null:
		joint_5.rotation.z = PI/2
		joint_5.rotation.x = -PI/2 - t3
		theta_3_box.text = "%.2f" % (rad_to_deg(t3))
	if x != null and y != null and a != null:
		current_pose.text = "X=%.3f   Y=%.3f   A=%.2f" % [x,y,rad_to_deg(a)]
