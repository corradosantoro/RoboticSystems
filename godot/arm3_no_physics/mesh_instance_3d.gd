extends MeshInstance3D

var target_sequence : int = 0

func _input(event: InputEvent) -> void:
	if event.is_action("ui_left"):
		position.x -= 0.01
	elif event.is_action("ui_right"):
		position.x += 0.01
	elif event.is_action("ui_up"):
		position.y += 0.01
	elif event.is_action("ui_down"):
		position.y -= 0.01
	elif event.is_action("ui_page_down"):
		rotation_degrees.z -= 5
	elif event.is_action("ui_page_up"):
		rotation_degrees.z += 5
	elif event.is_action("ui_accept"):
		var target_x = position.x
		var target_y = position.y - 0.21
		print(target_x, " ", target_y)
		DDS.publish("target_x", DDS.DDS_TYPE_FLOAT, target_x)
		DDS.publish("target_y", DDS.DDS_TYPE_FLOAT, target_y)
		DDS.publish("target_alpha", DDS.DDS_TYPE_FLOAT, rotation_degrees.z + 90)
		DDS.publish("sequence", DDS.DDS_TYPE_INT, target_sequence)
		target_sequence += 1
