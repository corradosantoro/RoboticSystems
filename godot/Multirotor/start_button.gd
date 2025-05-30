extends Button

@onready var robot: Node3D = $"/root/World/Robot"

func _ready() -> void:
	pressed.connect(on_start)
	
func on_start():
	pass
