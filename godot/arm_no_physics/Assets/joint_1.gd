extends Node3D

func _ready() -> void:
	DDS.subscribe("theta")

func _physics_process(delta: float) -> void:
	var p = DDS.read("theta")
	if p != null:
		rotation.y = p + PI/2
