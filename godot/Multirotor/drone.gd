extends RigidBody3D

@onready var L = 0.195
@onready var p1 = Vector3(L,0,L)
@onready var p2 = Vector3(-L,0,L)
@onready var p3 = Vector3(-L,0,-L)
@onready var p4 = Vector3(L,0,-L)
var f1 = Vector3(0,0,0)
var f2 = Vector3(0,0,0)
var f3 = Vector3(0,0,0)
var f4 = Vector3(0,0,0)

# Called when the node enters the scene tree for the first time.
func _ready():
	#self.set_forces(2.5,2.5,2.5,2.5)
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	#var f = 2.5
	#var f1 = Vector3(0,f + 0.5,0)
	#var f2 = Vector3(0,f + 0.5,0)
	#var f3 = Vector3(0,f - 0.5,0)
	#var f4 = Vector3(0,f - 0.5,0)
	self.apply_local_force(f1, p1)
	self.apply_local_force(f2, p2)
	self.apply_local_force(f3, p3)
	self.apply_local_force(f4, p4)

func apply_local_force(force: Vector3, pos: Vector3):
	var pos_local = self.transform.basis * pos
	var force_local = self.transform.basis * force
	self.apply_force(force_local, pos_local)

func set_forces(_f1,_f2,_f3,_f4):
	f1 = Vector3(0,_f1,0)
	f2 = Vector3(0,_f2,0)
	f3 = Vector3(0,_f3,0)
	f4 = Vector3(0,_f4,0)

func get_pose():
	return [global_position, global_rotation]

func get_velocity():
	return [linear_velocity, angular_velocity]
