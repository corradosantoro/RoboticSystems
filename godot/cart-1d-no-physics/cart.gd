extends Control

@onready var dds: Node2D = $DDS
@onready var cart_sprite: Sprite2D = $CartSprite
@onready var telemetry: Label = $telemetry

var t : float = 0
var x0 : float

func _ready() -> void:
	dds.subscribe("position")
	x0 = cart_sprite.position.x

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	t += delta
	telemetry.text = "t = %.3f seconds\np = %.3f meters" % \
		[t, (cart_sprite.position.x - x0) / 10.0]
	dds.publish("tick", dds.DDS_TYPE_FLOAT, delta)
	var p = dds.read("position")
	if p != null:
		#print(v)
		p = p * 10
		cart_sprite.position.x = p + x0
