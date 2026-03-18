extends Node2D

const COMMAND_KEEP_ALIVE = 0x80
const COMMAND_SUBSCRIBE = 0x81
const COMMAND_PUBLISH = 0x82

const TIME_TO_LIVE = 2   # 2 seconds

var server_port = 4444
var udp_server : UDPServer

var udp_peers = []


var subscribers : Dictionary
var variables : Dictionary
var subscribed_vars : Dictionary

const DDS_TYPE_UNKNOWN = 0
const DDS_TYPE_INT = 1
const DDS_TYPE_FLOAT = 2

class DDSVariable:
	var name : String
	var type : int
	var value
	var peers
	var packet : PackedByteArray
	var value_offset : int
	
	func init(n : String):
		self.peers = []
		self.name = n
		self.type = DDS_TYPE_UNKNOWN
		self.value = 0
		self.value_offset = 0
	
	func add_peer(p : PacketPeerUDP):
		self.peers.append(p)
		
	func set_value(t : int, value):
		if t != self.type:
			self.type = t
			self.prepare_packet()
		match self.type:
			DDS_TYPE_UNKNOWN:
				pass
			DDS_TYPE_INT:
				self.packet.encode_s32(self.value_offset, int(value))
			DDS_TYPE_FLOAT:
				self.packet.encode_float(self.value_offset, float(value))
					
	func prepare_packet():
		self.packet = PackedByteArray()
		self.packet.resize(len(self.name) + 3 + 4)
		self.packet.encode_u8(0, COMMAND_PUBLISH)
		self.packet.encode_u8(1, self.type)
		self.packet.encode_u8(2, len(self.name))
		var i = 3
		for c in self.name.to_ascii_buffer():
			self.packet.encode_u8(i, c)
			i += 1
		self.value_offset = i
		
	func publish():
		for p : PacketPeerUDP in self.peers:
			#print(self.name, self.packet)
			p.put_packet(self.packet)


class SubscribedVarCollection:
	var var_list : Dictionary
	var ttl : float
	var peer : PacketPeerUDP
	
	func set_var_list(v : Dictionary):
		self.var_list = v
		
	func init(peer : PacketPeerUDP):
		self.var_list = {}
		self.ttl = 0
		self.peer = peer
		
	func process(delta : float):
		self.ttl += delta
		if self.ttl > TIME_TO_LIVE:
			return true
		else:
			return false
			
	func keep_alive():
		self.ttl = 0

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	udp_server = UDPServer.new()
	udp_server.listen(server_port)
	subscribers = {}
	variables = {}

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	for k in subscribers.keys():
		if subscribers[k].process(delta):
			subscribers.erase(k)
	udp_server.poll()
	if udp_server.is_connection_available():
		var peer : PacketPeerUDP = udp_server.take_connection()
		udp_peers.append(peer)
		
	for peer in udp_peers:
		if peer.get_available_packet_count() <= 0:
			continue
		var var_collection : SubscribedVarCollection
		if subscribers.get(peer) == null:
			var_collection = SubscribedVarCollection.new()
			subscribers[peer] = var_collection
		else:
			var_collection = subscribers[peer]
		var packet = peer.get_packet()
		var command = packet.decode_u8(0)
		#print(packet)
		match command:
			COMMAND_KEEP_ALIVE:
				var_collection.keep_alive()
			COMMAND_SUBSCRIBE:
				subscribe_from_remote(peer, packet)
			COMMAND_PUBLISH:
				publish_from_remote(peer, packet)


func subscribe_from_remote(peer : PacketPeerUDP, packet : PackedByteArray):
	# subscribe packet format
	# U8 : numbers of vars
	# for each var : u8=string lenght + sequence of chars
	var number_of_vars = packet.decode_u8(1)
	var index = 2
	var peer_var = {}
	for i in range(number_of_vars):
		var len = packet.decode_u8(index)
		var subpacket = packet.slice(index + 1, index + len + 1)
		#print(subpacket)
		var name : String = subpacket.get_string_from_utf8()
		var variable
		if variables.get(name) == null:
			variable = DDSVariable.new()
			variable.init(name)
			variables[name] = variable
		else:
			variable = variables[name]
		variable.add_peer(peer)
		peer_var[name] = variable
		index += (len + 1)
	subscribers[peer].set_var_list(peer_var)
	#print(variables)

func publish_from_remote(peer : PacketPeerUDP, packet : PackedByteArray):
	# publish packet format
	# u8=type
	# u8=string lenght + sequence of chars
	# 4 bytes var value
	var typ = packet.decode_u8(1)
	var len = packet.decode_u8(2)
	var subpacket = packet.slice(3, len + 3)
	var name : String = subpacket.get_string_from_utf8()
	var value = 0.0
	match typ:
		DDS_TYPE_UNKNOWN:
			pass
		DDS_TYPE_INT:
			value = packet.decode_s32(len + 3)
		DDS_TYPE_FLOAT:
			value = packet.decode_float(len + 3)
	subscribed_vars[name] = value

	var number_of_vars = packet.decode_u8(1)
	var index = 2
	var peer_var = {}
	for i in range(number_of_vars):
		var variable
		if variables.get(name) == null:
			variable = DDSVariable.new()
			variable.init(name)
			variables[name] = variable
		else:
			variable = variables[name]
		variable.add_peer(peer)
		peer_var[name] = variable
		index += (len + 1)
	subscribers[peer].set_var_list(peer_var)
	#print(variables)


func publish(name : String, type : int, value):
	#print(name, variables)
	if variables.get(name) == null:
		#print("not found")
		return
	var item = variables[name]
	#print(item)
	item.set_value(type, value)
	item.publish()

func subscribe(name : String):
	subscribed_vars[name] = null
	
func read(name : String):
	return subscribed_vars[name]
