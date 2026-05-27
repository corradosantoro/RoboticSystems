extends Node

@onready var server : TCPServer
@onready var connection_list  = []

func _ready() -> void:
	server = TCPServer.new()
	server.listen(4445)
	
func _process(delta: float) -> void:
	if server.is_connection_available():
		var connection = server.take_connection()
		connection.poll()
		if connection.get_status() == StreamPeerTCP.STATUS_CONNECTED:
			var packet_streamer = PacketPeerStream.new()
			packet_streamer.set_output_buffer_max_size(1024000)
			packet_streamer.set_stream_peer(connection)
			connection_list.push_back(packet_streamer)
		
func send_data(packet : PackedByteArray):
	for c : PacketPeerStream in connection_list:
		c.put_packet(packet)
