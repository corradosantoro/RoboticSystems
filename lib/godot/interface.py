#
# interface.py
#

import socket
import struct
import time

class GodotInterface:

    def __init__(self, uPort = 4444):
        self.port = uPort
        self.sd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class GodotCart1D(GodotInterface):

    def __init__(self, uPort = 4444):
        super().__init__(uPort)

    def process(self, force):
        packet = struct.pack("<f", force)
        self.sd.sendto(packet, ('localhost', self.port))
        (reply, remote) = self.sd.recvfrom(1024)
        (delta, pos, vel) = struct.unpack("<fff", reply[8:])
        return (delta, pos, vel)


class GodotArm1D(GodotInterface):

    def __init__(self, uPort = 4444):
        super().__init__(uPort)

    def process(self, force):
        packet = struct.pack("<f", force)
        self.sd.sendto(packet, ('localhost', self.port))
        (reply, remote) = self.sd.recvfrom(1024)
        (delta, theta, omega) = struct.unpack("<fff", reply[8:])
        return (delta, theta, omega)


class GodotCartSimple(GodotInterface):

    def __init__(self, uPort = 4444):
        super().__init__(uPort)

    def process(self, force, torque):
        packet = struct.pack("<ff", force, torque)
        self.sd.sendto(packet, ('localhost', self.port))
        (reply, remote) = self.sd.recvfrom(1024)
        (delta, x, y, theta, v, w) = struct.unpack("<ffffff", reply[8:])
        return (delta, x, y, theta, v, w)


class GodotCartTwoWheels(GodotInterface):

    def __init__(self, uPort = 4444):
        super().__init__(uPort)

    def process(self, vl, vr):
        packet = struct.pack("<ff", vl, vr)
        self.sd.sendto(packet, ('localhost', self.port))
        (reply, remote) = self.sd.recvfrom(1024)
        (delta, x, y, theta, v, w) = struct.unpack("<ffffff", reply[8:])
        return (delta, x, y, theta, v, w)


class GodotDrone(GodotInterface):

    def __init__(self, uPort = 4444):
        super().__init__(uPort)

    def process(self, f1, f2, f3, f4):
        packet = struct.pack("<ffff", f1,f2,f3,f4)
        self.sd.sendto(packet, ('localhost', self.port))
        (reply, remote) = self.sd.recvfrom(1024)
        (delta, x, y, z, roll, pitch, yaw, vx, vy, vz, w_roll, w_pitch, w_yaw) = struct.unpack("<fffffffffffff", reply[8:])
        return (delta, x, y, z, roll, pitch, yaw, vx, vy, vz, w_roll, w_pitch, w_yaw)



if __name__ == "__main__":
    # g = GodotArm1D()
    # while True:
    #     print(g.process(0.5))

    #g = GodotCartSimple()
    #g = GodotCartTwoWheels()
    #while True:
    #    print(g.process(6, 10))
    g = GodotDrone()
    while True:
        print(g.process(2.5, 2.5, 2.5, 2.5))

