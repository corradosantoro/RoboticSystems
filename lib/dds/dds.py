#
# dds.py
#

import socket
import threading
import select
import io
import time
import struct

class MonitoredVariable:

    def __init__(self):
        self.mutex = threading.Lock()
        self.condition = threading.Condition(self.mutex)
        self.value = None

    def get_value(self):
        self.mutex.acquire()
        v = self.value
        self.mutex.release()
        return v

    def wait_value(self):
        self.mutex.acquire()
        self.condition.wait()
        v = self.value
        self.mutex.release()
        return v

    def enter(self):
        self.mutex.acquire()

    def wait(self):
        self.condition.wait()

    def notify(self, val):
        self.mutex.acquire()
        self.value = val
        self.condition.notify_all()
        self.mutex.release()

    def exit(self):
        self.mutex.release()


class DDS(threading.Thread):

    DDS_TYPE_UNKNOWN = 0
    DDS_TYPE_INT = 1
    DDS_TYPE_FLOAT = 2
    DDS_TYPE_BLOB = 3

    COMMAND_KEEP_ALIVE = 0x80
    COMMAND_SUBSCRIBE = 0x81
    COMMAND_PUBLISH = 0x82


    def __init__(self, uPort = None):
        super(DDS, self).__init__()
        self.setDaemon(True)
        self.variables = {}
        self.sd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if uPort is not None:
            self.sd.bind( ('', uPort) )


    def start(self, _remote_host = '127.0.0.1', _remote_port = 4444):
        self.remote_host = _remote_host
        self.remote_port = _remote_port
        super(DDS, self).start()

    def stop(self):
        self.__running = False


    def subscribe(self, _varlist):
        data = io.BytesIO()
        data.write(bytes([DDS.COMMAND_SUBSCRIBE, len(_varlist)]))
        for _varname in _varlist:
            self.variables[_varname] = MonitoredVariable()
            data.write(bytes([len(_varname)]))
            data.write(_varname.encode("utf-8"))
        self.sd.sendto(data.getvalue(), (self.remote_host, self.remote_port))


    def publish(self, _name, _value, _type):
        data = io.BytesIO()
        data.write(bytes([DDS.COMMAND_PUBLISH, _type, len(_name)]))
        data.write(_name.encode("utf-8"))
        match _type:
            case DDS.DDS_TYPE_FLOAT:
                data.write(struct.pack("<f", _value))
            case DDS.DDS_TYPE_INT:
                data.write(struct.pack("<i", _value))
        self.sd.sendto(data.getvalue(), (self.remote_host, self.remote_port))


    def read(self, _varname):
        if _varname in self.variables:
            return self.variables[_varname].get_value()
        else:
            return None


    def wait(self, _varname):
        if _varname in self.variables:
            return self.variables[_varname].wait_value()
        else:
            return None


    def run(self):
        self.__running = True
        while self.__running:
            sel = select.select([self.sd],[],[], 0.5)

            if sel == []:
                continue

            p = self.sd.recvfrom(1024)

            if p == None:
                break
            (data, address) = p
            if data[0] == DDS.COMMAND_KEEP_ALIVE:
                continue
            if data[0] == DDS.COMMAND_PUBLISH:
                self.__on_remote_publish(data)

        self.sd.close()


    def __on_remote_publish(self, data):
        typ = data[1]
        l = data[2]
        name = ""
        for i in range(l):
            c = data[i+3]
            name = name + chr(c)

        value = None
        match typ:
            case DDS.DDS_TYPE_FLOAT:
                value = struct.unpack("<f", data[l+3:l+7])
                #print(name, value)

            case DDS.DDS_TYPE_INT:
                value = struct.unpack("<i", data[l+3:l+7])

        if (value is not None)and(name in self.variables):
            m = self.variables[name]
            m.notify(value[0])


if __name__ == "__main__":
    dds = DDS()
    dds.start('127.0.0.1', 4444)
    dds.subscribe(['speed', 'position'])
    dds.publish('force', 5000.0, DDS.DDS_TYPE_FLOAT)
    while True:
        v = dds.wait('speed')
        p = dds.read('position')
        print(p, v)
        dds.publish('force', 0.0, DDS.DDS_TYPE_FLOAT)
