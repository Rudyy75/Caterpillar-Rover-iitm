import struct

class Blitz:
    def __init__(self, topic, msg_id, struct_fmt, fields, ros_msg, from_mcu):
        self.topic = topic
        self.id = msg_id
        self.struct = struct_fmt
        self.fields = fields
        self.ros_msg = ros_msg
        self.from_mcu = from_mcu
        self.payload_data = None
        self.pub = None

    def pack(self, msg):
        values = []

        for field in self.fields:
            obj = getattr(msg, field)
            values.append(obj)
        fmt = "<BB" + self.struct
        return struct.pack(fmt, 0xAA, self.id, *values)

    def unpack(self, data):
        fmt = "="+self.struct
        unpacked = struct.unpack(fmt, data)
        msg = self.ros_msg()
        for field, value in zip(self.fields, unpacked):
            setattr(msg, field, value)
        return msg
    