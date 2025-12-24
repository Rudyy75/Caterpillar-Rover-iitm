from robot_interfaces.msg import Counter
from blitz import Blitz

blitz_interfaces = {str : Blitz}

# add an interface in this form
blitz_interfaces = {

    "counter": Blitz(

        # the topic whose data you want to send to the mcu
        # this is the topic you will PUBLISH to
        topic="/counter",

        # msg id should match the id in the microcontroller
        msg_id=1,

        # format of the msg packet 
        # uint8 : B, int8 b, uint16 H, int16 h, uint32 I, int32 i, float32 f
        struct_fmt="hhff",

        # fields just as in the ros interface 
        fields=["a","b","c","d"],

        # ros interface name, import it first
        ros_msg=Counter,

        # false id you are writing to the microcontroller
        from_mcu=False
    ),

    "counter_response": Blitz(

        # to receive from the microcontroller

        # the data coming from the mcu will be published in this topic
        # this is the topic you will SUBSCRIBE drom
        topic="/counter_response",

        # msg_id should match the id in the microcontroller
        msg_id=2,

        # same as above
        struct_fmt="hhff",
        fields=["a","b","c","d"],
        ros_msg=Counter,

        # set this as true rest same
        from_mcu=True
    ),
}
