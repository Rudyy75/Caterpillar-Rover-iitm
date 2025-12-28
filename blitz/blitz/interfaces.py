from robot_interfaces.msg import Velocity
from robot_interfaces.msg import Odom
from robot_interfaces.msg import EncoderRaw
from robot_interfaces.msg import LimitSwitches
from robot_interfaces.msg import ModeSwitch
from robot_interfaces.msg import BnoReading
from robot_interfaces.msg import ActuatorCmd
from blitz import Blitz

blitz_interfaces = {str : Blitz}

blitz_interfaces = {

    # ============ ROS → MCU (from_mcu = False) ============
    
    "velocity": Blitz(
        topic="/velocity",
        msg_id=1,
        struct_fmt="fff",  # 3 floats: vx, vy, vw
        fields=["vx", "vy", "vw"],
        ros_msg=Velocity,
        from_mcu=False
    ),

    "actuator_cmd": Blitz(
        topic="/actuator_cmd",
        msg_id=6,
        struct_fmt="BB",  # 2 uint8: lead_screw, tub_angle
        fields=["lead_screw", "tub_angle"],
        ros_msg=ActuatorCmd,
        from_mcu=False
    ),

    # ============ MCU → ROS (from_mcu = True) ============
    
    "encoder_raw": Blitz(
        topic="/encoder_raw",
        msg_id=2,
        struct_fmt="iiii",  # 4 int32: fl, fr, bl, br ticks
        fields=["fl_ticks", "fr_ticks", "bl_ticks", "br_ticks"],
        ros_msg=EncoderRaw,
        from_mcu=True
    ),

    "limit_switches": Blitz(
        topic="/rover_limit_sw",
        msg_id=3,
        struct_fmt="???",  # 3 bools: ls1, ls2, ls3
        fields=["ls1", "ls2", "ls3"],
        ros_msg=LimitSwitches,
        from_mcu=True
    ),

    "mode_switch": Blitz(
        topic="/mode_switch",
        msg_id=4,
        struct_fmt="?",  # 1 bool: autonomous
        fields=["autonomous"],
        ros_msg=ModeSwitch,
        from_mcu=True
    ),

    "bno_reading": Blitz(
        topic="/bno",
        msg_id=5,
        struct_fmt="fff",  # 3 floats: yaw, pitch, roll
        fields=["yaw", "pitch", "roll"],
        ros_msg=BnoReading,
        from_mcu=True
    ),

}

