"""D1 arm DDS message types, ported to Python (cyclonedds).

Ported from the generated headers in `marm_code/src/msg/*.hpp`. If a type does
not match exactly, DDS silently never delivers any data (the reader just doesn't
match the writer). ArmString_ / PubServoInfo_ / SetServoAngle_ are confirmed on the
real arm.
"""

from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class ArmString_(idl.IdlStruct, typename="unitree_arm.msg.dds_.ArmString_"):
    """JSON command / feedback string. Topics: rt/arm_Command, rt/arm_Feedback."""

    data: str


@dataclass
@annotate.final
@annotate.autoid("sequential")
class PubServoInfo_(idl.IdlStruct, typename="unitree_arm.msg.dds_.PubServoInfo_"):
    """Raw servo angles [deg], J0..J6 (J6 = gripper). Topic: current_servo_angle."""

    servo0_data: types.float32
    servo1_data: types.float32
    servo2_data: types.float32
    servo3_data: types.float32
    servo4_data: types.float32
    servo5_data: types.float32
    servo6_data: types.float32

    def as_list(self) -> list:
        return [
            self.servo0_data,
            self.servo1_data,
            self.servo2_data,
            self.servo3_data,
            self.servo4_data,
            self.servo5_data,
            self.servo6_data,
        ]


@dataclass
@annotate.final
@annotate.autoid("sequential")
class SetServoAngle_(idl.IdlStruct, typename="unitree_arm.msg.dds_.SetServoAngle_"):
    """Single-joint angle command [deg]. Topic: set_servo_angle (read by marm_controller_node)."""

    seq: types.int32
    id: types.uint8
    angle: types.float32
    delay_ms: types.int16


@dataclass
@annotate.final
@annotate.autoid("sequential")
class SetServoDumping_(idl.IdlStruct, typename="unitree_arm.msg.dds_.SetServoDumping_"):
    """Single-joint damping (limp) command. Topic: set_servo_dumping."""

    seq: types.int32
    id: types.uint8
    power: types.uint16
