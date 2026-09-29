"""D1 arm DDS message types, ported to Python (cyclonedds).

TODO(verify): Field names/types are reconstructed from the official C++ samples
(`pm->data_()`, `pm->servo0_data_()` ...). Check them against the IDL / generated
.hpp in the D1 SDK zip or `marm_code/src/msg/`. If a type does not match exactly,
DDS silently never delivers any data (the reader just doesn't match the writer).
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

    # TODO(verify): float32 vs float64
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
