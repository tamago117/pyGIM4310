import os
import can
import time
from enum import Enum
from typing import List, Tuple

class GIM4310:
    class OperationMode(Enum):
        POSITION = 1
        VELOCITY = 2
        CURRENT = 3

    GIM4310_CODE_PARAM = {
        "P_MAX": 32768.0,
        "V_MAX": 2048.0,
        "C_MAX": 2048.0,
        "KP_MAX": 4096.0,
        "KD_MAX": 4096.0
    }

    GIM4310_PARAM = {
        "P_MAX": 12.5,
        "V_MAX": 65.0,
        "C_MAX": 4.0,
        "KP_MAX": 500.0,
        "KD_MAX": 5.0
    }

    def __init__(self, id, can_interface="can0"):
        self.id = id
        self.can_interface = can_interface
        self.bitrate = 1000000
        self._can_setup(self.can_interface, self.bitrate)

        self.canbus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')

    def __del__(self):
        self._can_cleanup(self.can_interface)

    def _can_setup(self, interface, bitrate=1000000):
        """ Setup CAN interface with given bitrate """
        os.system(f'sudo ifconfig {interface} down')
        os.system(f'sudo ip link set {interface} type can bitrate {bitrate}')
        os.system(f'sudo ifconfig {interface} txqueuelen 100000')
        os.system(f'sudo ifconfig {interface} up')

    def _can_cleanup(self, interface):
        os.system(f'sudo ifconfig {interface} down')

    def _can_send(self, message):
        frame = can.Message(arbitration_id=self.id, data=message, is_extended_id=False)

        try:
            self.canbus.send(frame)
        except can.CanError as e:
            print(f"Error while sending the message: {e}")

    def can_receive(self, timeout=10.0):
        return self.canbus.recv(timeout)

    def send_receive(self, sender, receiver):
        """ Function to send and receive CAN messages """
        # Base frame
        sff_frame = can.Message(arbitration_id=0x123, data=[0,1,2,3,4,5,6,7])
        sender.send(sff_frame)
        msg = receiver.recv(10.0)
        if msg is None:
            print("USB2CAN hardware connection failure.")
        else:
            print(f"Received base frame: \n{msg}\n")

        # Extended frame
        eff_frame = can.Message(arbitration_id=0x1FFF6666, data=[7,6,5,4,3,2,1,0],is_extended_id=True)
        sender.send(eff_frame)
        msg = receiver.recv(10.0)
        if msg is None:
            print("USB2CAN hardware connection failure.")
        else:
            print(f"Received extended frame: \n{msg}\n")

        # Remote frame
        rtr_frame = can.Message(arbitration_id=0x321, data=[0,1,2,3,4,5,6,7],is_remote_frame=True)    
        sender.send(rtr_frame)
        msg = receiver.recv(10.0)
        if msg is None:
            print("USB2CAN hardware connection failure.")
        else:
            print(f"Received remote frame: \n{msg}\n")

    @staticmethod
    def _float_to_uint(v: float, v_min: float, v_max: float, width: float) -> int:
        utemp = int(v)
        utemp = max(0, min(width, utemp))
        return int(utemp)

    def turn_on_motor(self, mode: bool):
        message = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC if mode else 0xFD]
        self._can_send(message)

    def set_zero_position(self):
        message = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
        self._can_send(message)

    def switch_operation_mode(self, mode: OperationMode):
        if mode == GIM4310.OperationMode.POSITION:
            message = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB]
        elif mode == GIM4310.OperationMode.VELOCITY:
            message = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFA]
        elif mode == GIM4310.OperationMode.CURRENT:
            message = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF9]

        self._can_send(message)

    # rad: -12.5 ~ 12.5 kp: 0 ~ 500 kd: 0 ~ 5
    def _set_command(self, position, kp, kd, velocity, current):
        a = self.GIM4310_PARAM["P_MAX"]
        b = self.GIM4310_CODE_PARAM["P_MAX"]
        pos_code = position * b / a + b

        a = self.GIM4310_PARAM["KP_MAX"]
        b = self.GIM4310_CODE_PARAM["KP_MAX"]
        kp_code = kp * b / a

        a = self.GIM4310_PARAM["KD_MAX"]
        b = self.GIM4310_CODE_PARAM["KD_MAX"]
        kd_code = kd * b / a

        a = self.GIM4310_PARAM["V_MAX"]
        b = self.GIM4310_CODE_PARAM["V_MAX"]
        vel_code = velocity * b / a + b

        a = self.GIM4310_PARAM["C_MAX"]
        b = self.GIM4310_CODE_PARAM["C_MAX"]
        current_code = current * b / a + b

        s_p_int = self._float_to_uint(pos_code, -self.GIM4310_CODE_PARAM["P_MAX"], self.GIM4310_CODE_PARAM["P_MAX"], self.GIM4310_CODE_PARAM["P_MAX"]*2)
        s_v_int = self._float_to_uint(vel_code, -self.GIM4310_CODE_PARAM["V_MAX"], self.GIM4310_CODE_PARAM["V_MAX"], self.GIM4310_CODE_PARAM["V_MAX"]*2)
        s_Kp_int = self._float_to_uint(kp_code, 0 , self.GIM4310_CODE_PARAM["KP_MAX"], self.GIM4310_CODE_PARAM["KP_MAX"]*2)
        s_Kd_int = self._float_to_uint(kd_code, 0 , self.GIM4310_CODE_PARAM["KD_MAX"], self.GIM4310_CODE_PARAM["KD_MAX"]*2)
        s_c_int = self._float_to_uint(current_code, -self.GIM4310_CODE_PARAM["C_MAX"], self.GIM4310_CODE_PARAM["C_MAX"], self.GIM4310_CODE_PARAM["C_MAX"]*2)

        transmit_message = bytearray(8)
        transmit_message[0] = s_p_int >> 8
        transmit_message[1] = s_p_int & 0xFF
        transmit_message[2] = s_v_int >> 4
        transmit_message[3] = ((s_v_int & 0xF) << 4) + (s_Kp_int >> 8)
        transmit_message[4] = s_Kp_int & 0xFF
        transmit_message[5] = s_Kd_int >> 4
        transmit_message[6] = ((s_Kd_int & 0xF) << 4) + (s_c_int >> 8)
        transmit_message[7] = s_c_int & 0xFF

        self._can_send(transmit_message)

    def set_position(self, position, Kp, Kd):
        position = max(-self.GIM4310_PARAM["P_MAX"], min(position, self.GIM4310_PARAM["P_MAX"]))
        Kp = max(0, min(Kp, self.GIM4310_PARAM["KP_MAX"]))
        Kd = max(0, min(Kd, self.GIM4310_PARAM["KD_MAX"]))
            
        self.switch_operation_mode(GIM4310.OperationMode.POSITION)
        self._set_command(position, Kp, Kd, 0, 0)

    def set_velocity(self, velocity: float):
        velocity = max(-self.GIM4310_PARAM["V_MAX"], min(velocity, self.GIM4310_PARAM["V_MAX"]))

        self.switch_operation_mode(GIM4310.OperationMode.VELOCITY)
        self._set_command(0, 0, 0, velocity, 0)

    def set_current(self, current: float):
        current = max(-self.GIM4310_PARAM["C_MAX"], min(current, self.GIM4310_PARAM["C_MAX"]))

        self.switch_operation_mode(GIM4310.OperationMode.CURRENT)
        self._set_command(0, 0, 0, 0, current)

    

# Usage:
motor = GIM4310(0x001, "can0")
time.sleep(3)
motor.turn_on_motor(True)
"""motor.set_zero_position()
motor.set_position(1, 500, 1)
time.sleep(3)
motor.set_position(-1, 500, 1)
time.sleep(3)
motor.set_position(0, 500, 1)
time.sleep(3)"""
motor.set_current(2)
time.sleep(3)
motor.set_current(-2)
time.sleep(3)

motor.turn_on_motor(False)
print("operation finish")

