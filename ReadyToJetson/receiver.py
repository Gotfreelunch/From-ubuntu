#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from msg_joy_orca.msg import OrcaJoystick
from pymavlink import mavutil
import time

class ROVTeleopReceiver(Node):
    def __init__(self):
        super().__init__('rov_joystick_receiver_node')
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
        self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat dari sistem didapatkan")
        
        self.subs_from_joy = self.create_subscription(OrcaJoystick, '/joy_msg', self.receive_joydata, 10)
        self.pwm = 0
        self.mode = ''
        self.prev_msg = None
        self.arm = 0


# Fungsi untuk mengirim RC override
    def receive_joydata(self,msg):

        if msg != self.prev_msg:
            self.get_logger().info(f"""
            Forward pwm  : {msg.forward} \t| Arm     : {msg.arm_mode}\n
            Lateral pwm  : {msg.lateral} \t| Mode    : {msg.control_mode}\n
            Throttle pwm : {msg.throttle}\t| Gripper : {msg.gripper_mode}\n
            Yaw pwm      : {msg.yaw}     \t| \n
            """)
            self.prev_msg = msg

        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            0,   # Channel 1 (Roll)
            0,   # Channel 2 (Pitch)
            (msg.throttle),  # Channel 3 (Throttle dikendalikan Depth Hold)
            (msg.yaw),       # Channel 4 (Yaw)
            (msg.forward),   # Channel 5 (Forward)
            (msg.lateral),   # Channel 6 (lateral)
            0, 0        
        )

        
        if msg.arm_mode == 'arm':
            self.arm = 1  # ARM
        else:
            self.arm = 0  # DISARM
        self.master.mav.command_long_send(
            self.master.target_system,            
            self.master.target_component,         
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
            0,                              
            self.arm,                           
            0, 0, 0, 0, 0, 0                
        )
        # Tunggu konfirmasi arming/disarming
        # ack = self.master.recv_match(type='COMMAND_ACK', blocking=True)
   
        # servo_channel: MAVLink channel (AUX1 = channel 9)
        # pwm: nilai PWM, contoh 1000 (lepas), 2000 (ambil)
        servo_channel=9
        if (msg.gripper_mode == 'open_grip'):
            self.pwm=1000
        else:
            self.pwm=2000   
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,                # confirmation
            servo_channel,    # channel (AUX1 = 9)
            self.pwm,              # PWM value
            0, 0, 0, 0, 0     # unused parameters
        )
        # print(f"Gripper di channel {servo_channel} diset ke PWM {pwm}")


        mode_name = msg.control_mode
        mode_id = self.master.mode_mapping().get(mode_name.upper())

        if mode_id is None:
            print(f"Mode {mode_name} tidak dikenal!")
            return

        # Kirim perintah SET_MODE
        self.master.set_mode(mode_id)
        # Tunggu respon ACK dari kendaraan
        # ack = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        # if ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
        #     print(f"ACK diterima: {ack.result}")
        # else:
        #     print("ACK tidak sesuai atau gagal")

    def reset_override(self):
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *[65535] * 8
        )

    def destroy(self):
        """Memberhentikan pygame."""
        super().destroy_node()
        self.get_logger().info("JoystickReader dimatikan.") 

def main(args=None):
    rclpy.init(args=args)
    node = ROVTeleopReceiver()
    rclpy.spin(node)
    node.reset_override()
    node.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
