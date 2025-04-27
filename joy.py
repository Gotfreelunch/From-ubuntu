import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from msg_joy_orca.msg import OrcaJoystick
import pygame

pygame.init()
pygame.joystick.init()

class ROVTeleop(Node):
    def __init__(self):
        super().__init__('rov_joystick_node')

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise Exception("Tidak ada joystick yang terdeteksi!")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Joystick '{self.joystick.get_name()}' berhasil diinisialisasi.")
        
        self.counter_ = 0
        self.get_logger().info("Program joystick ORCA dimulai!")
        
        # self.subscribe_ke_sensor_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publish_ke_joy_msg = self.create_publisher(OrcaJoystick, '/joy_msg', 10)

        #status mode -> toggle
        self.arm_toggle = True        # default = arm
        self.control_toggle = True    # default = manual
        self.gripper_toggle = True    # default = open_grip

        #deteksi rising edge
        self.prev_buttons = [0] * 10
        self.timer = self.create_timer(0.05, self.joy_callback)

        if(self.joystick.get_numaxes()==6) and (self.joystick.get_numbuttons()==11):
           self.get_logger().info("Program Stopped!!, Switch joystick first to D Mode") 
           self.destroy()


    def joy_callback(self):
        pygame.event.pump()
        joy_msg = OrcaJoystick() 
        

        def mapping_range(val):
            return int(1500 + (val * 400))

        #GERAKAN STIK
        joy_msg.forward = mapping_range(-1*self.joystick.get_axis(3))    # Kanan stick vertikal
        joy_msg.lateral = mapping_range(self.joystick.get_axis(2))    # Kanan stick horizontal
        joy_msg.throttle = mapping_range(-1*self.joystick.get_axis(1))   # Kiri stik vertikal
        joy_msg.yaw = mapping_range(self.joystick.get_axis(0))        # Kiri stick horizontal

        #KONDISI BUTTON -> default: false
        x = self.joystick.get_button(0)
        y = self.joystick.get_button(3)
        b = self.joystick.get_button(2)
        a = self.joystick.get_button(1)
        r2 = self.joystick.get_button(7)
        l2 = self.joystick.get_button(6)

        #ARM MODE
        if x == 1 and self.prev_buttons[3] == 0 and y != 1:
            self.arm_toggle = True
        elif y == 1 and self.prev_buttons[4] == 0 and x != 1:
            self.arm_toggle = False
        joy_msg.arm_mode = "arm" if self.arm_toggle else "disarm"

        #CONTROL MODE
        if b == 1 and self.prev_buttons[1] == 0 and a != 1:
            self.control_toggle = True
        elif a == 1 and self.prev_buttons[0] == 0 and b != 1:
            self.control_toggle = False
        joy_msg.control_mode = "MANUAL" if self.control_toggle else "ALT_HOLD"

        #GRIPPER MODE
        if l2 == 1 and self.prev_buttons[8] == 0 and r2 != 1:
            self.gripper_toggle = True
        elif r2 == 1 and self.prev_buttons[9] == 0 and l2 != 1:
            self.gripper_toggle = False
        joy_msg.gripper_mode = "open_grip" if self.gripper_toggle else "close_grip"

        #update previous button
        self.prev_buttons[3] = x
        self.prev_buttons[4] = y
        self.prev_buttons[1] = b
        self.prev_buttons[0] = a
        self.prev_buttons[9] = r2
        self.prev_buttons[8] = l2

        self.publish_ke_joy_msg.publish(joy_msg)

    def destroy(self):
        """Memberhentikan pygame."""
        pygame.quit()
        super().destroy_node()
        self.get_logger().info("JoystickReader dimatikan.")    

def main(args=None):
    rclpy.init(args=args)
    node = ROVTeleop()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == "__main__":
    main()