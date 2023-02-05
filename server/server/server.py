import rclpy
import time
from rclpy.node import Node
from uro_control_msgs.msg import MotionControlMode
from uro_control_msgs.srv import ControllerManager


class ServiceServer(Node):
    def __init__(self):
        super().__init__("service_server")
        self.mode = MotionControlMode()
        self.srv = self.create_service(
            ControllerManager, "/motion_control_node/controller_manager", self.callback
        )
        self.pub = self.create_publisher(MotionControlMode, '/motion_control_node/motion_control_mode', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def callback(self, request, response):
        self.mode.main_mode = request.main_mode
        response.success = True
        response.message = "aaa"
        mode_str = "OFF"
        if self.mode.main_mode == MotionControlMode.WBC:
            mode_str = "WBS"
        elif self.mode.main_mode == MotionControlMode.ROV:
            mode_str = "ROV"
        self.get_logger().info(f'main_mode={mode_str}')
        time.sleep(2)
        return response

    def timer_callback(self):
        self.pub.publish(self.mode)

def main(args=None):
    rclpy.init(args=args)
    server = ServiceServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()