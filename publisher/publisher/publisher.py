import rclpy
from rclpy.node import Node
from uro_control_msgs.msg import RovState
from uro_control_msgs.msg import RovControlConfig

class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(RovState, '/rov_control_node/rov_states', 10)
        self.publisher_gain_ = self.create_publisher(RovControlConfig, '/rov_control_node/rov_control_config', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = RovState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.i % 50 < 10:
            msg.flight_mode = 'MANUAL'
            msg.is_armed = False
            msg.voltage_battery = 15.678
            msg.current_battery = 2.345
            msg.battery_remaining = 1.0
        elif self.i % 50 < 20:
            msg.flight_mode = 'STABILIZE'
            msg.is_armed = True
            msg.voltage_battery = 345.678
            msg.current_battery = 2.345
            msg.battery_remaining = 0.789
        elif self.i % 50 < 30:
            msg.flight_mode = 'STABILIZE'
            msg.is_armed = False
            msg.voltage_battery = 15.678
            msg.current_battery = 2.345
            msg.battery_remaining = 0.789
        elif self.i % 50 < 40:
            msg.flight_mode = 'STABILIZE'
            msg.is_armed = False
            msg.voltage_battery = -0.1
            msg.current_battery = 0.0
            msg.battery_remaining = 0.0
        else:
            msg.flight_mode = 'MANUAL'
            msg.is_armed = True
            msg.voltage_battery = 5.678
            msg.current_battery = 0.345
            msg.battery_remaining = 0.123
        self.publisher_.publish(msg)

        gain = RovControlConfig()
        gain.p.linear.x = 0.8
        gain.p.linear.y = 0.8
        gain.p.linear.z = 0.8
        gain.p.angular.x = 2.0
        gain.p.angular.y = 2.0
        gain.p.angular.z = 2.0
        gain.d.linear.x = 2.0
        gain.d.linear.y = 2.0
        gain.d.linear.z = 2.0
        gain.d.angular.x = 2.0
        gain.d.angular.y = 2.0
        gain.d.angular.z = 2.0
        self.publisher_gain_.publish(gain)

        self.get_logger().info(f'@@@aaa {self.i}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()