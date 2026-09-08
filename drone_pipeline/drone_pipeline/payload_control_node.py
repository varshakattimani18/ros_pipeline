import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from mavros_msgs.srv import CommandLong

MAV_CMD_DO_SET_SERVO = 183
PWM_OPEN = 1900
PWM_CLOSED = 1100

# Map payload bay index -> servo channel. Adjust to match your actual wiring.
SERVO_CHANNEL_MAP = {
    0: 9,
    1: 9,
    2: 11,
    3: 11,
}

class PayloadControlNode(Node):
    def __init__(self):
        super().__init__('payload_control_node')

        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.create_subscription(Int32, '/payload/trigger_drop', self.trigger_callback, 10)

        self.get_logger().info('Payload control node started, waiting for /payload/trigger_drop')

    def trigger_callback(self, msg):
        bay_index = msg.data
        if bay_index not in SERVO_CHANNEL_MAP:
            self.get_logger().error(f'Unknown payload bay index: {bay_index}')
            return

        channel = SERVO_CHANNEL_MAP[bay_index]
        self.get_logger().info(f'Dropping payload {bay_index} via servo channel {channel}')
        self.set_servo(channel, PWM_OPEN)

    def set_servo(self, channel, pwm):
        if not self.command_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('MAVROS command service not available')
            return
        req = CommandLong.Request()
        req.command = MAV_CMD_DO_SET_SERVO
        req.param1 = float(channel)
        req.param2 = float(pwm)
        future = self.command_client.call_async(req)
        future.add_done_callback(lambda f: self.command_response(f, channel, pwm))

    def command_response(self, future, channel, pwm):
        result = future.result()
        if result and result.success:
            self.get_logger().info(f'Servo {channel} set to {pwm} successfully')
        else:
            self.get_logger().warn(f'Failed to set servo {channel} to {pwm}, retrying may be needed')

def main(args=None):
    rclpy.init(args=args)
    node = PayloadControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
