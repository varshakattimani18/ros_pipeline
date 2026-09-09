import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.srv import SetMode
from drone_msgs.msg import TargetList
import math

ARRIVAL_THRESHOLD_METERS = 3.0
TYPE_MASK_POSITION_ONLY = 4088  # ignore velocity, accel, yaw, yaw rate

STATE_WAITING = 'WAITING'
STATE_NAVIGATING = 'NAVIGATING'
STATE_DROPPING = 'DROPPING'
STATE_LANDING = 'LANDING'
STATE_DONE = 'DONE'

class NavDeliveryNode(Node):
    def __init__(self):
        super().__init__('nav_delivery_node')
        self.declare_parameter('drone_id', 'rudra')
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().string_value
        mavros_prefix = f'/{self.drone_id}/mavros'

        self.state = STATE_WAITING
        self.remaining_targets = []
        self.current_target = None
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.guided_mode_set = False

        self.create_subscription(TargetList, f'/{self.drone_id}/target_list', self.target_list_callback, 10)
        self.create_subscription(NavSatFix, f'{mavros_prefix}/global_position/global', self.gps_callback, 10)

        self.setpoint_pub = self.create_publisher(GlobalPositionTarget, f'{mavros_prefix}/setpoint_position/global', 10)
        self.drop_pub = self.create_publisher(Int32, f'/{self.drone_id}/trigger_drop', 10)
        self.mode_client = self.create_client(SetMode, f'{mavros_prefix}/set_mode')

        self.timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info('Nav delivery node started, waiting for target list')

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

    def target_list_callback(self, msg):
        if self.state != STATE_WAITING:
            return
        self.remaining_targets = list(msg.targets)
        self.get_logger().info(f'Received {len(self.remaining_targets)} targets for delivery')

    def control_loop(self):
        if self.current_lat is None:
            return

        if self.state == STATE_WAITING:
            if self.remaining_targets:
                self.set_guided_mode()
            return

        if self.state == STATE_NAVIGATING:
            if self.current_target is None:
                self.pick_nearest_target()
            self.publish_setpoint(self.current_target)
            dist = self.haversine_m(
                self.current_lat, self.current_lon,
                self.current_target.latitude, self.current_target.longitude
            )
            if dist < ARRIVAL_THRESHOLD_METERS:
                self.get_logger().info(f'Arrived at target {self.current_target.target_id}, dist={dist:.1f}m')
                self.state = STATE_DROPPING
                self.drop_pub.publish(Int32(data=self.current_target.target_id))

        elif self.state == STATE_DROPPING:
            self.remaining_targets = [
                t for t in self.remaining_targets if t.target_id != self.current_target.target_id
            ]
            self.current_target = None
            if self.remaining_targets:
                self.state = STATE_NAVIGATING
            else:
                self.state = STATE_LANDING

        elif self.state == STATE_LANDING:
            self.land()

    def set_guided_mode(self):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            return
        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        future = self.mode_client.call_async(req)
        future.add_done_callback(self.guided_mode_response)

    def guided_mode_response(self, future):
        result = future.result()
        if result and result.mode_sent:
            self.get_logger().info('GUIDED mode set, beginning delivery navigation')
            self.state = STATE_NAVIGATING
        else:
            self.get_logger().warn('Failed to set GUIDED mode, retrying')

    def pick_nearest_target(self):
        self.current_target = min(
            self.remaining_targets,
            key=lambda t: self.haversine_m(self.current_lat, self.current_lon, t.latitude, t.longitude)
        )
        self.get_logger().info(f'Heading to target {self.current_target.target_id}')

    def publish_setpoint(self, target):
        msg = GlobalPositionTarget()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.type_mask = TYPE_MASK_POSITION_ONLY
        msg.latitude = target.latitude
        msg.longitude = target.longitude
        msg.altitude = self.current_alt if self.current_alt else 10.0
        self.setpoint_pub.publish(msg)

    def land(self):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            return
        req = SetMode.Request()
        req.custom_mode = 'LAND'
        future = self.mode_client.call_async(req)
        future.add_done_callback(self.land_response)

    def land_response(self, future):
        result = future.result()
        if result and result.mode_sent:
            self.get_logger().info('LAND mode set, mission complete')
            self.state = STATE_DONE

    def haversine_m(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        return 2 * R * math.asin(math.sqrt(a))

def main(args=None):
    rclpy.init(args=args)
    node = NavDeliveryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
