import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from drone_msgs.msg import Detection, TargetList, Target
import math

DEDUP_DISTANCE_METERS = 4.0

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.scout_done = False
        self.payload_done = False
        self.all_detections = []
        self.fused_published = False

        self.create_subscription(Bool, '/scout/scan_complete', self.scout_done_callback, 10)
        self.create_subscription(Bool, '/payload/scan_complete', self.payload_done_callback, 10)
        self.create_subscription(Detection, '/scout/detections', self.detection_callback, 10)
        self.create_subscription(Detection, '/payload/detections', self.detection_callback, 10)

        self.targets_pub = self.create_publisher(TargetList, '/payload/target_list', 10)

        self.get_logger().info('Fusion node started, collecting detections from both drones')

    def scout_done_callback(self, msg):
        if msg.data:
            self.scout_done = True
            self.get_logger().info('Scout scan complete signal received')
            self.try_fuse()

    def payload_done_callback(self, msg):
        if msg.data:
            self.payload_done = True
            self.get_logger().info('Payload scan complete signal received')
            self.try_fuse()

    def detection_callback(self, msg):
        self.all_detections.append(msg)

    def try_fuse(self):
        if self.fused_published:
            return
        if not (self.scout_done and self.payload_done):
            return

        self.get_logger().info(f'Both scans complete. Fusing {len(self.all_detections)} raw detections')
        targets = self.dedup_detections(self.all_detections)

        target_list_msg = TargetList()
        target_list_msg.targets = targets
        self.targets_pub.publish(target_list_msg)
        self.fused_published = True

        self.get_logger().info(f'Published {len(targets)} fused targets on /payload/target_list')

    def dedup_detections(self, detections):
        clusters = []

        for det in detections:
            placed = False
            for cluster in clusters:
                rep = cluster[0]
                dist = self.haversine_m(rep.latitude, rep.longitude, det.latitude, det.longitude)
                if dist < DEDUP_DISTANCE_METERS:
                    cluster.append(det)
                    placed = True
                    break
            if not placed:
                clusters.append([det])

        targets = []
        for i, cluster in enumerate(clusters):
            best = max(cluster, key=lambda d: d.confidence)
            t = Target()
            t.target_id = i
            t.latitude = best.latitude
            t.longitude = best.longitude
            t.confidence = best.confidence
            t.source_drone_id = best.drone_id
            targets.append(t)

        # Cap at 4 payloads — keep highest-confidence targets if more were found
        targets.sort(key=lambda t: t.confidence, reverse=True)
        targets = targets[:4]
        for i, t in enumerate(targets):
            t.target_id = i  # reassign sequential IDs 0-3 after capping

        return targets
    def haversine_m(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
        return 2 * R * math.asin(math.sqrt(a))

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
