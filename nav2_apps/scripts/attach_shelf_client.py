import sys
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time

from rclpy.node import Node
from attach_shelf.srv import GoToLoading
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point32

class AttachShelfClient(Node):

    def __init__(self):
        super().__init__('approach_shelf_client_async')

        # Define QoS profile with reliability set to reliable and durability set to transient local
        qos_profile_elevator = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  
        )

        #elevator publishers
        self.elevate_up_pub_ = self.create_publisher(String, 'elevator_up', qos_profile_elevator)
        self.elevate_down_pub_ = self.create_publisher(String, 'elevator_down', qos_profile_elevator)

        # Define QoS profile with reliability set to reliable and durability set to transient local
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, 
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  
        )

        #create footprint publisher
        self.global_footprint_pub_ = self.create_publisher(Polygon, 'global_costmap/footprint', qos_profile)
        self.local_footprint_pub_ = self.create_publisher(Polygon, 'local_costmap/footprint', qos_profile)

        #once client is inited, init elevator as down
        self.lift_down()

        #create client
        self.cli = self.create_client(GoToLoading, 'approach_shelf')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        #init request BUT NOT SENT: ####### REQUEST MSG IS FIXED ############
        self.req = GoToLoading.Request()
        self.req.attach_to_shelf = True

    def set_footprint(self, length, width):
        self.footprint_msg = Polygon()
        self.footprint_msg.points = [
            Point32(x= length/2, y= width/2),
            Point32(x= length/2, y=-width/2),
            Point32(x=-length/2, y=-width/2),
            Point32(x=-length/2, y= width/2)
        ]

    def set_shelf_footprint(self):
        self.set_footprint(0.8, 0.8)
        print('Shelf_footprint is set.')
        self.publish_footprint()
    
    def set_normal_footprint(self):
        self.set_footprint(0.5, 0.5)
        print('Normal_footprint is set.')
        self.publish_footprint()

    def publish_footprint(self):
        # Publish the footprint message
        self.global_footprint_pub_.publish(self.footprint_msg)
        self.local_footprint_pub_.publish(self.footprint_msg)
        print("published foot print")

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        result = self.future.result()
        if result:
            self.set_shelf_footprint()
        return result

    def lift_up(self):
        msg = String()
        msg.data = ""
        self.elevate_up_pub_.publish(msg)
        #set flag for up flag
        self.get_logger().info('Lifting up ...')
        self.set_shelf_footprint()
        #force sleep for 8 seconds
        for_loop_sleeper(16)

    def lift_down(self):
        msg = String()
        msg.data = ""
        self.elevate_down_pub_.publish(msg)
        #set flag for up flag
        self.get_logger().info('Lifting down ...')
        self.set_normal_footprint()
        #force sleep for 8 seconds
        for_loop_sleeper(16)


#FUNCTIONS BELOW ARE FOR TESTING PURPOSE ONLY
def for_loop_sleeper(num_iterations):
    for i in range(1, num_iterations + 1):
        print(f"Sleeping for {i * 0.5} seconds...")
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)

    approach_shelf_client = AttachShelfClient()

    # print("request", approach_shelf_client.send_request())

    # for i in range(10):
    #     approach_shelf_client.lift_up()

    # for_loop_sleeper(20)
    
    # approach_shelf_client.lift_down()

    # try:
    #     # Spin the executor
    #     executor.spin()
    # except KeyboardInterrupt:
    #     pass

    # Cleanup
    approach_shelf_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
