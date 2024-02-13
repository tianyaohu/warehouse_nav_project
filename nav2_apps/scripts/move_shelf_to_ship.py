import time
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
import math

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from attach_shelf_client import AttachShelfClient
from move_controller import MovementController

from rclpy.executors import MultiThreadedExecutor

START_POS_X = -0.159
START_POS_Y = 0.0
START_ORN_Z = 0.0

# Shelf positions for picking
shelf_positions = {
    "shelf_start": [0.05, 0.0, 0],
    "loading_pose": [4.5, -0.35, -1.57],
    "initial_position": [START_POS_X, START_POS_Y, START_ORN_Z]
}

shipping_destinations = {
    "office_corner" : [0.31, -2.8, -1.72]
}

def get_sim_initial_pose(nav):
    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = START_POS_X
    initial_pose.pose.position.y = START_POS_Y
    initial_pose.pose.orientation.z = START_ORN_Z
    return initial_pose


def go_to_pose(navigator_obj, pose, location_str = "", recovery_pose = None):
    x,y,yaw = pose

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator_obj.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = x
    shelf_item_pose.pose.position.y = y
    shelf_item_pose.pose.orientation.z = math.sin(yaw / 2)
    shelf_item_pose.pose.orientation.w =  math.cos(yaw / 2)
    if len(location_str) > 0 :
        print("Going towards ", location_str)
    navigator_obj.goToPose(shelf_item_pose)

    #Wait until task is compelte
    i = 0
    while not navigator_obj.isTaskComplete():
        i = i + 1
        feedback = navigator_obj.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + location_str +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    navigation_result = navigator_obj.getResult()

    if navigation_result == TaskResult.SUCCEEDED:
        print('Task at ' + location_str +
                    ' Reached.')

    elif navigation_result == TaskResult.CANCELED:
        print('Task at ' + location_str +
              ' was canceled. Returning to staging point...')
        recovery_pose.header.stamp = navigator_obj.get_clock().now().to_msg()
        navigator_obj.goToPose(recovery_pose)

    elif navigation_result == TaskResult.FAILED:
        print('Task at ' + location_str + ' failed!')
        exit(-1)

    return navigator_obj.getResult()

#TODO
# def move_for_x_sec(controller):
    # controller.move_for_x_sec()


def main():
    rclpy.init()

    #init navigator
    navigator = BasicNavigator()

    #init move controller
    movement_controller = MovementController()

    # (1) init shelf client
    shelf_client = AttachShelfClient()

    # Set robot initial pose
    initial_pose = get_sim_initial_pose(navigator)
    
    #announcing initial pose set
    print("Simulation initial pose set")

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    loading_success = go_to_pose(navigator, shelf_positions['loading_pose'],  'loading_pose', recovery_pose=initial_pose)

    if loading_success == TaskResult.SUCCEEDED:
        ############ ATTACH SHELF CLIENT #############

        # (1) get request future
        approach_shelf_result = shelf_client.send_request()
        print(str(type(approach_shelf_result)))

        print("approach_shelf_result", approach_shelf_result)

        if approach_shelf_result:
            print("shelf should be lifted by now")

            #(3) Do a small pause 
            movement_controller.move_for_x_sec(0,0,1)

            # if shelf was approach shelf service return successful
            if approach_shelf_result:
                #backing from the shelf
                movement_controller.move_for_x_sec(-0.1, 0, 12)

                #go to shipping position
                go_to_pose(navigator, shipping_destinations['office_corner'],  'office_corner')

                #lowering elevator
                shelf_client.lift_down()

                # Do a small pause 
                movement_controller.move_for_x_sec(0,0,1)

                #move backwards
                movement_controller.move_for_x_sec(-0.2,0,5)

                #go back to initial position
                go_to_pose(navigator, shelf_positions['initial_position'],  'initial_position')
            else:
                print("Approach Shelf Failed")
        else:
            print("Failed to approach shelf. Please check if tf if properly broadcasted in RVIZ and check if the cart is placed backwards")
    else:
        print("Failed reaching loading pose, Terminating...")
        return


    exit(0)


if __name__ == '__main__':
    main()