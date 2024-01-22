import time
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
import math

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from attach_shelf_client import AttachShelfClient
from move_controller import MovementController

from rclpy.executors import MultiThreadedExecutor
# Shelf positions for picking
shelf_positions = {
    "shelf_start": [5.273, -2.853, 0],
    "loading_pose": [5.72, 0.0, -1.78]
}

shipping_destinations = {
    "office_corner" : [0.5, -3.0, 1.78]
}

'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''

SIM_START_POS_X = -0.159
SIM_START_POS_Y = 0.0
SIM_START_ORN_Z = 0.0

def get_sim_initial_pose(nav):
    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = SIM_START_POS_X
    initial_pose.pose.position.y = SIM_START_POS_Y
    initial_pose.pose.orientation.z = SIM_START_ORN_Z
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

        # Wait for the future result
        # while rclpy.ok() and not approach_result_future.done():
        #     time.sleep(0.2)
        #     print("not done? ", approach_result_future.done())
        #     print("result? ", approach_result_future.result())


        # (2) SHould have wait until request is finsihed before print
        print("shelf should be lifted by now")

        print("approach_shelf_result", approach_shelf_result)

        # if shelf was approach shelf service return successful
        if approach_shelf_result:
            movement_controller.move_for_x_sec(-0.1, 0, 22)

            go2shipping_success = go_to_pose(navigator, shipping_destinations['office_corner'],  'office_corner')

            print("go to shipping success? ", go2shipping_success)
        else:
            print("Approach Shelf Failed")
    else:
        print("Failed reaching loading pose, Terminating...")
        return


    exit(0)


if __name__ == '__main__':
    main()