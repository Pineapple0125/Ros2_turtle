import threading
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy.time
from turtlesim.msg import Pose
import math
from turtlesim.srv import Spawn, SetPen
import random
from scipy.spatial import KDTree

turtleList = []
tutleFollowList = []
tutleListUncatch = []
turtle_index = 2


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.turtle_x = 5.0
        self.turtle_y = 5.0
        self.turtle_angle = 0.0
        self.angular_speed = 1.0
        self.linear_speed = 8.0
        self.target_x = None
        self.target_y = None
        self.angle_tolerance = 0.01
        self.distance_tolerance = 0.1
        self.turtle_index = 1

        client_set_pen = self.create_client(SetPen, 'turtle1/set_pen')
        while not client_set_pen.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service /turtle1/set_pen not available, waiting again...')
        request_set_pen = SetPen.Request()
        request_set_pen.r = 0
        request_set_pen.g = 0
        request_set_pen.b = 0
        request_set_pen.width = 0
        request_set_pen.off = 1
        client_set_pen.call_async(request_set_pen)

    def pose_callback(self, pose):
        global tutleFollowList, tutleListUncatch, lasttank, nexttankIndex

        # update the location of turtle
        self.turtle_x = pose.x
        self.turtle_y = pose.y
        self.turtle_angle = pose.theta

        if self.target_x is not None and self.target_y is not None:
            self.move_to_target()

        if len(tutleListUncatch) > 0:

            if len(tutleListUncatch) <= 1:
                distance = math.sqrt(pow((tutleListUncatch[0].turtle_x - self.turtle_x), 2) + pow(
                    (tutleListUncatch[0].turtle_y - self.turtle_y), 2))
                if distance <= 0.5:
                    tutleFollowList.append(tutleListUncatch.pop(0))

            else:
                for i in range(len(tutleListUncatch) - 1, -1, -1):
                    distance = math.sqrt(pow((tutleListUncatch[i].turtle_x - self.turtle_x), 2) + pow(
                        (tutleListUncatch[i].turtle_y - self.turtle_y), 2))
                    if distance <= 0.5:
                        tutleFollowList.append(tutleListUncatch.pop(i))

        for i in range(1, len(tutleFollowList)):
            twist_data = Twist()
            distance = math.sqrt(pow((tutleFollowList[i - 1].turtle_x - tutleFollowList[i].turtle_x), 2) + pow(
                (tutleFollowList[i - 1].turtle_y - tutleFollowList[i].turtle_y), 2))
            angle = math.atan2(tutleFollowList[i - 1].turtle_y - tutleFollowList[i].turtle_y,
                               tutleFollowList[i - 1].turtle_x - tutleFollowList[i].turtle_x)

            angle_diff = angle - tutleFollowList[i].turtle_angle

            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            if distance > 0.1:
                twist_data.linear.x = 1.5 * distance
                twist_data.angular.z = 3.0 * angle_diff

            tutleFollowList[i].publisher.publish(twist_data)

    def set_target(self, x, y):
        # setting the position of target
        self.target_x = x
        self.target_y = y

    def move_to_target(self):
        dist_x = self.target_x - self.turtle_x
        dist_y = self.target_y - self.turtle_y
        distance = math.sqrt(pow(dist_x, 2) + pow(dist_y, 2))
        msg = Twist()
        if distance > 0.5:
            # update position components
            msg.linear.x = 1.5

            # update orientation
            goal_theta = math.atan2(dist_y, dist_x)
            dif_theta = goal_theta - self.turtle_angle

            # make angle within 0 to 2pi
            if dif_theta > math.pi:
                dif_theta -= 2 * math.pi
            elif dif_theta < -math.pi:
                dif_theta += 2 * math.pi

            msg.angular.z = 4 * dif_theta

        else:
            # make them 0 to stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher.publish(msg)


class SubTurtle(Node):
    def __init__(self, turtle_name):
        super().__init__(turtle_name)  # init the node of ROS
        self.turtle_name = turtle_name

        # create the publisher to control the turtle
        self.publisher = self.create_publisher(Twist, self.turtle_name + '/cmd_vel', 10)
        # create a 监听器 to substripe the location of turtle
        self.subscription = self.create_subscription(Pose, self.turtle_name + '/pose', self.pose_callback, 10)

        self.client = self.create_client(Spawn, '/spawn')  # create a spawn survive
        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.turtle_angle = 0.0

        self.client_set_pen = self.create_client(SetPen, self.turtle_name + '/set_pen')

        self.create_new_turtle()
        # self.timer = self.create_timer(1.0, self.print_terminal_turtle_position)

    def pose_callback(self, pose):
        self.turtle_x = pose.x
        self.turtle_y = pose.y
        self.turtle_angle = pose.theta

        pass

    def create_new_turtle(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        request = Spawn.Request()
        request.x = random.uniform(0, 11)  # random x
        request.y = random.uniform(0, 11)  # random y
        request.theta = random.uniform(0, 2 * 3.1416)  # random angle
        self.turtle_x = request.x
        self.turtle_y = request.y
        self.turtle_angle = request.theta
        # next part is create a new turtle
        self.client.call_async(request)

        while not self.client_set_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.turtle_name}/set_pen not available, waiting again...')

        # shezhi bishuia
        self.request_set_pen = SetPen.Request()
        self.request_set_pen.r = 0
        self.request_set_pen.g = 0
        self.request_set_pen.b = 0
        self.request_set_pen.width = 0
        self.request_set_pen.off = 1

        self.client_set_pen.call_async(self.request_set_pen)

def create_subTurtle():
    global turtle_index
    newTurtle = SubTurtle("turtle" + str(turtle_index))
    tutleListUncatch.append(newTurtle)
    turtleList.append(newTurtle)
    turtle_index += 1


def loop_create():
    create_subTurtle()
    threading.Timer(3, loop_create).start()


def find_nearest_turtle_advanced(target_turtle, all_turtles):
    if not all_turtles:
        pass
    else:
        # create a KD tree
        tree = KDTree(all_turtles)
        # find the closest subturle that havent catched
        distance, index = tree.query(target_turtle)
        return all_turtles[index]


def main(args=None):
    global turtle_index, tutleFollowList, turtleList, tutleListUncatch
    rclpy.init(args=args)
    controller = TurtleController()
    tutleFollowList.append(controller)
    turtleList.append(controller)

    for i in range(2, 10):
        create_subTurtle()
    loop_create()

    try:
        while rclpy.ok():  # main loop
            if len(tutleListUncatch) != 0:
                masterTurtle = (controller.turtle_x, controller.turtle_y)
                all_turtles = [(i.turtle_x, i.turtle_y) for i in tutleListUncatch]
                nearest_turtle = find_nearest_turtle_advanced(masterTurtle, all_turtles)
                controller.set_target(nearest_turtle[0], nearest_turtle[1])
                try:
                    rclpy.spin_once(controller)
                    for i in turtleList:
                        rclpy.spin_once(i)
                except KeyboardInterrupt:
                    pass

            for i in turtleList:
                rclpy.spin_once(i)

    except KeyboardInterrupt:
        pass
    finally:
        # Destruction node
        controller.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
