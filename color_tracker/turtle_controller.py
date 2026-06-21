import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from turtlesim.msg import Pose


IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
TURTLE_MAX = 11.0

KP_LINEAR = 1.5
KP_ANGULAR = 6.0

FOLLOW_DISTANCE = 0.5
TRACK_RESUME_DISTANCE = 1.0

SEARCH_ANGULAR_SPEED = 1.0
SEARCH_AFTER_SECONDS = 5.0
GIVE_UP_AFTER_SECONDS = 10.0


class State(Enum):
    IDLE = auto()
    TRACKING = auto()
    FOLLOWING = auto()
    LOST_TARGET = auto()
    SEARCHING = auto()
    STOP = auto()


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        self.position_sub = self.create_subscription(
            Point, '/object_position', self.on_position, 10)
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.on_pose, 10)
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.target_x = 0.0
        self.target_y = 0.0
        self.object_detected = False
        self.pose = None

        self.state = State.IDLE
        self.last_seen_time = None
        self.search_start_time = None

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('turtle_controller running in IDLE state')

    def on_position(self, msg):
        self.object_detected = msg.z > 0.5
        if self.object_detected:
            self.target_x = (msg.x / IMAGE_WIDTH) * TURTLE_MAX
            self.target_y = (1.0 - msg.y / IMAGE_HEIGHT) * TURTLE_MAX
            self.last_seen_time = self.get_clock().now()

    def on_pose(self, msg):
        self.pose = msg

    def change_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'{self.state.name} -> {new_state.name}')
            self.state = new_state

    def seconds_since(self, stamp):
        if stamp is None:
            return float('inf')
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def acquire_state(self, distance):
        return State.FOLLOWING if distance < FOLLOW_DISTANCE else State.TRACKING

    def control_loop(self):
        if self.pose is None:
            return

        distance = None
        dx = dy = 0.0
        if self.object_detected:
            dx = self.target_x - self.pose.x
            dy = self.target_y - self.pose.y
            distance = math.hypot(dx, dy)

        self.update_state(distance)
        self.publish_command(distance, dx, dy)

    def update_state(self, distance):
        if self.state == State.IDLE:
            if self.object_detected:
                self.change_state(self.acquire_state(distance))

        elif self.state == State.TRACKING:
            if not self.object_detected:
                self.change_state(State.LOST_TARGET)
            elif distance < FOLLOW_DISTANCE:
                self.change_state(State.FOLLOWING)

        elif self.state == State.FOLLOWING:
            if not self.object_detected:
                self.change_state(State.LOST_TARGET)
            elif distance > TRACK_RESUME_DISTANCE:
                self.change_state(State.TRACKING)

        elif self.state == State.LOST_TARGET:
            if self.object_detected:
                self.change_state(self.acquire_state(distance))
            elif self.seconds_since(self.last_seen_time) >= SEARCH_AFTER_SECONDS:
                self.search_start_time = self.get_clock().now()
                self.change_state(State.SEARCHING)

        elif self.state == State.SEARCHING:
            if self.object_detected:
                self.change_state(self.acquire_state(distance))
            elif self.seconds_since(self.search_start_time) >= GIVE_UP_AFTER_SECONDS:
                self.change_state(State.STOP)

        elif self.state == State.STOP:
            if self.object_detected:
                self.change_state(self.acquire_state(distance))

    def publish_command(self, distance, dx, dy):
        cmd = Twist()

        if self.state in (State.TRACKING, State.FOLLOWING):
            heading_error = math.atan2(dy, dx) - self.pose.theta
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            cmd.linear.x = KP_LINEAR * distance
            cmd.angular.z = KP_ANGULAR * heading_error

        elif self.state == State.SEARCHING:
            cmd.angular.z = SEARCH_ANGULAR_SPEED

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
