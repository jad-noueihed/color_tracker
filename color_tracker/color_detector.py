import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.publisher = self.create_publisher(Point, '/object_position', 10)

        self.lower = np.array([99, 54, 74])
        self.upper = np.array([119, 127, 255])
        self.min_area = 500
        self.kernel = np.ones((5, 5), np.uint8)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open the webcam.')
            return

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._latest_frame = None
        self._frame_lock = threading.Lock()

        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info('color_detector running on /object_position')

    def _grab_loop(self):
        while rclpy.ok():
            ok, frame = self.cap.read()
            if ok:
                with self._frame_lock:
                    self._latest_frame = frame

    def process_frame(self):
        with self._frame_lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame.copy()

        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        position = Point()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > self.min_area:
                moments = cv2.moments(largest)
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])

                position.x = float(cx)
                position.y = float(cy)
                position.z = 1.0

                hull = cv2.convexHull(largest)
                cv2.drawContours(frame, [hull], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(frame, f'({cx}, {cy})', (cx + 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        self.publisher.publish(position)

        cv2.imshow('Camera', frame)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
