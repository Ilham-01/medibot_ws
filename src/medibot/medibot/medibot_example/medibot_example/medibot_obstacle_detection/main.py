
import rclpy

from medibot_example.medibot_obstacle_detection.medibot_obstacle_detection \
    import MedibotObstacleDetection


def main(args=None):
    rclpy.init(args=args)
    medibot_obstacle_detection = MedibotObstacleDetection()
    rclpy.spin(medibot_obstacle_detection)

    medibot_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
