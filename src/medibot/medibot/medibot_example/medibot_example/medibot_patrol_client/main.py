
import rclpy

from medibot_example.medibot_patrol_client.medibot_patrol_client \
    import MedibotPatrolClient


def main(args=None):
    rclpy.init(args=args)
    medibot_patrol_client = MedibotPatrolClient()
    rclpy.spin(medibot_patrol_client)


if __name__ == '__main__':
    main()
