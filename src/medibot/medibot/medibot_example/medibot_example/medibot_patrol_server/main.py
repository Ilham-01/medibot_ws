
import rclpy

from medibot_example.medibot_patrol_server.medibot_patrol_server \
    import MedibotPatrolServer


def main(args=None):
    rclpy.init(args=args)
    medibot_patrol_server = MedibotPatrolServer()
    rclpy.spin(medibot_patrol_server)

    medibot_patrol_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
