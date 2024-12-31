
import rclpy

from medibot_example.medibot_position_control.medibot_position_control \
    import MedibotPositionControl


def main(args=None):
    rclpy.init(args=args)
    medibot_position_control = MedibotPositionControl()
    rclpy.spin(medibot_position_control)

    medibot_position_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
