"""
PMSM FOC Controller ROS2 Node.

This node provides the ROS2 interface for the PMSM FOC controller.
It subscribes to motor feedback (currents, position) and publishes
PWM duty cycles.

Topics:
    Subscribed:
        /pmsm/feedback   (pmsm_msgs/MotorFeedback) - Motor sensor data
        /pmsm/speed_ref  (std_msgs/Float64)         - Speed reference [rad/s]
    Published:
        /pmsm/pwm_duty   (pmsm_msgs/PWMDuty)        - PWM duty cycles
        /pmsm/status     (pmsm_msgs/ControllerStatus)- Controller status

Parameters:
    motor.Rs, motor.Ld, motor.Lq, motor.flux_pm, motor.pole_pairs
    controller.Kp_id, Ki_id, Kp_iq, Ki_iq, Kp_speed, Ki_speed
    inverter.Vdc, inverter.fsw
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from .foc_bindings import FOCController, FOCConfig


class PMSMControllerNode(Node):
    """ROS2 node wrapping the C++/Python FOC controller."""

    def __init__(self):
        super().__init__('pmsm_foc_controller')

        # Declare parameters
        self._declare_parameters()

        # Initialize FOC controller
        config = self._build_config()
        self.controller = FOCController(config)

        # State
        self.speed_ref = 0.0
        self.id_ref = 0.0

        # Subscribers
        self.feedback_sub = self.create_subscription(
            Float64MultiArray,
            '/pmsm/feedback',
            self._feedback_callback,
            10
        )
        self.speed_ref_sub = self.create_subscription(
            Float64,
            '/pmsm/speed_ref',
            self._speed_ref_callback,
            10
        )

        # Publishers
        self.pwm_pub = self.create_publisher(
            Float64MultiArray,
            '/pmsm/pwm_duty',
            10
        )
        self.status_pub = self.create_publisher(
            Float64MultiArray,
            '/pmsm/status',
            10
        )

        self.get_logger().info('PMSM FOC Controller node started.')

    def _declare_parameters(self):
        self.declare_parameter('motor.Rs', 0.5)
        self.declare_parameter('motor.Ld', 1.4e-3)
        self.declare_parameter('motor.Lq', 1.4e-3)
        self.declare_parameter('motor.flux_pm', 0.0577)
        self.declare_parameter('motor.pole_pairs', 4)
        self.declare_parameter('controller.Kp_id', 5.0)
        self.declare_parameter('controller.Ki_id', 1000.0)
        self.declare_parameter('controller.Kp_iq', 5.0)
        self.declare_parameter('controller.Ki_iq', 1000.0)
        self.declare_parameter('controller.Kp_speed', 0.5)
        self.declare_parameter('controller.Ki_speed', 10.0)
        self.declare_parameter('controller.iq_max', 10.0)
        self.declare_parameter('controller.id_max', 10.0)
        self.declare_parameter('inverter.Vdc', 24.0)
        self.declare_parameter('inverter.fsw', 20000.0)

    def _build_config(self):
        config = FOCConfig()
        config.Rs = self.get_parameter('motor.Rs').value
        config.Ld = self.get_parameter('motor.Ld').value
        config.Lq = self.get_parameter('motor.Lq').value
        config.flux_pm = self.get_parameter('motor.flux_pm').value
        config.pole_pairs = self.get_parameter('motor.pole_pairs').value
        config.Kp_id = self.get_parameter('controller.Kp_id').value
        config.Ki_id = self.get_parameter('controller.Ki_id').value
        config.Kp_iq = self.get_parameter('controller.Kp_iq').value
        config.Ki_iq = self.get_parameter('controller.Ki_iq').value
        config.Kp_speed = self.get_parameter('controller.Kp_speed').value
        config.Ki_speed = self.get_parameter('controller.Ki_speed').value
        config.iq_max = self.get_parameter('controller.iq_max').value
        config.id_max = self.get_parameter('controller.id_max').value
        config.Vdc = self.get_parameter('inverter.Vdc').value
        fsw = self.get_parameter('inverter.fsw').value
        config.Ts = 1.0 / fsw
        return config

    def _feedback_callback(self, msg):
        """
        Process motor feedback and compute PWM output.
        Expected data: [ia, ib, ic, theta_e, omega_m]
        """
        if len(msg.data) < 5:
            self.get_logger().warn('Invalid feedback message length')
            return

        ia = msg.data[0]
        ib = msg.data[1]
        ic = msg.data[2]
        theta_e = msg.data[3]
        omega_m = msg.data[4]

        # Run FOC controller step
        output = self.controller.step(
            ia, ib, ic, theta_e, omega_m,
            self.speed_ref, self.id_ref
        )

        # Publish PWM duty cycles
        pwm_msg = Float64MultiArray()
        pwm_msg.data = [output.duty_a, output.duty_b, output.duty_c]
        self.pwm_pub.publish(pwm_msg)

        # Publish status
        status_msg = Float64MultiArray()
        status_msg.data = [
            output.id_meas, output.iq_meas,
            output.vd, output.vq,
            output.iq_ref, self.speed_ref, omega_m
        ]
        self.status_pub.publish(status_msg)

    def _speed_ref_callback(self, msg):
        self.speed_ref = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = PMSMControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
