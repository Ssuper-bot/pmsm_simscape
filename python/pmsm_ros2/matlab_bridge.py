"""
ROS2-MATLAB Bridge Node.

Bridges between ROS2 topics and MATLAB/Simulink via:
  1. MATLAB Engine API for Python (direct call)
  2. Shared memory / file-based data exchange
  3. ROS Toolbox in Simulink (recommended for real-time co-simulation)

For Simscape co-simulation, the recommended approach is:
  - Use MATLAB's ROS Toolbox blocks in Simulink to subscribe/publish ROS2 topics
  - This bridge node handles format conversion and data routing
"""

import json
import struct
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class MatlabBridgeNode(Node):
    """
    Bridge node connecting ROS2 to MATLAB/Simulink.

    Approach 1 (Default): File-based data exchange
      - Writes ROS2 data to a shared JSON file
      - MATLAB reads the file in its control loop
      - Simple, works across processes

    Approach 2 (Optional): MATLAB Engine API
      - Requires matlab.engine Python package
      - Direct function calls into MATLAB workspace
    """

    def __init__(self):
        super().__init__('matlab_bridge')

        self.declare_parameter('bridge_mode', 'file')  # 'file' or 'engine'
        self.declare_parameter('data_dir', '/tmp/pmsm_bridge')
        self.declare_parameter('matlab_model', 'pmsm_foc_model')

        self.bridge_mode = self.get_parameter('bridge_mode').value
        self.data_dir = Path(self.get_parameter('data_dir').value)
        self.data_dir.mkdir(parents=True, exist_ok=True)

        self.matlab_engine = None
        if self.bridge_mode == 'engine':
            self._init_matlab_engine()

        # Subscribe to controller outputs (from foc_node)
        self.pwm_sub = self.create_subscription(
            Float64MultiArray,
            '/pmsm/pwm_duty',
            self._pwm_callback,
            10
        )

        # Subscribe to status
        self.status_sub = self.create_subscription(
            Float64MultiArray,
            '/pmsm/status',
            self._status_callback,
            10
        )

        # Publish simulated motor feedback (from MATLAB)
        self.feedback_pub = self.create_publisher(
            Float64MultiArray,
            '/pmsm/feedback',
            10
        )

        # Timer for reading MATLAB output
        self.create_timer(0.001, self._read_matlab_output)  # 1kHz

        self.get_logger().info(
            f'MATLAB Bridge started (mode={self.bridge_mode}, dir={self.data_dir})')

    def _init_matlab_engine(self):
        try:
            import matlab.engine
            self.get_logger().info('Starting MATLAB engine...')
            self.matlab_engine = matlab.engine.start_matlab()
            self.get_logger().info('MATLAB engine started.')
        except ImportError:
            self.get_logger().error(
                'matlab.engine not available. Install MATLAB Engine API for Python.')
            self.bridge_mode = 'file'

    def _pwm_callback(self, msg):
        """Forward PWM data to MATLAB."""
        data = {
            'timestamp': time.time(),
            'duty_a': msg.data[0],
            'duty_b': msg.data[1],
            'duty_c': msg.data[2],
        }

        if self.bridge_mode == 'file':
            output_path = self.data_dir / 'pwm_input.json'
            output_path.write_text(json.dumps(data))
        elif self.bridge_mode == 'engine' and self.matlab_engine:
            self.matlab_engine.workspace['pwm_duty'] = list(msg.data)

    def _status_callback(self, msg):
        """Forward status data to MATLAB."""
        data = {
            'timestamp': time.time(),
            'values': list(msg.data),
        }
        if self.bridge_mode == 'file':
            output_path = self.data_dir / 'status.json'
            output_path.write_text(json.dumps(data))

    def _read_matlab_output(self):
        """Read motor feedback from MATLAB simulation."""
        if self.bridge_mode == 'file':
            feedback_path = self.data_dir / 'motor_feedback.json'
            if feedback_path.exists():
                try:
                    data = json.loads(feedback_path.read_text())
                    msg = Float64MultiArray()
                    msg.data = [
                        data.get('ia', 0.0),
                        data.get('ib', 0.0),
                        data.get('ic', 0.0),
                        data.get('theta_e', 0.0),
                        data.get('omega_m', 0.0),
                    ]
                    self.feedback_pub.publish(msg)
                except (json.JSONDecodeError, KeyError):
                    pass
        elif self.bridge_mode == 'engine' and self.matlab_engine:
            try:
                feedback = self.matlab_engine.eval(
                    "motor_feedback", nargout=1)
                if feedback:
                    msg = Float64MultiArray()
                    msg.data = list(feedback)
                    self.feedback_pub.publish(msg)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = MatlabBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.matlab_engine:
            node.matlab_engine.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
