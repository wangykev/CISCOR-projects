import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from pymodbus.client import ModbusSerialClient
import glob

class CFW11Controller(Node):
    def __init__(self):
        super().__init__('cfw11_controller')

        # Find USB device dynamically
        port = self.find_serial_port()
        if not port:
            self.get_logger().error("No matching USB serial device found.")
            return

        # Set up Modbus RTU connection
        self.client = ModbusSerialClient(
            port=port,
            baudrate=9600,
            parity='E',
            stopbits=1,
            bytesize=8,
            timeout=1
        )
        if not self.client.connect():
            self.get_logger().error(f"Failed to connect to CFW-11 on {port}")
            return
        else:
            self.get_logger().info(f"Connected to CFW-11 on {port}")

        # ROS 2 Subscriptions
        self.create_subscription(Float32, 'cfw11/set_rpm', self.set_rpm_callback, 10)
        self.create_subscription(Bool, 'cfw11/run', self.run_callback, 10)

        # Timer to publish actual speed
        self.speed_pub = self.create_publisher(Float32, 'cfw11/actual_rpm', 10)
        self.create_timer(1.0, self.read_speed)

    def find_serial_port(self):
        for path in glob.glob('/dev/serial/by-id/*'):
            if '1a86' in path or 'QinHeng' in path or 'USB_Single_Serial' in path:
                return path
        return None

    def set_rpm_callback(self, msg):
        rpm = msg.data
        raw_value = int(rpm / 0.22)
        self.client.write_register(address=683, value=raw_value, slave=1)
        self.get_logger().info(f"Set speed to {rpm:.2f} RPM")

    def run_callback(self, msg):
        value = 3 if msg.data else 0
        self.client.write_register(address=682, value=value, slave=1)
        self.get_logger().info("Run command sent" if msg.data else "Stop command sent")

    def read_speed(self):
        res = self.client.read_holding_registers(address=2, count=1, slave=1)
        if not res.isError():
            rpm = res.registers[0] * 0.22
            self.speed_pub.publish(Float32(data=rpm))


def main(args=None):
    rclpy.init(args=args)
    node = CFW11Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
