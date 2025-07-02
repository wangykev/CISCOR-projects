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

        # ROS 2 Publishers
        self.speed_pub = self.create_publisher(Float32, 'cfw11/actual_rpm', 10)
        self.status_pub = self.create_publisher(Float32, 'cfw11/status', 10)

        # Timer to poll CFW-11 parameters
        self.create_timer(0.1, self.read_status_and_rpm)

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

    def read_status_and_rpm(self):
        # P0002: Read current RPM
        res_rpm = self.client.read_holding_registers(address=2, count=1, slave=1)
        if not res_rpm.isError():
            rpm = float(res_rpm.registers[0])
            #self.get_logger().info(f"P0002 RPM: {rpm:.2f}")
            self.speed_pub.publish(Float32(data=rpm))

        # P0006: Read current status word
        res_status = self.client.read_holding_registers(address=6, count=1, slave=1)
        if not res_status.isError():
            status = float(res_status.registers[0])
            #self.get_logger().info(f"P0006 Status: {status}")
            self.status_pub.publish(Float32(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = CFW11Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
