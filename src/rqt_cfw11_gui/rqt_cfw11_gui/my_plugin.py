import rclpy
import threading
from rqt_gui_py.plugin import Plugin
from rclpy.executors import SingleThreadedExecutor
from .gui_widget import CFW11GUI
from std_msgs.msg import Bool  


class MyPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('MyPlugin')

        if not rclpy.ok():
            rclpy.init(args=None)


        self.executor = SingleThreadedExecutor()
        self.node = rclpy.create_node('cfw11_gui_node')
        self.executor.add_node(self.node)

        self._widget = CFW11GUI(self.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(f"{self._widget.windowTitle()} ({context.serial_number()})")

        context.add_widget(self._widget)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def shutdown_plugin(self):
        # Publish disable command on shutdown
        if self.node:
            run_pub = self.node.create_publisher(Bool, 'cfw11/run', 10)
            run_pub.publish(Bool(data=False))  # 🚨 Disable the motor
            self.node.get_logger().info("Sent /cfw11/run False on shutdown")

        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

        

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
