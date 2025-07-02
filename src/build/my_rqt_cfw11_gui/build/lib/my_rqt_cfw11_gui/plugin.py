from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout, QLabel, QSlider
from std_msgs.msg import Float32, Bool
import rclpy
from rclpy.node import Node

class CFW11GUIPlugin(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('CFW11GUIPlugin')

        # QWidget setup
        self._widget = QWidget()
        self._layout = QVBoxLayout()

        self.start_button = QPushButton('Start')
        self.stop_button = QPushButton('Stop')
        self.speed_slider = QSlider()
        self.speed_slider.setOrientation(1)  # Horizontal
        self.speed_slider.setRange(0, 60)
        self.speed_slider.setValue(20)

        self.speed_label = QLabel('Speed: 20 RPM')
        self.actual_rpm_label = QLabel('Current Speed: -- RPM')

        self._layout.addWidget(self.speed_label)
        self._layout.addWidget(self.speed_slider)
        self._layout.addWidget(self.start_button)
        self._layout.addWidget(self.stop_button)
        self._layout.addWidget(self.actual_rpm_label)
        self._widget.setLayout(self._layout)
        context.add_widget(self._widget)

        # Init ROS node (singleton-style for rqt)
        rclpy.init(args=None)
        self.node = rclpy.create_node('cfw11_gui_rqt')

        # Publishers
        self.rpm_pub = self.node.create_publisher(Float32, 'cfw11/set_rpm', 10)
        self.run_pub = self.node.create_publisher(Bool, 'cfw11/run', 10)

        # Subscriber
        self.actual_rpm_sub = self.node.create_subscription(Float32, 'cfw11/actual_rpm', self.actual_rpm_callback, 10)

        # Connect events
        self.start_button.clicked.connect(self.on_start)
        self.stop_button.clicked.connect(self.on_stop)
        self.speed_slider.valueChanged.connect(self.on_speed_change)

        # Timer to spin ROS
        from python_qt_binding.QtCore import QTimer
        self._ros_timer = QTimer()
        self._ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.01))
        self._ros_timer.start(10)

    def on_start(self):
        self.run_pub.publish(Bool(data=True))

    def on_stop(self):
        self.run_pub.publish(Bool(data=False))

    def on_speed_change(self, value):
        rpm = float(value)
        self.speed_label.setText(f'Speed: {rpm:.1f} RPM')
        self.rpm_pub.publish(Float32(data=rpm))

    def actual_rpm_callback(self, msg):
        rpm = msg.data
        self.actual_rpm_label.setText(f'Current Speed: {rpm:.1f} RPM')
