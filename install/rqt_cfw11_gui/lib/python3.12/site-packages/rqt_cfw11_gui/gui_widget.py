from python_qt_binding.QtWidgets import QDialog
from python_qt_binding import loadUi
from std_msgs.msg import Float32, Bool
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os



class CFW11GUI(QDialog):
    def __init__(self, node):
        super().__init__()
        self.node = node

        ui_file = Path(
            get_package_share_directory('rqt_cfw11_gui')
        ) / 'resource' / 'MyPlugin.ui'
        loadUi(str(ui_file), self)

        self.rpm_pub = node.create_publisher(Float32, 'cfw11/set_rpm', 10)
        self.run_pub = node.create_publisher(Bool, 'cfw11/run', 10)

        self.horizontalSlider.setMinimum(0)
        self.horizontalSlider.setMaximum(60)

        node.create_subscription(Float32, 'cfw11/actual_rpm', self.update_actual_rpm, 10)

        self.horizontalSlider.valueChanged.connect(self.on_rpm_change)
        self.pushButton_2.setCheckable(True)
        self.pushButton_2.toggled.connect(self.on_run_toggle)

    def on_rpm_change(self, value):
        self.lineEdit.setText(str(value))
        self.rpm_pub.publish(Float32(data=float(value)))

    def on_run_toggle(self, checked):
        self.run_pub.publish(Bool(data=checked))
        self.pushButton_2.setText("Stop" if checked else "Start")
        self.textBrowser.setText("Running" if checked else "Stopped")

    def update_actual_rpm(self, msg):
        self.lcdNumber.display(int(msg.data))
