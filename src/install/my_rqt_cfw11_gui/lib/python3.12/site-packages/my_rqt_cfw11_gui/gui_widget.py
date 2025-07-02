from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import Float32, Bool
from pathlib import Path

class CFW11GUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # Load the .ui file
        ui_file = Path(__file__).parent / 'cfw11.ui'
        loadUi(str(ui_file), self)

        # ROS 2 publishers and subscribers
        self.rpm_pub = node.create_publisher(Float32, 'cfw11/set_rpm', 10)
        self.run_pub = node.create_publisher(Bool, 'cfw11/run', 10)
        node.create_subscription(Float32, 'cfw11/actual_rpm', self.update_actual_rpm, 10)

        # Setup slider
        self.horizontalSlider.setMinimum(0)
        self.horizontalSlider.setMaximum(60)
        self.horizontalSlider.setValue(0)
        self.horizontalSlider.valueChanged.connect(self.slider_changed)

        # Setup line edit to track RPM from slider
        self.lineEdit.setReadOnly(True)
        self.lineEdit.setText("0")

        # Start/Stop button (pushButton_2)
        self.pushButton_2.setCheckable(True)
        self.pushButton_2.toggled.connect(self.toggle_run)
        self.pushButton_2.setText("Start")

        # Status browser
        self.textBrowser.setText("Stopped")

        # Optional: disable "Enable" button for now
        self.pushButton.setEnabled(False)

    def slider_changed(self, value):
        self.lineEdit.setText(str(value))
        self.rpm_pub.publish(Float32(data=float(value)))

    def toggle_run(self, checked):
        msg = Bool(data=checked)
        self.run_pub.publish(msg)

        if checked:
            self.pushButton_2.setText("Stop")
            self.textBrowser.setText("Running")
        else:
            self.pushButton_2.setText("Start")
            self.textBrowser.setText("Stopped")

    def update_actual_rpm(self, msg):
        self.lcdNumber.display(int(msg.data))
