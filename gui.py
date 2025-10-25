import sys 
from PyQt6 import QtCore, QtWidgets
from shared_state import SharedState

class GUI:
    
    def __init__(self, a_shared_state : SharedState):
        self._shared_state = a_shared_state
        self._state_data = {}
        self._data_labels = ["position", "velocity", "orientation", "ang_vel"]
        self._axis_labels = ["x", "y", "z"]

        self.app = QtWidgets.QApplication(sys.argv)
        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle("Robot GUI")
        self.window.setStyleSheet("background-color: #2b2b2b; color: white;")

        central_widget = QtWidgets.QWidget()
        self.window.setCentralWidget(central_widget)
        self.layout = QtWidgets.QVBoxLayout(central_widget)
        
        self.test_label = QtWidgets.QLabel()
        self.test_label.setText("NaN")
        self.layout.addWidget(self.test_label)
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)
        
        self.plots = {}
        self.plot_titles = ["Position", "Velocity", "Orientation", "Ang. Velocity"]

        for i, label in enumerate(self._data_labels):
            self._state_data[label] = {axis: [] for axis in self._axis_labels}
            self.plots[label] = LivePlot(self.plot_titles[i])
            self.layout.addWidget(self.plots[label])
    
    def run(self):
        
        self.window.resize(1200, 800)
        self.window.show()
        self.app.exec()
        
    def update(self):
        new_state = self._shared_state.get_state()
        
        for label in self._data_labels:
            new = new_state[label]
            
            for i, axis in enumerate(self._axis_labels):
                self._state_data[label][axis].append(new[i])

            self.plots[label].update_plot(self._state_data[label])
            
import pyqtgraph as pg 
            
class LivePlot(QtWidgets.QWidget):
    
    def __init__(self, title, color='cyan'):
        super().__init__()
        
        layout = QtWidgets.QVBoxLayout() # vertical box layout
        self.plot_widget = pg.PlotWidget(title=title)
        
        self.plot_widget.setBackground((30, 30, 30))
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setTitle(title, color='white', size='11pt')
        self.plot_widget.getAxis('left').setTextPen('white')
        self.plot_widget.getAxis('bottom').setTextPen('white')

        self.curve_x = self.plot_widget.plot(pen=pg.mkPen(color='r', width=2))
        self.curve_y = self.plot_widget.plot(pen=pg.mkPen(color='g', width=2))
        self.curve_z = self.plot_widget.plot(pen=pg.mkPen(color='b', width=2))
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)
        
    def update_plot(self, data):
        if data:
            x_vals = list(range(len(data["x"])))
            self.curve_x.setData(x_vals, data["x"])
            self.curve_y.setData(x_vals, data["y"])
            self.curve_z.setData(x_vals, data["z"])

            
        
        
    