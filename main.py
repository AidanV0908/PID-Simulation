'''
Model a spring-mass system controlled by a PID controller.
The user inputs the PID constants, reference position, and initial state.
Program simulates the system's response over time and plots the results.
System starts from rest at position 0.
'''
import matplotlib.pyplot as plt
# Use QT for Python
from PySide6 import QtCore, QtWidgets, QtGui
import sys

class PIDController:
    def __init__(self, kp, ki, kd, r):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = r

    # Computes the next control input
    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        u = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return u
    
class FloatSpinBox(QtWidgets.QDoubleSpinBox):
    def __init__(self, parent=None,
                 minimum=-1e6,
                 maximum=1e6,
                 decimals=4,
                 step=1):
        super().__init__(parent)
        self.setRange(minimum, maximum)
        self.setDecimals(decimals)
        self.setSingleStep(step)
        self.setKeyboardTracking(False)
    
class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Spring-Mass System with PID Control")

        layout = QtWidgets.QVBoxLayout(self)

        # Optional image
        image_label = QtWidgets.QLabel()
        pixmap = QtGui.QPixmap("system.jpg")
        if not pixmap.isNull():
            image_label.setPixmap(pixmap)
            layout.addWidget(image_label)

        '''
        SYSTEM PARAMETERS INPUT
        '''
        QLabel_mass = QtWidgets.QLabel("Mass (kg):")
        layout.addWidget(QLabel_mass)
        self.mass_input = FloatSpinBox()
        layout.addWidget(self.mass_input)

        QLabel_k = QtWidgets.QLabel("Spring Constant k (N/m):")
        layout.addWidget(QLabel_k)
        self.k_input = FloatSpinBox()
        layout.addWidget(self.k_input)

        QLabel_c = QtWidgets.QLabel("Damping Constant c (Ns/m):")
        layout.addWidget(QLabel_c)
        self.c_input = FloatSpinBox()
        layout.addWidget(self.c_input)

        '''
        PID PARAMETERS INPUT
        '''
        QLabel_kp = QtWidgets.QLabel("PID Proportional Gain Kp:")
        layout.addWidget(QLabel_kp)
        self.kp_input = FloatSpinBox()
        layout.addWidget(self.kp_input)

        QLabel_ki = QtWidgets.QLabel("PID Integral Gain Ki:")
        layout.addWidget(QLabel_ki)
        self.ki_input = FloatSpinBox()
        layout.addWidget(self.ki_input)

        QLabel_kd = QtWidgets.QLabel("PID Derivative Gain Kd:")
        layout.addWidget(QLabel_kd)
        self.kd_input = FloatSpinBox()
        layout.addWidget(self.kd_input)

        '''
        REFERENCE POSITION
        '''
        QLabel_r = QtWidgets.QLabel("Reference Position r (m):")
        layout.addWidget(QLabel_r)
        self.r_input = FloatSpinBox()
        layout.addWidget(self.r_input)

        '''
        SIMULATION INPUTS
        '''
        QLabel_sim_time = QtWidgets.QLabel("Simulation Time (s):")
        layout.addWidget(QLabel_sim_time)
        self.sim_time_input = FloatSpinBox()
        layout.addWidget(self.sim_time_input)

        QLabel_dt = QtWidgets.QLabel("Time Step dt (s):")
        layout.addWidget(QLabel_dt)
        self.dt_input = FloatSpinBox()
        layout.addWidget(self.dt_input)
        
        # Submit button
        run_button = QtWidgets.QPushButton("Run Simulation")
        run_button.clicked.connect(self.on_run_clicked)
        layout.addWidget(run_button)

    def on_run_clicked(self):
        try:
            mass = float(self.mass_input.text())
            k = float(self.k_input.text())
            c = float(self.c_input.text())
            kp = float(self.kp_input.text())
            ki = float(self.ki_input.text())
            kd = float(self.kd_input.text())
            r = float(self.r_input.text())
            sim_time_text = float(self.sim_time_input.text())
            dt_text = float(self.dt_input.text())
        except ValueError:
            QtWidgets.QMessageBox.critical(self, "Input error", "All fields must be valid numbers.")
            return

        if mass <= 0 or k <= 0 or c < 0:
            QtWidgets.QMessageBox.critical(self, "Input error", "Mass and k must be > 0, c must be >= 0.")
            return

        run_simulation(mass=mass, k=k, c=c, kp=kp, ki=ki, kd=kd, r=r)

def run_simulation(mass, k, c, kp, ki, kd, r):
    # simulation parameters
    sim_time = 10.0  # total simulation time in seconds
    dt = 0.01       # time step in second
    # calculate number of steps
    t = 0
    pos = 0
    vel = 0
    acc = 0
    # store simulation data for plotting
    output = [[t, pos, vel, acc, 0]]
    # define PID controller
    pid = PIDController(kp, ki, kd, r)
    # begin simulation
    while t <= sim_time:
        error = r - pos
        u = pid.compute(error, dt)
        # system dynamics
        acc = (u - c * vel - k * pos) / mass
        vel += acc * dt
        pos += vel * dt
        t += dt
        output.append([t, pos, vel, acc, u])
    # set the first control input to the second value to avoid zero initial spike
    output[0][4] = output[1][4]
    # plot results
    output = list(zip(*output))  # transpose for easy access
    plt.figure(figsize=(12, 8))
    plt.subplot(3, 1, 1)
    plt.plot(output[0], output[1], label='Position (m)')
    plt.axhline(y=r, color='r', linestyle='--', label='Reference Position')
    plt.title('Spring-Mass System Response with PID Control')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid()
    plt.subplot(3, 1, 2)
    plt.plot(output[0], output[2], label='Velocity (m/s)', color='g')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid()
    plt.subplot(3, 1, 3)
    plt.plot(output[0], output[4], label='Control Input (u)', color='m')
    plt.xlabel('Time (s)')
    plt.ylabel('Control Input (u)')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())