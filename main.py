import interactive_animation
from robot_arm_gui_app import Robot_Arm_Gui
from multiprocessing import Process
import matplotlib.pyplot as plt


def func1():
    interactive_animation.run()


def func2():
    robot = Robot_Arm_Gui()
    robot.update_arm(0.0, 0.0, 0.0)
    robot.setup_gui()
    plt.show()


if __name__ == '__main__':
    p1 = Process(target=func1)
    p1.start()
    p2 = Process(target=func2)
    p2.start()
    p1.join()
    p2.join()