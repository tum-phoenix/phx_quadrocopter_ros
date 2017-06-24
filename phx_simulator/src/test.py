import numpy as np
from simulator.drone import Drone
from simulator.controller import Controller
from simulator.simulator import Simulator

if __name__ == '__main__':

    drone = Drone()
    controller = Controller(drone)
    # Simulate some disturbance in the angular velocity.
    angular_disturbance = np.array([[0.0], [0.0], [0.0]])
    drone.thetadot = angular_disturbance
    simulator = Simulator(drone, controller)
    simulator.simulate(30)  # simulate n secs
