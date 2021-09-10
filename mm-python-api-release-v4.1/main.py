#!/usr/bin/python
import sys
import time
import signal
from dataclasses import dataclass, field
from typing import List
from MachineMotion import *

sys.path.append("../..")

N_PODS = 3
TOTAL_AXES_MM = 3

# Configuration mapping - example for 2 pods
# IO number are zero indexed in this mapping
ip_address = ['192.168.7.5', '192.168.7.4']
dir_mapper = {'forward': -1, 'back': -1}

sensor = {'forward': ['mm1_io1_pin0', 'mm1_io2_pin0', 'mm1_io3_pin0'],
          'back': ['mm1_io1_pin1', 'mm1_io2_pin1', 'mm1_io3_pin1']}

gate = {'forward': ['mm1_io1_pin01', 'mm1_io2_pin01', 'mm1_io3_pin01'],
        'back': ['mm1_io1_pin23', 'mm1_io2_pin23', 'mm1_io3_pin23']}

servo = {'forward': ['mm1_drive2', 'mm2_drive1', 'mm2_drive3'],
         'back': ['mm1_drive1', 'mm1_drive3', 'mm2_drive2']}

# Kinematics parameters
CRUISE_SPEED = 600
CRUISE_ACC = 10000
FINE_SPEED = 200
FINE_ACC = 10000
STOP_ACC = 1000


class ConveyorMotion:
    def __init__(self):
        self.mmList = []
        self.initialise_machine_motion()
        self.stop_all_conveyors()
        self.release_estop()

    def cleanup(self, *args):
        for pod in range(N_PODS):
            for direction in ['forward', 'back']:
                self.stop_conveyor(pod + 1, direction)
        print('Exiting, stopping conveyor')
        sys.exit(0)

    def stop_all_conveyors(self):
        for pod in range(N_PODS):
            for direction in ['forward', 'back']:
                self.stop_conveyor(pod + 1, direction)
        print('Exiting, stopping conveyor')

    def initialise_machine_motion(self):
        # Create Machine Motion instance
        for count, ip in enumerate(ip_address):
            self.mmList.append(MachineMotion(machineIp=ip))
            # Configure all available axes
            for axis in range(TOTAL_AXES_MM):
                self.mmList[count].configAxis(axis + 1, MICRO_STEPS.ustep_8, MECH_GAIN.roller_conveyor_mm_turn)
                print(f'Axis {axis + 1} of machine {count + 1} configured correctly!')

    def release_estop(self):
        for count, _ in enumerate(self.mmList):
            print(count)
            # When starting a program, one must remove the software stop before moving
            print('--> Removing software stop')
            self.mmList[count].releaseEstop()
            print('--> Resetting system')
            self.mmList[count].resetSystem()
        print('Resetting completed!')

    def read_sensor(self, pod, direction):
        """
        :param pod: int, number of pod [1->N_PODS]
        :param direction: string, ['forward', 'back']
        :return:
        """
        self.check_pod_number_validity(pod)

        # self.check_pod_number_validity(pod)
        label = sensor[direction][pod - 1]  # Extract mapping
        label_ids = [int(s) for s in label if s.isdigit()]

        # Check validity of inputs
        self.check_machine_number_validity(label_ids[0] - 1)
        self.check_input_number_validity(label_ids[1])

        # Detect all connected digital IO Modules
        detected_io_modules = self.mmList[label_ids[0] - 1].detectIOModules()

        # Toggles the output pins
        if detected_io_modules is not None:
            pin_value = self.mmList[label_ids[0] - 1].digitalRead(deviceNetworkId=label_ids[1],
                                                                  pin=label_ids[2])

            return pin_value

    def close_gate(self, pod, direction, action):
        """
        :param pod: int, number of pod [1->N_PODS]
        :param direction: string, ['forward', 'back']
        :param action: string, ['open', 'close']
        :return:
        """
        # self.check_pod_number_validity(pod)
        label = gate[direction][pod - 1]  # Extract mapping
        label_ids = [int(s) for s in label if s.isdigit()]

        # Check validity of inputs
        self.check_machine_number_validity(label_ids[0] - 1)
        self.check_input_number_validity(label_ids[2])
        self.check_input_number_validity(label_ids[3])

        # Detect all connected digital IO Modules
        detected_io_modules = self.mmList[label_ids[0] - 1].detectIOModules()

        # Toggles the output pins
        if detected_io_modules is not None:
            if action == 'close':
                self.mmList[label_ids[0] - 1].digitalWrite(deviceNetworkId=int(label_ids[1]), pin=label_ids[2], value=0)
                time.sleep(0.1)
                self.mmList[label_ids[0] - 1].digitalWrite(deviceNetworkId=int(label_ids[1]), pin=label_ids[3], value=1)
            elif action == 'open':
                self.mmList[label_ids[0] - 1].digitalWrite(deviceNetworkId=int(label_ids[1]), pin=label_ids[2], value=1)
                time.sleep(0.1)
                self.mmList[label_ids[0] - 1].digitalWrite(deviceNetworkId=int(label_ids[1]), pin=label_ids[3], value=0)
            else:
                print('Action unknown')
                exit(1)
        time.sleep(0.1)

    def move_conveyor(self, pod, direction, duration=None, speed=CRUISE_SPEED, acc=CRUISE_ACC):
        """
        :param pod: int, number of pod [1->N_PODS]
        :param direction: string, ['forward', 'back']
        :param duration: int/float, duration of movement in seconds
        :param speed: int/float, linear speed of belt, in [mm/s]
        :param acc: int/float, linear acceleration/deceleration of belt, in [mm/s^2]
        :return:
        """
        self.check_pod_number_validity(pod)

        label = servo[direction][pod - 1]  # Extract mapping
        label_ids = [int(s) for s in label if s.isdigit()]

        # Check validity of inputs
        self.check_machine_number_validity(label_ids[0] - 1)
        self.check_axis_number_validity(label_ids[1])

        # Move conveyor
        self.mmList[label_ids[0] - 1].setContinuousMove(label_ids[1], dir_mapper[direction]*speed, acc)
        # if duration is not None:
        #     try:
        #         duration = int(duration)
        #         time.sleep(duration)
        #         # Stop conveyor
        #         self.mmList[label_ids[0] - 1].stopContinuousMove(label_ids[1], acc)
        #     except ValueError:
        #         try:
        #             # Convert it into float
        #             duration = float(duration)
        #             time.sleep(duration)
        #             # Stop conveyor
        #             self.mmList[label_ids[0] - 1].stopContinuousMove(label_ids[1], acc)
        #         except ValueError:
        #             print("Duration input is not a number.")

    def stop_conveyor(self, pod, direction, acc=STOP_ACC):
        """
        :param pod: int, number of pod [1->N_PODS]
        :param direction: string, ['forward', 'back']
        :param acc: int/float, linear deceleration of belt, in [mm/s^2]
        :return:
        """
        self.check_pod_number_validity(pod)

        label = servo[direction][pod - 1]  # Extract mapping
        label_ids = [int(s) for s in label if s.isdigit()]
        print('label is', label)
        print('label indexes are', label_ids)
        print('machine', label_ids[0], '@', self.mmList[label_ids[0] - 1], 'drive', label_ids[1])

        # Check provided motor id
        self.check_machine_number_validity(label_ids[0] - 1)
        self.check_axis_number_validity(label_ids[1])

        # Stop conveyor
        self.mmList[label_ids[0] - 1].stopContinuousMove(label_ids[1], acc)

    @staticmethod
    def check_axis_number_validity(axis):
        # Axis number is not zero indexed
        if axis not in [1, 2, 3] and isinstance(axis, int):
            print('Axis number out of range. Permitted values are [1, 2, 3].')
            exit(1)

    @staticmethod
    def check_io_board_number_validity(io_board):
        # IO board number is zero indexed
        if io_board not in range(4) and isinstance(io_board, int):
            print('IO board number out of range. Permitted values are [0, 1, 2].')
            exit(1)

    @staticmethod
    def check_input_number_validity(input_pin):
        # Input pin number is zero indexed
        if input_pin not in range(4) and isinstance(input_pin, int):
            print('Input pin number out of range. Permitted values are [0, 1, 2, 3].')
            exit(1)

    @staticmethod
    def check_pod_number_validity(pod):
        # Pod number is not zero indexed
        if pod - 1 not in range(N_PODS) and isinstance(pod, int):
            print('Pods number out of range. Check the number of pods defined.')
            exit(1)

    def check_machine_number_validity(self, machine):
        # Machine number is zero indexed
        if machine not in range(len(self.mmList)) and isinstance(machine, int):
            print('Machine number out of range. Check the number of Machine Motion machines connected.')
            exit(1)

    def move_all_gates(self, action):
        """
        :param action: string, ['open', 'close']
        :return:
        """
        for pod in range(0, N_PODS):
            for direction in ['back', 'forward']:
                print('moving pod', pod + 1, 'direction', direction, 'action', action)
                self.close_gate(pod + 1, direction, action)

    def move_between_pods(self, pod_initial, pod_final):
        self.check_pod_number_validity(pod_initial)
        self.check_pod_number_validity(pod_final)

        # Sort pod numbers for range looping later on
        pod_list = [pod_initial, pod_final]
        pod_list.sort()

        if pod_initial == pod_final:
            print('No movement to be performed, pod numbers are the same!')
            return
        direction = 'forward' if pod_final > pod_initial else 'back'

        # Close all gates
        self.move_all_gates('close')

        # Open gates for movement
        for pod in range(pod_list[0], pod_list[1] + 1):
            self.close_gate(pod=pod, direction=direction, action='open')

        # Start conveyors
        for pod in range(pod_list[0], pod_list[1] + 1):
            self.move_conveyor(pod=pod, direction=direction)

        # Listen for sensor
        while True:
            value = self.read_sensor(pod=pod_final, direction=direction)
            if not value:
                print('Found obstacle')
                break

        # Close final gates
        self.close_gate(pod=pod_final, direction=direction, action='close')
        self.move_conveyor(pod=pod_final, direction=direction, speed=FINE_SPEED, acc=FINE_ACC, duration=5)
        for pod in range(pod_list[0], pod_list[1] + 1):
            self.move_conveyor(pod=pod, direction=direction, speed=FINE_SPEED, acc=FINE_ACC, duration=5)
        time.sleep(3)
        for pod in range(pod_list[0], pod_list[1] + 1):
            self.stop_conveyor(pod=pod, direction=direction)


conveyor = ConveyorMotion()
conveyor.move_between_pods(3, 1)
