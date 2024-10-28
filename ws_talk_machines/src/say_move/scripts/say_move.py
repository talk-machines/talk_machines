#!/usr/bin/env python

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, QPushButton, QLineEdit
import re
from openai import OpenAI

import sys

sys.path.insert(0, "/home/research/Desktop/ur5_drl_ammar/ws_ur_openai_ros_sim/src/ur5/ur_control/src")
sys.path.insert(0, "ur5/ur_control/src/")

import numpy as np
import gymnasium as gym
import argparse
import os
import copy
from pathlib import Path
from datetime import datetime

import rospy
from std_msgs.msg import String
from ur_openai.common import load_environment, log_ros_params, clear_gym_params, load_ros_params

from gymnasium.wrappers import TimeAwareObservation, NormalizeObservation, NormalizeReward, RecordEpisodeStatistics, \
    FlattenObservation
        
            
class ChatWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('TalkMachines')
        self.setGeometry(100, 100, 400, 400)

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        self.chat_display = QTextEdit()
        self.chat_display.setReadOnly(True)
        self.layout.addWidget(self.chat_display)

        self.input_layout = QHBoxLayout()
        self.input_text = QLineEdit()
        self.input_text.returnPressed.connect(self.send_message)
        self.input_layout.addWidget(self.input_text)
        self.send_button = QPushButton('Send Command')
        self.send_button.clicked.connect(self.send_message)
        self.input_layout.addWidget(self.send_button)
        self.layout.addLayout(self.input_layout)

        self.client = OpenAI(api_key='sk-proj-drkRmfXKuf8kWxfOAB0HT3BlbkFJ3puFwMzVJAgliM1Qayd4')

        ros_param_path = load_ros_params(rospackage_name="ur_rl",
                                         rel_path_from_package_to_file="config",
                                         yaml_file_name="simulation/task_space_pick_and_place.yaml")

        max_episode_steps = rospy.get_param("/ur_gym/rl/steps_per_episode", 200)
        self.env = load_environment(rospy.get_param('ur_gym/env_id'),
                                    max_episode_steps=max_episode_steps).unwrapped
        self.env.reset()
            
    def send_message(self):
        user_input = self.input_text.text()
        if user_input.strip():  # Check if input is not empty or whitespace
                self.input_text.clear()
                self.add_to_chat_display(f'<b>Operator:</b> {user_input}')
                command = self.parse_command(user_input)        
                self.add_to_chat_display(f'<b>Control Pattern:</b> {command}')
                system_response = self.generate_response(command)
                self.add_to_chat_display(f'<b>Robot:</b> {system_response}')
                self.add_to_chat_display('(The robot is moving...)')
                self.move_robot()
                self.add_to_chat_display('\n')

    def add_to_chat_display(self, message):
        self.chat_display.append(message)

    def parse_command(self, response):        
        pattern_string =  response

        pattern_string = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": 'You are a robot arm manipulation control pattern expert. Your job is to give an end effector position control pattern based on the input. You will always give the output in the correct format no matter what the input is. The following are description about robot movements: 1. Moving left or right is represented as moving in the positive or negative X direction for 1cm or -1cm, respectively. 2. Moving forward or backward is represented as moving in the positive or negative Y direction for 1cm or -1cm, respectively. 3. Moving up or down is represented as moving in the positive or negative Z direction for 1cm or -1cm, respectively. The following are rules for describing the robot movement patterns: 1. You should output the movement patterns in X, Y, and Z format. 2. There are three values to choose from for each of the axis: [-1, 0, 1], which represents movement along that axis for 1cm. 3. A pattern has 3 lines, each of which represents the robot movement pattern of the end effector. 4. Each line has a label. "X" is the movement in left or right direction, "Y" is the movement in forward or backward direction, and "Z" is the movement in up or down direction. 5. In each line, "0" represents no movement in that direction, "1" represents positive movement in that direction, and "-1" represents negative movement in that direction.'},
                {"role": "user", "content": "Input: Move forward"},
                {"role": "assistant", "content": "X: 00000000000000000000000000... Y: 11111111111111111111111111... Z: 00000000000000000000000000..."},
                {"role": "user", "content": "Input: Move backward"},
                {"role": "assistant", "content": "X: 00000000000000000000000000... Y: -1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1... Z: 00000000000000000000000000..."},
                {"role": "user", "content": "Input: Move right"},
                {"role": "assistant", "content": "X: 11111111111111111111111111... Y: 00000000000000000000000000... Z: 00000000000000000000000000..."},
                {"role": "user", "content": response},
                ],
            temperature=0.,
            ).choices[0].message.content

        # Define the pattern to search for
        X = r'X: -?\d+(?:-?\d+)*\b'
        Y = r'Y: -?\d+(?:-?\d+)*\b'
        Z = r'Z: -?\d+(?:-?\d+)*\b'

        # Use regular expression to find the pattern
        x = re.search(X, pattern_string).group(0)
        y = re.search(Y, pattern_string).group(0)
        z = re.search(Z, pattern_string).group(0)

        # Extracting the part containing numbers
        x_values = x.split(': ')[1]
        y_values = y.split(': ')[1]
        z_values = z.split(': ')[1]

        # Converting the string to a list of numbers
        x_coords = []
        prev_char = '0'
        for char in x_values:
            if char == '-':
                if not prev_char == '-': x_coords.append(-1) 
            elif char.isdigit():
                if not prev_char == '-': x_coords.append(int(char))
            prev_char = char

        y_coords = []
        prev_char = '0'
        for char in y_values:
            if char == '-':
                if not prev_char == '-': y_coords.append(-1) 
            elif char.isdigit():
                if not prev_char == '-': y_coords.append(int(char))
            prev_char = char

        z_coords = []
        prev_char = '0'
        for char in z_values:
            if char == '-':
                if not prev_char == '-': z_coords.append(-1) 
            elif char.isdigit():
                if not prev_char == '-': z_coords.append(int(char))
            prev_char = char

        self.coordinates = list(zip(x_coords, y_coords, z_coords))
        print(self.coordinates)
        return [x, y, z]

    def move_robot(self):
        self.env.talk_machines(self.coordinates, 1)


    def generate_response(self, command):
        response = self.client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": 'You are a robot arm manipulation control pattern expert. Your job is to give an end effector position control pattern based on the input. You will always give the output in the correct format no matter what the input is. The following are description about robot movements: 1. Moving left or right is represented as moving in the positive or negative X direction for 1cm or -1cm, respectively. 2. Moving forward or backward is represented as moving in the positive or negative Y direction for 1cm or -1cm, respectively. 3. Moving up or down is represented as moving in the positive or negative Z direction for 1cm or -1cm, respectively. The following are rules for describing the robot movement patterns: 1. You should output the movement patterns in X, Y, and Z format. 2. There are three values to choose from for each of the axis: [-1, 0, 1], which represents movement along that axis for 1cm. 3. A pattern has 3 lines, each of which represents the robot movement pattern of the end effector. 4. Each line has a label. "X" is the movement in left or right direction, "Y" is the movement in forward or backward direction, and "Z" is the movement in up or down direction. 5. In each line, "0" represents no movement in that direction, "1" represents positive movement in that direction, and "-1" represents negative movement in that direction.'},
                {"role": "user", "content": "X: 00000000000000000000000000... Y: 11111111111111111111111111... Z: 00000000000000000000000000..."},
                {"role": "assistant", "content": "Robot is moving forward for 25cm"},
                {"role": "user", "content": "X: 00000000000000000000000000... Y: -1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1... Z: 00000000000000000000000000..."},
                {"role": "assistant", "content": "Robot is moving backward by 25cm"},
                {"role": "user", "content": "X: 11111111111111111111111111... Y: 00000000000000000000000000... Z: 00000000000000000000000000..."},
                {"role": "assistant", "content": "Robot is moving to the right"},
                {"role": "user", "content": str(command) + "explain what is the robot's movement pattern"},
                ],
            temperature=0.,
            ).choices[0].message.content
        return response


if __name__ == '__main__':
    rospy.init_node('say_move_ur5',
                    anonymous=True, log_level=rospy.WARN)
    app = QApplication(sys.argv)
    window = ChatWindow()
    window.show()
    sys.exit(app.exec_())

