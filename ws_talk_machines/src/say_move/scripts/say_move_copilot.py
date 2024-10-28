#!/usr/bin/env python
import numpy as np
import gymnasium as gym
import sys
import re
import os

import rospy
from std_msgs.msg import String
from ur_openai.common import load_environment, log_ros_params, clear_gym_params, load_ros_params

from gymnasium.wrappers import TimeAwareObservation, NormalizeObservation, NormalizeReward, RecordEpisodeStatistics, \
    FlattenObservation

import asyncio
from sydney import SydneyClient


sys.path.insert(0, "/home/research/Desktop/ur5_drl_ammar/ws_ur_openai_ros_sim/src/ur5/ur_control/src")
sys.path.insert(0, "ur5/ur_control/src/")


def parse_command(response):
    try:
        pattern_string = response

        # Define the pattern to search for
        X = r'X:(?:\s*(-?1|0))+\s*(?![A-Za-z])'
        Y = r'Y:(?:\s*(-?1|0))+\s*(?![A-Za-z])'
        Z = r'Z:(?:\s*(-?1|0))+\s*(?![A-Za-z])'
        G = r'G:(?:\s*(-?1|0))+\s*(?![A-Za-z])'

        # Use regular expression to find the pattern
        x = re.search(X, pattern_string).group(0)
        y = re.search(Y, pattern_string).group(0)
        z = re.search(Z, pattern_string).group(0)
        g = re.search(G, pattern_string).group(0)

        # Extracting the part containing numbers
        x_values = x.split(': ')[1].replace(' ','')
        y_values = y.split(': ')[1].replace(' ','')
        z_values = z.split(': ')[1].replace(' ','')
        g_values = g.split(': ')[1].replace(' ','')

        # Find the maximum length among the strings
        max_length = max(len(s) for s in [x_values.replace('-', '').replace(' ', ''), y_values.replace('-', '').replace(' ', ''), z_values.replace('-', '').replace(' ', '')])
        lengths = [len(s) for s in [x_values.replace('-', '').replace(' ', ''), y_values.replace('-', '').replace(' ', ''), z_values.replace('-', '').replace(' ', '')]]
        print(lengths)

        # Pad the shorter strings with zeros
        x_values = x_values.ljust(x_values.count("-")+max_length, '0')
        y_values = y_values.ljust(y_values.count("-")+max_length, '0')
        z_values = z_values.ljust(z_values.count("-")+max_length, '0')
        print(g_values, max_length, len(g_values))
        g_values = '0' * (max_length-len(g_values)) + g_values if len(g_values) < max_length else g_values[-max_length-1:]

        print(x_values,y_values,z_values)
        
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

        g_val = []
        prev_char = '0'
        for char in g_values:
            if char.isdigit():
                g_val.append(int(char))
            prev_char = char

        coordinates = list(zip(x_coords, y_coords, z_coords))
        print(coordinates)
        print(g_val)
        return [x, y, z], coordinates, g_val
    except Exception as e:
        print("Error parsing generated pattern, please input again")
        print(str(e), str(X,Y,Z))
        return None, None


def move_robot(env, coordinates, g_val):
    env.talk_machines(coordinates, g_val, 1)
        

async def main() -> None:    
    async with SydneyClient(style="creative") as sydney:
        reset = True
        obstacle = rospy.get_param("/ur_gym/object_disturbance", False)

        while True: 
            if reset:
                # prompt = 'You are a UR5 robot control pattern manipulation expert. Your job is to give a full end effector position control pattern without spaces based on the input and not only the single values. There is only one trial allowed and the robot moves simultaneously in the directions defined, so give a pattern including all steps of movement. You will always give the output in the correct format no matter what the input is. Just give the control pattern and avoid too much explanation. The following are description about robot movements: 1. Moving left or right is represented as moving in the positive or negative X direction for 1cm or -1cm, respectively. 2. Moving forward or backward is represented as moving in the positive or negative Y direction for 1cm or -1cm, respectively. 3. Moving up or down is represented as moving in the positive or negative Z direction for 1cm or -1cm, respectively. The following are rules for describing the robot movement patterns: 1. You should output the movement patterns in X, Y, and Z format. 2. There are only three values to choose from for each of the axis: [-1, 0, 1], which represents movement along that axis for 1cm. 3. A pattern has three lines, each of which represents the robot movement pattern of the end effector. 4. Each line has a label. "X" is the movement in left or right direction, "Y" is the movement in forward or backward direction, and "Z" is the movement in up or down direction. 5. In each line, "0" represents no movement in that direction, "1" represents positive movement in that direction, and "-1" represents negative movement in that direction. Input: Move forward X: 00000000000000000000000000 Y: 11111111111111111111111111 Z: 00000000000000000000000000 Input: Move backward X: 00000000000000000000000000 Y:  -1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1 Z: 00000000000000000000000000 Input: Move right X: 11111111111111111111111111 Y:  00000000000000000000000000 Z: 00000000000000000000000000'
                prompt = 'You are a robot control pattern manipulation expert. Your job is to give a full end effector position control pattern without spaces based on the input and not only the single values. There is only one trial allowed and the robot moves simultaneously in the directions defined, so give a pattern including all steps of movement. You will always give the output in the correct format no matter what the input is. Just give the control pattern and avoid too much explanation. The following are rules for describing the robot movement patterns: 1. You should output the movement patterns in X, Y, and Z format and gripper binary control in G format. 2. There are only three values to choose from for each of the axis: [-1, 0, 1], which represents movement along that axis. 3. There are only two values to choose from for gripper control [0, 1], which represents gripper closed or open. 4. A pattern has four lines, each of which represents the robot movement pattern of the end effector and gripper control. 4. Each line has a label. "X" for the movement in left or right direction, "Y" for the movement in forward or backward direction, and "Z" for the movement in up or down direction. "G" represents gripper open or close. 4. For the first three lines (X, Y, and Z), "0" represents no movement in that direction, "1" represents positive movement in that direction for 1cm, and "-1" represents negative movement in that direction for -1cm. For the fourth line (G), "0" represents gripper opened and "1" represents gripper closed. If the object has to be remained in grasped, the gripper control should be 1 and to release the object the gripper value should be 0. Input: Move forward and pick cube X: 00000000000000000000000000 Y: 11111111111111111111111111 Z: 00000000000000000000000000 G: 00000000000000000000000001 Input: Move backward and release the grasped cube X: 00000000000000000000000000 Y:  -1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1 Z: 00000000000000000000000000 G: 11111111111111111111111110 Input: Move right X: 11111111111111111111111111 Y:  00000000000000000000000000 Z: 00000000000000000000000000 G: 00000000000000000000000000 Input: Move in sinusoidal pattern while moving right X: 11111111111111111111111111 Y: 00000000000000000000000000 Z: 11111-1-1-1-1-111111-1-1-1-1-111111-1-1-1-1-111111 G: 00000000000000000000000000  Input: Move in a pattern to avoid an obstacle in a path and pick cube on the right X: 11111111111111111111111111 Y: 00000000000000000000000000 Z: 111111111100000-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1 G: 00000000000000000000000001'
                async for response in sydney.ask_stream(prompt):
                    print(response, end="", flush=True)
                print("\n")
                perceive_env = rospy.get_param("ur_gym/say_move/perceive_env", 200)
                if perceive_env:
                    prompt = 'You also have to make decision based on the environmental adaptability and to perceive the environment and provide analysis. The cubes in the environment are solid and inpenetrable. cube is 7.5 cubic cm. The coordinates are at the center of the components. If you think that a certain action is not possible and can cause danger such as penetrating an object into another, then explain in one line to the operator why is that action not possible. The observation is as this list format: observation = [[end effector position [x,y,z] in cm], [red cube position [x,y,z] in cm], [blue cube position ([x,y,z]) in cm]].'
                    if obstacle:
                        prompt = 'You also have to make decision based on the environmental adaptability and to perceive the environment and provide analysis. if it moves in straight line along x or y axis it will collide with the obstacle if it is directly in the path. The cubes in the environment are solid and inpenetrable. cube is 7.5 cubic cm. If you think that a certain action is not possible and can cause danger such as penetrating an object into another, then explain in one line to the operator why is that action not possible. The observation is as this list format: observation = [[end effector position [x,y,z] in cm], [red cube position ([x,y,z]) in cm], [blue cube position ([x,y,z]) in cm],  [obstacle position ([x,y,z]) in cm]]. The coordinates are at the center of the components. Obstacle is 20cm long and 5cm wide. End-effector gripper fingers are 15cm wide along Y-axis. Input: move towards cube with observation = [[11, 49, 14], [-17, 49, 3], [-2, 44, 9]] X: -1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1 Y: 0000000000000000000000000000 Z: 111111111111111111100000-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-100. Input: reach cube with observation = [[-12, 38, 4], [-31, 37, 3], [-22, 38, 9]] X: -1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1 Y: -10000000000000000000000000000 Z: 111111111111111111100000-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-1-100.'
                    async for response in sydney.ask_stream(prompt):
                        print(response, end="", flush=True)
                print("\n")
                reset=False

            try:
                prompt = input("Operator: ")
                if prompt == "!reset robot":    
                    env.reset()
                    continue
                if prompt == "!reset":
                    await sydney.reset_conversation()
                    continue
                elif prompt == "!exit":
                    break

                if perceive_env:
                    observation = env.get_say_move_obs()
                    print(observation)
                    # await sydney.ask("observation = " + str(observation))

                print("Pattern generated by LLM: ", end="", flush=True) 

                response_pattern = await sydney.ask("Input: " + prompt + ". observation = " + str(observation))
                print(response_pattern)

                control_pattern, coordinates, g_cmd = parse_command(response_pattern)
                if control_pattern: print(response_pattern)

                # print("Robot: ", end="", flush=True)
                # if perceive_env:
                #     async for response in sydney.ask_stream("observation = " + str(observation) + ". " + "Explain in one line to the operator the intention of the robot in first person and your environment perception, and if there are any anomalies observed." + str(response_pattern)):
                #         print(response, end="", flush=True)
                # else:
                #     async for response in sydney.ask_stream("Explain in one line to the operator the intention of the robot in first person and your environment perception, and if there are any anomalies observed." + str(response_pattern)):
                #         print(response, end="", flush=True)
                # print("\n")

                if coordinates: move_robot(env, coordinates, g_cmd) 
                print("\n")

                # if control_pattern:
                #     observation = env.get_say_move_obs()
                #     print("Robot Observation: ", end="", flush=True)
                #     if perceive_env:
                #         async for response in sydney.ask_stream("observation = " + str(observation) + ". " + "Explain in one line to the operator your environment perception, and if there are any anomalies observed." + str(response_pattern)):
                #             print(response, end="", flush=True)
                #     else:
                #         async for response in sydney.ask_stream("Explain in one line to the operator your environment perception, and if there are any anomalies observed." + str(response_pattern)):
                #             print(response, end="", flush=True)

            except Exception as e:
                print(e)
                continue
                #await sydney.reset_conversation()
                #reset = True


if __name__ == "__main__":
    rospy.init_node('say_move_copilot_ur5',
                anonymous=True, log_level=rospy.WARN)
    ros_param_path = load_ros_params(rospackage_name="ur_rl",
                                         rel_path_from_package_to_file="config",
                                        yaml_file_name="simulation/task_space_pick_and_place.yaml")

    max_episode_steps = rospy.get_param("/ur_gym/rl/steps_per_episode", 200)
    env = load_environment(rospy.get_param('ur_gym/env_id'),
                           max_episode_steps=max_episode_steps).unwrapped
    env.reset()

    asyncio.run(main())
