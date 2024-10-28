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
    def flatten_list(nested_list):
        flat_list = []
        for sublist in nested_list:
            for item in sublist:
                flat_list.append(item)
        return flat_list

    try:
        patterns = ['X', 'Y', 'Z', 'G']
        control_patterns = {pattern: [] for pattern in patterns}
        # for pattern in patterns:
        #     matches = re.findall(f'{pattern}\s*:\s*(.*)', response)
        #     for match in matches:
        #         control_patterns[pattern].append(eval(match))

        lines = response.split('\n')
        for line in lines:
            line = line.strip()
            if not line: continue  # skip empty lines
            pattern = line[0]
            if pattern in patterns:
                matches = eval(line.split(':')[1].strip())
                control_patterns[pattern].append(matches)
                
        coordinates = []
        g_vals = []
        for i in range(len(control_patterns['X'])):
            x_coords = control_patterns['X'][i]
            y_coords = control_patterns['Y'][i]
            z_coords = control_patterns['Z'][i]
            g_val = control_patterns['G'][i]
            # Find the maximum length among the lists
            max_len = max(len(x_coords), len(y_coords), len(z_coords), len(g_val))
            # Pad the lists with zeros to make them the same length
            x_coords = x_coords + [0]*(max_len - len(x_coords))
            y_coords = y_coords + [0]*(max_len - len(y_coords))
            z_coords = z_coords + [0]*(max_len - len(z_coords))
            g_val = g_val + [0]*(max_len - len(g_val)) if g_val[0] == 1 else [0]*(max_len - len(g_val)) + g_val

            coordinates.append(list(zip(x_coords, y_coords, z_coords)))
            g_vals.append(g_val)

        coordinates = flatten_list(coordinates)
        g_vals = flatten_list(g_vals)
        return control_patterns, coordinates, g_vals

    except Exception as e:  
        print("Error parsing generated pattern, please input again")
        print(str(e))
        return None, None, None


def move_robot(env, coordinates, g_val):
    env.talk_machines(coordinates, g_val, 0.01)
        

async def main() -> None:    
    async with SydneyClient(style="precise") as sydney:
        reset = True
        obstacle = rospy.get_param("/ur_gym/object_disturbance", False)

        while True: 
            if reset:
                prompt = '''
                        You are a robotic end-effector position control pattern expert. Your task is to provide a control pattern in the format of multiples based on the given input, following the specified rules and examples.

                        Rules:

                        1. Output patterns in the format: X (left/right), Y (forward/backward), Z (up/down), G (gripper open/close).
                        2. X, Y, Z values: [-1 (negative), 0 (no movement), 1 (positive)] representing movement in mm.
                        3. G values: [0 (open), 1 (closed)].
                        4. Each pattern has 4 lines: X, Y, Z, G.
                        5. Break down complex tasks into simpler sequences.
                        6. The robot moves simultaneously in the directions defined.
                        7. Avoid sliding/dragging objects on the floor. Lift before moving.

                        Input: [Task description]
                        Output: (Follow the specified format, provide only the control pattern)

                        Examples:

                        Input: Move forward 100mm and pick up a cube
                        X: [0]*50
                        Y: [1]*100  
                        Z: [0]*30
                        G: [0]*99 + [1]*1

                        Input: Move backward 50mm and release the grasped cube  
                        X: [0]*10
                        Y: [-1]*50
                        Z: [0]*20
                        G: [1]*49 + [0]*1

                        Input: Move left 70mm
                        X: [-1]*70
                        Y: [0]*20
                        Z: [0]*10
                        G: [0]*70
                        '''
                async for response in sydney.ask_stream(prompt):
                    print(response, end="", flush=True)
                print("\n")
                perceive_env = rospy.get_param("ur_gym/perceive_env", 200)
                zones = rospy.get_param("ur_gym/zones", False)
                if perceive_env:
                    prompt = '''
                            You also have to make decisions based on environmental adaptability and to perceive the environment and provide analysis. 
                            The cubes in the environment are solid and impenetrable.
                            cube is (75*75*75) cubic mm. 
                            The coordinates are at the center of the components. 
                            If you think that a certain action is not possible and can cause danger such as penetrating an object into another or picking an obstructed object, then explain in one line to the operator why is that action not possible. 
                            The observation is as this list format: observation = [[end effector position [x,y,z] in mm], [red cube position [x,y,z] in mm], [blue cube position ([x,y,z]) in mm]].
                            '''
                    if obstacle:
                        prompt = '''
                                You also have to make decisions based on environmental adaptability and to perceive the environment and provide analysis.

                                If it moves in a straight line along the x or y axis it will collide with the obstacle if it is directly in the path.
                                The cubes in the environment are solid and impenetrable. if you moved up to avoid an obstacle you have to compensate for that, add the moving up position back to move down to reach the previous elevation and then the difference between the end-effector elevation to the cube elevation.

                                If you think that a certain action is not possible and can cause danger such as penetrating an object into another or colliding with an obstacle, then explain in one line to the operator why is that action not possible.

                                The observation is as this list format:
                                observation = [[end effector position [x,y,z] in mm], [red cube position ([x,y,z]) in mm],  [blue cube position ([x,y,z]) in mm],  [obstacle position ([x,y,z]) in mm]].

                                The coordinates are at the center of the objects.
                                The obstacle is 200mm long and 50mm wide.
                                End-effector gripper fingers are 150mm wide along the Y-axis.
                                The cube is (75*75*75) cubic mm.

                                Input: grasp red cube. Observation = [[110, 490, 140], [-270, 190, 30],  [-170, 490, 30], [-20, 440, 90]]
                                X: [0]*100 + [-1]*280 + [0]*10
                                Y: [0]*390
                                Z: [1]*100 + [0]*280 + [-1]*100+ [-1]*110
                                G: [0]*589 + [1]*1

                                Input: grasp red cube. Observation = [[-120, 380, 40], [200, -500, 30], [-310, 370, 30], [-220, 380, 90]]
                                X: [0]*100 + [-1]*190 + [0]*10
                                Y: [0]*300
                                Z: [1]*100 + [0]*190 + [-1]*100 + [-1]*70
                                G: [0]*459 + [1]*1
                                '''
                    if zones:
                        prompt = '''
                                You are an intelligent robotic control system capable of perceiving the environment and making decisions based on observations and constraints. Here are the key points:

                                - The environment contains solid, impenetrable cubes (75x75x75 mm) and two zones (green and yellow, 400x300 mm).
                                - Coordinates represent the center of components.
                                - Avoid sliding grasped cubes; lift them before moving.
                                - Observations are provided in the format: [[end effector position [x,y,z]], [red cube position [x,y,z]], [blue cube position [x,y,z]], [green zone position [x,y,z]], [yellow zone position [x,y,z]]].
                                - If an action is impossible or dangerous (e.g., penetrating objects), provide a one-line explanation to the operator.

                                Your task is to provide control patterns and analysis based on the given observations and environmental constraints.

                                Input: [Task description]. Observation = [[end effector position [x,y,z]], [red cube position [x,y,z]], [blue cube position [x,y,z]], [green zone position [x,y,z]], [yellow zone position [x,y,z]]]

                                Output: [control pattern] 
                                '''
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
                print("Robot: ", end="", flush=True) 

                response_pattern = await sydney.ask("Input: " + prompt + ". Observation = " + str(observation))
                print(response_pattern)

                control_pattern, coordinates, g_cmd = parse_command(response_pattern)

                if coordinates: move_robot(env, coordinates, g_cmd) 
                print("\n")

            except Exception as e:
                print(e)
                continue


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
