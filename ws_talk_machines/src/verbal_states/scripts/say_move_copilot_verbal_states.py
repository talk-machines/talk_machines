#!/usr/bin/env python3
import numpy as np
import gymnasium as gym
import sys
import re
import os

import time
import subprocess
import rospy
import asyncio

from std_msgs.msg import String
from ur_openai.common import load_environment, log_ros_params, clear_gym_params, load_ros_params
from std_srvs.srv import Empty
from PIL import Image

from gymnasium.wrappers import TimeAwareObservation, NormalizeObservation, NormalizeReward, RecordEpisodeStatistics, \
    FlattenObservation

from sydney import SydneyClient


sys.path.insert(0, "/home/research/Desktop/ur5_drl_ammar/ws_ur_openai_ros_sim/src/ur5/ur_control/src")
sys.path.insert(0, "ur5/ur_control/src/")



def take_screenshot(filename):
    command = f"rosservice call /rviz/screenshot {filename}"
    subprocess.run(command, shell=True)

def collate_screenshot_with_obs():
    fileroot = "/home/imr/Desktop/ws_talk_machines/say_move/src/talk_machine/scripts/_misc/screenshots/"
    frame_stacks = 5
    sensor_obs = []
    for i in range(frame_stacks):
        env.expert_pick_talk_machines()
        filename = fileroot + f"screenshot_{i}.png"
        env.get_say_move_obs()
        # take_screenshot(filename)
        sensor_obs.append(env.get_verbal_states_obs())

    # images = [Image.open(fileroot + f"screenshot_{i}.png") for i in range(2)]
    # widths, heights = zip(*(i.size for i in images))

    # total_width = sum(widths)
    # max_height = max(heights)

    # collage = Image.new('RGB', (total_width, max_height))

    # x_offset = 0
    # for img in images:
    #     collage.paste(img, (x_offset, 0))
    #     x_offset += img.width

    images = []
    max_width = 800  # Maximum width for the resized images
    for i in range(frame_stacks):
        img = Image.open(fileroot + f"screenshot_{i}.png")
        # Resize the image to reduce resolution
        ratio = max_width / img.width
        new_height = int(img.height * ratio)
        resized_img = img.resize((max_width, new_height))
        images.append(resized_img)

    widths, heights = zip(*(i.size for i in images))
    total_width = sum(widths)
    max_height = max(heights)

    collage = Image.new('RGB', (total_width, max_height))
    x_offset = 0
    for img in images:
        collage.paste(img, (x_offset, 0))
        x_offset += img.width

    collage.save("/home/imr/Desktop/ws_talk_machines/say_move/src/talk_machine/scripts/_misc/robot_state.png")
    return sensor_obs

prompt = '''
        You are the robot and the sensor observation is given in a list of observations.
        Each observation list is ordered as [if cube is grasped or not in boolean value,
                                            position of end effector (x, y, z),
                                            velocity of end effector (x, y, z),
                                            cube 1 position (x, y, z),
                                            cube 2 position (x, y, z),
                                            force on end effector in z]
        To help visually, camera image is given as a real-time frame stack starting from left. 
        It is a grasping task with object being the red cube. 
        The object should be properly aligned in the gap with the gripper fingers otherwise the object will collide with the it while gripper is moving towards it. 
        Describe robot state and if at any point it is going to or has already collided etc.
        Check if the object is within range to grasp.
        Also check if the red cube is not obstructed by the black cube.
        Predict the future state or if any dangerous anomaly is about to occur.
        give output response only in 50 characters.
        give reason for the decision based on observation sequence or images only in another 50 characters.
        observation = 
        '''
                                                
async def main() -> None:    
    sensor_obs_list = collate_screenshot_with_obs()    
    print(sensor_obs_list)
    async with SydneyClient(style="creative") as sydney:
        async for response in sydney.ask_stream(prompt + str(sensor_obs_list), attachment="./src/talk_machine/scripts/_misc/robot_state.png"):
            print(response, end="", flush=True)

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
