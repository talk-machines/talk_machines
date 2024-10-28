#!/usr/bin/env python3

import numpy as np
import gymnasium as gym
import sys
import os
import time
import subprocess
import rospy
import asyncio
import yaml
from std_msgs.msg import String
from ur_openai.common import load_environment, log_ros_params, clear_gym_params, load_ros_params
from std_srvs.srv import Empty
from view_controller_msgs.msg import CameraPlacement
from PIL import Image
from gymnasium.wrappers import TimeAwareObservation, NormalizeObservation, NormalizeReward, RecordEpisodeStatistics, FlattenObservation
from sydney import SydneyClient

sys.path.insert(0, "/home/research/Desktop/ur5_drl_ammar/ws_ur_openai_ros_sim/src/ur5/ur_control/src")
sys.path.insert(0, "ur5/ur_control/src/")

camera_placement_pub = rospy.Publisher('/rviz/camera_placement', CameraPlacement, queue_size=1)


def load_views(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)
views = load_views('/home/imr/Desktop/ws_talk_machines/say_move/src/talk_machine/config/views.yaml')

def change_view(view):
    camera_placement_msg = CameraPlacement()
    camera_placement_msg.target_frame = view['frame']
    camera_placement_msg.eye.header.frame_id = view['frame']
    camera_placement_msg.eye.point.x = view['eye'][0]
    camera_placement_msg.eye.point.y = view['eye'][1]
    camera_placement_msg.eye.point.z = view['eye'][2]
    camera_placement_msg.focus.point.x = view['focus'][0]
    camera_placement_msg.focus.point.y = view['focus'][1]
    camera_placement_msg.focus.point.z = view['focus'][2]
    camera_placement_msg.up.vector.x = view['up'][0]
    camera_placement_msg.up.vector.y = view['up'][1]
    camera_placement_msg.up.vector.z = view['up'][2]

    camera_placement_pub.publish(camera_placement_msg)
    # rospy.sleep(1)
    rospy.loginfo(f"Changed view to '{view['name']}'.")

def change_viewpoint(viewpoint_name):
    if viewpoint_name in views:
        view = views[viewpoint_name]
        change_view(view)
    else:
        rospy.logwarn(f"View '{viewpoint_name}' not found in configuration.")

def take_screenshot(filename_prefix):
    for view in views:
        change_viewpoint(view)
        # time.sleep(1)  # Wait for the view to change
        filename = f"{filename_prefix}_{view}.png"
        command = f"rosservice call /rviz/screenshot '{filename}'"
        subprocess.run(command, shell=True)

def collate_screenshot_with_obs():
    fileroot = "/home/imr/Desktop/ws_talk_machines/say_move/src/talk_machine/scripts/_misc/screenshots/"
    frame_stacks = 5
    sensor_obs = []
    images = []
    max_width = 600  # Maximum width for the resized images
    max_height = 600

    for i in range(frame_stacks):
        # env.expert_pick_talk_machines()
        filename_prefix = fileroot + f"screenshot_{i}"
        # env.get_say_move_obs()
        take_screenshot(filename_prefix)
        # sensor_obs.append(env.get_verbal_states_obs())

    for i in range(frame_stacks):
        row_images = []
        for view in views:
            filename = f"{fileroot}screenshot_{i}_{view}.png"
            img = Image.open(filename)
            ratio = max_width / img.width
            new_height = int(img.height * ratio)
            
            # Adjust the resizing dimensions based on max_height
            if new_height > max_height:
                ratio = max_height / img.height
                new_height = max_height
                new_width = max_width
            else:
                new_width = max_width
            
            resized_img = img.resize((new_width, new_height))
            row_images.append(resized_img)
        
        # Ensure all images have the same height
        row_images = [img.resize((new_width, max_height)) for img in row_images]
        images.append(row_images)

    num_cols = frame_stacks
    num_rows = 4 # Number of views 
    grid_width = num_cols * max_width
    grid_height = num_rows * max_height

    grid = Image.new('RGB', (grid_width, grid_height))

    for j, row_images in enumerate(images):
        for i, img in enumerate(row_images):
            col_index = j
            row_index = i
            x_offset = col_index * max_width
            y_offset = row_index * max_height
            grid.paste(img, (x_offset, y_offset))

    grid.save("/home/imr/Desktop/ws_talk_machines/say_move/src/talk_machine/scripts/_misc/grid.png")
    return sensor_obs

prompt = '''
You are the robot and the sensor observation is given in a list of observations.
Each observation list is ordered as [if cube is grasped or not in boolean value,
                                    position of end effector (x, y, z),
                                    velocity of end effector (x, y, z),
                                    cube 1 position (x, y, z),
                                    cube 2 position (x, y, z),
                                    force on end effector in z]
To help visually, the camera image is given as a real-time 5-frame stack starting from left to right column-wise.
Additionally, there are 4 rows each representing different viewpoints of the same time-stamp for better analysis.
The first row is the orthographic view.
The second row is the top view.
The third row is the side view.
The fourth row is the front view.
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
    change_viewpoint('Ortho')
    time.sleep(0.5)
    sensor_obs_list = []
    sensor_obs_list = collate_screenshot_with_obs()
    async with SydneyClient(style="precise") as sydney:
        async for response in sydney.ask_stream(prompt + str(sensor_obs_list), 
                                                attachment="./src/talk_machine/scripts/_misc/grid.png"):
            print(response, end="", flush=True)

if __name__ == "__main__":
    rospy.init_node('say_move_copilot_ur5', anonymous=True, log_level=rospy.WARN)
    ros_param_path = load_ros_params(rospackage_name="ur_rl", rel_path_from_package_to_file="config", yaml_file_name="simulation/task_space_pick_and_place.yaml")

    max_episode_steps = rospy.get_param("/ur_gym/rl/steps_per_episode", 200)
    # env = load_environment(rospy.get_param('ur_gym/env_id'), max_episode_steps=max_episode_steps).unwrapped
    # env.reset()

    asyncio.run(main())
