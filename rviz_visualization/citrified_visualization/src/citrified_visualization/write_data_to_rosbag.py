#!/usr/bin/env python

import os
from os.path import join as pjoin

import rosbag
import rospkg
import rospy
from geometry_msgs.msg import TransformStamped, Point, WrenchStamped
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker

import pandas as pd
import numpy as np


def update_trace_msg(pos, color, time_stamp, count, tool_id, max_length=500, marker_size=0.002):
    if not len(trace_marker_array[tool_id].points):
        trace_marker_array[tool_id].header.frame_id = "world"
        trace_marker_array[tool_id].type = trace_marker_array[tool_id].LINE_STRIP
        trace_marker_array[tool_id].action = trace_marker_array[tool_id].ADD
        trace_marker_array[tool_id].color.a = 0.8
        trace_marker_array[tool_id].scale.x = marker_size
    while len(trace_marker_array[tool_id].points) >= max_length:
        trace_marker_array[tool_id].points.pop(0)
        trace_marker_array[tool_id].colors.pop(0)
    trace_marker_array[tool_id].header.stamp = time_stamp
    trace_marker_array[tool_id].header.seq = count
    trace_marker_array[tool_id].pose.orientation.w = 1
    color_vec = get_colorRGBA(color)
    trace_marker_array[tool_id].colors.append(color_vec)
    point = Point()
    point.x = pos[0]
    point.y = pos[1]
    point.z = pos[2]
    trace_marker_array[tool_id].points.append(point)


def get_colorRGBA(color, startcolor=[0.0, 0.0, 1.0], stopcolor=[1.0, 0.0, 0.0], max=5.0):
    color_vec = ColorRGBA()
    prop = color / max
    if prop > 1.0:
        prop = 1.0
    vec = tuple(prop * (b - a) + a for a, b in zip(startcolor, stopcolor))
    color_vec.a = 1.
    color_vec.r = vec[0]
    color_vec.g = vec[1]
    color_vec.b = vec[2]
    return color_vec


def get_tf_message(pose, time_stamp, frame_id, child_frame_id, count):
    """write the information in a ros transform message.
    normalize the quaternion."""
    translation = [pose[0], pose[1], pose[2], 1]
    quat = [pose[3], pose[4], pose[5], pose[6]]
    quat = quat / np.linalg.norm(quat)

    msg = TransformStamped()
    msg.header.seq = count
    msg.header.stamp = time_stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.transform.translation.x = translation[0]
    msg.transform.translation.y = translation[1]
    msg.transform.translation.z = translation[2]
    msg.transform.rotation.x = quat[0]
    msg.transform.rotation.y = quat[1]
    msg.transform.rotation.z = quat[2]
    msg.transform.rotation.w = quat[3]

    return msg


def get_wrench_message(force, time_stamp, frame_id, count, scale=10.0):
    msg = WrenchStamped()
    msg.header.seq = count
    msg.header.stamp = time_stamp
    msg.header.frame_id = frame_id
    msg.wrench.force.x = force[0] / float(scale)
    msg.wrench.force.y = force[1] / float(scale)
    msg.wrench.force.z = force[2] / float(scale)
    return msg


def initialize_marker(marker, frame, pos, orientation, scale, marker_type, marker_id, color):
    marker.header.frame_id = frame
    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.type = marker_type
    marker.id = marker_id
    marker.action = marker.ADD
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]


def update_marker(marker, time_stamp, count):
    marker.header.stamp = time_stamp
    marker.header.seq = count


tools = {"knife": 0}

trace_marker_array = []
tool_marker_array = []
fruit_marker_msg = Marker()
table_marker_msg = Marker()


def main(experiment="", fruit="", cut_quality="", desired_run=None):
    # iterate through all the cuts data folder
    # data_dir = pjoin(rospkg.RosPack().get_path("citrified_visualization"), "data", "preprocessed_transformed_data")
    data_dir = pjoin(os.getenv("HOME"), "data", "preprocessed_transformed_data")
    print(data_dir)
    experiment_config_dir = pjoin(data_dir, experiment, fruit, cut_quality)

    if not os.path.isdir(data_dir):
        rospy.logwarn("This combination of experiment, fruit, and cut quality is not available! Abort")
        exit(1)

    if desired_run is not None:
        runs = [run for run in os.listdir(experiment_config_dir) if run.split("_")[-1].split(".")[0] == desired_run]
    else:
        runs = os.listdir(experiment_config_dir)

    export_dir = pjoin(data_dir, os.pardir, "rosbags", experiment, fruit, cut_quality)
    if not os.path.isdir(export_dir):
        os.makedirs(export_dir)

    for tool_name, tool_info in tools.items():
        trace_marker_msg = Marker()
        trace_marker_array.append(trace_marker_msg)
        tool_marker_msg = Marker()
        initialize_marker(tool_marker_msg, tool_name, [0, 0.05, 0], [-0.7071068, 0, 0, 0.7071068], [0.005, 0.005, 0.1],
                          tool_marker_msg.CYLINDER, tool_info, [0, 1, 1, 1])
        tool_marker_array.append(tool_marker_msg)

    initialize_marker(fruit_marker_msg, "world", [0.01, 0, 0.115], [0, 0, 0, 1], [0.12, 0.12, 0.12],
                      table_marker_msg.SPHERE, 11, [1, 69.0 / 255.0, 0, 0.5])
    initialize_marker(table_marker_msg, "world", [0, 0, 0.05], [0, 0, 0, 1], [0.3, 0.3, 0.01], table_marker_msg.CUBE,
                      22, [165.0 / 255.0, 42.0 / 255.0, 42.0 / 255.0, 0.5])

    for run in runs:
        print("Processing cut " + run)
        with rosbag.Bag(
                pjoin(data_dir, os.pardir, "rosbags", experiment, fruit, cut_quality, run.split(".")[0] + ".bag"),
                "w") as bag:
            data = pd.read_csv(pjoin(experiment_config_dir, run))
            for index, row in data.iterrows():
                tf_msg = TFMessage()
                timestamp = rospy.Time.from_sec(row["relative_time"])
                for tool_name, tool_info in tools.items():
                    transform_msg = get_tf_message(np.array(row["ExactoKnife_x":"ExactoKnife_qw"]), timestamp,
                                                   "world", tool_name, index)
                    tf_msg.transforms.append(transform_msg)

                    update_marker(tool_marker_msg, timestamp, index)
                    bag.write("/" + tool_name, tool_marker_msg, timestamp)

                    force_norm = np.linalg.norm(np.array(row["force_x":"force_z"]))
                    if force_norm > 0:
                        update_trace_msg([transform_msg.transform.translation.x, transform_msg.transform.translation.y,
                                          transform_msg.transform.translation.z], force_norm, timestamp, index,
                                         tool_info)
                        bag.write("/" + tool_name + "_trace", trace_marker_array[tool_info], timestamp)
                    else:
                        trace_marker_array[tool_info] = Marker()

                    wrench_msg = get_wrench_message(np.array(row["force_x":"force_z"]), timestamp, tool_name, index)
                    bag.write("/" + tool_name + "_wrench", wrench_msg, timestamp)

                update_marker(fruit_marker_msg, timestamp, index)
                update_marker(table_marker_msg, timestamp, index)

                bag.write("/tf", tf_msg, timestamp)
                bag.write("/fruit", fruit_marker_msg, timestamp)
                bag.write("/table", table_marker_msg, timestamp)
