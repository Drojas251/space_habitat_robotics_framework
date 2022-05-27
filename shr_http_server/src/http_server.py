#! /usr/bin/env python3

import os
import rospy
import threading
import json

from manipulation_client import ManipulationClient
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion

from flask import Flask, request, jsonify, json
from werkzeug.exceptions import HTTPException


def get_pose_msg(pose_str):
    parts = pose_str.split(';')
    parts = [float(x) for x in parts]
    pose = Pose()

    if len(parts) == 6:
        pose.position.x = parts[0]
        pose.position.y = parts[1]
        pose.position.z = parts[2]
        quat_tf = quaternion_from_euler(parts[3], parts[4], parts[5])
        orientation = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
        pose.orientation = orientation   

    elif len(parts) == 7:
        pose.position.x = parts[0]
        pose.position.y = parts[1]
        pose.position.z = parts[2]
        pose.orientation.x = parts[3]
        pose.orientation.y = parts[4]
        pose.orientation.z = parts[5]
        pose.orientation.w = parts[6]
    else:
        raise Exception("Invalid pose string")

    return pose


threading.Thread(target=lambda: rospy.init_node('http_server', disable_signals=True)).start()
manipulation_client = ManipulationClient('/vx300s')

app = Flask(__name__)

@app.route('/move-to-pose', methods=['PUT'])
def move_to_pose():
    request_data = request.get_json()

    pose_str = request_data['pose']

    try:
        pose = get_pose_msg(pose_str)
    except:
        return jsonify({'message': 'invalid pose'})

    success = manipulation_client.move_to_pose_client(pose)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'move_to_pose failed'})

@app.route('/move-to-target', methods=['POST'])
def move_to_target():
    req = request.form

    target = req.get("target")

    success = manipulation_client.move_to_target_client(target)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'move_to_target failed'})

@app.route('/move-gripper-to-target', methods=['PUT'])
def move_gripper_to_target():
    request_data = request.get_json()

    target = request_data['target']

    success = manipulation_client.move_gripper_to_target_client(target)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'move_gripper_to_target failed'})

@app.route('/move-gripper', methods=['PUT'])
def move_gripper():
    request_data = request.get_json()

    gripper_width = request_data['gripper_width']

    try:
        gripper_width = float(gripper_width)
    except:
        return jsonify({'message': 'invalid gripper_width'})

    success = manipulation_client.gripper_width_client(gripper_width)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'move_gripper failed'})

@app.route('/pick', methods=['PUT'])
def pick():
    request_data = request.get_json()

    object_id = request_data['object_id']
    retreat = '' if 'retreat' not in request_data else request_data['retreat']

    success = manipulation_client.pick_client(object_id, retreat)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'pick failed'})

@app.route('/place', methods=['PUT'])
def place():
    request_data = request.get_json()

    object_id = request_data['object_id']
    try:
        pose = get_pose_msg(request_data['pose'])
    except:
        return jsonify({'message': 'invalid pose'})

    success = manipulation_client.place_client(object_id, pose)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'place failed'})

@app.route('/add-object', methods=['PUT'])
def add_object():
    request_data = request.get_json()

    type = request_data['type']
    object_id = request_data['object_id']
    try:
        pose = get_pose_msg(request_data['pose'])
    except:
        return jsonify({'message': 'invalid pose'})
    dimensions = request_data['dimensions']

    success = manipulation_client.add_object_client(type, object_id, pose, dimensions)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'add_object failed'})

@app.route('/remove-object', methods=['PUT'])
def remove_object():
    request_data = request.get_json()

    object_id = request_data['object_id']

    success = manipulation_client.remove_object_client(object_id)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'remove_object failed'})

@app.route('/get-scene-objects', methods=['GET'])
def get_scene():
    scene, success = manipulation_client.get_scene_objects()

    if success:
        return jsonify(scene)
    else:
        return jsonify({'message': 'get_scene failed'})

@app.route('/move-object', methods=['POST'])
def move_object():
    print(request.data)
    print()
    print(request.headers)
    req = request.get_json()

    object_id = req['id']

    pose = Pose()
    pose.position.x = float(req['newPosX'].replace(',', '.'))
    pose.position.y = -float(req['newPosY'].replace(',', '.'))
    pose.position.z = float(req['newPosZ'].replace(',', '.'))
    pose.orientation.x = float(req['newRotX'].replace(',', '.'))
    pose.orientation.y = float(req['newRotY'].replace(',', '.'))
    pose.orientation.z = float(req['newRotZ'].replace(',', '.'))
    pose.orientation.w = float(req['newRotW'].replace(',', '.'))

    pose = manipulation_client.transform_pose(pose, "ipad_camera", "world")

    pose_str = '%s;%s;%s;%s;%s;%s;%s' % (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        0,#pose.orientation.x,
        0,#pose.orientation.y,
        0,#pose.orientation.z,
        1#pose.orientation.w
    )


    xml = f"""
    <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <Pick object_id="{object_id}" />
                <MoveToPose pose="{pose_str}" />
                <MoveGripperToTarget target="Open"/>
                <MoveToTarget target="inspect_ground"/>
            </Sequence>
        </BehaviorTree>
    </root>
     """

    rospy.loginfo(xml)

    success = manipulation_client.execute_behavior_tree_client(xml)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'get_scene failed'})

@app.route('/test', methods=['GET'])
def test():
    xml = """
    <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <Place object_id="cube" pose="0.3;0;0.2;0;0;1.5"/>
            </Sequence>
        </BehaviorTree>
    </root>
    """

    success = manipulation_client.execute_behavior_tree_client(xml)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'failed!'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000,  debug=True)


# [ERROR] [1653618175.008503218]: interbotix_arm/interbotix_arm[RRTConnect]: Motion planning start tree could not be initialized!