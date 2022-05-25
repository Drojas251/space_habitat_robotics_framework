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
    object_id = req = request.get_json()

    object_id = req['id']
    pose_str = '%s;%s;%s;%s;%s;%s;%s' % (
        req['posX'],
        req['posY'],
        req['posZ'],
        req['rotX'],
        req['rotY'],
        req['rotZ'],
        req['rotW']
    )


    xml = f"""
    <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <ClearOctomap />
                <EnableCamera enable="true"/>
                <MoveToTarget target="inspect_ground"/>
                <Wait sec="2" />
                <Pick object_id="{object_id}" />
                <Place object_id="{object_id}" pose="{pose_str}"/>
                <MoveToTarget target="inspect_ground"/>
            </Sequence>
        </BehaviorTree>
    </root>
    """

    success = manipulation_client.execute_behavior_tree_client(xml)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'get_scene failed'})

@app.route('/test', methods=['POST'])
def test():
    xml = """
    <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <AddObject object_id="cube" pose="0.3;0;0.1;0;0;0"/>
        </BehaviorTree>
    </root>
    """

    success = manipulation_client.execute_behavior_tree_client(xml)

    if success:
        return jsonify({'message': 'success!'})
    else:
        return jsonify({'message': 'failed!'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8000, debug=True)
