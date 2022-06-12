#! /usr/bin/env python3

import os
import rospy
import threading
import json
import tf2_ros

from manipulation_client import ManipulationClient
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion, TransformStamped

from flask import Flask, request, jsonify, json
from werkzeug.exceptions import HTTPException

threading.Thread(target=lambda: rospy.init_node('http_server', disable_signals=True)).start()
manipulation_client = ManipulationClient('/vx300s')

app = Flask(__name__)

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
    print()


    req = request.get_json()

    #get data
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
    pose = manipulation_client.flatten_pose(pose)

    pose.position.z = 0.015 + manipulation_client.objects[object_id]['dimensions'][2] / 2

    br = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.frame_id = "world"
    t.child_frame_id = "final_pose"
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation = pose.orientation
    br.sendTransform(t)

    pose_str = '%s;%s;%s;%s;%s;%s;%s' % (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )

    xml = f"""
    <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <EnableCamera enable="true"/>
                <MoveToTarget target="inspect_ground"/>
                <Wait sec="1"/>
                <Pick object_id="{object_id}" />
                <Place object_id="{object_id}" pose="{pose_str}" />
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

@app.route('/test', methods=['POST'])
def test():
    xml = """
    <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <Place object_id="cube" pose="0.3;0;0.2;0;0;0"/>
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