#! /usr/bin/env python3

import rospy
import tkinter
from manipulation_client import ManipulationClient


if __name__ == '__main__':
    rospy.init_node('semiautomated_control')
    manipulation_client = ManipulationClient('/vx300s')
    cur_obj = None

    def reset():
        xml = """
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <Sequence name="root_sequence">
                        <Reset/>
                        <MoveGripperToTarget target="Open"/>
                        <EnableCamera enable="true"/>
                        <MoveToTarget target="inspect_ground"/>
                    </Sequence>
                </BehaviorTree>
            </root>
            """
        success = manipulation_client.execute_behavior_tree_client(xml)

    def small_cb():
        global cur_obj
        if cur_obj == None:
            xml = """
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <Pick object_id="small"/>
                </BehaviorTree>
            </root>
            """
            success = manipulation_client.execute_behavior_tree_client(xml)
            if success:
                cur_obj = 'small'

    def medium_cb():
        global cur_obj
        if cur_obj == None:
            xml = """
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <Pick object_id="medium"/>
                </BehaviorTree>
            </root>
            """
            success = manipulation_client.execute_behavior_tree_client(xml)
            if success:
                cur_obj = 'medium'

    def large_cb():
        global cur_obj
        if cur_obj == None:
            xml = """
            <root main_tree_to_execute = "MainTree" >
                <BehaviorTree ID="MainTree">
                    <Pick object_id="large"/>
                </BehaviorTree>
            </root>
            """
            success = manipulation_client.execute_behavior_tree_client(xml)
            if success:
                cur_obj = 'large'

    def drop_cb():
        global cur_obj
        xml = f"""
        <root main_tree_to_execute = "MainTree" >
            <BehaviorTree ID="MainTree">
                <Sequence name="root_sequence">
                    <MoveGripperToTarget target="Open"/>
                    <DetachObject object_id="{cur_obj}"/>
                    <MoveToTarget target="inspect_ground"/>
                    <EnableCamera enable="true"/>
                </Sequence>
            </BehaviorTree>
        </root>
        """
        success = manipulation_client.execute_behavior_tree_client(xml)
        if success:
            cur_obj = None

    reset()

    top = tkinter.Tk()

    s = tkinter.Button(top, text ="Pick small", command = small_cb)
    s.pack(pady=16, padx=16)

    m = tkinter.Button(top, text ="Pick medium", command = medium_cb)
    m.pack(pady=16, padx=16)

    l = tkinter.Button(top, text ="Pick large", command = large_cb)
    l.pack(pady=16, padx=16)

    d = tkinter.Button(top, text ="Drop object", command = drop_cb)
    d.pack(pady=16, padx=16)

    top.mainloop()

    