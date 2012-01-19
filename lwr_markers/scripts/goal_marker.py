#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest("lwr_markers")
import rospy
import copy

import actionlib
import arm_navigation_msgs.msg

from arm_navigation_msgs.msg import PositionConstraint
from arm_navigation_msgs.msg import OrientationConstraint

from arm_navigation_msgs.msg import Shape

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

menu_handler = MenuHandler()
client = actionlib.SimpleActionClient('move_right', arm_navigation_msgs.msg.MoveArmAction)


def processFeedback(feedback):
    #s = "Feedback from marker '" + feedback.marker_name
    #if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
    #    rospy.loginfo( s + ": button click" + mp + "." )



    if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        if (feedback.menu_entry_id == 1):
            p = feedback.pose
            print p

            goalA = arm_navigation_msgs.msg.MoveArmGoal()
            goalA.motion_plan_request.group_name = "right";
            goalA.motion_plan_request.num_planning_attempts = 1;
            goalA.motion_plan_request.planner_id = "";
            goalA.planner_service_name = "ompl_planning/plan_kinematic_path";
            goalA.motion_plan_request.allowed_planning_time = rospy.Duration(5.0);

            desired_pose = PositionConstraint()
            desired_pose.header.frame_id = "/world";
            desired_pose.link_name = "right_arm_7_link";
            desired_pose.position = p.position

            desired_pose.constraint_region_shape.type = Shape.BOX
            desired_pose.constraint_region_shape.dimensions = [0.02, 0.02, 0.02]
            desired_pose.constraint_region_orientation.w = 1.0


            goalA.motion_plan_request.goal_constraints.position_constraints.append(desired_pose)

            oc = OrientationConstraint()
            oc.header.stamp = rospy.Time.now()
            oc.header.frame_id = "/world";
            oc.link_name = "right_arm_7_link";
            oc.orientation = p.orientation

            oc.absolute_roll_tolerance = 0.04
            oc.absolute_pitch_tolerance = 0.04
            oc.absolute_yaw_tolerance = 0.04
            oc.weight = 1.

            goalA.motion_plan_request.goal_constraints.orientation_constraints.append(oc)

            client.send_goal(goalA)




#        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked." )
 #   elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
 #       rospy.loginfo( s + ": pose changed")
 #       p = feedback.pose.position
 #       print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

if __name__=="__main__":
    rospy.init_node("simple_marker")



    client = actionlib.SimpleActionClient('move_right', arm_navigation_msgs.msg.MoveArmAction)
    client.wait_for_server()








    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("simple_marker")

    menu_handler.insert( "Go to", callback=processFeedback )

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/world"
    int_marker.name = "my_marker"
    int_marker.description = "Right"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.1
    box_marker.scale.y = 0.1
    box_marker.scale.z = 0.1
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 0.5

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )

    # add the control to the interactive marker
    int_marker.controls.append( box_control )

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    # make one control using default visuals
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Right"
    control.name = "menu_only_control"
    int_marker.controls.append(copy.deepcopy(control))

    int_marker.scale=0.2

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

    menu_handler.apply( server, int_marker.name )

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()
