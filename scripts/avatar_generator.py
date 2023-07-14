#!/usr/bin/python3

import rospy
from xsens_mvn_ros_msgs.msg import LinkStateArray
from avatar import XSensAwindaModel
import odio_urdf as urdf

if __name__ == '__main__':

    rospy.init_node('urdf_model_generator', anonymous=True)

    # Get ROS params
    model_name = rospy.get_param("~model_name", "human")
    link_state_topic = rospy.get_param("~link_state_topic", "link_states")

    # Wait for link state message
    rospy.loginfo("Waiting for message from \""+link_state_topic+"\" ROS topic...")
    link_state_msg = rospy.wait_for_message("/xsens/link_states", LinkStateArray)
    rospy.loginfo("Message from \""+link_state_topic+"\" ROS topic received.")

    xsens_awinda_model = XSensAwindaModel(model_name, 
                                        link_state_msg)

    xsens_awinda_model.generate_avatar()

    rospy.set_param('robot_description', xsens_awinda_model.get_urdf())