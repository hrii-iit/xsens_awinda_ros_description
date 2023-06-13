from odio_urdf import *
import math 

class Dim():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def to_str(self):
        return str(self.x)+" "+str(self.y)+" "+str(self.z)

class XSensAwindaModel():
    def __init__(self, 
                model_prefix,       
                namespace,
                color, 
                link_state_msg):
        
        self.namespace = namespace
        self.color = color
        hips_width = 0.5
        self.hips_width = hips_width
        # self.torso_height = torso_height
        # self.chest_width = chest_width
        # self.head_size = head_size
        # self.arm_length = arm_length
        # self.forearm_length = forearm_length
        # self.upperleg_length = upperleg_length
        # self.leg_length = leg_length
        link_size = 0.05
        
        self.link_size = link_size
        self.model_prefix = model_prefix
        self.link_material = Material(name=self.color.name)
        self.link_state_msg = link_state_msg 

        self.human = Robot(self.namespace, self.color)

    def get_urdf(self):
        return self.human.urdf()

    def generate_model(self):
        print(self.model_prefix)
        
        # self.human = list()

        # Xacro properties
        # self.addXacroProperty("hips_width", self.hips_width)
        # self.addXacroProperty("chest_width", self.chest_width)
        # self.addXacroProperty("head_size", self.head_size)
        # self.addXacroProperty("link_size", self.link_size)
        # self.addXacroProperty("torso_height", self.torso_height)
        # self.addXacroProperty("arm_length", self.arm_length)
        # self.addXacroProperty("forearm_length", self.forearm_length)
        # self.addXacroProperty("upperleg_length", self.upperleg_length)
        # self.addXacroProperty("leg_length", self.leg_length)

        # pelvis_state = self.get_link_state("pelvis")
        # print(pelvis_state)

        # l5_state = self.get_link_state("l5")
        # print(l5_state)

        pelvis_l5_length = self.get_link_length("pelvis", "l5")
        l5_s3_length = self.get_link_length("l5", "l3")
        l3_t12_length = self.get_link_length("l3", "t12")
        t12_t8_length = self.get_link_length("t12", "t8")
        t12_neck_length = self.get_link_length("t8", "neck")
        neck_head_length = self.get_link_length("neck", "head")

        pelvis_mass = 0
        l5_mass = 1.23

        # Add torso
        # TODO:
        # - REMOVE SELF MODEL PREFIX
        # - Add yaml file
        # self.addLink("base_link")
        # self.addFixedJoint("base_link_pelvis", "base_link", "pelvis", Origin(rpy="0 0 0",xyz="0 0 0"))
        # Spinal segments
        self.addLink("pelvis", Origin(), Dim(self.link_size, self.hips_width, self.link_size), pelvis_mass)
        self.addLink("l5", Origin(), Dim(self.link_size, self.link_size, self.link_size), l5_mass)
        self.addLink("l3", Origin(), Dim(self.link_size, self.link_size, self.link_size), l5_mass)
        self.addLink("t12", Origin(), Dim(self.link_size, self.link_size, self.link_size), l5_mass)
        self.addLink("t8", Origin(), Dim(self.link_size, self.link_size, self.link_size), l5_mass)
        self.addLink("neck", Origin(), Dim(self.link_size, self.link_size, self.link_size), l5_mass)
        self.addLink("head", Origin(), Dim(self.link_size, self.link_size, self.link_size), l5_mass)

        self.addSphericalJoint("l5_s1", "pelvis", "l5", Origin(xyz="0 0 "+str(pelvis_l5_length), rpy="0 0 0"))
        self.addSphericalJoint("l4_l3", "l5", "l3", Origin(xyz="0 0 "+str(l5_s3_length), rpy="0 0 0"))
        self.addSphericalJoint("l1_t12", "l3", "t12", Origin(xyz="0 0 "+str(l3_t12_length), rpy="0 0 0"))
        self.addSphericalJoint("t9_t8", "t12", "t8", Origin(xyz="0 0 "+str(t12_t8_length), rpy="0 0 0"))
        self.addSphericalJoint("t1_c7", "t8", "neck", Origin(xyz="0 0 "+str(t12_neck_length), rpy="0 0 0"))
        self.addSphericalJoint("c1_head", "neck", "head", Origin(xyz="0 0 "+str(neck_head_length), rpy="0 0 0"))

        print(self.human)



        # - skeleton_left_ankle_x
        # - skeleton_left_ankle_y
        # - skeleton_left_ankle_z
        # - skeleton_left_ball_foot_x
        # - skeleton_left_ball_foot_y
        # - skeleton_left_ball_foot_z
        # - skeleton_left_c7_shoulder_x
        # - skeleton_left_c7_shoulder_y
        # - skeleton_left_c7_shoulder_z
        # - skeleton_left_elbow_x
        # - skeleton_left_elbow_y
        # - skeleton_left_elbow_z
        # - skeleton_left_hip_x
        # - skeleton_left_hip_y
        # - skeleton_left_hip_z
        # - skeleton_left_knee_x
        # - skeleton_left_knee_y
        # - skeleton_left_knee_z
        # - skeleton_left_shoulder_x
        # - skeleton_left_shoulder_y
        # - skeleton_left_shoulder_z
        # - skeleton_left_wrist_x
        # - skeleton_left_wrist_y
        # - skeleton_left_wrist_z

        # - skeleton_right_ankle_x
        # - skeleton_right_ankle_y
        # - skeleton_right_ankle_z
        # - skeleton_right_ball_foot_x
        # - skeleton_right_ball_foot_y
        # - skeleton_right_ball_foot_z
        # - skeleton_right_c7_shoulder_x
        # - skeleton_right_c7_shoulder_y
        # - skeleton_right_c7_shoulder_z
        # - skeleton_right_elbow_x
        # - skeleton_right_elbow_y
        # - skeleton_right_elbow_z
        # - skeleton_right_hip_x
        # - skeleton_right_hip_y
        # - skeleton_right_hip_z
        # - skeleton_right_knee_x
        # - skeleton_right_knee_y
        # - skeleton_right_knee_z
        # - skeleton_right_shoulder_x
        # - skeleton_right_shoulder_y
        # - skeleton_right_shoulder_z
        # - skeleton_right_wrist_x
        # - skeleton_right_wrist_y
        # - skeleton_right_wrist_z
        
    def addLink(self, link_name, link_origin=None, link_dim=None, mass=0.0):
        if mass == 0.0:
            self.human.append(Link(name=self.model_prefix+link_name))
        else:
            self.human(Link(
                            Inertial(link_origin, 
                                        Mass(mass), 
                                        Inertia(ixx=1.0/12.0*mass*(pow(2*link_dim.y,2) + pow(2*link_dim.z,2)), ixy=0.0, iyz=0.0,
                                                iyy=1.0/12.0*mass*(pow(2*link_dim.x,2) + pow(2*link_dim.z,2)), ixz=0.0,
                                                izz=1.0/12.0*mass*(pow(2*link_dim.x,2) + pow(2*link_dim.y,2)))),
                            Visual(link_origin,
                                self.link_material,
                                Geometry(Box(size=link_dim.to_str()), name=self.model_prefix+"pelvis_visual")),
                            Collision(link_origin,
                                Geometry(Box(size=link_dim.to_str()), name=self.model_prefix+"pelvis_collision")),
                            name=self.model_prefix+link_name))
    
    def addXacroProperty(self, name, value):
        self.human.append(Xacroproperty(name=name, value=str(value)))

    def addSphericalJoint(self, joint_name, parent_link_name, child_link_name, joint_origin):
        self.addLink(parent_link_name+"_"+joint_name+"_x")
        self.addLink(parent_link_name+"_"+joint_name+"_y")

        self.addRevoluteJoint(joint_name+"_x", parent_link_name, parent_link_name+"_"+joint_name+"_x",
                            joint_origin, Axis(xyz="1 0 0"))
        self.addRevoluteJoint(joint_name+"_y", parent_link_name+"_"+joint_name+"_x", parent_link_name+"_"+joint_name+"_y",
                            Origin(rpy="0 0 0",xyz="0 0 0"), Axis(xyz="0 1 0"))
        self.addRevoluteJoint(joint_name+"_z", parent_link_name+"_"+joint_name+"_y", child_link_name,
                            Origin(rpy="0 0 0",xyz="0 0 0"), Axis(xyz="0 0 1"))

    def addRevoluteJoint(self, joint_name, parent_link_name, child_link_name, joint_origin, axis):
        self.human.append(Joint(Parent(self.model_prefix+parent_link_name), 
                                Child(self.model_prefix+child_link_name),
                                joint_origin, axis, Limit(effort="1000", velocity="1000", lower="-2", upper="2"),
                                name=self.model_prefix+joint_name, type="revolute")) 

    def addFixedJoint(self, joint_name, parent_link_name, child_link_name, joint_origin):
        self.human.append(Joint(Parent(self.model_prefix+parent_link_name), 
                                Child(self.model_prefix+child_link_name),
                                joint_origin, name=joint_name, type="fixed"))     
    
    def get_link_state(self, link_name):
        for link_state in self.link_state_msg.states:
            if link_state.header.frame_id == link_name:
                return link_state
        
        return None

    def get_link_length(self, link_name1, link_name2):
        link_pos1 = self.get_link_state(link_name1).pose.position
        link_pos2 = self.get_link_state(link_name2).pose.position

        return math.sqrt(pow(link_pos1.x-link_pos2.x,2) + pow(link_pos1.y-link_pos2.y,2) + pow(link_pos1.z-link_pos2.z,2))
