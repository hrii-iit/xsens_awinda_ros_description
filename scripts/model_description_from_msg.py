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
        link_size = 0.1
        
        self.link_size = link_size
        self.model_prefix = model_prefix
        self.link_material = Material(name=self.color.name)
        self.link_state_msg = link_state_msg 

        self.human = Robot(self.namespace, self.color)

    def get_urdf(self):
        return self.human.urdf()

    def generate_model(self):
        print(self.model_prefix)
        
        self.pelvis_mass = 1
        self.l5_mass = 1.23

        # Add torso
        # TODO:
        # - REMOVE SELF MODEL PREFIX
        # - Add yaml file
        self.add_link("base_link")
        self.addFixedJoint("base_link_pelvis", "base_link", "pelvis", Origin())

        # Spinal segments MVN manual section 23.6.1 pag. 140 
        self.add_spinal_segments()

        # Arm segments
        self.add_arm("right", "-")
        self.add_arm("left", "+")

        # Leg segments
        self.add_leg("right", "-")
        self.add_leg("left", "+")
        
        # print(self.human)

    def add_arm(self, prefix, multiplier):
        # Compute arm bone lengths
        t8_c7_shoulder_length = self.get_link_length("t8", prefix+"_shoulder")
        c7_shoulder_length = self.get_link_length(prefix+"_shoulder", prefix+"_upper_arm")
        upper_arm_forearm_length = self.get_link_length(prefix+"_upper_arm", prefix+"_forearm")
        forearm_hand_length = self.get_link_length(prefix+"_forearm", prefix+"_hand")

        # Add links
        self.add_link(prefix+"_shoulder", Origin(), Dim(self.link_size, self.hips_width, self.link_size), self.pelvis_mass, "package://xsens_mvn_ros_description/meshes/arm_leg_bone.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*t8_c7_shoulder_length))
        self.add_link(prefix+"_upper_arm", Origin(), Dim(self.link_size, self.link_size, self.link_size), self.l5_mass, "package://xsens_mvn_ros_description/meshes/arm_leg_bone.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*upper_arm_forearm_length))
        self.add_link(prefix+"_forearm", Origin(), Dim(self.link_size, self.link_size, self.link_size), self.l5_mass, "package://xsens_mvn_ros_description/meshes/arm_leg_bone.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*forearm_hand_length))
        self.add_link(prefix+"_hand", Origin(rpy="0 3.1415 ${-1.57075"+multiplier+"1.57075}"), Dim(self.link_size, self.link_size, self.link_size), self.l5_mass, "package://xsens_mvn_ros_description/meshes/hand.stl", str(0.01*self.link_size)+" "+str(0.01*self.link_size)+" "+str(0.005*forearm_hand_length))

        # Add joints
        self.add_spherical_joint(prefix+"_c7_shoulder", "t8", prefix+"_shoulder", Origin(xyz="0 "+multiplier+str(t8_c7_shoulder_length)+" 0", rpy="0 0 0"))
        self.add_spherical_joint(prefix+"_shoulder", prefix+"_shoulder", prefix+"_upper_arm", Origin(xyz="0 "+multiplier+str(c7_shoulder_length)+" 0", rpy="0 0 0"))
        self.add_spherical_joint(prefix+"_elbow", prefix+"_upper_arm", prefix+"_forearm", Origin(xyz="0 0 -"+str(upper_arm_forearm_length), rpy="0 0 0"))
        self.add_spherical_joint(prefix+"_wrist", prefix+"_forearm", prefix+"_hand", Origin(xyz="0 0 -"+str(forearm_hand_length), rpy="0 0 0"))

    def add_leg(self, prefix, multiplier):
        # Compute leg bone lengths
        pelvis_upper_leg_length = self.get_link_length("pelvis", prefix+"_upper_leg")
        upper_leg_lower_leg_length = self.get_link_length(prefix+"_upper_leg", prefix+"_lower_leg")
        lower_leg_foot_length = self.get_link_length(prefix+"_lower_leg", prefix+"_foot")
        foot_toe_length = self.get_link_length(prefix+"_foot", prefix+"_toe")

        # Add links
        self.add_link(prefix+"_upper_leg", Origin(), Dim(self.link_size, pelvis_upper_leg_length, self.link_size), self.pelvis_mass, "package://xsens_mvn_ros_description/meshes/arm_leg_bone.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*upper_leg_lower_leg_length))
        self.add_link(prefix+"_lower_leg", Origin(), Dim(self.link_size, self.link_size, upper_leg_lower_leg_length), self.l5_mass, "package://xsens_mvn_ros_description/meshes/arm_leg_bone.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*lower_leg_foot_length))
        self.add_link(prefix+"_foot", Origin(rpy="0 0 1.57075"), Dim(self.link_size, self.link_size, lower_leg_foot_length), self.l5_mass, "package://xsens_mvn_ros_description/meshes/foot.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*lower_leg_foot_length))
        self.add_link(prefix+"_toe", Origin(), Dim(foot_toe_length, self.link_size, self.link_size), self.l5_mass, "package://xsens_mvn_ros_description/meshes/toe.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*foot_toe_length))

        # Add joints
        self.add_spherical_joint(prefix+"_hip", "pelvis", prefix+"_upper_leg", Origin(xyz="0 "+multiplier+str(pelvis_upper_leg_length)+" 0", rpy="0 0 0"))
        self.add_spherical_joint(prefix+"_knee", prefix+"_upper_leg", prefix+"_lower_leg", Origin(xyz="0 0 -"+str(upper_leg_lower_leg_length), rpy="0 0 0"))
        self.add_spherical_joint(prefix+"_ankle", prefix+"_lower_leg", prefix+"_foot", Origin(xyz="0 0 -"+str(lower_leg_foot_length), rpy="0 0 0"))
        self.add_spherical_joint(prefix+"_ball_foot", prefix+"_foot", prefix+"_toe", Origin(xyz=str(foot_toe_length)+" 0 0", rpy="0 0 0"))

    def add_spinal_segments(self, ):
        # Compute spinal bone lengths
        pelvis_l5_length = self.get_link_length("pelvis", "l5")
        l5_s3_length = self.get_link_length("l5", "l3")
        l3_t12_length = self.get_link_length("l3", "t12")
        t12_t8_length = self.get_link_length("t12", "t8")
        t8_neck_length = self.get_link_length("t8", "neck")
        neck_head_length = self.get_link_length("neck", "head")

        # Add links
        self.add_link("pelvis", Origin(), Dim(self.link_size, pelvis_l5_length*2, self.link_size), self.pelvis_mass)
        self.add_link("l5", Origin(), Dim(self.link_size, self.link_size, l5_s3_length), self.l5_mass, "package://xsens_mvn_ros_description/meshes/spinal_segment.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*l5_s3_length))
        self.add_link("l3", Origin(), Dim(self.link_size, self.link_size, l3_t12_length), self.l5_mass, "package://xsens_mvn_ros_description/meshes/spinal_segment.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*l3_t12_length))
        self.add_link("t12", Origin(), Dim(self.link_size, self.link_size, t12_t8_length), self.l5_mass, "package://xsens_mvn_ros_description/meshes/spinal_segment.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*t12_t8_length))
        self.add_link("t8", Origin(), Dim(self.link_size, self.link_size, t8_neck_length), self.l5_mass, "package://xsens_mvn_ros_description/meshes/chest.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*t8_neck_length))
        self.add_link("neck", Origin(), Dim(self.link_size, self.link_size, neck_head_length), self.l5_mass, "package://xsens_mvn_ros_description/meshes/spinal_segment.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*neck_head_length))
        self.add_link("head", Origin(), Dim(self.link_size, self.link_size, self.link_size), self.l5_mass, "package://xsens_mvn_ros_description/meshes/head.stl", str(0.01*neck_head_length*0.7)+" "+str(0.01*neck_head_length*0.7)+" "+str(0.01*neck_head_length))

        # Add spherical joints
        # self.add_spherical_joint("l5_s1", "pelvis", "l5", Origin(xyz="0 0 "+str(pelvis_l5_length), rpy="0 0 0"))
        # self.add_spherical_joint("l4_l3", "l5", "l3", Origin(xyz="0 0 "+str(l5_s3_length), rpy="0 0 0"))
        # self.add_spherical_joint("l3_t12", "l3", "t12", Origin(xyz="0 0 "+str(l3_t12_length), rpy="0 0 0"))
        # self.add_spherical_joint("t9_t8", "t12", "t8", Origin(xyz="0 0 "+str(t12_t8_length), rpy="0 0 0"))
        # self.add_spherical_joint("t1_c7", "t8", "neck", Origin(xyz="0 0 "+str(t8_neck_length), rpy="0 0 0"))
        # self.add_spherical_joint("c1_head", "neck", "head", Origin(xyz="0 0 "+str(neck_head_length), rpy="0 0 0"))
        self.add_spherical_joint("pelvis_l5", "pelvis", "l5", Origin(xyz="0 0 "+str(pelvis_l5_length), rpy="0 0 0"))
        self.add_spherical_joint("l5_l3", "l5", "l3", Origin(xyz="0 0 "+str(l5_s3_length), rpy="0 0 0"))
        self.add_spherical_joint("l3_t12", "l3", "t12", Origin(xyz="0 0 "+str(l3_t12_length), rpy="0 0 0"))
        self.add_spherical_joint("t9_t8", "t12", "t8", Origin(xyz="0 0 "+str(t12_t8_length), rpy="0 0 0"))
        self.add_spherical_joint("t1_c7", "t8", "neck", Origin(xyz="0 0 "+str(t8_neck_length), rpy="0 0 0"))
        self.add_spherical_joint("c1_head", "neck", "head", Origin(xyz="0 0 "+str(neck_head_length), rpy="0 0 0"))

        
    def add_link(self, link_name, link_origin=None, link_dim=None, mass=0.0, mesh_filename="package://xsens_mvn_ros_description/meshes/spinal_segment.stl", scale="0.001 0.001 0.001"):
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
                                # Geometry(Box(size=link_dim.to_str()), name=self.model_prefix+"pelvis_visual")),
                                Geometry(Mesh(filename=mesh_filename, scale=scale), name=self.model_prefix+"pelvis_visual")),
                            Collision(link_origin,
                                Geometry(Box(size=link_dim.to_str()), name=self.model_prefix+"pelvis_collision")),
                            name=self.model_prefix+link_name))
    
    def addXacroProperty(self, name, value):
        self.human.append(Xacroproperty(name=name, value=str(value)))

    def add_spherical_joint(self, joint_name, parent_link_name, child_link_name, joint_origin):
        self.add_link(parent_link_name+"_"+joint_name+"_x")
        self.add_link(parent_link_name+"_"+joint_name+"_y")

        self.add_revolute_joint(joint_name+"_x", parent_link_name, parent_link_name+"_"+joint_name+"_x",
                            joint_origin, Axis(xyz="1 0 0"))
        self.add_revolute_joint(joint_name+"_y", parent_link_name+"_"+joint_name+"_x", parent_link_name+"_"+joint_name+"_y",
                            Origin(rpy="0 0 0",xyz="0 0 0"), Axis(xyz="0 1 0"))
        self.add_revolute_joint(joint_name+"_z", parent_link_name+"_"+joint_name+"_y", child_link_name,
                            Origin(rpy="0 0 0",xyz="0 0 0"), Axis(xyz="0 0 1"))

    def add_revolute_joint(self, joint_name, parent_link_name, child_link_name, joint_origin, axis):
        self.human.append(Joint(Parent(self.model_prefix+parent_link_name), 
                                Child(self.model_prefix+child_link_name),
                                joint_origin, axis, Limit(effort="1000", velocity="1000", lower="-2", upper="2"),
                                name=self.model_prefix+joint_name, type="revolute")) 

    def addFixedJoint(self, joint_name, parent_link_name, child_link_name, joint_origin):
        self.human.append(Joint(Parent(self.model_prefix+parent_link_name), 
                                Child(self.model_prefix+child_link_name),
                                joint_origin, name=joint_name, type="fixed"))     
    
    def get_link_state(self, link_name):
        # return None
        for link_state in self.link_state_msg.states:
            if link_state.header.frame_id == link_name:
                return link_state
        
        return None

    def get_link_length(self, link_name1, link_name2):
        # return 0.2
        link_pos1 = self.get_link_state(link_name1).pose.position
        link_pos2 = self.get_link_state(link_name2).pose.position

        return math.sqrt(pow(link_pos1.x-link_pos2.x,2) + pow(link_pos1.y-link_pos2.y,2) + pow(link_pos1.z-link_pos2.z,2))
