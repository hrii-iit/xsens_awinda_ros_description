from odio_urdf import *
import avatar_materials
import avatar_properties
import math 


package_path = 'package://xsens_mvn_ros_description'
separator = '_'

class Dim():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def to_str(self):
        return str(self.x)+" "+str(self.y)+" "+str(self.z)

class XSensAwindaModel():
    def __init__(self, 
                avatar_prefix,
                link_state_msg):
        
        self.avatar_prefix = avatar_prefix
        self.link_state_msg = link_state_msg 

        self.avatar = Robot(avatar_properties.xacro_properties, 
                           avatar_materials.materials, 
                           name = avatar_prefix)

    def get_urdf(self):
        return self.avatar.urdf()

    def generate_avatar(self):
        
        self.add_link("base_link")
        self.addFixedJoint("base_link_pelvis", "base_link", "pelvis", Origin())

        # Spinal segments MVN manual section 23.6.1 pag. 140 
        self.add_spinal_segments()
        
        self.add_leg('right')
        self.add_leg('left')        
        
        print(self.avatar)
        
    def add_spinal_segments(self):
        pelvis_width = self.get_link_length("right_upper_leg", "left_upper_leg")
        
        # TODO 
        # Use anthropometric tables to calculate links' masses
        pelvis_mass = 1.0

        self.add_link("pelvis", Origin(), Dim(0.1, pelvis_width, 0.1), pelvis_mass, scale_factor = pelvis_width/avatar_properties.mesh_dim['pelvis']['hips_width'])




        


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

    def add_leg(self, prefix):
        
        multiplier = '+'
        
        if prefix == 'right':
            multiplier = '-'
        elif prefix == 'left':
            multiplier = '+'
        
        # Compute leg bone lengths
        pelvis_width = self.get_link_length("right_upper_leg", "left_upper_leg")
        upper_leg_length = self.get_link_length(prefix + "_upper_leg", prefix + "_lower_leg")
        # lower_leg_foot_length = self.get_link_length(prefix+"_lower_leg", prefix+"_foot")
        # foot_toe_length = self.get_link_length(prefix+"_foot", prefix+"_toe")
        
        # TODO 
        # Use anthropometric tables to calculate links' masses
        upper_leg_mass = 1.0

        # Add links
        self.add_link(prefix + "_upper_leg", Origin(), Dim(0.1, 0.1, upper_leg_length), upper_leg_mass, scale_factor = upper_leg_length/avatar_properties.mesh_dim['upper_leg']['upper_leg_length'])
        
        # self.add_link(prefix+"_lower_leg", Origin(), Dim(self.link_size, self.link_size, upper_leg_lower_leg_length), self.l5_mass, "package://xsens_mvn_ros_description/meshes/arm_leg_bone.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*lower_leg_foot_length))
        # self.add_link(prefix+"_foot", Origin(rpy="0 0 1.57075"), Dim(self.link_size, self.link_size, lower_leg_foot_length), self.l5_mass, "package://xsens_mvn_ros_description/meshes/foot.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*lower_leg_foot_length))
        # self.add_link(prefix+"_toe", Origin(), Dim(foot_toe_length, self.link_size, self.link_size), self.l5_mass, "package://xsens_mvn_ros_description/meshes/toe.stl", str(0.02*self.link_size)+" "+str(0.02*self.link_size)+" "+str(0.01*foot_toe_length))

        # Add joints
        self.add_spherical_joint(prefix + "_hip", "pelvis", prefix + "_upper_leg", Origin(xyz="0 " + multiplier + str(pelvis_width/2) + " 0", rpy = "0 0 0"))
        # self.add_spherical_joint(prefix+"_knee", prefix+"_upper_leg", prefix+"_lower_leg", Origin(xyz="0 0 -"+str(upper_leg_lower_leg_length), rpy="0 0 0"))
        # self.add_spherical_joint(prefix+"_ankle", prefix+"_lower_leg", prefix+"_foot", Origin(xyz="0 0 -"+str(lower_leg_foot_length), rpy="0 0 0"))
        # self.add_spherical_joint(prefix+"_ball_foot", prefix+"_foot", prefix+"_toe", Origin(xyz=str(foot_toe_length)+" 0 0", rpy="0 0 0"))


    def add_link(self, link_name, link_origin = None, link_dim = None, mass = 0.0, scale_factor = 1):
        """
        Adds a link to the urdf tree structure using the odio_urdf library syntax.

        Args:
            link_name (string): Name of the link.
            link_origin (Origin(xyz="x y z", rpy="r p y"), optional): Frame origin of the link (default is None).
            link_dim (Dim(x,y,z), optional): Approximate x,y,z dimensions of the link (default is None).
            mass (float, optional): Mass of the link (default is 0)
            scale_factor: scale multiplication factor (default is 1)
        """
        if mass == 0.0:
            self.avatar(Link(name = '_'.join([self.avatar_prefix, link_name])))
        else:
            
            scale = [scale_factor] * 3 
            
            self.avatar(Link(
                            Inertial(link_origin, 
                                        Mass(mass), 
                                        Inertia(ixx=1.0/12.0*mass*(pow(2*link_dim.y,2) + pow(2*link_dim.z,2)), ixy=0.0, iyz=0.0,
                                                iyy=1.0/12.0*mass*(pow(2*link_dim.x,2) + pow(2*link_dim.z,2)), ixz=0.0,
                                                izz=1.0/12.0*mass*(pow(2*link_dim.x,2) + pow(2*link_dim.y,2)))),
                            Visual(link_origin, Material(name='bone'), Geometry(Mesh(filename = package_path + '/meshes/visual/' + link_name + '.stl', scale = scale), name = '_'.join([self.avatar_prefix, "pelvis_visual"]))),
                            Visual(link_origin, Material(name='blue'), Geometry(Mesh(filename = package_path + '/meshes/visual/' + link_name + '_armour.stl', scale = scale), name = '_'.join([self.avatar_prefix, "pelvis_armour"]))),
                            Collision(link_origin, Geometry(Box(size = link_dim.to_str()), name = '_'.join([self.avatar_prefix, "pelvis_collision"]))), name = '_'.join([self.avatar_prefix, link_name])))

    def add_spherical_joint(self, joint_name, parent_link_name, child_link_name, joint_origin):
        
        self.add_link('_'.join([parent_link_name, joint_name, "x"]))
        self.add_link('_'.join([parent_link_name, joint_name, "y"]))

        self.add_revolute_joint(joint_name + "_x", parent_link_name, parent_link_name + "_" + joint_name + "_x", joint_origin, Axis(xyz="1 0 0"))
        
        self.add_revolute_joint(joint_name + "_y", parent_link_name + "_" + joint_name + "_x", parent_link_name + "_" + joint_name + "_y", Origin(rpy="0 0 0",xyz="0 0 0"), Axis(xyz="0 1 0"))
        
        self.add_revolute_joint(joint_name + "_z", parent_link_name + "_" + joint_name + "_y", child_link_name, Origin(rpy="0 0 0",xyz="0 0 0"), Axis(xyz="0 0 1"))

    def add_revolute_joint(self, joint_name, parent_link_name, child_link_name, joint_origin, axis):
        self.avatar(Joint(Parent('_'.join([self.avatar_prefix, parent_link_name])), 
                                Child('_'.join([self.avatar_prefix, child_link_name])),
                                joint_origin, axis, Limit(effort="1000", velocity="1000", lower="-2", upper="2"),
                                name = '_'.join([self.avatar_prefix, joint_name]), type="revolute")) 

    def addFixedJoint(self, joint_name, parent_link_name, child_link_name, joint_origin):
        self.avatar.append(Joint(Parent('_'.join([self.avatar_prefix, parent_link_name])), 
                                Child('_'.join([self.avatar_prefix, child_link_name])),
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
