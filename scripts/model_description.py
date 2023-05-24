from odio_urdf import *

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
                hips_width,
                torso_height,
                chest_width,
                head_size,
                arm_length,
                forearm_length,
                upperleg_length,
                leg_length,
                link_size):
        
        self.namespace = namespace
        self.color = color
        self.hips_width = hips_width
        self.torso_height = torso_height
        self.chest_width = chest_width
        self.head_size = head_size
        self.arm_length = arm_length
        self.forearm_length = forearm_length
        self.upperleg_length = upperleg_length
        self.leg_length = leg_length
        self.link_size = link_size
        self.model_prefix = model_prefix
        self.link_material = Material(name=self.color.name)

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

        pelvis_mass = 1.23

        # Add torso
        # TODO:
        # - REMOVE SELF MODEL PREFIX
        # - Add yaml file
        self.addLink("base_link")
        self.addFixedJoint(self, "base_link", "pelvis", Origin(rpy="0 0 0",xyz="0 0 0"))
        self.addLink("pelvis", Origin(rpy="0 0 0",xyz="0 0 0"), 
                     Dim(self.link_size, self.hips_width, self.link_size), pelvis_mass)
        self.addLink("l5", Origin(rpy="0 0 0",xyz="0 0 0"), 
                     Dim(self.link_size, self.hips_width, self.link_size), pelvis_mass)
        
        # self.addRevoluteJoint("l5_s1", "pelvis", "l5",
                            #    Origin(rpy="0 0 0",xyz="0 0 0"), Axis(xyz="0 0 1"))


        self.addSphericalJoint("l5_s1", "pelvis", "l5", Origin(rpy="0 0 0",xyz="0 0 0"))
        print(self.human)
        
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
        self.addLink(parent_link_name+"_link_rot_x")
        self.addLink(parent_link_name+"_link_rot_y")
        # self.addLink(parent_link_name+"_link_rot_z")
        
        self.addRevoluteJoint(joint_name+"_x", parent_link_name, parent_link_name+"_link_rot_x",
                            joint_origin, Axis(xyz="1 0 0"))
        self.addRevoluteJoint(joint_name+"_y", parent_link_name+"_link_rot_x", parent_link_name+"_link_rot_y",
                            Origin(), Axis(xyz="0 1 0"))
        self.addRevoluteJoint(joint_name+"_z", parent_link_name+"_link_rot_y", child_link_name,
                            Origin(), Axis(xyz="0 0 1"))
        

    def addRevoluteJoint(self, joint_name, parent_link_name, child_link_name, joint_origin, axis):
        self.human.append(Joint(Parent(self.model_prefix+parent_link_name), 
                                Child(self.model_prefix+child_link_name),
                                joint_origin, axis, Limit(effort="1000", velocity="1000", lower="-2", upper="2"),
                                name=self.model_prefix+joint_name, type="revolute")) 

    def addFixedJoint(self, joint_name, parent_link_name, child_link_name, joint_origin):
        self.human.append(Joint(Parent(self.model_prefix+parent_link_name), 
                                Child(self.model_prefix+child_link_name),
                                joint_origin, name=joint_name, type="fixed"))     
    