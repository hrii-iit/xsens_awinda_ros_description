import odio_urdf as urdf

class XSensAwindaModel():
    def __init__(self, namespace,
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
        
        self.human = urdf.Robot(namespace, self.color)
    
    def human_base(self):
        return urdf.Group(
            urdf.Xacroproperty(name = "PI",             value = "3.14159265359"),
            urdf.Xacroproperty(name= "hips_width",      value = str(self.hips_width)),
            urdf.Xacroproperty(name= "chest_width",     value = str(self.chest_width)),
            urdf.Xacroproperty(name= "head_size",       value = str(self.head_size)),
            urdf.Xacroproperty(name= "link_size",       value = str(self.link_size)),
            urdf.Xacroproperty(name= "torso_height",    value = str(self.torso_height)),
            urdf.Xacroproperty(name= "arm_length",      value = str(self.arm_length)),
            urdf.Xacroproperty(name= "forearm_length",  value = str(self.forearm_length)),
            urdf.Xacroproperty(name= "upperleg_length", value = str(self.upperleg_length)),
            urdf.Xacroproperty(name= "leg_length",      value = str(self.leg_length)),

            urdf.Link(name="base_link"),
            urdf.Link(
                urdf.Visual(
                    urdf.Origin(rpy = "0 0 0", xyz = "0 0 0"),
                    urdf.Material(name = self.color.name),
                    urdf.Geometry(urdf.Box(size = "${link_size} ${hips_width} ${link_size}"), name = "pelvis_visual")),
                name="pelvis"),
                
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 ${torso_height/2}"),
                Material(name = self.color.name),
                Geometry(Box(size = "${link_size} ${link_size} ${torso_height}"), name = "torso_visual")
                ),
            name = "torso"
            ),
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 0"),
                Material(name = self.color.name),
                Geometry(Box(size = "${link_size} ${chest_width} ${link_size}"), name = "chest_visual")
                ),
            name = "chest"
            ),
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 ${head_size}"),
                Material(name = self.color.name),
                Geometry(Sphere(radius = "${head_size}"), name = "head_visual")
            ),
            name = "head"
            ),
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 -${arm_length/2}"),
                Material(name = self.color.name),
                Geometry(Cylinder(radius = "${link_size/2}", length="${arm_length}"), name = "r_arm_visual")
            ),
            name = "r_arm"
            ),
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 -${arm_length/2}"),
                Material(name = self.color.name),
                Geometry(Cylinder(radius = "${link_size/2}", length="${arm_length}"), name = "l_arm_visual")
            ),
            name = "l_arm"
            ),
        
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 -${forearm_length/2}"),
                Material(name = self.color.name),
                Geometry(Cylinder(radius = "${link_size/2}", length="${forearm_length}"), name = "r_forearm_visual")
            ),
            name = "r_forearm"
            ),
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 -${forearm_length/2}"),
                Material(name = self.color.name),
                Geometry(Cylinder(radius = "${link_size/2}", length="${forearm_length}"), name = "l_forearm_visual")
            ),
            name = "l_forearm"
            ),
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 -${upperleg_length/2}"),
                Material(name = self.color.name),
                Geometry(Cylinder(radius = "${link_size/2}", length="${upperleg_length}"), name = "r_upperleg_visual")
            ),
            name = "r_upperleg"
            ),
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 -${upperleg_length/2}"),
                Material(name = self.color.name),
                Geometry(Mesh(scale= "0.03937 0.03937 0.03937", filename= "package://xsens_mvn_ros/meshes/visual/l_femur.stl"), name= "l_femur")
            ),
            name = "l_upperleg"
            ),
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 -${leg_length/2}"),
                Material(name = self.color.name),
                Geometry(Cylinder(radius = "${link_size/2}", length="${leg_length}"), name = "r_leg_visual")
            ),
            name = "r_leg"
            ),
        Link(
            Visual(
                Origin(rpy = "0 0 0", xyz = "0 0 -${leg_length/2}"),
                Material(name = self.color.name),
                Geometry(Cylinder(radius = "${link_size/2}", length="${leg_length}"), name = "l_leg_visual")
            ),
            name = "l_leg"
            ),
        Joint(
            Parent(link = "base_link"),
            Child(link = "pelvis"),
            Origin(rpy = "0 0 0", xyz = "0 0 0"),
            name = "base_joint",
            type = "fixed"
        ),
        Joint(
            Parent(link = "pelvis"),
            Child(link = "torso"),
            Origin(rpy = "0 0 0", xyz = "0 0 ${link_size/2}"),
            name = "torso_joint",
            type= "fixed"
        ),
        Joint(
            Parent(link = "torso"),
            Child(link = "chest"),
            Origin(rpy = "0 0 0", xyz = "0 0 ${torso_height + link_size/2}"),
            name = "chest_joint",
            type= "fixed"
        ),
        Joint(
            Parent(link = "chest"),
            Child(link = "head"),
            Origin(rpy = "0 0 0", xyz = "0 0 ${link_size/2}"),
            name = "head_joint",
            type= "fixed"
        ),
        Joint(
            Parent(link = "chest"),
            Child(link = "r_arm"),
            Origin(rpy = "0 0 0", xyz = "0 -${chest_width/2} 0"),
            name = "r_shoulder_joint",
            type= "fixed"
        ),
        Joint(
            Parent(link = "chest"),
            Child(link = "l_arm"),
            Origin(rpy = "0 0 0", xyz = "0 ${chest_width/2} 0"),
            name = "l_shoulder_joint",
            type= "fixed"
        ),
        Joint(
            Parent(link = "r_arm"),
            Child(link = "r_forearm"),
            Origin(rpy = "0 0 0", xyz = "0 0 -${arm_length}"),
            name = "r_elbow_joint",
            type= "fixed"
        ),
        Joint(
            Parent(link = "l_arm"),
            Child(link = "l_forearm"),
            Origin(rpy = "0 0 0", xyz = "0 0 -${arm_length}"),
            name = "l_elbow_joint",
            type= "fixed"
        ),
        Joint(
            Parent(link = "pelvis"),
            Child(link = "r_upperleg"),
            Origin(rpy = "0 0 0", xyz = "0 -${hips_width/2} 0"),
            name = "r_hip_joint",
            type= "fixed"
        ),
        Joint(
            Parent(link = "pelvis"),
            Child(link = "l_upperleg"),
            Origin(rpy = "0 0 0", xyz = "0 ${hips_width/2} 0"),
            name = "l_hip_joint",
            type= "fixed"
        ),
        Joint(
            Parent(link = "r_upperleg"),
            Child(link = "r_leg"),
            Origin(rpy = "0 0 0", xyz = "0 0 -${upperleg_length}"),
            name = "r_knee_joint",
            type= "fixed"
        ),
        Joint(
            Parent(link = "l_upperleg"),
            Child(link = "l_leg"),
            Origin(rpy = "0 0 0", xyz = "0 0 -${upperleg_length}"),
            name = "l_knee_joint",
            type= "fixed"
        )
        
        
    )