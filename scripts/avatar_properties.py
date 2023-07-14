from odio_urdf import Group, Xacroproperty

xacro_properties=Group(
    Xacroproperty(name = "PI", value = "3.14159265359"),
    Xacroproperty(name = "hips_width", value = "0.18276"))


mesh_dim = {'pelvis': {'hips_width': 0.18276},
            'upper_leg': {'upper_leg_length': 0.444236}}