#!/usr/bin/python3

from odio_urdf import Group, Material, Color

materials=Group(
    Material("bone", Color(rgba="1 1 1 1")),
    Material("orange", Color(rgba="0.8 0.3 0.0 1.0")),
    Material("blue", Color(rgba="0.176 0.404 0.694 1.0")))