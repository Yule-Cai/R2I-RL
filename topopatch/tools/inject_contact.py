#!/usr/bin/env python3
"""Inject a Gazebo contact sensor into the waffle base_link.

Reads the xacro-generated robot SDF, appends a contact sensor watching
base_collision, writes the result. Deterministic, no template edits.
Bridged to ROS separately (contact_bridge.yaml).
"""
import sys
import xml.etree.ElementTree as ET

SRC = sys.argv[1] if len(sys.argv) > 1 else \
    "/opt/ros/lyrical/share/nav2_minimal_tb3_sim/urdf/gz_waffle.sdf"
DST = sys.argv[2] if len(sys.argv) > 2 else "/tmp/waffle_contact.sdf"

tree = ET.parse(SRC)
root = tree.getroot()
n = 0
for link in root.iter("link"):
    if link.get("name") == "base_link":
        if link.find("./sensor[@name='base_contact_sensor']") is not None:
            continue
        sens = ET.SubElement(link, "sensor")
        sens.set("name", "base_contact_sensor")
        sens.set("type", "contact")
        ET.SubElement(sens, "topic").text = "/base_contact"
        ET.SubElement(sens, "update_rate").text = "30"
        contact = ET.SubElement(sens, "contact")
        ET.SubElement(contact, "collision").text = "base_collision"
        n += 1
assert n == 1, f"expected 1 base_link, got {n}"
tree.write(DST, xml_declaration=True)
print(f"wrote {DST} (contact sensor injected)")
