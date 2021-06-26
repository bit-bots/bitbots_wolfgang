# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from controller import Supervisor
import trimesh
import traceback
import transforms3d
import math
import numpy as np
import trimesh.viewer
import trimesh.creation
import trimesh.boolean
import json
import time
import os

# Giving a classical name to avoid robot invalid proto when spawned without
# color or id
ROBOT_NAME = "RED_PLAYER_1"

# Values initialized at the beginning of script
MODEL_NAME = None
ROBOT_PATH = None
ROBOT_DIR = None
EXPORT_MF = True


def spawn_robot():
    """Spawn and returns the robot that should be verified"""
    string = f'DEF {ROBOT_NAME} {MODEL_NAME}' \
        '{name "red player 1" translation 0 0 0 rotation 0 0 1 0 controller "void"}'
    s.getRoot().getField('children').importMFNodeFromString(-1, string)
    return s.getFromDef(ROBOT_NAME)


def get_node_desc(node):
    # TODO ideally the node path should be provided but it's unclear if that can still be accessed from the dictionary
    # represenation
    node_type = node[1].get("__type")
    node_name = node[1].get("name", "unknown")
    return f"(name: {node_name}, type: {node_type})"


def build_dict_field(field):
    """
    :type field: Field
    """
    if field is None:
        return None

    type_name = field.getTypeName()
    value_s = None
    if type_name == "SFBool":
        value_s = field.getSFBool()
    elif type_name == "SFInt32":
        value_s = field.getSFInt32()
    elif type_name == "SFFloat":
        value_s = field.getSFFloat()
    elif type_name == "SFVec2f":
        value_s = field.getSFVec2f()
    elif type_name == "SFVec3f":
        value_s = field.getSFVec3f()
    elif type_name == "SFRotation":
        value_s = field.getSFRotation()
    elif type_name == "SFColor":
        value_s = field.getSFColor()
    elif type_name == "SFString":
        value_s = field.getSFString()
    elif type_name == "SFNode":
        value_s = build_dict_node(field.getSFNode())
    elif type_name == "MFNode":
        vals = []
        for i in range(field.getCount()):
            vals.append(build_dict_node(field.getMFNode(i)))
        value_s = vals
    elif EXPORT_MF:
        if type_name == "MFBool":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFBool(i))
            value_s = vals
        elif type_name == "MFInt32":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFInt32(i))
            value_s = vals
        elif type_name == "MFFloat":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFFloat(i))
            value_s = vals
        elif type_name == "MFVec2f":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFVec2f(i))
            value_s = vals
        elif type_name == "MFVec3f":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFVec3f(i))
            value_s = vals
        elif type_name == "MFColor":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFColor(i))
            value_s = vals
        elif type_name == "MFRotation":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFRotation(i))
            value_s = vals
        elif type_name == "MFString":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFString(i))
            value_s = vals
        else:
            print(f"type {type_name} not known")

    return type_name, value_s

def build_dict_node(node):
    """
    :param node: Node
    :return:
    """
    if node is None:
        return {}
    local_fields = {}
    local_fields["__type"] = node.getTypeName()
    nb_fields = node.getProtoNumberOfFields()
    for i in range(nb_fields):
        field = node.getProtoFieldByIndex(i)
        if field is None:
            print(f"None field reached {i+1}/{nb_fields} in {get_node_desc(('SFNode',local_fields))}\n")
            continue
        local_fields[field.getName()] = build_dict_field(field)
    return local_fields


def mesh_to_indexed_face_set(mesh):
    coordIndex = []
    for face in mesh.faces:
        coordIndex.extend(face)
        coordIndex.append(-1)
    coordinates = []
    for vert in mesh.vertices:
        coordinates.append(vert)
    return "SFNode", \
           {"__type": "IndexedFaceSet",
            "coord": ("SFNode", {"__type": "Coordinate", "point": ("MFVec3f", coordinates)}),
            "coordIndex": ("MFInt32", coordIndex)}

def get_physics_nodes(node, transform_list=[],):
    """Return a list of tuples (mass, joint_and_tf, mass_name)"""
    physics_nodes = []
    if node[0] == "SFNode":
        translation = node[1].get("translation")
        rotation = node[1].get("rotation")
        if translation is not None:
            transform_list.append((translation[1], rotation[1]))

        if node[1].get("physics") is not None:
            physics = node[1]["physics"][1]
            if physics != {}:  # empty dict if null
                com = physics["centerOfMass"][1]
                if len(com) == 0:
                    name = node[1].get("name")
                    if name is None:
                        warning("Empty center of mass in unnamed node\n")
                    else:
                        warning(f"Empty center of mass in node {name}\n")
                else:
                    new_tf_list = transform_list[:]
                    new_tf_list.append((com[0], [0, 0, 1, 0]))
                    physics_nodes.append((physics["mass"][1], new_tf_list[:], node[1]["name"]))
        for mass_name in ['gearMass', 'gearMass2']:
            mass_node = node[1].get(mass_name)
            if mass_node is not None:
                physics_nodes.append((mass_node[1], transform_list[:], node[1]["name"]))
        node_type = node[1].get("__type")
        if node_type in JOINT_TYPES:
            motor_name = None
            for device in node[1]["device"][1]:
                if device["__type"] == "RotationalMotor":
                    motor_name = device["name"][1]

            transform_list.append((node_type,
                                   node[1]["jointParameters"][1]["anchor"][1],
                                   node[1]["jointParameters"][1]["axis"][1],
                                   node[1]["position"][1],
                                   motor_name))
        for k, v in node[1].items():
            physics_nodes.extend(get_physics_nodes(v, transform_list[:]))
    elif node[0] == "MFNode":
        for child_node in node[1]:
            physics_nodes.extend(get_physics_nodes(("SFNode", child_node), transform_list[:]))

    return physics_nodes


s = Supervisor()
controller_start = time.time()
try:
    for required_variable in ["ROBOT_NAME", "ROBOT_PATH"]:
        if required_variable not in os.environ:
            raise RuntimeError(f"Environment variable {required_variable} is missing")
    MODEL_NAME = os.environ["ROBOT_NAME"]
    ROBOT_PATH = os.environ["ROBOT_PATH"]
    ROBOT_DIR = os.path.dirname(ROBOT_PATH)

    before_spawning = time.time()
    robot_node = spawn_robot()
    spawn_time = time.time() - before_spawning
    if robot_node is None:
        raise RuntimeError(f"Failed to spawn robot {MODEL_NAME}")
    robot = build_dict_node(robot_node)
except Exception:
    print(f"Failed execution of model verifier with error:\n{traceback.format_exc()}\n")


# todo visual model complexity (count vertices) (optional)
# todo collision model complexity (count primitives) (optional)