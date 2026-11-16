"""GLB collision reader extracted verbatim from existing prepared-scene loaders.

Only read_glb/accessor/load_triangles are included; no scene construction code.
"""

import json
import struct
import numpy as np
from scipy.spatial.transform import Rotation

def read_glb(path):
    blob = path.read_bytes()
    assert struct.unpack_from("<4sII", blob) == (b"glTF", 2, len(blob)), path
    length, kind = struct.unpack_from("<I4s", blob, 12)
    assert kind == b"JSON"
    start = 20 + length
    size, kind = struct.unpack_from("<I4s", blob, start)
    assert kind == b"BIN\0" and start + 8 + size == len(blob)
    return json.loads(blob[20:start]), bytearray(blob[start + 8:])

def accessor(doc, blob, index):
    field = doc["accessors"][index]
    view = doc["bufferViews"][field["bufferView"]]
    dtype = np.dtype({5121: "u1", 5123: "<u2", 5125: "<u4", 5126: "<f4"}[field["componentType"]])
    width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[field["type"]]
    assert "sparse" not in field and view["buffer"] == 0
    return np.ndarray((field["count"], width), dtype, blob,
                      offset=view.get("byteOffset", 0) + field.get("byteOffset", 0),
                      strides=(view.get("byteStride", dtype.itemsize * width), dtype.itemsize))

def load_triangles(path):
    doc, blob = read_glb(path)
    pieces = []

    def visit(index, parent):
        node = doc["nodes"][index]
        if "matrix" in node:
            transform = np.asarray(node["matrix"], dtype=float).reshape(4, 4, order="F")
        else:
            transform = np.eye(4)
            rotation = Rotation.from_quat(node.get("rotation", [0, 0, 0, 1])).as_matrix()
            transform[:3, :3] = rotation * node.get("scale", [1, 1, 1])
            transform[:3, 3] = node.get("translation", [0, 0, 0])
        placement = parent @ transform
        if "mesh" in node:
            for primitive in doc["meshes"][node["mesh"]]["primitives"]:
                assert primitive.get("mode", 4) == 4
                coords = accessor(doc, blob, primitive["attributes"]["POSITION"])
                indices = accessor(doc, blob, primitive["indices"]).reshape(-1, 3)
                world = coords @ placement[:3, :3].T + placement[:3, 3]
                pieces.append(world[indices])
        for child in node.get("children", []):
            visit(child, placement)

    for index in doc["scenes"][doc.get("scene", 0)]["nodes"]:
        visit(index, np.eye(4))
    return np.concatenate(pieces)
