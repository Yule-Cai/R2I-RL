"""Bring one content-pinned static HSSD scene into the existing Gazebo runtime."""
import argparse
import hashlib
from io import BytesIO
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from prepare_scene import add


Y_UP_TO_Z_UP = Rotation.from_euler("x", math.pi / 2)
TEXTURE_EXTENSION = "KHR_texture_basisu"


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def read_glb(path):
    blob = path.read_bytes()
    assert struct.unpack_from("<4sII", blob) == (b"glTF", 2, len(blob)), path
    length, kind = struct.unpack_from("<I4s", blob, 12)
    assert kind == b"JSON"
    start = 20 + length
    size, kind = struct.unpack_from("<I4s", blob, start)
    assert kind == b"BIN\0" and start + 8 + size == len(blob)
    return json.loads(blob[20:start]), bytearray(blob[start + 8:])


def aligned(data, padding=b"\0"):
    return data + padding * (-len(data) % 4)


def write_glb(dest, doc, blob):
    doc["buffers"][0]["byteLength"] = len(blob)
    header = aligned(json.dumps(doc, separators=(",", ":")).encode(), b" ")
    with dest.open("xb") as stream:
        stream.write(struct.pack("<4sIII4s", b"glTF", 2, 28 + len(header) + len(blob), len(header), b"JSON"))
        stream.write(header)
        stream.write(struct.pack("<I4s", len(blob), b"BIN\0"))
        stream.write(blob)


def accessor(doc, blob, index):
    field = doc["accessors"][index]
    view = doc["bufferViews"][field["bufferView"]]
    dtype = np.dtype({5121: "u1", 5123: "<u2", 5125: "<u4", 5126: "<f4"}[field["componentType"]])
    width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[field["type"]]
    assert "sparse" not in field and view["buffer"] == 0
    return np.ndarray((field["count"], width), dtype, blob,
                      offset=view.get("byteOffset", 0) + field.get("byteOffset", 0),
                      strides=(view.get("byteStride", dtype.itemsize * width), dtype.itemsize))


def mirror_opening(source, dest):
    doc, blob = read_glb(source)
    assert all(not set(node) & {"matrix", "scale", "rotation", "translation"} for node in doc["nodes"])
    mirrored_attributes, reversed_indices = set(), set()
    for geom in doc["meshes"]:
        for primitive in geom["primitives"]:
            assert primitive.get("mode", 4) == 4
            for label, index in primitive["attributes"].items():
                if label not in {"POSITION", "NORMAL", "TANGENT"} or index in mirrored_attributes:
                    continue
                mirrored_attributes.add(index)
                array = accessor(doc, blob, index)
                array[:, 0] *= -1
                if label == "TANGENT":
                    array[:, 3] *= -1
                field = doc["accessors"][index]
                if "min" in field:
                    field["min"], field["max"] = array.min(axis=0).tolist(), array.max(axis=0).tolist()
            index = primitive["indices"]
            if index not in reversed_indices:
                reversed_indices.add(index)
                triangles = accessor(doc, blob, index).reshape(-1, 3)
                triangles[:, [1, 2]] = triangles[:, [2, 1]]
    write_glb(dest, doc, blob)


def reencode_textures(source, dest, runtime):
    doc, blob = read_glb(source)
    images = doc.get("images", [])
    indices = [i for i, image in enumerate(images) if image["mimeType"] == "image/ktx2"]
    if not indices:
        shutil.copyfile(source, dest)
        return 0

    with TemporaryDirectory() as work:
        decoder = Path(__file__).with_name("decode_hssd_ktx2.cjs")
        sizes = json.loads(subprocess.check_output(
            ["node", str(decoder), str(runtime), str(source), work, *map(str, indices)], text=True
        ))
        for index in indices:
            width, height = sizes[str(index)]
            rgba = (Path(work) / f"{index}.rgba").read_bytes()
            image = Image.frombytes("RGBA", (width, height), rgba)
            if image.getextrema()[3] == (255, 255):
                image = image.convert("RGB")
            png = BytesIO()
            image.save(png, format="PNG")
            contents = png.getvalue()
            doc["bufferViews"].append(dict(buffer=0, byteOffset=len(blob), byteLength=len(contents)))
            images[index].update(bufferView=len(doc["bufferViews"]) - 1, mimeType="image/png")
            blob.extend(aligned(contents))

    for texture in doc["textures"]:
        extensions = texture.get("extensions", {})
        encoded = extensions.pop(TEXTURE_EXTENSION, None)
        if encoded:
            texture["source"] = encoded["source"]
        if not extensions:
            texture.pop("extensions", None)
    for key in ("extensionsUsed", "extensionsRequired"):
        if key in doc:
            doc[key] = [name for name in doc[key] if name != TEXTURE_EXTENSION]

    write_glb(dest, doc, blob)
    return len(indices)


def collision_mesh(source, dest):
    doc, blob = read_glb(source)
    for field in ("images", "textures", "samplers", "materials"):
        doc.pop(field, None)
    for group in doc["meshes"]:
        for primitive in group["primitives"]:
            primitive.pop("material", None)
    for key in ("extensionsUsed", "extensionsRequired"):
        if key in doc:
            doc[key] = [name for name in doc[key] if
                        not name.startswith(("KHR_materials_", "KHR_texture_"))]
    write_glb(dest, doc, blob)


def mesh(link, role, name, uri, scale):
    geom = add(add(add(link, role, name=name), "geometry"), "mesh")
    add(geom, "uri", uri)
    add(geom, "scale", " ".join(map(str, scale)))


def insert(world, scene_id, label, instance, render, collision):
    scale = instance.get("non_uniform_scale", [1, 1, 1])
    scale = np.asarray(scale) * instance.get("uniform_scale", 1)
    assert all(scale != 0)
    source_wxyz = instance.get("rotation", [1, 0, 0, 0])
    source_rotation = Rotation.from_quat([*source_wxyz[1:], source_wxyz[0]])
    rotation = Y_UP_TO_Z_UP * source_rotation
    xyz = Y_UP_TO_Z_UP.apply(instance.get("translation", [0, 0, 0]))
    pose = [*xyz.tolist(), *rotation.as_euler("xyz").tolist()]
    model = add(world, "model", name=label)
    add(model, "static", "true")
    add(model, "pose", " ".join(map(str, pose)))
    link = add(model, "link", name="source_geometry")
    mesh(link, "visual", "original_visual", f"/assets/{scene_id}/{render}", abs(scale))
    mesh(link, "collision", "original_collision", f"/assets/{scene_id}/{collision}", abs(scale))
    return dict(name=label, source_instance=instance, pose=pose, scale=scale.tolist(),
                geometry_scale=abs(scale).tolist(),
                visual=render, collision=collision)


def main(source, runtime, template, output, scene_number):
    scene_id = f"hssd_{scene_number}"
    assert output.name == scene_id
    inputs = json.loads((source / f"{scene_number}_object_assets.json").read_text())
    source_scene = source / f"{scene_number}.scene_instance.json"
    scene = json.loads(source_scene.read_text())
    stage_config = json.loads((source / f"{scene_number}.stage_config.json").read_text())
    assert scene["translation_origin"] == "asset_local"
    assert not scene.get("articulated_object_instances")
    assert scene["stage_instance"]["template_name"] == f"stages/{scene_number}"
    assert stage_config["render_asset"] == f"{scene_number}.glb"
    assert {r["name"] for r in inputs["records"]} == {r["template_name"] for r in scene["object_instances"]}
    assert all("error" not in r for r in inputs["records"])
    for path, expected in inputs["sha256"].items():
        assert sha(source / "source_files" / path) == expected, path

    assets = output / "assets"
    assets.mkdir(parents=True, exist_ok=False)
    tree = ET.parse(template)
    tree.getroot().set("version", "1.10")
    world = tree.getroot().find("world")
    for tag in ("model", "include"):
        for child in world.findall(tag):
            world.remove(child)
    stage = source / stage_config["render_asset"]
    stage_count = reencode_textures(stage, assets / "stage.glb", runtime)
    collision_mesh(stage, assets / "stage_collision.glb")
    rows = [insert(world, scene_id, "hssd_stage", scene["stage_instance"], "stage.glb", "stage_collision.glb")]
    conversions = []
    models = {}
    for record in inputs["records"]:
        name, cfg = record["name"], record["config"]
        assert cfg["up"] == [0, 1, 0] and cfg["front"] == [0, 0, -1]
        root = source / "source_files" / record["folder"]
        original_visual = root / cfg["render_asset"]
        visual = f"{name}_visual.glb"
        count = reencode_textures(original_visual, assets / visual, runtime)
        original_collision = root / cfg.get("collision_asset", cfg["render_asset"])
        collision = f"{name}_collision.glb"
        collision_mesh(original_collision, assets / collision)
        models[name] = visual, collision
        conversions.append(dict(name=name, visual_source_sha256=sha(original_visual),
                                collision_source_sha256=sha(original_collision), decoded_ktx2_images=count))
    for index, instance in enumerate(scene["object_instances"]):
        assert instance["motion_type"] == "STATIC"
        visual, collision = models[instance["template_name"]]
        scale = np.asarray(instance["non_uniform_scale"])
        if min(scale) < 0:
            assert scale[0] < 0 and min(scale[1:]) > 0
            mirrored = [f"mirror_x_{item}" for item in (visual, collision)]
            for item, reflected in zip((visual, collision), mirrored):
                if not (assets / reflected).exists():
                    mirror_opening(assets / item, assets / reflected)
            visual, collision = mirrored
        rows.append(insert(world, scene_id, f"hssd_object_{index:03d}", instance, visual, collision))

    world_file = output / f"{scene_id}.sdf"
    tree.write(world_file, encoding="utf-8", xml_declaration=True)
    record = dict(scene=scene_number, world_provenance_label=f"HSSD {scene_number} static furnished layout",
                  source_revision=inputs["repo_revision"], source_scene_sha256=sha(source_scene),
                  stage_source_sha256=sha(stage), world_sha256=sha(world_file), models=rows,
                  conversions=conversions, stage_decoded_ktx2_images=stage_count,
                  texture_transcoder_sha256=sha(runtime / "basis_transcoder.wasm"),
                  asset_files_sha256={p.name: sha(p) for p in sorted(assets.iterdir())})
    (output / "migration_manifest.json").write_text(json.dumps(record, indent=2) + "\n")
    print(f"{len(rows) - 1} objects; {stage_count + sum(r['decoded_ktx2_images'] for r in conversions)} textures")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("runtime", type=lambda p: Path(p).resolve())
    parser.add_argument("template", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--scene", required=True)
    args = parser.parse_args()
    main(args.source, args.runtime, args.template, args.output, args.scene)
