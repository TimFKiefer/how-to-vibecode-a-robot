# SPDX-License-Identifier: MIT
"""Visual meshes from viewer/meshes.json, written as OBJ in metres."""

import base64
import struct

import numpy as np

from robot2sim.paths import mesh_dir, meshes_json, read_json
from robot2sim.units import mm_to_m


def load_entries():
    return read_json(meshes_json())


def _arrays(entry):
    nv, nt = entry["nv"], entry["nt"]
    pos = np.array(struct.unpack(f"<{nv * 3}f", base64.b64decode(entry["pos"])), dtype=np.float64).reshape(nv, 3)
    code = "I" if entry["idx_type"] == "u32" else "H"
    faces = np.array(struct.unpack(f"<{nt * 3}{code}", base64.b64decode(entry["idx"])), dtype=np.int64).reshape(nt, 3)
    return pos, faces


def volume_centroid(entry):
    """Signed-volume centroid of a closed triangle mesh, in the mesh frame (mm)."""
    pos, faces = _arrays(entry)
    a = pos[faces[:, 0]]
    b = pos[faces[:, 1]]
    c = pos[faces[:, 2]]
    cross = np.cross(b, c)
    vol = np.einsum("ij,ij->i", a, cross)
    total = float(vol.sum())
    if abs(total) < 1e-9:
        return None
    centroid = ((a + b + c) * vol[:, None]).sum(axis=0) / (total * 4.0)
    return centroid


def centroids_mm(keys):
    entries = load_entries()
    out = {}
    for key in keys:
        if key not in entries:
            continue
        c = volume_centroid(entries[key])
        if c is not None:
            out[key] = [float(v) for v in c]
    return out


def write_obj(path, entry):
    pos, faces = _arrays(entry)
    pos = pos * mm_to_m(1.0)
    lines = [f"v {p[0]:.8f} {p[1]:.8f} {p[2]:.8f}" for p in pos]
    lines.extend(f"f {int(a) + 1} {int(b) + 1} {int(c) + 1}" for a, b, c in faces)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_used(keys):
    entries = load_entries()
    out = mesh_dir()
    out.mkdir(parents=True, exist_ok=True)
    missing = [k for k in keys if k not in entries]
    if missing:
        raise RuntimeError(f"no mesh in viewer/meshes.json for {missing}")
    for key in sorted(set(keys)):
        write_obj(out / f"{key}.obj", entries[key])
    return out


def write_hex(circumradius_mm, z0_mm, z1_mm):
    """Convex hexagonal prism. A vertex lies on +X, so the flats face ±Y."""
    r = mm_to_m(circumradius_mm)
    z0, z1 = mm_to_m(z0_mm), mm_to_m(z1_mm)
    verts = []
    for z in (z0, z1):
        for k in range(6):
            a = np.deg2rad(60.0 * k)
            verts.append((r * np.cos(a), r * np.sin(a), z))
    faces = []
    for k in range(1, 5):
        faces.append((0, k, k + 1))
        faces.append((6, 6 + k + 1, 6 + k))
    for k in range(6):
        n = (k + 1) % 6
        faces.append((k, n, 6 + n))
        faces.append((k, 6 + n, 6 + k))
    path = mesh_dir() / "body_hex.obj"
    lines = [f"v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}" for v in verts]
    lines.extend(f"f {a + 1} {b + 1} {c + 1}" for a, b, c in faces)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path
