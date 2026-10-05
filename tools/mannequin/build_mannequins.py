#!/usr/bin/env python3
"""Build the male and female mannequin assets from MakeHuman's CC0 body.

    python3 tools/mannequin/build_mannequins.py [--cache out/makehuman-cache] [--out assets/mannequins]

Per body: bisect MakeHuman's height macro until the posed body is the target height, build it
(macro targets, egg head, smooth feet, 45 degree arms and 5 degree legs, Drape's frame), measure
it (`measure.py`), and write `<body>.obj`, `<body>.json` and `<body>.checks.json`; then
`LICENSE.md`. The source files come from the pinned MakeHuman commit, checked against
`tools/mannequin/manifest.sha256`, and are downloaded into `--cache` only when missing. The
outputs are committed; running this twice gives byte-identical files. One report line per body
goes to stdout.

Everything that is written is rounded to 6 decimals first and measured from the rounded
vertices, so the numbers in the JSON describe exactly the mesh in the OBJ.
Python 3.13 and numpy only.
"""
import argparse
import json
import math
import sys
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import makehuman
import measure
import pose
import style

BUILDER_VERSION = 1
FORMAT_VERSION = 1
VERTEX_ORDER = "makehuman base.obj body group, ascending index"

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MANIFEST_PATH = Path(__file__).resolve().parent / "manifest.sha256"
BASE_OBJ = "makehuman/data/3dobjs/base.obj"
SKELETON = "makehuman/data/rigs/default.mhskel"
SKIN_WEIGHTS = "makehuman/data/rigs/default_weights.mhw"

# MakeHuman slider settings shared by both bodies (gender and height differ).
AGE, MUSCLE, WEIGHT, PROPORTIONS = 0.5, 0.75, 0.5, 1.0
ARM_DEG, LEG_DEG = 45.0, 5.0
HEIGHT_TOLERANCE = 0.0005         # metres
MAX_BISECTIONS = 40
HEAD_PIN_WEIGHT = 0.999           # head vertices at least this weighted sit on the egg

BODIES = {"male": (1.0, 1.80), "female": (0.0, 1.70)}    # name -> (gender slider, target height in metres)


@dataclass
class Source:
    """The MakeHuman files, read once, and the skinning-weight regions derived from them."""
    base: makehuman.BaseMesh
    rig: makehuman.Rig
    cache: Path
    manifest: dict[str, str]
    head_weights: np.ndarray          # (all base vertices,): weight on the head and its descendants; 0 past the body
    toe_weights: np.ndarray           # same, for the toes (the foot bones below the foot bone itself)


def load_source(cache: Path) -> Source:
    manifest = makehuman.load_manifest(MANIFEST_PATH)
    base = makehuman.read_base_obj(makehuman.fetch(BASE_OBJ, cache, manifest))
    rig = makehuman.read_rig(makehuman.fetch(SKELETON, cache, manifest),
                             makehuman.fetch(SKIN_WEIGHTS, cache, manifest))
    n = len(base.body_ids)
    # The body group is vertices 0..n-1 of base.obj (the helper geometry follows), so a body
    # vertex's OBJ index is its base index. The regions are computed over the body only: the rig
    # also weights the eyeballs, teeth and tongue to the head, and those must not shape the egg.
    if not np.array_equal(base.body_ids, np.arange(n)):
        raise RuntimeError("base.obj: the body group is not vertices 0..n-1; the builder assumes it is")
    head = np.zeros(len(base.vertices))
    head[:n] = style.region_weights(rig, ["head"], n)
    toes = np.zeros(len(base.vertices))
    toes[:n] = style.region_weights(rig, ["foot.L", "foot.R"], n, exclude=frozenset({"foot.L", "foot.R"}))
    return Source(base=base, rig=rig, cache=cache, manifest=manifest, head_weights=head, toe_weights=toes)


@dataclass
class Built:
    vertices: np.ndarray              # (all base vertices, 3) in Drape's frame
    offset: np.ndarray                # (3,) subtracted from MakeHuman's frame to get Drape's
    joints: dict[str, np.ndarray]     # posed limb joints plus the unposed breast, neck01 and eye.L, in Drape's frame
    egg: style.Egg                    # in Drape's frame
    head_vertices: np.ndarray         # ascending indices of the egg-surface vertices (cavities excluded)
    height: float                     # floor to top of the head


def build_body(src: Source, gender: float, height_macro: float) -> Built:
    """One evaluation: weighted targets, egg head, smooth feet, pose, Drape's frame."""
    n = len(src.base.body_ids)
    quads = src.base.body_quads
    values = makehuman.macro_values(gender, AGE, MUSCLE, WEIGHT, height_macro, PROPORTIONS)
    vertices = makehuman.apply_targets(src.base.vertices, makehuman.macro_target_weights(values),
                                       src.cache, src.manifest)

    # The egg is fitted to the unposed skull, in MakeHuman's frame.
    eye_y = makehuman.joint_pos(vertices, src.rig, "eye.L____head")[1]
    egg = style.fit_egg(vertices, src.head_weights, eye_y)
    cavity = style.cavity_vertices(vertices, quads, src.head_weights, egg)    # before the egg moves them
    head_vertices = np.flatnonzero((src.head_weights[:n] >= HEAD_PIN_WEIGHT) & ~cavity[:n])

    vertices = style.make_egg_head(vertices, quads, src.head_weights, egg)
    vertices = style.smooth_toes(vertices, quads, src.toe_weights)
    posed = pose.pose(vertices, src.rig, ARM_DEG, LEG_DEG)
    framed, offset = pose.to_drape_frame(posed.vertices, src.base.body_ids, posed.joints["foot.L____head"],
                                         posed.joints["foot.R____head"])

    joints = {name: p - offset for name, p in posed.joints.items()}
    for name in ("breast.L____head", "neck01____head", "eye.L____head"):
        joints[name] = makehuman.joint_pos(posed.vertices, src.rig, name) - offset
    return Built(vertices=framed, offset=offset, joints=joints,
                 egg=style.Egg(centre=egg.centre - offset, radii=egg.radii, taper=egg.taper),
                 head_vertices=head_vertices, height=float(framed[:n, 1].max()))


def bisect_height(evaluate: Callable[[float], float], target: float, log: Callable[[str], None]) -> float:
    """The height macro in [0, 1] at which `evaluate` (the body's height, rising with the macro)
    is within HEIGHT_TOLERANCE of `target`. [0.5, 1.0] is tried first; if it does not bracket the
    target the search widens to [0, 1] (said on stderr), never beyond: MakeHuman's sliders stop there."""
    for low, high in ((0.5, 1.0), (0.0, 1.0)):
        f_low, f_high = evaluate(low), evaluate(high)
        log(f"  height({low}) = {f_low:.4f}, height({high}) = {f_high:.4f}, target {target}")
        if f_low <= target <= f_high:
            break
        print(f"note: [{low}, {high}] does not bracket {target} m (heights {f_low:.4f} to {f_high:.4f})"
              + ("; widening to [0, 1]" if low > 0.0 else ""), file=sys.stderr)
    else:
        raise RuntimeError(f"height {target} m is outside what the height macro reaches")
    for macro in (low, high):
        if abs(evaluate(macro) - target) <= HEIGHT_TOLERANCE:
            return macro
    for step in range(1, MAX_BISECTIONS + 1):
        mid = (low + high) / 2.0
        f_mid = evaluate(mid)
        log(f"  {step:2d}: macro {mid:.6f} -> height {f_mid:.5f}")
        if abs(f_mid - target) <= HEIGHT_TOLERANCE:
            return mid
        if f_mid < target:
            low = mid
        else:
            high = mid
    raise RuntimeError(f"height bisection did not reach {target} m +- {HEIGHT_TOLERANCE} in {MAX_BISECTIONS} steps")


# --------------------------------------------------------------------------- writing

def format_json(value, level: int = 0) -> str:
    """json.dumps(value, sort_keys=True, indent=1), except floats print with 6 decimals ("-0.000000"
    is "0.000000"). Numpy arrays and scalars are accepted."""
    pad = " " * (level + 1)
    if isinstance(value, dict):
        if not value:
            return "{}"
        items = [f"{pad}{json.dumps(key)}: {format_json(value[key], level + 1)}" for key in sorted(value)]
        return "{\n" + ",\n".join(items) + "\n" + " " * level + "}"
    if isinstance(value, (list, tuple, np.ndarray)):
        if len(value) == 0:
            return "[]"
        return "[\n" + ",\n".join(pad + format_json(item, level + 1) for item in value) + "\n" + " " * level + "]"
    if isinstance(value, (bool, np.bool_)):
        return "true" if value else "false"
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        if not math.isfinite(value):
            raise ValueError(f"cannot write {value} to JSON")
        text = f"{float(value):.6f}"
        return "0.000000" if text == "-0.000000" else text
    return json.dumps(value)


def write_text(path: Path, text: str) -> None:
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def obj_text(name: str, vertices: np.ndarray, tris: np.ndarray) -> str:
    lines = [f"# Drape mannequin {name} — tools/mannequin/build_mannequins.py, "
             f"MakeHuman {makehuman.PINNED_COMMIT[:7]} (CC0)", "o body"]
    lines += [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in tris]
    return "\n".join(lines) + "\n"


LICENSE_TEXT = f"""# Mannequin assets

`male.*` and `female.*` in this folder are generated by `tools/mannequin/build_mannequins.py`
from MakeHuman's base mesh, shape targets and default rig.

- Source: <https://github.com/makehumancommunity/makehuman>, commit `{makehuman.PINNED_COMMIT}`
- License of the source assets: Creative Commons CC0 1.0 Universal (public domain dedication),
  see `LICENSE.ASSETS.md` at that commit. Commercial use is allowed and no attribution is
  required. This folder is released on the same terms.
- Builder version: {BUILDER_VERSION}
- Vertex order: {VERTEX_ORDER}. Each `.obj` holds the 13,380 vertices of
  MakeHuman's `body` group and no others, so shape targets defined on MakeHuman's mesh map
  one to one.

Files per body: `<body>.obj` (triangles, metres), `<body>.json` (height, girths, landmarks and
region cylinders) and `<body>.checks.json` (the egg and vertex lists the loader's tests use).
Regenerate with `python3 tools/mannequin/build_mannequins.py`; the output is byte-identical
from run to run.
"""


def write_body(out_dir: Path, name: str, body: np.ndarray, tris: np.ndarray, m: measure.Measurement,
               built: Built) -> None:
    height = float(body[:, 1].max())
    description = {
        "format": FORMAT_VERSION, "body": name,
        "source": {"makehuman_commit": makehuman.PINNED_COMMIT, "builder_version": BUILDER_VERSION,
                   "vertex_order": VERTEX_ORDER},
        "height": height, "girths": m.girths,
        "landmarks": {key: p.tolist() for key, p in m.landmarks.items()},
        "regions": {key: {"origin": c["origin"].tolist(), "axis": c["axis"].tolist(), "e0": c["e0"].tolist(),
                          "radius": c["radius"]} for key, c in m.regions.items()}}
    checks = {
        "format": FORMAT_VERSION,
        "egg": {"centre": np.round(built.egg.centre, 6).tolist(), "radii": np.round(built.egg.radii, 6).tolist(),
                "taper": built.egg.taper},
        "head_vertices": built.head_vertices.tolist(),
        "region_vertices": {key: ids.tolist() for key, ids in m.region_vertices.items()}}
    write_text(out_dir / f"{name}.obj", obj_text(name, body, tris))
    write_text(out_dir / f"{name}.json", format_json(description) + "\n")
    write_text(out_dir / f"{name}.checks.json", format_json(checks) + "\n")


# --------------------------------------------------------------------------- driver

def build_and_write(src: Source, name: str, out_dir: Path, log: Callable[[str], None]) -> str:
    gender, target = BODIES[name]
    n = len(src.base.body_ids)
    evaluations: dict[float, Built] = {}

    def evaluate(macro: float) -> float:
        if macro not in evaluations:
            evaluations[macro] = build_body(src, gender, macro)
        return evaluations[macro].height

    log(f"{name}: bisecting the height macro")
    macro = bisect_height(evaluate, target, log)
    built = evaluations[macro]

    # Round first: everything below is measured from the numbers that get written.
    body = np.round(built.vertices[:n], 6) + 0.0
    tris = pose.triangulate(body, src.base.body_quads)
    dominant = measure.dominant_bones(src.rig, n)
    m = measure.measure_body(body, tris, dominant, src.rig, built.joints, built.egg, built.head_vertices)
    write_body(out_dir, name, body, tris, m, built)

    # The angles are read back from the rounded landmarks that go into the JSON.
    def angle(side: str, tip: str, root: str) -> float:
        return pose.frontal_angle(m.landmarks[f"{tip}_{side}"] - m.landmarks[f"{root}_{side}"])

    g = m.girths
    return (f"{name} height={body[:, 1].max():.3f} chest={g['chest']:.3f} waist={g['waist']:.3f} "
            f"hip={g['hip']:.3f} neck={g['neck']:.3f} "
            f"arm_l={angle('left', 'wrist', 'shoulder_joint'):.2f} "
            f"arm_r={angle('right', 'wrist', 'shoulder_joint'):.2f} "
            f"leg_l={angle('left', 'ankle', 'hip_joint'):.2f} leg_r={angle('right', 'ankle', 'hip_joint'):.2f}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build the male and female mannequin assets.")
    parser.add_argument("--cache", type=Path, default=REPO_ROOT / "out" / "makehuman-cache",
                        help="where the pinned MakeHuman files are kept (downloaded if missing)")
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "assets" / "mannequins",
                        help="where the assets are written")
    parser.add_argument("-v", "--verbose", action="store_true", help="trace the height bisection on stderr")
    args = parser.parse_args(argv)
    log = (lambda text: print(text, file=sys.stderr)) if args.verbose else (lambda text: None)

    src = load_source(args.cache)
    args.out.mkdir(parents=True, exist_ok=True)
    for name in BODIES:
        print(build_and_write(src, name, args.out, log), flush=True)
    write_text(args.out / "LICENSE.md", LICENSE_TEXT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
