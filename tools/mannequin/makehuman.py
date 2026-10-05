"""Pinned, hash-verified access to MakeHuman source assets, plus the macro-modifier math.

Everything the mannequin builder needs from MakeHuman (CC0 assets at one pinned commit)
goes through this module: `fetch` downloads a file once and checks it against
`manifest.sha256`; the readers parse base mesh, targets and rig; `macro_values` and
`macro_target_weights` reproduce MakeHuman's macro sliders (apps/human.py) so a build
can be reproduced without running MakeHuman.

Units: MakeHuman files are in decimetres, everything returned here is in metres.
Python 3.13 and numpy only.
"""
import hashlib
import http.client
import itertools
import json
import os
import ssl
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PINNED_COMMIT = "a8bc2d54ff0ac92e78ff71431b1023eda42bf482"
RAW_BASE = ("https://raw.githubusercontent.com/makehumancommunity/makehuman/"
            + PINNED_COMMIT + "/")

_TARGETS_DIR = "makehuman/data/targets/"
_DECIMETRES_TO_METRES = 0.1
_DOWNLOAD_TIMEOUT_S = 60


class FetchError(Exception):
    """A pinned source file could not be obtained or did not match its manifest hash."""


# --------------------------------------------------------------------------- fetching

def load_manifest(path: Path) -> dict[str, str]:
    """Parse a `sha256sum`-format file (`<64 hex>  <relative path>`) into {path: hash}."""
    manifest: dict[str, str] = {}
    for lineno, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        digest, sep, relpath = line.partition("  ")
        if (not sep or not relpath or len(digest) != 64
                or any(c not in "0123456789abcdef" for c in digest)):
            raise ValueError(f"{path}:{lineno}: expected '<sha256 hex>  <path>', got {line!r}")
        if relpath in manifest:
            raise ValueError(f"{path}:{lineno}: duplicate entry for {relpath}")
        manifest[relpath] = digest
    return manifest


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _download(url: str) -> bytes:
    # The default context verifies certificates and honours SSL_CERT_FILE / SSL_CERT_DIR;
    # urlopen picks up HTTPS_PROXY from the environment.
    context = ssl.create_default_context()
    try:
        with urllib.request.urlopen(url, context=context, timeout=_DOWNLOAD_TIMEOUT_S) as response:
            return response.read()
    except (OSError, http.client.HTTPException) as exc:    # URLError is an OSError
        raise FetchError(f"download failed: {url}: {exc}") from exc


def fetch(relpath: str, cache_dir: Path, manifest: dict[str, str]) -> Path:
    """Return `cache_dir/relpath`, downloading it from the pinned commit if absent.

    The file's sha256 must match `manifest[relpath]`. A download that fails the check is
    never written to the cache; a cached file that fails it is left alone for inspection.
    """
    expected = manifest.get(relpath)
    if expected is None:
        raise FetchError(f"{relpath}: not in the manifest, refusing to use an unverified file")
    dest = Path(cache_dir) / relpath
    if dest.exists():
        actual = _sha256(dest.read_bytes())
        if actual != expected:
            raise FetchError(f"{relpath}: sha256 mismatch for cached {dest} "
                             f"(expected {expected}, got {actual}); delete it to download again")
        return dest

    url = RAW_BASE + relpath
    data = _download(url)
    actual = _sha256(data)
    if actual != expected:
        raise FetchError(f"{relpath}: sha256 mismatch for download from {url} "
                         f"(expected {expected}, got {actual})")
    dest.parent.mkdir(parents=True, exist_ok=True)
    partial = dest.with_name(dest.name + ".part")
    partial.write_bytes(data)
    os.replace(partial, dest)
    return dest


# --------------------------------------------------------------------------- parsers

@dataclass
class BaseMesh:
    vertices: np.ndarray      # (19158, 3) float64, metres; every vertex, helpers included
    body_quads: np.ndarray    # (13378, 4) int64, 0-based, faces of group `body` only
    body_ids: np.ndarray      # sorted unique vertex indices used by body_quads (13380)


def read_base_obj(path: Path) -> BaseMesh:
    """Read MakeHuman's base.obj. All vertices are kept because the `joint-*` helper
    vertices drive the skeleton; only quads of group `body` are returned as faces."""
    vertices: list[tuple[float, float, float]] = []
    quads: list[list[int]] = []
    group = ""
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "v":
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif parts[0] == "g":
                group = " ".join(parts[1:])
            elif parts[0] == "f" and group == "body":
                ids = [int(token.split("/")[0]) for token in parts[1:]]
                if len(ids) != 4 or min(ids) < 1:
                    raise ValueError(f"{path}:{lineno}: body faces must be quads "
                                     f"with positive vertex indices, got {line.strip()!r}")
                quads.append([i - 1 for i in ids])
    body_quads = np.array(quads, dtype=np.int64).reshape(-1, 4)
    if body_quads.size and body_quads.max() >= len(vertices):
        raise ValueError(f"{path}: body face refers to vertex {body_quads.max() + 1} "
                         f"but only {len(vertices)} are defined")
    return BaseMesh(vertices=np.array(vertices, dtype=np.float64).reshape(-1, 3) * _DECIMETRES_TO_METRES,
                    body_quads=body_quads,
                    body_ids=np.unique(body_quads))


def read_target(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read a `.target` file: (vertex indices int64, offsets (K, 3) float64 in metres)."""
    indices: list[int] = []
    offsets: list[tuple[float, float, float]] = []
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            text = line.strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            try:
                index, dx, dy, dz = parts      # ValueError unless exactly four fields
                indices.append(int(index))
                offsets.append((float(dx), float(dy), float(dz)))
            except ValueError as exc:
                raise ValueError(f"{path}:{lineno}: expected '<index> <dx> <dy> <dz>', "
                                 f"got {text!r}") from exc
    return (np.array(indices, dtype=np.int64),
            np.array(offsets, dtype=np.float64).reshape(-1, 3) * _DECIMETRES_TO_METRES)


def apply_targets(base: np.ndarray, weights: dict[str, float], cache_dir: Path,
                  manifest: dict[str, str]) -> np.ndarray:
    """Return base + sum(weight * target offsets); `weights` keys are paths relative to
    makehuman/data/targets/. `base` is not modified."""
    result = np.array(base, dtype=np.float64)
    # Sorted so the floating-point summation order does not depend on the caller's dict order.
    for key in sorted(weights):
        indices, offsets = read_target(fetch(_TARGETS_DIR + key, cache_dir, manifest))
        np.add.at(result, indices, weights[key] * offsets)
    return result


# --------------------------------------------------------------------------- macro sliders

def macro_values(gender: float, age: float, muscle: float, weight: float, height: float,
                 proportions: float, races: tuple[float, float, float] = (1/3, 1/3, 1/3)
                 ) -> dict[str, float]:
    """MakeHuman's macro `*Val` blend values for slider settings in [0, 1].

    Mirrors apps/human.py (`_setGenderVals`, `_setAgeVals`, `_setMuscleVals`,
    `_setWeightVals`, `_setHeightVals`, `_setBodyProportionVals`). `races` is ordered
    (african, asian, caucasian) and is used as given, not renormalised.
    """
    sliders = {"gender": gender, "age": age, "muscle": muscle, "weight": weight,
               "height": height, "proportions": proportions}
    for name, x in sliders.items():
        if not 0.0 <= x <= 1.0:
            raise ValueError(f"{name} must be in [0, 1], got {x}")
    if len(races) != 3 or not all(0.0 <= r <= 1.0 for r in races):
        raise ValueError(f"races must be three values in [0, 1] (african, asian, caucasian), got {races}")
    african, asian, caucasian = races

    if age < 0.5:
        old = 0.0
        baby = max(0.0, 1.0 - age * 5.333)
        young = max(0.0, (age - 0.1875) * 3.2)
        child = max(0.0, min(1.0, 5.333 * age) - young)
    else:
        child = baby = 0.0
        old = max(0.0, 2.0 * age - 1.0)
        young = 1.0 - old

    def three_way(x: float) -> tuple[float, float, float]:
        """(low, middle, high) blend of a slider; the middle is 1 minus the active side.
        human.py spells this `1 - max - min` for muscle and weight but `1 - max` or `1 - min`
        for height and proportions; at most one side is nonzero, so the forms agree exactly."""
        high, low = max(0.0, 2.0 * x - 1.0), max(0.0, 1.0 - 2.0 * x)
        return low, 1.0 - high - low, high

    minmuscle, averagemuscle, maxmuscle = three_way(muscle)
    minweight, averageweight, maxweight = three_way(weight)
    minheight, averageheight, maxheight = three_way(height)
    uncommonproportions, regularproportions, idealproportions = three_way(proportions)

    return {"male": gender, "female": 1.0 - gender,
            "baby": baby, "child": child, "young": young, "old": old,
            "caucasian": caucasian, "asian": asian, "african": african,
            "minmuscle": minmuscle, "averagemuscle": averagemuscle, "maxmuscle": maxmuscle,
            "minweight": minweight, "averageweight": averageweight, "maxweight": maxweight,
            "minheight": minheight, "averageheight": averageheight, "maxheight": maxheight,
            "uncommonproportions": uncommonproportions, "regularproportions": regularproportions,
            "idealproportions": idealproportions}


_RACES = ("african", "asian", "caucasian")
_GENDERS = ("male", "female")
_AGES = ("baby", "child", "young", "old")
_MUSCLES = ("minmuscle", "averagemuscle", "maxmuscle")
_WEIGHTS = ("minweight", "averageweight", "maxweight")
_GROUP_WORDS = frozenset({"universal", "height", "proportions"})   # name a family, not a blend value


def _macro_target_files() -> list[str]:
    """Every macro target MakeHuman ships, relative to makehuman/data/targets/. There are no
    files for averageheight or regularproportions: those are the unmodified state."""
    names = [f"macrodetails/{race}-{gender}-{age}.target"
             for race, gender, age in itertools.product(_RACES, _GENDERS, _AGES)]
    for gender, age, muscle, weight in itertools.product(_GENDERS, _AGES, _MUSCLES, _WEIGHTS):
        build = f"{gender}-{age}-{muscle}-{weight}"
        names.append(f"macrodetails/universal-{build}.target")
        names += [f"macrodetails/height/{build}-{h}.target" for h in ("minheight", "maxheight")]
        names += [f"macrodetails/proportions/{build}-{p}.target"
                  for p in ("idealproportions", "uncommonproportions")]
    return names


def macro_target_weights(values: dict[str, float]) -> dict[str, float]:
    """Target weights for `macro_values` output: {path relative to makehuman/data/targets/: weight}.

    A target's weight is the product of the values named by the dash-separated parts of its
    file name (group words count as 1). Zero weights are dropped.
    """
    weights: dict[str, float] = {}
    for rel in _macro_target_files():
        stem = rel.rsplit("/", 1)[-1].removesuffix(".target")
        weight = 1.0
        for part in stem.split("-"):
            if part not in _GROUP_WORDS:
                weight *= values[part]
        if weight > 0.0:
            weights[rel] = weight
    return weights


# --------------------------------------------------------------------------- rig

@dataclass
class Rig:
    parent: dict[str, str | None]                            # bone -> parent bone (None for the root)
    head_joint: dict[str, str]                               # bone -> name of its head joint
    joints: dict[str, list[int]]                             # joint name -> base-mesh vertex indices
    weights: dict[str, tuple[np.ndarray, np.ndarray]]        # bone -> (vertex indices, skin weights)


def read_rig(skel_path: Path, weights_path: Path) -> Rig:
    """Read MakeHuman's `.mhskel` (bones and joints) and `.mhw` (skin weights) JSON files."""
    skeleton = json.loads(Path(skel_path).read_text(encoding="utf-8"))
    skin = json.loads(Path(weights_path).read_text(encoding="utf-8"))
    bones = skeleton["bones"]
    return Rig(
        parent={name: bone["parent"] for name, bone in bones.items()},
        head_joint={name: bone["head"] for name, bone in bones.items()},
        joints={name: [int(i) for i in ids] for name, ids in skeleton["joints"].items()},
        weights={name: (np.array([int(i) for i, _ in pairs], dtype=np.int64),
                        np.array([float(w) for _, w in pairs], dtype=np.float64))
                 for name, pairs in skin["weights"].items()})


def joint_pos(vertices: np.ndarray, rig: Rig, joint: str) -> np.ndarray:
    """Position of a named joint: the mean of its vertices."""
    return vertices[rig.joints[joint]].mean(axis=0)


def descendants(rig: Rig, bone: str) -> list[str]:
    """`bone` and every bone below it, depth first with children in file order."""
    if bone not in rig.parent:
        raise KeyError(bone)
    children: dict[str, list[str]] = {}
    for name, parent in rig.parent.items():
        if parent is not None:
            children.setdefault(parent, []).append(name)
    order: list[str] = []
    pending = [bone]
    while pending:
        current = pending.pop()
        order.append(current)
        pending.extend(reversed(children.get(current, [])))
    return order
