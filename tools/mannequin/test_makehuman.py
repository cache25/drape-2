"""Offline unit tests for makehuman.py: synthetic inputs only, never the network."""
import hashlib
import json
import tempfile
import unittest
import urllib.error
from pathlib import Path
from unittest import mock

import numpy as np

import makehuman

M = "macrodetails/"


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def write(path: Path, data: str | bytes) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(data, str):
        data = data.encode()
    path.write_bytes(data)
    return path


def fake_response(data: bytes) -> mock.MagicMock:
    """What urllib.request.urlopen returns: a context manager with read()."""
    resp = mock.MagicMock()
    resp.__enter__.return_value.read.return_value = data
    return resp


class MacroValues(unittest.TestCase):
    def test_corner_settings(self):
        v = makehuman.macro_values(gender=1.0, age=0.5, muscle=0.75, weight=0.5, height=0.75, proportions=1.0)
        expect = {"male": 1, "female": 0, "young": 1, "old": 0, "child": 0, "baby": 0,
                  "maxmuscle": 0.5, "averagemuscle": 0.5, "minmuscle": 0, "averageweight": 1,
                  "maxweight": 0, "minweight": 0, "maxheight": 0.5, "averageheight": 0.5, "minheight": 0,
                  "idealproportions": 1, "regularproportions": 0, "uncommonproportions": 0,
                  "african": 1/3, "asian": 1/3, "caucasian": 1/3}
        self.assertEqual(set(v), set(expect))
        for k, x in expect.items():
            self.assertAlmostEqual(v[k], x, places=12, msg=k)

    def test_low_height_uses_minheight(self):
        v = makehuman.macro_values(1.0, 0.5, 0.75, 0.5, 0.25, 1.0)
        self.assertAlmostEqual(v["minheight"], 0.5)
        self.assertAlmostEqual(v["averageheight"], 0.5)

    def test_gender_is_a_blend(self):
        v = makehuman.macro_values(0.25, 0.5, 0.5, 0.5, 0.5, 0.5)
        self.assertAlmostEqual(v["male"], 0.25)
        self.assertAlmostEqual(v["female"], 0.75)

    def test_age_below_half_blends_baby_child_young(self):
        v = makehuman.macro_values(1.0, 0.25, 0.5, 0.5, 0.5, 0.5)
        self.assertAlmostEqual(v["baby"], 0.0)
        self.assertAlmostEqual(v["young"], (0.25 - 0.1875) * 3.2)
        self.assertAlmostEqual(v["child"], min(1.0, 5.333 * 0.25) - v["young"])
        self.assertAlmostEqual(v["old"], 0.0)
        v = makehuman.macro_values(1.0, 0.0, 0.5, 0.5, 0.5, 0.5)
        self.assertEqual((v["baby"], v["child"], v["young"], v["old"]), (1.0, 0.0, 0.0, 0.0))

    def test_age_above_half_blends_young_old(self):
        v = makehuman.macro_values(1.0, 0.75, 0.5, 0.5, 0.5, 0.5)
        self.assertAlmostEqual(v["old"], 0.5)
        self.assertAlmostEqual(v["young"], 0.5)
        self.assertEqual((v["baby"], v["child"]), (0.0, 0.0))

    def test_weight_and_proportions_use_the_same_three_way_split(self):
        v = makehuman.macro_values(1.0, 0.5, 0.5, 0.25, 0.5, 0.25)
        self.assertAlmostEqual(v["minweight"], 0.5)
        self.assertAlmostEqual(v["averageweight"], 0.5)
        self.assertAlmostEqual(v["maxweight"], 0.0)
        self.assertAlmostEqual(v["uncommonproportions"], 0.5)
        self.assertAlmostEqual(v["regularproportions"], 0.5)
        self.assertAlmostEqual(v["idealproportions"], 0.0)
        v = makehuman.macro_values(1.0, 0.5, 0.5, 0.5, 0.5, 0.5)
        self.assertEqual((v["averageweight"], v["regularproportions"], v["averageheight"]), (1.0, 1.0, 1.0))

    def test_races_order_is_african_asian_caucasian(self):
        v = makehuman.macro_values(1.0, 0.5, 0.5, 0.5, 0.5, 0.5, races=(0.2, 0.3, 0.5))
        self.assertEqual((v["african"], v["asian"], v["caucasian"]), (0.2, 0.3, 0.5))

    def test_out_of_range_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            makehuman.macro_values(1.5, 0.5, 0.5, 0.5, 0.5, 0.5)
        with self.assertRaises(ValueError):
            makehuman.macro_values(1.0, 0.5, 0.5, 0.5, -0.1, 0.5)
        with self.assertRaises(ValueError):
            makehuman.macro_values(1.0, 0.5, 0.5, 0.5, 0.5, 0.5, races=(0.5, 0.5))


class MacroTargetWeights(unittest.TestCase):
    def test_male_build(self):
        w = makehuman.macro_target_weights(makehuman.macro_values(1.0, 0.5, 0.75, 0.5, 0.75, 1.0))
        expect = {M+"african-male-young.target": 1/3, M+"asian-male-young.target": 1/3,
                  M+"caucasian-male-young.target": 1/3,
                  M+"universal-male-young-averagemuscle-averageweight.target": 0.5,
                  M+"universal-male-young-maxmuscle-averageweight.target": 0.5,
                  M+"height/male-young-averagemuscle-averageweight-maxheight.target": 0.25,
                  M+"height/male-young-maxmuscle-averageweight-maxheight.target": 0.25,
                  M+"proportions/male-young-averagemuscle-averageweight-idealproportions.target": 0.5,
                  M+"proportions/male-young-maxmuscle-averageweight-idealproportions.target": 0.5}
        self.assertEqual(set(w), set(expect))
        for k, x in expect.items():
            self.assertAlmostEqual(w[k], x, places=12, msg=k)

    def test_female_build_names_female_files_only(self):
        w = makehuman.macro_target_weights(makehuman.macro_values(0.0, 0.5, 0.75, 0.5, 0.5, 1.0))
        self.assertTrue(w)
        parts = {k: k.rsplit("/", 1)[-1].split("-") for k in w}
        self.assertTrue(all("female" in p and "male" not in p for p in parts.values()), sorted(w))
        # height 0.5 is "average", for which MakeHuman has no target files
        self.assertFalse([k for k in w if k.endswith("height.target")])
        self.assertAlmostEqual(w[M+"caucasian-female-young.target"], 1/3, places=12)

    def test_age_blends_name_every_age_family(self):
        w = makehuman.macro_target_weights(makehuman.macro_values(1.0, 0.25, 0.5, 0.5, 0.5, 0.5))
        young, child = (0.25 - 0.1875) * 3.2, min(1.0, 5.333 * 0.25) - (0.25 - 0.1875) * 3.2
        self.assertAlmostEqual(w[M+"african-male-child.target"], child / 3, places=12)
        self.assertAlmostEqual(w[M+"african-male-young.target"], young / 3, places=12)
        self.assertAlmostEqual(w[M+"universal-male-child-averagemuscle-averageweight.target"], child, places=12)
        self.assertNotIn(M+"african-male-baby.target", w)
        self.assertNotIn(M+"african-male-old.target", w)

    def test_zero_weights_are_dropped(self):
        w = makehuman.macro_target_weights(makehuman.macro_values(0.3, 0.2, 0.6, 0.4, 0.8, 0.3))
        self.assertTrue(w)
        self.assertTrue(all(x > 0 for x in w.values()))

    def test_each_family_partitions_unity(self):
        # Every family is a product of partitions of unity, so its weights sum to a known constant.
        v = makehuman.macro_values(0.3, 0.7, 0.6, 0.4, 0.8, 0.3, races=(0.2, 0.5, 0.3))
        w = makehuman.macro_target_weights(v)

        def total(pred):
            return sum(x for k, x in w.items() if pred(k))

        self.assertAlmostEqual(total(lambda k: k.startswith(M + "universal-")), 1.0, places=12)
        self.assertAlmostEqual(total(lambda k: k.count("/") == 1 and not k.startswith(M + "universal-")),
                               1.0, places=12)
        self.assertAlmostEqual(total(lambda k: k.startswith(M + "height/")),
                               v["minheight"] + v["maxheight"], places=12)
        self.assertAlmostEqual(total(lambda k: k.startswith(M + "proportions/")),
                               v["idealproportions"] + v["uncommonproportions"], places=12)

    def test_unknown_value_names_fail_loudly(self):
        with self.assertRaises(KeyError):
            makehuman.macro_target_weights({"male": 1.0})


class Parsers(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.tmp = Path(self._tmp.name)

    def test_read_target_scales_to_metres(self):
        p = write(self.tmp / "t.target", "# c\n\n12 1 -2 0.5\n")
        idx, off = makehuman.read_target(p)
        self.assertEqual(idx.tolist(), [12])
        self.assertTrue(np.issubdtype(idx.dtype, np.integer))
        np.testing.assert_allclose(off, [[0.1, -0.2, 0.05]], atol=1e-15)

    def test_read_target_with_only_comments_is_empty(self):
        idx, off = makehuman.read_target(write(self.tmp / "t.target", "# nothing\n  \n"))
        self.assertEqual(idx.shape, (0,))
        self.assertEqual(off.shape, (0, 3))

    def test_read_target_keeps_file_order(self):
        p = write(self.tmp / "t.target", "7 1 0 0\n3 0 1 0\n9 0 0 1\n")
        idx, off = makehuman.read_target(p)
        self.assertEqual(idx.tolist(), [7, 3, 9])
        np.testing.assert_allclose(off, 0.1 * np.eye(3)[[0, 1, 2]], atol=1e-15)

    def test_read_target_names_file_and_line_on_bad_data(self):
        p = write(self.tmp / "bad.target", "# c\n12 1 2\n")
        with self.assertRaises(ValueError) as cm:
            makehuman.read_target(p)
        self.assertIn("bad.target", str(cm.exception))
        self.assertIn(":2", str(cm.exception))

    OBJ = ("# six vertices\n"
           "v 0 0 0\nv 10 0 0\nv 20 0 0\nv 30 0 0\nv 40 0 0\nv 50 10 -10\n"
           "g helper\nf 1/1 2/2 3/3 4/4\n"
           "g body\nf 3/5 4/6 5/7 6/8\n")

    def test_read_base_obj_keeps_body_group_only(self):
        mesh = makehuman.read_base_obj(write(self.tmp / "base.obj", self.OBJ))
        self.assertEqual(mesh.vertices.shape, (6, 3))      # all vertices kept
        np.testing.assert_allclose(mesh.vertices[5], [5.0, 1.0, -1.0], atol=1e-15)
        self.assertEqual(mesh.body_quads.tolist(), [[2, 3, 4, 5]])
        self.assertEqual(mesh.body_ids.tolist(), [2, 3, 4, 5])
        self.assertTrue(np.issubdtype(mesh.body_quads.dtype, np.integer))
        self.assertTrue(np.issubdtype(mesh.body_ids.dtype, np.integer))

    def test_read_base_obj_merges_split_body_group_into_sorted_unique_ids(self):
        obj = ("v 0 0 0\nv 1 0 0\nv 2 0 0\nv 3 0 0\nv 4 0 0\nv 5 0 0\n"
               "g body\nf 4 2 1 3\n"
               "g joint-pelvis\nf 1 2 3 4\n"
               "g body\nf 4 5 6 3\n")
        mesh = makehuman.read_base_obj(write(self.tmp / "base.obj", obj))
        self.assertEqual(mesh.body_quads.tolist(), [[3, 1, 0, 2], [3, 4, 5, 2]])
        self.assertEqual(mesh.body_ids.tolist(), [0, 1, 2, 3, 4, 5])

    def test_read_base_obj_rejects_non_quad_body_face(self):
        obj = "v 0 0 0\nv 1 0 0\nv 2 0 0\ng body\nf 1 2 3\n"
        with self.assertRaises(ValueError):
            makehuman.read_base_obj(write(self.tmp / "base.obj", obj))


class Manifest(unittest.TestCase):
    def test_load_manifest_parses_sha256sum_format(self):
        with tempfile.TemporaryDirectory() as d:
            p = write(Path(d) / "m", f"{'a' * 64}  x/y.target\n\n{'b' * 64}  z.md\n")
            self.assertEqual(makehuman.load_manifest(p), {"x/y.target": "a" * 64, "z.md": "b" * 64})

    def test_load_manifest_rejects_malformed_lines(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):
                makehuman.load_manifest(write(Path(d) / "m", "not-a-hash  x\n"))

    def test_committed_manifest_covers_every_pinned_source(self):
        m = makehuman.load_manifest(Path(makehuman.__file__).with_name("manifest.sha256"))
        self.assertTrue(all(len(h) == 64 and set(h) <= set("0123456789abcdef") for h in m.values()))
        self.assertEqual(m["makehuman/data/3dobjs/base.obj"],
                         "8e761e6624b8f54536409135d1636da63b32486a90d4897f84e121d144f6fb4c")
        for rel in ("LICENSE.ASSETS.md",
                    "makehuman/data/rigs/default.mhskel",
                    "makehuman/data/rigs/default_weights.mhw",
                    "makehuman/data/targets/macrodetails/caucasian-female-young.target",
                    "makehuman/data/targets/macrodetails/height/male-young-maxmuscle-averageweight-maxheight.target"):
            self.assertIn(rel, m)


class Fetch(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.cache = Path(self._tmp.name)

    def test_hash_mismatch_names_the_file(self):
        write(self.cache / "a" / "b.txt", "x")
        with self.assertRaises(makehuman.FetchError) as cm:
            makehuman.fetch("a/b.txt", self.cache, {"a/b.txt": "0" * 64})
        self.assertIn("a/b.txt", str(cm.exception))

    def test_cached_file_with_matching_hash_needs_no_network(self):
        p = write(self.cache / "a" / "b.txt", "x")
        with mock.patch("urllib.request.urlopen") as urlopen:
            got = makehuman.fetch("a/b.txt", self.cache, {"a/b.txt": sha(b"x")})
        urlopen.assert_not_called()
        self.assertEqual(got, p)

    def test_path_missing_from_manifest_is_refused_before_any_download(self):
        with mock.patch("urllib.request.urlopen") as urlopen:
            with self.assertRaises(makehuman.FetchError) as cm:
                makehuman.fetch("a/b.txt", self.cache, {})
        urlopen.assert_not_called()
        self.assertIn("a/b.txt", str(cm.exception))

    def test_missing_file_is_downloaded_from_the_pinned_commit_and_cached(self):
        with mock.patch("urllib.request.urlopen", return_value=fake_response(b"payload")) as urlopen:
            got = makehuman.fetch("a/b.txt", self.cache, {"a/b.txt": sha(b"payload")})
        self.assertEqual(urlopen.call_args.args[0], makehuman.RAW_BASE + "a/b.txt")
        self.assertEqual(got, self.cache / "a" / "b.txt")
        self.assertEqual(got.read_bytes(), b"payload")
        self.assertEqual([p.name for p in got.parent.iterdir()], ["b.txt"])   # no stray temp file

    def test_failed_download_names_the_url_and_leaves_no_cache_entry(self):
        with mock.patch("urllib.request.urlopen", side_effect=urllib.error.URLError("offline")):
            with self.assertRaises(makehuman.FetchError) as cm:
                makehuman.fetch("a/b.txt", self.cache, {"a/b.txt": sha(b"x")})
        self.assertIn(makehuman.RAW_BASE + "a/b.txt", str(cm.exception))
        self.assertFalse((self.cache / "a" / "b.txt").exists())

    def test_corrupt_download_is_reported_and_not_cached(self):
        with mock.patch("urllib.request.urlopen", return_value=fake_response(b"tampered")):
            with self.assertRaises(makehuman.FetchError) as cm:
                makehuman.fetch("a/b.txt", self.cache, {"a/b.txt": sha(b"payload")})
        self.assertIn("a/b.txt", str(cm.exception))
        self.assertFalse((self.cache / "a" / "b.txt").exists())

    def test_source_is_pinned_to_the_reviewed_commit(self):
        self.assertEqual(makehuman.PINNED_COMMIT, "a8bc2d54ff0ac92e78ff71431b1023eda42bf482")
        self.assertEqual(makehuman.RAW_BASE,
                         "https://raw.githubusercontent.com/makehumancommunity/makehuman/"
                         "a8bc2d54ff0ac92e78ff71431b1023eda42bf482/")


class ApplyTargets(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.cache = Path(self._tmp.name)
        self.manifest = {}
        for key, text in {"m/a.target": "# a\n0 10 0 0\n2 0 10 0\n", "m/b.target": "2 0 0 10\n"}.items():
            rel = "makehuman/data/targets/" + key
            write(self.cache / rel, text)
            self.manifest[rel] = sha(text.encode())

    def test_sums_weighted_offsets_and_leaves_base_alone(self):
        base = np.arange(9, dtype=float).reshape(3, 3)
        before = base.copy()
        out = makehuman.apply_targets(base, {"m/a.target": 0.5, "m/b.target": 2.0}, self.cache, self.manifest)
        expect = before.copy()
        expect[0] += [0.5, 0, 0]
        expect[2] += [0, 0.5, 2.0]
        np.testing.assert_allclose(out, expect, atol=1e-15)
        np.testing.assert_array_equal(base, before)
        self.assertIsNot(out, base)

    def test_no_weights_returns_an_equal_copy(self):
        base = np.ones((3, 3))
        out = makehuman.apply_targets(base, {}, self.cache, self.manifest)
        np.testing.assert_array_equal(out, base)
        self.assertIsNot(out, base)

    def test_target_files_are_verified_through_the_manifest(self):
        bad = {k: "0" * 64 for k in self.manifest}
        with self.assertRaises(makehuman.FetchError) as cm:
            makehuman.apply_targets(np.zeros((3, 3)), {"m/a.target": 1.0}, self.cache, bad)
        self.assertIn("m/a.target", str(cm.exception))


class RigTests(unittest.TestCase):
    SKEL = {"bones": {
        "root": {"head": "root_h", "tail": "root_t", "parent": None},
        "spine": {"head": "spine_h", "tail": "spine_t", "parent": "root"},
        "arm.L": {"head": "armL_h", "tail": "armL_t", "parent": "spine"},
        "arm.R": {"head": "armR_h", "tail": "armR_t", "parent": "spine"},
        "hand.L": {"head": "handL_h", "tail": "handL_t", "parent": "arm.L"}},
        "joints": {"root_h": [0], "spine_h": [1, 2], "armL_h": [3], "armR_h": [4], "handL_h": [5]}}
    WEIGHTS = {"weights": {"root": [[0, 1.0]], "spine": [[1, 0.25], [2, 0.75]]}}

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        tmp = Path(self._tmp.name)
        self.rig = makehuman.read_rig(write(tmp / "s.mhskel", json.dumps(self.SKEL)),
                                      write(tmp / "w.mhw", json.dumps(self.WEIGHTS)))

    def test_read_rig(self):
        self.assertEqual(self.rig.parent, {"root": None, "spine": "root", "arm.L": "spine",
                                           "arm.R": "spine", "hand.L": "arm.L"})
        self.assertEqual(self.rig.head_joint["hand.L"], "handL_h")
        self.assertEqual(self.rig.joints["spine_h"], [1, 2])
        idx, w = self.rig.weights["spine"]
        self.assertEqual(idx.tolist(), [1, 2])
        self.assertTrue(np.issubdtype(idx.dtype, np.integer))
        np.testing.assert_allclose(w, [0.25, 0.75])
        self.assertNotIn("arm.L", self.rig.weights)

    def test_joint_pos_is_the_mean_of_the_joint_vertices(self):
        verts = np.arange(18, dtype=float).reshape(6, 3)
        np.testing.assert_allclose(makehuman.joint_pos(verts, self.rig, "spine_h"), [4.5, 5.5, 6.5])
        np.testing.assert_allclose(makehuman.joint_pos(verts, self.rig, "root_h"), [0, 1, 2])

    def test_descendants_include_the_bone_and_follow_the_tree(self):
        self.assertEqual(makehuman.descendants(self.rig, "spine"), ["spine", "arm.L", "hand.L", "arm.R"])
        self.assertEqual(makehuman.descendants(self.rig, "hand.L"), ["hand.L"])
        self.assertEqual(len(makehuman.descendants(self.rig, "root")), 5)
        with self.assertRaises(KeyError):
            makehuman.descendants(self.rig, "no-such-bone")


if __name__ == "__main__":
    unittest.main()
