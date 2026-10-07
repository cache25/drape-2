# Drape

Commercial 3D fashion design and garment simulation software. See
`docs/superpowers/specs/` for the design and `docs/superpowers/plans/` for build plans.

## Developer setup (Linux, portable modules)

```
sudo apt-get install -y cmake ninja-build g++ libeigen3-dev python3-matplotlib
cmake --preset dev && cmake --build --preset dev && ctest --preset dev
```

Mac (Homebrew): `brew install cmake ninja eigen@3 git`

Quick test loop: `ctest --preset dev -LE "slow|perf"`. Timing tests: `ctest --preset release -L perf`.

## Developer tools

`drape_dump` drapes the Phase A test tee on a mannequin (`--body male|female`, default male) and writes the
mannequin and the tee to an OBJ file as objects `body` and `tee`. The male wears the men's test tee (`test_tee`),
the female its women's cut (`test_tee_womens`: narrower shoulders and neck, shorter body and
sleeves). It prints one summary line and exits 0 when the garment settled, 1 when it did not, and 2 on bad arguments:

```
./build/dev/tools/drape_dump/drape_dump --out out/tee_M.obj [--size XS|S|M|L|XL|XXL] [--quality draft|standard|fine] [--body male|female]
settled=1 failed=0 seconds=5.43 retries=0 seam_gap_mm=0.03 penetration_mm=0.00 particles=2647
```

`tools/render_obj.py` turns that OBJ into a 1600×1000 preview with front and side views (body off-white, garment blue):

```
python3 tools/render_obj.py out/tee_M.obj out/tee_M.png
```

## Mannequins

`assets/mannequins/` holds the male (1.80 m) and female (1.70 m) display mannequins: egg head, toned build,
smooth feet, arms 45° and legs 5° apart. Each body is a triangle OBJ (metres, Y up, facing +Z), a JSON file with
height, girths, landmarks and region cylinders, and a test-only `checks.json`. They are built from MakeHuman's
CC0 assets (pinned commit, see `assets/mannequins/LICENSE.md`) and loaded with `drape::avatar::loadMannequin`.

Regenerate them with `python3 tools/mannequin/build_mannequins.py` (Python 3 with numpy; the network is needed
once to fill `out/makehuman-cache`). Two runs give byte-identical files.
