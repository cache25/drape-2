# Drape

Commercial 3D fashion design and garment simulation software. See
`docs/superpowers/specs/` for the design and `docs/superpowers/plans/` for build plans.

## Developer setup (Linux, portable modules)

```
sudo apt-get install -y cmake ninja-build g++ libeigen3-dev python3-matplotlib
cmake --preset dev && cmake --build --preset dev && ctest --preset dev
```

Quick test loop: `ctest --preset dev -LE "slow|perf"`. Timing tests: `ctest --preset release -L perf`.

## Developer tools

`drape_dump` drapes the Phase A test tee on the test body and writes both to an OBJ file. It prints one summary
line and exits 0 when the garment settled, 1 when it did not, and 2 on bad arguments:

```
./build/dev/tools/drape_dump/drape_dump --out out/tee_M.obj [--size XS|S|M|L|XL|XXL] [--quality draft|standard|fine]
settled=1 failed=0 seconds=5.43 retries=0 seam_gap_mm=0.03 penetration_mm=0.00 particles=2647
```

`tools/render_obj.py` turns that OBJ into a 1600×1000 preview with front and side views (body grey, garment blue):

```
python3 tools/render_obj.py out/tee_M.obj out/tee_M.png
```
