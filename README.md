# Drape

Commercial 3D fashion design and garment simulation software. See
`docs/superpowers/specs/` for the design and `docs/superpowers/plans/` for build plans.

## Developer setup (Linux, portable modules)

```
sudo apt-get install -y cmake ninja-build g++ libeigen3-dev python3-matplotlib
cmake --preset dev && cmake --build --preset dev && ctest --preset dev
```

Quick test loop: `ctest --preset dev -LE "slow|perf"`. Timing tests: `ctest --preset release -L perf`.
