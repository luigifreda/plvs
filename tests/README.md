Focused line mapping regression tests use the actual PLVS classes and synthetic
geometry; no dataset, vocabulary, display, or GPU is required.

With the project's dependencies and an existing CMake build configured:

```sh
./tests/run_line_mapping_tests.sh
```

The script follows the same steps. `BUILD_DIR` selects the CMake build directory
(default `build`), `JOBS` sets compile parallelism (default `2`), and any extra
arguments are forwarded to the configure step:

```sh
cmake -S . -B build -DPLVS_BUILD_TESTS=ON
cmake --build build --target test_line_mapping -j2
ctest --test-dir build --output-on-failure -R '^line_'
```

The tests cover stereo association and count balance, repeated/conflicting
registration and stale feature-slot cleanup, transferring camera observations
during replacement, all three
TUM-VI line pyramid levels alongside eight point levels, reference-view
projection, missing-camera fusion, recovery of valid one-sided triangulation
matches, metadata change reporting, and valid/invalid stereo triangulation.
Checks remain enabled in Release builds. These tests do not replace a TUM-VI
trajectory and map-quality replay.

Run the evaluation-script alignment regressions (requires NumPy) with:

```sh
python3 tests/test_evaluation_alignment.py
```

The pointer helper regression is standalone and needs only a C++17 compiler:

```sh
c++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests/test_pointers.cc -o /tmp/plvs_test_pointers
/tmp/plvs_test_pointers
```

It checks shared control-block identity, lifetime extension, weak-pointer expiry,
rejection of unmanaged objects, null wrapping, move-only factory arguments, and
the existing raw-pointer declarations and deletion behavior.

The octree cleanup regressions exercise both segmented and non-segmented map
updates. They check that a centroid across a voxel face cannot cause deletion of
a stable neighbor, that multiple old unstable leaves are removed, and that recent
unstable leaves survive. A second update checks that the stable leaf remains in
the octree as well as the output cloud.

```sh
cmake -S . -B build -DPLVS_BUILD_TESTS=ON
cmake --build build --target test_octree_cleanup -j2
ctest --test-dir build --output-on-failure -R '^octree_cleanup_'
```
