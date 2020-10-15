Panorama - exercise for the 3D Computer vision course
=====================================================

This program is an exercise for the 3D computer vision course, as part of the IPP MVA master.

## Requirements

- cmake (2.6 minimum)
- GNU Make
- Imagine++ library

## Usage

In a terminal in this directory, type:

```bash
mkdir build
cd build
cmake ..
make
./Panorama
```

## Grading

The sources are in `Panorama.cpp`

## Additional info

- The panorama rendering mode can be switched between "Pull" and "Push" using a preprocessor variable in the source. Some postprocessing functions are available to try and reduce the rendering artefacts when using push method.
- Precalculated points can be used instead of the clicked points. One only needs to uncomment the commented lines in the `getClicks` function.
