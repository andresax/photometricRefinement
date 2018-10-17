# Photometric Mesh Refinement

The code in this repository implements a classical mesh refinement method that minimizes the ZNCC photometric error among pairs of images.

## Installation

Clone the repository

```
git clone --recurse-submodules https://github.com/andresax/photometricRefinement
```


Compile GLFW
```
cd external/glfw/
mkdir build/
cd build
cmake ..
make
```

Compile TransforMesh

```
cd ../../TransforMesh/
mkdir build/
cd build
cmake ..
make
```

Compile Photometric Refinement:

From the partend folder of the project:

```
mkdir build/
cd build
cmake ..
make
```