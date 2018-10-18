# Photometric Mesh Refinement

The code in this repository implements a classical mesh refinement method that minimizes the ZNCC photometric error among pairs of images.
It takes a json file in the format defined by openMVG [here](https://openmvg.readthedocs.io/en/latest/software/SfM/SfM_OutputFormat/).


## Dependencies

The codde is implemented in c++ and OpenGL. You can run the following in Ubuntu/Debian to install the required dependencies

sudo apt install libboost-filesystem-dev libboost-thread-dev libboost-system-dev libopencv-dev libglew-dev libcgal-dev freeglut3-dev libmpfr-dev libglu1-mesa-dev

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

From the partent folder of the project:

```
mkdir build/
cd build
cmake ..
make
```


Run the example.

From the partent folder of the project:

First uncompress the json (with the output of openMVG):

```
unzip sfm_data.zip sfm_data.json
```

Then, execute the program with the configuration file as parameter.

```
./build/photometricRefinement ./config/configDatasetCastle.conf
```

The Results will be saved into the folder ```Result->Result_Castle_*``` where * will be replaced by the values of the parameter of the current configuration.