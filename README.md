# Photometric Mesh Refinement

The code in this repository implements a classical mesh refinement method that minimizes the ZNCC photometric error among pairs of images.
It takes a json file in the format defined by openMVG [here](https://openmvg.readthedocs.io/en/latest/software/SfM/SfM_OutputFormat/).


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

```
./build/photometricRefinement ./config/configDatasetCastle.conf
```

The Results will be saved into the folder ```Result->Result_Castle_*``` where * will be replaced by the values of the parameter of the current configuration.