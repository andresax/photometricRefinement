FROM nvidia/opengl:base-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NONINTERACTIVE_SEEN true
ENV NUM_CORES 10

RUN apt-get update --fix-missing 
RUN apt-get -y upgrade
RUN apt-get update
RUN apt install -y --no-install-recommends apt-utils

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    libgflags-dev \
    libglew-dev \
    freeglut3-dev \
    wget \
    git \
    unzip \
    pkg-config \
    libjpeg-dev \
    libtiff-dev \
    libpng-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libatlas-base-dev \
    gfortran \
    libtbb2 \
    libtbb-dev \
    libpq-dev \
    libgmp-dev \
    libmpfr-dev \
    libboost-filesystem-dev \
    libboost-thread-dev \
    libboost-system-dev \
    libopencv-dev \
    libcgal-dev \
    libglfw3-dev \
    libglu1-mesa-dev\
    xorg-dev \
    libglu1-mesa-dev\
    libsoil-dev\
    && rm -rf /var/lib/apt/lists/*


# Install TransforMesh 
COPY external/TransforMesh /src/external/TransforMesh
COPY external/Utils /src/external/Utils
COPY external/CEP /src/external/CEP
RUN mkdir -p /src/external/TransforMesh/build
WORKDIR /src/external/TransforMesh/build
RUN cmake .. && make
RUN cp /src/external/TransforMesh/build/libTransforMesh.so /usr/local/lib/libTransforMesh.so

# Install glfw 
COPY external/glfw /src/external/glfw
RUN mkdir -p /src/external/glfw/build
WORKDIR /src/external/glfw/build
RUN cmake .. && make 
RUN cp /src/external/glfw/build/src/libglfw3.a /usr/local/lib/libglfw3.a

COPY . /src/refinement
RUN mkdir -p /src/refinement/build
WORKDIR /src/refinement/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make -j10 


RUN apt-get purge -y cmake && apt-get autoremove -y
WORKDIR /src/refinement/

CMD /bin/bash