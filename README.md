This project is archived. Please visit https://github.com/chenhaox/pytracik for the latest version and full installation instructions.

# Dependencies
- CMake
- Boost
- Eigen
- NLOpt
- KDL  

# Installation

## Windows

Download CMake
https://cmake.org/

### Install Boost

[Download](https://sourceforge.net/projects/boost/files/boost-binaries/) prebuild binary files

### Install Eigen

1. [Download](https://eigen.tuxfamily.org/index.php?title=Main_Page) Eigen (This library is tested in 3.4.0)

2. Build Using CMake

### Install NLOpt

1. Clone from repository

   ```shell 
    git clone https://github.com/stevengj/nlopt
    ```

2. Build Using CMake

### Install KDL

1. Clone from repository
    
    ```shell
    git clone https://github.com/orocos/orocos_kinematics_dynamics.git
    ```
    
2. Build Using CMake


## Linux (Ubuntu)

### Instal Boost, Eigen, KDL Dependencies

```
sudo apt install libboost-all-dev libeigen3-dev liborocos-kdl-dev
```

### Install and Build NLOpt
NLOpt

### Build the Program Using CMake
