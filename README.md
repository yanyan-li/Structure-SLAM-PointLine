# Structure-SLAM
Structure-SLAM is a real-time monocular SLAM library that computes the camera trajectory and a sparse 3D reconstruction based on the assumption of Manhattan World. We provide examples to run the system in the [TUM RGB-D dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) and [ICL NUIM dataset](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html). 

​            ![structure-slam](images/structure-slam.png)

# 1. License

Structure-SLAM is released under a [GPLv3 license](https://github.com/raulmur/StructureSLAM/blob/master/License-gpl.txt). 

For a closed-source version of Structure-SLAM for commercial purposes, please contact the us.

# 2. Prerequisites
We have tested the library in **Ubuntu** **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

# 3. Test Structure-SLAM

## Download and build

Clone the repository:
```
git clone https://github.com/yanyan-li/Structure-SLAM.git 
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *Structure-SLAM*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd Structure-SLAM
chmod +x build.sh
./build.sh
```

## Run on ICL NUIM dataset

1. Prepare a dataset to PATH_TO_SEQUENCE_FOLDER, which contains  RGB images, normal maps.
```
RGB File
Normal File
associations.txt
```
*You can train your own network for normal prediction or download the our normal prediction results for testing, which can be found [here](https://drive.google.com/open?id=12rgrg92CFt7DcdTO4a7tthqbVKsmcfGn) or in Examples/dataset.*

2. Execute the following command. 

```
./Examples/Structure-SLAM Vocabulary/ORBvoc.txt Examples/ICL.yaml PATH_TO_SEQUENCE_FOLDER
```

# Acknowledgements

We thank [Raul Mur-Artal](https://github.com/raulmurfor) for his impressive work, [**ORB-SLAM2**](https://github.com/raulmur/ORB_SLAM), which is a completed feature-based SLAM system.   

