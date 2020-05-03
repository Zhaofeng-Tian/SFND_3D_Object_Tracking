# SFND 3D Object Tracking

## (TODO) Overview  
Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Prerequisites/Dependencies  
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## (TODO) Project Description  
## Run the project  
* Clone this repository  
```
git clone https://github.com/jinchaolu/SFND-Term1-P3-3D-Object-Tracking.git
```
* Navigate to the `SFND-Term1-P3-3D-Object-Tracking` folder  
```
cd SFND-Term1-P3-3D-Object-Tracking
```
* Create and open `build` folder  
```
mkdir build && cd build
```
* Compile your code  
```
cmake .. && make
```
* Run `3D_object_tracking` application  
```
./3D_object_tracking
```

## Tips  
1. It's recommended to update and upgrade your environment before running the code.  
```
sudo apt-get update && sudo apt-get upgrade -y
```
2. You might encounter some segmentation fault. Might be caused by the incomplete yolov3.weights. Download the dataset again with the following command.  
```
wget https://pjreddie.com/media/files/yolov3.weights
```

## Code Style  
Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).  

## (TODO) Project Rubric  
### 1. FP.0 Final Report  
#### 1.1 Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.  

### 2. FP.1 Match 3D Objects  
#### 2.1 Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.  

### 3. FP.2 Compute Lidar-based TTC  
#### 3.1 Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.  

### 4. FP.3 Associate Keypoint Correspondences with Bounding Boxes  
#### 4.1 Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.  

### 5. FP.4 Compute Camera-based TTC  
#### 5.1 Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.  

### 6. FP.5 Performance Evaluation 1  
#### 6.1 Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.  

### 7. FP.6 Performance Evaluation 2  
#### 7.1 Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.  