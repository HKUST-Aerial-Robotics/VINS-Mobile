# VINS-Mobile
## Monocular Visual-Inertial State Estimator on Mobile Phones

VINS-Mobile is a real-time monocular visual-inertial state estimator developed by members of the [HKUST Aerial Robotics Group](http://uav.ust.hk/). It runs on compatible iOS devices, and provides localization services for augmented reality (AR) applications. It is also tested for state estimation and feedback control for autonomous drones. VINS-Mobile uses sliding window optimization-based formulation for providing high-accuracy visual-inertial odometry with automatic initialization and failure recovery. The accumulated odometry errors are corrected in real-time using global pose graph SLAM. An AR demonstration is provided to showcase its capability.

**Authors:** [Peiliang LI](https://github.com/PeiliangLi), [Tong QIN](https://github.com/qintony), [Zhenfei YANG](https://github.com/dvorak0), Kejie QIU, and [Shaojie SHEN](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [HKUST Aerial Robotics Group](http://uav.ust.hk/)

**Videos:** https://www.youtube.com/watch?v=qazzGT84Scc&feature=youtu.be

**Related Papers:**
* [**Monocular Visual-Inertial State Estimation for Mobile Augmented Reality**](http://www.ece.ust.hk/~eeshaojie/ismar2017peiliang.pdf), *P.Li et al (submitted to ISMAR 2017)*
* [**Robust Initialization of Monocular Visual-Inertial Estimation on Aerial Robots**](http://www.ece.ust.hk/~eeshaojie/iros2017tong.pdf), *T.Qin et al (submitted to IROS 2017)*
* [**Monocular Visual-Inertial State Estimation With Online Initialization and Camera-IMU Extrinsic Calibration**](http://ieeexplore.ieee.org/document/7463059/), *Z.Yang et al (T-ASE 2017)*

*If you use VINS-Mobile for your academic research, please cite at least one of our related papers.*

## 1. Build

The code has been compiled on macOS Sierra with Xcode 8.3.1 and tested with iOS 10.2.1 on iPhone7 Plus.

1.1 Install boost for macOS
```
$ ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
$ brew install boost
```

1.2 Download specific **opencv2.framework** from [here](http://uav.ust.hk/storage/opencv2.framework.zip), then unzip it to VINS_ThirdPartyLib/opencv2.framework
    **(Please make sure you haven't installed opencv for your OSX)**

1.3 In your Xcode, select **Product**-> **Scheme**-> **Edit Scheme**-> **Run**-> **Info**, set **Build Configuration** to **Release** (not debug)

1.4 **Slect your device** at upper left corner, then **choose your device size** at Main.storyboard, build and run

1.5 Compatible Devices and iOS version requiements

	iPhone7 Plus, iPhone7, iPhone6s Plus, iPhone6s, iPad Pro
	iOS 10.2.1 and above

## 2. Acknowledgements

We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBow](https://github.com/dorian3d/DBoW2) for loop detection.

Thanks the contributions of [Yang Liu](https://github.com/wandermyz) and [Botao Hu](http://amber.botao.hu/) from [Amber Garage](https://ambergarage.com/).

## 3. Licence

The source code is released under [GPLv3](http://www.gnu.org/licenses/) licence.

We are still working for improving the code readability. Welcome to contribute to VINS-Mobile or ask any issues via Github or contacting Peiliang LI <pliap@connect.ust.hk> or Tong QIN <tong.qin@connect.ust.hk>.

For commercial inqueries, please contact Shaojie SHEN <eeshaojie@ust.hk>
