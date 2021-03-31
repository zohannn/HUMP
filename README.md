# HUMP: a Human-like Upper-limb Motion Planner
This library is designed to generate human-like trajectories of the arms and the hands of humanoid robots.
Human-like single-arm and dual-arm motion is planned with a naturalistic obstacles-avoidance mechanism. The description of the planner and the results of different experimental tests are published in *Gulletta, G., Silva, E. C. e, Erlhagen, W., Meulenbroek, R., Costa, M. F. P., & Bicho, E. (2021). A Human-like Upper-limb Motion Planner: Generating naturalistic movements for humanoid robots. International Journal of Advanced Robotic Systems.* [https://doi.org/10.1177/1729881421998585](https://doi.org/10.1177/1729881421998585)
 
## Overview
The purpose of this document is to provide a brief installation guide of the library, while more technical details are described in the [Wiki pages](https://github.com/zohannn/HUMP/wiki). 

## Installation guide
Follow the instructions to guide your installation of the HUMP library.

### Dependencies
1. Install the [Eigen libraries](http://eigen.tuxfamily.org/index.php?title=Main_Page).
In Linux Mint anf Ubuntu , you can type:
```Bash
sudo apt install libeigen3-dev
```
2. Install the [boost libraries](https://www.boost.org/).
In Linux Mint anf Ubuntu, you can type:
```Bash
sudo apt-get install libboost-all-dev
```
3. Install [Coinipopt](https://coin-or.github.io/Ipopt/) at the link [https://github.com/zohannn/CoinIpopt](https://github.com/zohannn/CoinIpopt)

4. Install [Google Test](https://github.com/zohannn/googletest)

5. Download a licensed version of [AMPL](https://ampl.com/) into the folder `/home/${USER}/AMPL` 

Then, prepare your installation folders
```Bash
cd /home/${USER}
git clone https://github.com/zohannn/HUMP.git
cd HUMP
```
### Debug
```Bash
mkdir Debug
cd -DCMAKE_BUILD_TYPE=Debug ..
make
make test
make doc
```
### Release
```Bash
mkdir Release
cd -DCMAKE_BUILD_TYPE=Release ..
make
make doc
```

## Upgrades
* The HUMP has been upgraded by [Sara Sá](https://github.com/sarasa22) to support the collaborative robot Sawyer by Rethink Robotics&copy; (see [https://github.com/sarasa22/HUMP](https://github.com/sarasa22/HUMP))  
* The HUMP has been upgraded by [JoaoQPereira](https://github.com/JoaoQPereira) to support the collaborative robot UR5 by Universal Robots&copy; and integrate planing trajectories through a set of waypoints (see [https://github.com/JoaoQPereira/HUMP](https://github.com/JoaoQPereira/HUMP)) 
