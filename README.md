# HUMP: a Human-like Upper-limb Motion Planner
This library is designed to generate human-like trajectories of the arms and the hands of humanoid robots.
Human-like single-arm and dual-arm motion is planned with a naturalistic obstacles-avoidance mechanism.
## Overview
The purpose of this document is to provide a brief installation guide of the library, while more technical details are described in the [Wiki pages](https://github.com/zohannn/HUMP/wiki). 

## Installation guide
Folloe the instructions to guide your installation of the HUMP library.

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
