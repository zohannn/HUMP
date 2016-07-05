#ifndef HUMLConfig_HPP
#define HUMLConfig_HPP

#include <string>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <list>
#include <vector>
#include <cmath>
#include <boost/smart_ptr.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include "config.h" // hand configuration




// version of the library
#define HUML_VERSION_MAJOR 2
#define HUML_VERSION_MINOR 0

namespace HUMotion{

using namespace std;
using namespace Eigen;

#if HAND==0
    // Human hand
    const float THETA8_HOME = 0.0; //[rad] constant of the joint 8 (spread of the hand)
    const float THETA8_FINAL = -M_PI/2; //[rad] constant of the joint 8 (spread of the hand)
    const float TOL_GRIP = 5.0; // [mm] tolerance on the grip
#elif HAND==1
    // Barrett hand   
    const float THETA8_HOME = 0.0; //[rad] constant of the joint 8 (spread of the hand)
    const float THETA8_FINAL = 0.0; //[rad] constant of the joint 8 (spread of the hand)
    const float TOL_GRIP = 0.0; // [mm] tolerance on the grip
#endif

const int HAND_FINGERS = 3;
const int JOINTS_ARM = 7; // number of joints per arm
const int JOINTS_HAND = 4; // number of joints per hand
const int N_PHALANGE = 3; // number of phalanges per finger



const string ampl_path = string("/home/gianpaolo/catkin_ws/build/MotionPlanner/devel/lib/MotionPlanner/AMPL");
const string SEP = string(", ");
const string METERS = string(" [m]");
const string MILLIMETERS = string(" [mm]");
const string DEG = string(" [deg]");
const string RAD = string(" [rad]");
const string COLUMN = string(":");
const string SPACE = string(" ");
const string XposSTR = string("Xpos = ");
const string YposSTR = string("Ypos = ");
const string ZposSTR = string("Zpos = ");
const string RollSTR = string("Roll = ");
const string PitchSTR = string("Pitch = ");
const string YawSTR = string("Yaw = ");
const string XsizeSTR = string("Xsize = ");
const string YsizeSTR = string("Ysize = ");
const string ZsizeSTR = string("Zsize = ");
const string NOBJECTS = string("n_objects");


typedef struct{

    float Xpos; //[mm]
    float Ypos; //[mm]
    float Zpos; //[mm]

} pos;

typedef struct{

    float roll; //[rad]
    float pitch; //[rad]
    float yaw; //[rad]

} orient;

typedef struct{

    float Xsize; //[mm]
    float Ysize; //[mm]
    float Zsize; //[mm]

} dim;

typedef struct{

    std::vector<float> d; // [mm] distances between consecutive frames along the y axes
    std::vector<float> a; // [mm] distances between concecutive frames along the z axes
    std::vector<float> alpha; // [rad] angle around the x axes between consecutive z axes
    std::vector<float> theta; // [rad] angle around the z axes between consecutive x axes

} DHparams;


typedef struct{


    DHparams arm_specs; // DH parameters of the arm

} arm;


typedef struct{

    float maxAperture; // [mm] max aperture of the hand
    float Aw; //[mm] smallest distance between F1 and F2
    float A1; //[mm] length of the 1st part of the finger
    float A2; //[mm] length of the first phalax
    float A3; //[mm] length of the second phalax
    float D3; //[mm] depth of the fingertip
    float phi2; //[rad] angular displacement between the 1st part of the finger and the 1st phalax
    float phi3; //[rad] angular displacement between the 1st and the 2nd phalax

} barrett_hand;

typedef struct{

    // position of the base of the finger from the palm of the hand
    float ux; // [mm]
    float uy; // [mm]
    float uz; // [mm]
    DHparams finger_specs; // DH parameters of the finger

} human_finger;

typedef struct{

    // position of the base of the thumb from the palm of the hand
    float uTx;// [mm]
    float uTy;// [mm]
    float uTz;// [mm]
    DHparams thumb_specs; // DH parameters of the thumb

} human_thumb;

typedef struct{

  std::vector<human_finger> fingers;
  human_thumb thumb;
  float maxAperture; // [mm] max aperture of the hand

} human_hand;

typedef struct{

    float Xpos; // [m]
    float Ypos; // [m]
    float Zpos; // [m]
    float Roll; // [deg]
    float Pitch; // [deg]
    float Yaw; // [deg]
    float Xsize; // [m]
    float Ysize; // [m]
    float Zsize; // [m]

} humanoid_part;



typedef struct{

    std::vector<float> vel_0; // initial velocity of the joints
    std::vector<float> vel_f; // final velocity of the joints
    std::vector<float> acc_0; // initial acceleration of the joints
    std::vector<float> acc_f; // final acceleration of the joints

} boundaryConditions;

typedef struct{

    std::vector<float> tolsArm; // radius of the spheres along the arm
    MatrixXf tolsHand; // radius of the spheres along the fingers
    MatrixXf final_tolsObstacles; // tolerances of the final posture against the obstacles
    std::vector< MatrixXf > singleArm_tolsTarget; // tolerances of the trajectory against the target
    std::vector< MatrixXf > singleArm_tolsObstacles;// tolerances of the trajectory against the obstacles
    float tolTarPos; // tolerance of the final position of the hand against the target
    float tolTarOr; // tolerance of the final orientation of the hand against the target
    boundaryConditions bounds; // boundary condistions of the bounce problem
    int steps; // number of steps for the trajectory
    //float totalTime; // normalized time of the movement (0 < t <= 1)
    std::vector<float> lambda_final; // weights for the final posture optimization
    std::vector<float> lambda_bounce; // weights for the bounce posture optimization
    std::vector<float> w_max; // maximum angular velocity for each joint [rad/sec]
    std::vector<float> tols_table; // tolernaces for the table

    // engaging parameters
    float eng_dist;
    int eng_dir;
    std::vector<float> eng_tols;
    // disengaging parameters
    float diseng_dist;
    int diseng_dir;    
    float tol_stop; // tol_stop

} Tols;



} // namespace HUMotion

#endif // HUMLConfig_HPP
