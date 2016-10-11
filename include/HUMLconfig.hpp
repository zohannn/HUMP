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

#include "../config/config.hpp"




/** version of the library */
#define HUML_VERSION_MAJOR 1
#define HUML_VERSION_MINOR 0

/** definitions for printing output */
#define SEP ", "
#define METERS " [m]"
#define MILLIMETERS " [mm]"
#define DEG " [deg]"
#define RAD " [rad]"
#define COLUMN ":"
#define SPACE " "
#define XposSTR "Xpos = "
#define YposSTR "Ypos = "
#define ZposSTR "Zpos = "
#define RollSTR "Roll = "
#define PitchSTR "Pitch = "
#define YawSTR "Yaw = "
#define XsizeSTR "Xsize = "
#define YsizeSTR "Ysize = "
#define ZsizeSTR "Zsize = "


//definition of the macro ASSERT
#ifndef DEBUG
    #define ASSERT(x)
#else
    #define ASSERT(x) \
             if (! (x)) \
            { \
               cout << "ERROR!! Assert " << #x << " failed\n"; \
               cout << " on line " << __LINE__  << "\n"; \
               cout << " in file " << __FILE__ << "\n";  \
            }
#endif


/** This is the main namespace of the library */
namespace HUMotion{

using namespace std;
using namespace Eigen;

#if HAND==0
    // Human hand
    const float THETA8_HOME = 0.0; /**< constant of the joint 8 (spread of the hand) in [rad] */
    const float THETA8_FINAL = -M_PI/2; /**< constant of the joint 8 (spread of the hand) in [rad] */
    const float TOL_GRIP = 5.0; /**< tolerance on the grip in [mm] */
#elif HAND==1
    // Barrett hand   
    const float THETA8_HOME = 0.0; /**< constant of the joint 8 (spread of the hand) in [rad] */
    const float THETA8_FINAL = 0.0; /**< constant of the joint 8 (spread of the hand) in [rad] */
    const float TOL_GRIP = 0.0; /**< tolerance on the grip in [mm] */
    const int firstPartTorqueOvershootCountRequired = 1;/**< number of time that the torque applied to the first phalanx is bigger than firstPartMaxTorque */
    const float firstPartMaxTorque = 0.9f;/**< max torque that can be applied to the first phalanx of the fingers */
    const float closingOpeningTorque = 1.0f;/**< torque applied to the fingers when they are opening/closing */
    const float closingVel = 60.0f * static_cast<float>(M_PI) / 180.0f; /**< joint velocity of the fingers when they are closing */
    const float openingVel = -120.0f * static_cast<float>(M_PI) / 180.0f;/**< joint velocity of the fingers when they are opening */
#endif

const int HAND_FINGERS = 3; /**< number of fingers per hand */
const int JOINTS_ARM = 7; /**< number of joints per arm */
const int JOINTS_HAND = 4; /**< number of joints per hand */
const int N_PHALANGE = 3; /**< number of phalanges per finger */

/** this struct defines the position in the Cartesian space*/
typedef struct{
    float Xpos; /**< position along the x axis in [mm] */
    float Ypos; /**< position along the y axis in [mm] */
    float Zpos; /**< position along the z axis in [mm] */
} pos;

/** this struct defines the orientation in Roll-Pitch-Yaw */
typedef struct{
    float roll; /**< rotarion around the z axis in [rad] */
    float pitch; /**< rotarion around the y axis in [rad] */
    float yaw; /**< rotarion around the x axis in [rad] */
} orient;

/** this struct defines the dimention of an object */
typedef struct{
    float Xsize; /**< size of the object along the x axis in [mm] */
    float Ysize; /**< size of the object along the y axis in [mm] */
    float Zsize; /**< size of the object along the z axis in [mm] */
} dim;

/** this struct defines the Denavit-Hartenberg kinematic parameters */
typedef struct{
    std::vector<float> d; /**< distances between consecutive frames along the y axes in [mm] */
    std::vector<float> a; /**< distances between concecutive frames along the z axes in [mm] */
    std::vector<float> alpha; /**< angle around the x axes between consecutive z axes in [rad] */
    std::vector<float> theta; /**< angle around the z axes between consecutive x axes in [rad] */
} DHparams;

/** this struct defines the arm */
typedef struct{
    DHparams arm_specs; /**< the Denavit-Hartenberg parameters of the arm */
} arm;

/** this struct defines the barrett hand */
typedef struct{
    float maxAperture; /**< [mm] max aperture of the hand in [mm] */
    float Aw; /**< smallest distance between F1 and F2 in [mm] */
    float A1; /**< length of the 1st part of the finger in [mm] */
    float A2; /**< length of the first phalax in [mm] */
    float A3; /**< length of the second phalax in [mm] */
    float D3; /**< depth of the fingertip in [mm] */
    float phi2; /**< angular displacement between the 1st part of the finger and the 1st phalax in [rad] */
    float phi3; /**< angular displacement between the 1st and the 2nd phalax in [rad] */
} barrett_hand;

/** this struct defines a human finger */
typedef struct{
    float ux; /**<  position of the finger with respect to the center of the palm along the x axis in [mm] */
    float uy; /**<  position of the finger with respect to the center of the palm along the y axis in [mm] */
    float uz; /**<  position of the finger with respect to the center of the palm along the z axis in [mm] */
    DHparams finger_specs; /**< the Denavit-Hartenberg parameters of the finger */
} human_finger;

/** this struct defines a human thumb */
typedef struct{
    float uTx; /**<  position of the thumb with respect to the center of the palm along the x axis in [mm] */
    float uTy; /**<  position of the thumb with respect to the center of the palm along the y axis in [mm] */
    float uTz; /**<  position of the thumb with respect to the center of the palm along the z axis in [mm] */
    DHparams thumb_specs; /**< the Denavit-Hartenberg parameters of the thumb */
} human_thumb;

/** this struct defines a human hand */
typedef struct{
  std::vector<human_finger> fingers; /**< fingers of the hand */
  human_thumb thumb; /**<  thumb of the hand */
  float maxAperture; /**< max aperture of the hand in [mm] */
} human_hand;

/** this struct defines a generic part of a humanoid body */
typedef struct{
    float Xpos; /**< position of the part along the x axis in [mm] */
    float Ypos; /**< position of the part along the y axis in [mm] */
    float Zpos; /**< position of the part along the z axis in [mm] */
    float Roll; /**< orientation of the part around the z axis in [rad] */
    float Pitch; /**< orientation of the part around the y axis in [rad] */
    float Yaw; /**< orientation of the part around the x axis in [rad] */
    float Xsize; /**< size of the part along the x axis in [mm] */
    float Ysize; /**< size of the part along the y axis in [mm] */
    float Zsize; /**< size of the part along the z axis in [mm] */
} humanoid_part;

/** this struct defines the boundary conditions of the movement*/
typedef struct{
    std::vector<float> vel_0; /**< initial velocity of the joints in [rad/s] */
    std::vector<float> vel_f; /**< final velocity of the joints in [rad/s] */
    std::vector<float> acc_0; /**< initial acceleration of the joints in [rad/s²] */
    std::vector<float> acc_f; /**< final acceleration of the joints in [rad/s²] */
} boundaryConditions;

/** this struct defines the tolerances that have to be set before planning the trajectory*/
typedef struct{
    std::vector<float> tolsArm; /**< radius of the spheres along the arm in [mm] */
    MatrixXf tolsHand; /**< radius of the spheres along the fingers in [mm] */
    MatrixXf final_tolsObstacles; /**< tolerances of the final posture against the obstacles in [mm] */
    std::vector< MatrixXf > singleArm_tolsTarget; /**< tolerances of the trajectory against the target in [mm] */
    std::vector< MatrixXf > singleArm_tolsObstacles; /**< tolerances of the trajectory against the obstacles in [mm] */
    float tolTarPos; /**< tolerance of the final position of the hand against the target in [mm] */
    float tolTarOr; /**< tolerance of the final orientation of the hand against the target in [mm] */
    boundaryConditions bounds; /**< boundary condistions of the bounce problem */
    int steps; /**< number of steps for the trajectory */
    //float totalTime; // normalized time of the movement (0 < t <= 1)
    std::vector<float> lambda_final; /**< weights for the final posture optimization */
    std::vector<float> lambda_bounce; /**< weights for the bounce posture optimization */
    std::vector<float> w_max; /**< maximum angular velocity for each joint [rad/s] */
    //std::vector<float> tols_table; // tolernaces for the table
    bool obstacle_avoidance; /**< true to avoid obstacle */
    bool target_avoidance; /**< true to avoid the target during the motion */
    float eng_dist; /**< distance in [mm] of tolerance from the engage point along the axis  indicated by the eng_dir parameter.
                        It defines the target for the sub-engage movement */
    int eng_dir; /**< direction of tolerance. eng_dir=0 means no direction; eng_dir=1 means the x axis; eng_dir=2 means the y axis; eng_dir=3 means the z axis;*/
    std::vector<float> eng_tols; /**< tolerances in reaching the engage point in [mm].
                                    eng_tols(0) along the x axis; eng_tols(1) along the y axis; eng_tols(2) along the z axis;*/
    float diseng_dist; /**< distance in [mm] of tolerance from the engage point along the axis  indicated by the diseng_dir parameter.
                        It defines the target for the sub-disengage movement */
    int diseng_dir; /**< direction of tolerance. diseng_dir=0 means no direction; diseng_dir=1 means the x axis; diseng_dir=2 means the y axis; diseng_dir=3 means the z axis;*/

    float tol_stop; /**< this tolerance defines the error between the norm of the final posture and the norm the current posture.
                    It has to be set to stop the movement when the final posture is reached. A tipical value is 0.1  */
} Tols;



} // namespace HUMotion




#endif // HUMLConfig_HPP
