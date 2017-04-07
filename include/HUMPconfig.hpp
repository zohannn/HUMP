#ifndef HUMPConfig_HPP
#define HUMPConfig_HPP

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
#include "object.hpp"



/** version of the library */
#define HUMP_VERSION_MAJOR 2
#define HUMP_VERSION_MINOR 0


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


using namespace std;
using namespace Eigen;

/** This is the main namespace of the library */
namespace HUMotion{

typedef boost::shared_ptr<Object> objectPtr;/**< shared pointer to an object in the scenario */

const double PHI = (-log(2.0)/log(TB));/**< parameter to control when the bounce posture is reached */
const double AP = 30.0*static_cast<double>(M_PI)/180; /**< aperture of the fingers when approaching to pick [rad] */

const double BLANK_PERCENTAGE = 0.15; /**< percentage of steps to eliminate when either reaching to grasp an object OR move at the beginning of a move movement [0,1]*/

const int N_STEP_MIN = 5; /**< minimum number of steps */
const int N_STEP_MAX = 100; /**< maximum number of steps */

const double W_RED_MIN = 1; /**< minimum value of angular velocity reduction when approaching */
const double W_RED_MAX = 15; /**< maximum value of angular velocity reduction when approaching */

/** this struct defines the Denavit-Hartenberg kinematic parameters */
typedef struct{
    vector<double> d; /**< distances between consecutive frames along the y axes in [mm] */
    vector<double> a; /**< distances between concecutive frames along the z axes in [mm] */
    vector<double> alpha; /**< angle around the x axes between consecutive z axes in [rad] */
    vector<double> theta; /**< angle around the z axes between consecutive x axes in [rad] */
} DHparameters;

/** this struct defines the barrett hand */
typedef struct{
    double maxAperture; /**< [mm] max aperture of the hand in [mm] */
    double Aw; /**< smallest distance between F1 and F2 in [mm] */
    double A1; /**< length of the 1st part of the finger in [mm] */
    double A2; /**< length of the first phalax in [mm] */
    double A3; /**< length of the second phalax in [mm] */
    double D3; /**< depth of the fingertip in [mm] */
    double phi2; /**< angular displacement between the 1st part of the finger and the 1st phalax in [rad] */
    double phi3; /**< angular displacement between the 1st and the 2nd phalax in [rad] */
    vector<int> rk; /**< r parameters of the barrett hand */
    vector<int> jk; /**< j parameters of the barrett hand */
} BarrettHand;

/** this struct defines a human finger */
typedef struct{
    double ux; /**<  position of the finger with respect to the center of the palm along the x axis in [mm] */
    double uy; /**<  position of the finger with respect to the center of the palm along the y axis in [mm] */
    double uz; /**<  position of the finger with respect to the center of the palm along the z axis in [mm] */
    DHparameters finger_specs; /**< the Denavit-Hartenberg parameters of the finger */
} HumanFinger;

/** this struct defines a human thumb */
typedef struct{
    double uTx; /**<  position of the thumb with respect to the center of the palm along the x axis in [mm] */
    double uTy; /**<  position of the thumb with respect to the center of the palm along the y axis in [mm] */
    double uTz; /**<  position of the thumb with respect to the center of the palm along the z axis in [mm] */
    DHparameters thumb_specs; /**< the Denavit-Hartenberg parameters of the thumb */
} HumanThumb;

/** this struct defines a human hand */
typedef struct{
  vector<HumanFinger> fingers; /**< fingers of the hand */
  HumanThumb thumb; /**<  thumb of the hand */
  double maxAperture; /**< max aperture of the hand in [mm] */
} HumanHand;

/** this struct defines the parameters of the movement */
typedef struct{
    int arm_code; /**< the code of the arm: 0 = both arms, 1 = right arm, 2 = left arm */
    int hand_code;/**< the code of the hand: 0 = human hand, 1 = barrett hand */
    int griptype; /**< the type of the grip */
    string mov_infoline; /**< description of the movement */
    double dHO;/**< distanche hand-target*/
    std::vector<double> finalHand;/**< final posture of the hand */
    std::vector<double> target;/**< target / location to reach: target(0)=x, target(1)=y, target(2)=z, target(3)=roll, target(4)=pitch, target(6)=yaw,*/
    objectPtr obj; /**< object involved in the movement. The info of the object have to be updated according to the selected movement */
    bool approach;/**< true to use the approach options, false otherwise  */
    bool retreat;/**< true to use the retreat options, false otherwise */
    std::vector<double> pre_grasp_approach; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    std::vector<double> post_grasp_retreat; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    std::vector<double> pre_place_approach; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    std::vector<double> post_place_retreat; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    bool rand_init; /**< true to select randon initialization in "plan" stages */
    bool coll; /**< true to enable collisions with the environment (body of the robot included) */
    bool use_move_plane; /**< true to constrain the end-effector to move on a plane in move movements, false otherwise*/
    std::vector<double> plane_params; /**< plane cartesian parameters in move movements: a*x+b*y+c*z+d=0. a=plane_params(0), b=plane_params(1), c=plane_params(2), d=plane_params(3) */
}mov_params;
/** this struct defines the boundary conditions of the movement*/
typedef struct{
    vector<double> vel_0; /**< initial velocity of the joints in [rad/s] */
    vector<double> vel_f; /**< final velocity of the joints in [rad/s] */
    vector<double> acc_0; /**< initial acceleration of the joints in [rad/s²] */
    vector<double> acc_f; /**< final acceleration of the joints in [rad/s²] */
} boundaryConditions;

/** this struct defines the tolerances that have to be set before planning the trajectory*/
typedef struct{
    mov_params mov_specs; /**< specifications of the movement */
    vector<double> tolsArm; /**< radius of the spheres along the arm in [mm] */
    MatrixXd tolsHand; /**< radius of the spheres along the fingers in [mm] */
    MatrixXd final_tolsObstacles; /**< tolerances of the final posture against the obstacles in [mm] */
    vector< MatrixXd > singleArm_tolsTarget; /**< tolerances of the trajectory against the target in [mm] */
    vector< MatrixXd > singleArm_tolsObstacles; /**< tolerances of the trajectory against the obstacles in [mm] */
    double tolTarPos; /**< tolerance of the final position of the hand against the target in [mm] */
    double tolTarOr; /**< tolerance of the final orientation of the hand against the target in [mm] */
    boundaryConditions bounds; /**< boundary condistions of the bounce problem */
    vector<double> vel_approach; /**< velocity of the joints in [rad/s] at the beginning of the approach stage */
    vector<double> acc_approach; /**< acceleration of the joints in [rad/s²] at the beginning of the approach stage */
    vector<double> lambda_final; /**< weights for the final posture optimization */
    vector<double> lambda_bounce; /**< weights for the bounce posture optimization */
    vector<double> w_max; /**< maximum angular velocity for each joint [rad/s] */
    bool obstacle_avoidance; /**< true to avoid obstacle */
    bool target_avoidance; /**< true to avoid the target during the motion */
} hump_params;

/** This struct defines the result of the planned trajectory */
typedef struct{
    int mov_type;/**< type of the planned movement */
    int status;/**< status code of the planning */
    string status_msg;/**< status message of the planning */
    string object_id;/**< identity of the object involved in the movement */
    vector<MatrixXd> trajectory_stages;/**< sequence of the trajectories */
    vector<MatrixXd> velocity_stages;/**< sequence of the velocities */
    vector<MatrixXd> acceleration_stages;/**< sequence of the accelerations */
    vector<double> time_steps; /**< sequence of each time steps for each trajectory */
    vector<string> trajectory_descriptions;/**< description of the trajectories */
}planning_result;



} // namespace HUMotion




#endif // HUMPConfig_HPP
