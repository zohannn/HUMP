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

// Final posture selection problem ipopt options
const double FINAL_TOL = 1e-6; /**< desired convergence tolerance */
const double FINAL_ACC_TOL = 1e-2; /**< acceptable convergence tolerance */
const double FINAL_CONSTR_VIOL_TOL = 0.0001; /**< constraints violation tolerance */

const double FINAL_DUAL_TOL = 1e-2; /**< desired convergence tolerance */
const double FINAL_DUAL_ACC_TOL = 1e-1; /**< acceptable convergence tolerance */
const double FINAL_DUAL_CONSTR_VIOL_TOL = 0.0001; /**< constraints violation tolerance */

// Bounce posture selection problem ipopt options
const double BOUNCE_TOL = 1e-6; /**< desired convergence tolerance*/
const double BOUNCE_ACC_TOL = 1e-3; /**< acceptable convergence tolerance */
const double BOUNCE_CONSTR_VIOL_TOL = 0.0001; /**< constraints violation tolerance */
//const double BOUNCE_CONSTR_VIOL_TOL = 1e-6; /**< constraints violation tolerance */

const double BOUNCE_DUAL_TOL = 1e-2; /**< desired convergence tolerance*/
const double BOUNCE_DUAL_ACC_TOL = 1e-1; /**< acceptable convergence tolerance */
const double BOUNCE_DUAL_CONSTR_VIOL_TOL = 1e-3; /**< constraints violation tolerance */

const double BOUNCE_DUAL_OBJ_POS_TOL = 10; /**< tolerance in position when transporting one object with both hands [mm]*/
const double BOUNCE_DUAL_OBJ_OR_TOL = 0.3; /**< tolerance in orientation when transporting one object with both hands [mm]*/

const double SPACER = 1.0*static_cast<double>(M_PI)/180; /**< degree used to space the joint limits [deg]: IPOPT sometimes does not fully respect all the constraints,
                                                              but the joint limits has to be respected
                                                              This parameter helps to stay within the joint range */

const double SPACER_BOUNCE = 0.0*static_cast<double>(M_PI)/180; /**< degree used to space the joint limits [deg]: IPOPT sometimes does not fully respect all the constraints,
                                                                         but the joint limits has to be respected
                                                                         This parameter helps to stay within the joint range */

const double PHI = (-log(2.0)/log(TB));/**< parameter to control when the bounce posture is reached */
const double AP = 15.0*static_cast<double>(M_PI)/180; /**< aperture of the fingers when approaching to pick [rad] */

const double BLANK_PERCENTAGE_TAR = 0.10; /**< percentage of steps to eliminate when either reaching to grasp an object [0,1]*/
const double BLANK_PERCENTAGE_OBS = 0.20;/**< move at the beginning of a move movement [0,1] */

const double BLANK_PERCENTAGE_DUAL_TAR = 0.10; /**< percentage of steps to eliminate when reaching to grasp an object [0,1]*/
const double BLANK_PERCENTAGE_DUAL_OBS = 0.15;/**< move at the beginning of a move movement [0,1] */

//const int N_STEP_MIN = 5; /**< minimum number of steps */
//const int N_STEP_MAX = 50; /**< maximum number of steps */

// learning taks: reaching with one obstacle
const int N_STEP_MIN = 14; /**< minimum number of steps */
const int N_STEP_MAX = 14; /**< maximum number of steps */

const double WARM_START_BOUND_PUSH = 1e-6; /**< k1 in section 3.6 of the paper about IPOPT */
const double WARM_START_MULT_BOUND_PUSH = 1e-6; /**< bound push of the multipliers */
const double MU_INIT = 1e-6; /**< initial value of the barrier parameter */

const double W_RED_MIN = 1; /**< minimum value of angular velocity reduction when approaching and retreating */
//const double W_RED_APP_MAX = 5; /**< maximum value of angular velocity reduction when approaching */
//const double W_RED_RET_MAX = 5; /**< maximum value of angular velocity reduction when retreating */

const bool HAS_JOINT_ACCELEARATION_MAX_LIMIT = true; /**< true to check acceleration maximum limit, false otherwise */

static bool abs_compare(double a, double b)
{
    return (std::abs(a) < std::abs(b));
}

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

/** This struct defines the parameters of the warm start settings */
typedef struct{
    bool valid; /**< true if the struct is valid, false otherwise */
    int iterations; /**< number of iterations taken */
    double cpu_time; /**< cpu time in [sec] taken */
    double obj_value; /**< final value of the objective function */
    vector<double> x; /**< initial guess */
    vector<double> zL; /**< lower bounds multipliers */
    vector<double> zU; /**< upper bounds multipliers */
    vector<double> dual_vars; /**< lagrange multipliers of the constraints */
    string description; /**< description of the related problem. It must be either "plan" or "approach" or "retreat" or "bounce" */
}warm_start_params;

/** this struct defines the parameters of the movement */
typedef struct{
    int arm_code; /**< the code of the arm: 0 = both arms, 1 = right arm, 2 = left arm */
    int hand_code;/**< the code of the hand: 0 = human hand, 1 = barrett hand */
    //int griptype; /**< the type of the grip */
    string mov_infoline; /**< description of the movement */
    double dHO;/**< distanche hand-target */
    std::vector<double> finalHand;/**< final posture of the hand */
    std::vector<double> target;/**< target / location to reach: target(0)=x, target(1)=y, target(2)=z, target(3)=roll, target(4)=pitch, target(6)=yaw,*/
    objectPtr obj; /**< object involved in the movement. The info of the object have to be updated according to the selected movement */
    Matrix4d T_tar_to_obj; /**< transformation matrix from the target to the obj = inv(T_tar)*T_obj */
    std::string support_obj; /**< name of the object that is a support surface in place movements */
    bool approach; /**< true to use the approach options, false otherwise  */
    bool retreat; /**< true to use the retreat options, false otherwise */
    bool straight_line; /**< true to use the straight line option of the approach/retreat stage */
    double w_red_app_max;/**< maximum velocity reduction factor when approaching */
    double w_red_ret_max;/**< maximum velocity reduction factor when retreating */
    std::vector<double> pre_grasp_approach; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    std::vector<double> post_grasp_retreat; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    std::vector<double> pre_place_approach; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    std::vector<double> post_place_retreat; /**< (0)= x component, (1)= y component, (2)= z component, (3)= distance from the target*/
    bool rand_init; /**< true to select randon initialization in "plan" stages */
    bool coll; /**< true to enable collisions with the environment (body of the robot included) */
    bool use_move_plane; /**< true to constrain the end-effector to move on a plane in move movements, false otherwise*/
    std::vector<double> plane_params; /**< plane cartesian parameters in move movements: a*x+b*y+c*z+d=0. a=plane_params(0), b=plane_params(1), c=plane_params(2), d=plane_params(3) */
    bool warm_start; /**< true to use warm-start options, false otherwise */
    vector<warm_start_params> final_warm_start_params; /**< warm start params of the target posture selection problems */
    warm_start_params bounce_warm_start_params; /**< warm start params of the bounce posture selection problem */
}mov_params;

/** this struct defines the boundary conditions of the movement*/
typedef struct{
    vector<double> vel_0; /**< initial velocity of the joints in [rad/s] */
    vector<double> vel_f; /**< final velocity of the joints in [rad/s] */
    vector<double> acc_0; /**< initial acceleration of the joints in [rad/s²] */
    vector<double> acc_f; /**< final acceleration of the joints in [rad/s²] */
} boundaryConditions;

/** this struct defines the tolerances that have to be set before planning the trajectory (single-arm)*/
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
    vector<double> alpha_max; /**< maximum angular acceleration for each joint [rad/s²] */
    bool obstacle_avoidance; /**< true to avoid obstacle */
    bool target_avoidance; /**< true to avoid the target during the motion */
    bool coll_body; /**< true to enable collisions avoidance with the body */
} hump_params;

/** this struct defines the tolerances that have to be set before planning the trajectory (dual-arm) */
typedef struct{
    mov_params mov_specs_right; /**< specifications of the movement (right) */
    mov_params mov_specs_left; /**< specifications of the movement (left) */
    vector<double> tolsArm_right; /**< radius of the spheres along the right arm in [mm] */
    vector<double> tolsArm_left; /**< radius of the spheres along the left arm in [mm] */
    MatrixXd tolsHand_right; /**< radius of the spheres along the right hand fingers in [mm] */
    MatrixXd tolsHand_left; /**< radius of the spheres along the left hand fingers in [mm] */
    MatrixXd final_tolsObstacles_right; /**< tolerances of the final right posture against the obstacles in [mm] */
    MatrixXd final_tolsObstacles_left; /**< tolerances of the final left posture against the obstacles in [mm] */
    vector< MatrixXd > singleArm_tolsTarget_right; /**< tolerances of the trajectory against the target in [mm] (right) */
    vector< MatrixXd > singleArm_tolsTarget_left; /**< tolerances of the trajectory against the target in [mm] (left) */
    vector< MatrixXd > singleArm_tolsObstacles_right; /**< tolerances of the trajectory against the obstacles in [mm] (right) */
    vector< MatrixXd > singleArm_tolsObstacles_left; /**< tolerances of the trajectory against the obstacles in [mm] (left) */
    double tolTarPos_right; /**< tolerance of the final position of the right hand against the target in [mm] */
    double tolTarPos_left; /**< tolerance of the final position of the left hand against the target in [mm] */
    double tolTarOr_right; /**< tolerance of the final orientation of the right hand against the target in [mm] */
    double tolTarOr_left; /**< tolerance of the final orientation of the left hand against the target in [mm] */
    boundaryConditions bounds_right; /**< boundary condistions of the bounce problem (right) */
    boundaryConditions bounds_left; /**< boundary condistions of the bounce problem (left) */
    vector<double> vel_approach_right; /**< velocity of the joints in [rad/s] at the beginning of the approach stage (right) */
    vector<double> vel_approach_left; /**< velocity of the joints in [rad/s] at the beginning of the approach stage (left) */
    vector<double> acc_approach_right; /**< acceleration of the joints in [rad/s²] at the beginning of the approach stage (right) */
    vector<double> acc_approach_left; /**< acceleration of the joints in [rad/s²] at the beginning of the approach stage (left) */
    vector<double> lambda_final_right; /**< weights for the final posture optimization (right) */
    vector<double> lambda_final_left; /**< weights for the final posture optimization (left) */
    vector<double> lambda_bounce_right; /**< weights for the bounce posture optimization (right) */
    vector<double> lambda_bounce_left; /**< weights for the bounce posture optimization (left) */
    vector<double> w_max; /**< maximum angular velocity for each joint [rad/s] */
    vector<double> alpha_max; /**< maximum angular acceleration for each joint [rad/s²] */
    bool obstacle_avoidance; /**< true to avoid obstacle */
    bool target_avoidance; /**< true to avoid the target during the motion */
    bool coll_arms; /**< true to enable collisions avoidance between the arms-hands */
    bool coll_body; /**< true to enable collisions avoidance with the body */
}hump_dual_params;

/** This struct defines the result of the planned trajectory (single-arm) */
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
    vector<warm_start_params> final_warm_start_res; /**< warm start results of the target posture selection problems */
    warm_start_params bounce_warm_start_res; /**< warm start results of the bounce posture selection problem */
}planning_result;

/** This struct defines the result of the planned trajectory (dual-arm) */
typedef struct{
    int mov_type_right;/**< type of the planned movement (right) */
    int mov_type_left;/**< type of the planned movement (left) */
    int status;/**< status code of the planning */
    string status_msg;/**< status message of the planning */
    string object_right_id;/**< identity of the object involved in the movement (right) */
    string object_left_id;/**< identity of the object involved in the movement (left) */
    vector<MatrixXd> trajectory_stages;/**< sequence of the trajectories */
    vector<MatrixXd> velocity_stages;/**< sequence of the velocities */
    vector<MatrixXd> acceleration_stages;/**< sequence of the accelerations */
    vector<double> time_steps; /**< sequence of each time steps for each trajectory */
    vector<string> trajectory_descriptions;/**< description of the trajectories */
}planning_dual_result;



} // namespace HUMotion




#endif // HUMPConfig_HPP
