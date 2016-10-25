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


/** this struct defines an object in the scenario */
typedef struct{
    string name; /**< name of the object */
    double xpos; /**< position of the object along the x axis in [mm] */
    double ypos; /**< position of the object along the y axis in [mm] */
    double zpos; /**< position of the object along the z axis in [mm] */
    double roll; /**< rotarion of the object around the z axis in [rad] */
    double pitch;/**< rotarion of the object around the y axis in [rad] */
    double yaw;  /**< rotarion of the object around the x axis in [rad] */
    double xsize;/**< size of the object along the x axis in [mm] */
    double ysize;/**< size of the object along the y axis in [mm] */
    double zsize;/**< size of the object along the z axis in [mm] */
}object;

/** this struct defines a human finger */
typedef struct{
    double ux; /**<  position of the finger with respect to the center of the palm along the x axis in [mm] */
    double uy; /**<  position of the finger with respect to the center of the palm along the y axis in [mm] */
    double uz; /**<  position of the finger with respect to the center of the palm along the z axis in [mm] */
    vector<double> d; /**< distances between consecutive frames along the y axes in [mm] */
    vector<double> a; /**< distances between concecutive frames along the z axes in [mm] */
    vector<double> alpha; /**< angle around the x axes between consecutive z axes in [rad] */
    vector<double> theta; /**< angle around the z axes between consecutive x axes in [rad] */
} human_finger;


/** this struct defines the boundary conditions of the movement*/
typedef struct{
    vector<double> vel_0; /**< initial velocity of the joints in [rad/s] */
    vector<double> vel_f; /**< final velocity of the joints in [rad/s] */
    vector<double> acc_0; /**< initial acceleration of the joints in [rad/s²] */
    vector<double> acc_f; /**< final acceleration of the joints in [rad/s²] */
} boundaryConditions;

/** this struct defines the tolerances that have to be set before planning the trajectory*/
typedef struct{
    vector<double> tolsArm; /**< radius of the spheres along the arm in [mm] */
    MatrixXf tolsHand; /**< radius of the spheres along the fingers in [mm] */
    MatrixXf final_tolsObstacles; /**< tolerances of the final posture against the obstacles in [mm] */
    vector< MatrixXf > singleArm_tolsTarget; /**< tolerances of the trajectory against the target in [mm] */
    vector< MatrixXf > singleArm_tolsObstacles; /**< tolerances of the trajectory against the obstacles in [mm] */
    double tolTarPos; /**< tolerance of the final position of the hand against the target in [mm] */
    double tolTarOr; /**< tolerance of the final orientation of the hand against the target in [mm] */
    boundaryConditions bounds; /**< boundary condistions of the bounce problem */
    int steps; /**< number of steps for the trajectory */
    //float totalTime; // normalized time of the movement (0 < t <= 1)
    vector<double> lambda_final; /**< weights for the final posture optimization */
    vector<double> lambda_bounce; /**< weights for the bounce posture optimization */
    vector<double> w_max; /**< maximum angular velocity for each joint [rad/s] */
    //std::vector<float> tols_table; // tolernaces for the table
    bool obstacle_avoidance; /**< true to avoid obstacle */
    bool target_avoidance; /**< true to avoid the target during the motion */
    float eng_dist; /**< distance in [mm] of tolerance from the engage point along the axis  indicated by the eng_dir parameter.
                        It defines the target for the sub-engage movement */
    int eng_dir; /**< direction of tolerance. eng_dir=0 means no direction; eng_dir=1 means the x axis; eng_dir=2 means the y axis; eng_dir=3 means the z axis;*/
    vector<double> eng_tols; /**< tolerances in reaching the engage point in [mm].
                                    eng_tols(0) along the x axis; eng_tols(1) along the y axis; eng_tols(2) along the z axis;*/
    double diseng_dist; /**< distance in [mm] of tolerance from the engage point along the axis  indicated by the diseng_dir parameter.
                        It defines the target for the sub-disengage movement */
    int diseng_dir; /**< direction of tolerance. diseng_dir=0 means no direction; diseng_dir=1 means the x axis; diseng_dir=2 means the y axis; diseng_dir=3 means the z axis;*/

    double tol_stop; /**< this tolerance defines the error between the norm of the final posture and the norm the current posture.
                    It has to be set to stop the movement when the final posture is reached. A tipical value is 0.1  */
} huml_tols;



} // namespace HUMotion




#endif // HUMLConfig_HPP
