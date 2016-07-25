#include "../include/common_functions.hpp"


using namespace std;
using namespace Eigen;
using namespace HUMotion;


void CommonFunctions::getRPY_matrix(Matrix3f& Rot, orient orr){

    Rot = Matrix3f::Zero();

    float roll = orr.roll; // around z
    float pitch = orr.pitch; // around y
    float yaw = orr.yaw; // around x

    // Rot = Rot_z * Rot_y * Rot_x

    Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);


}

void CommonFunctions::getTrans_matrix(Matrix4f& Trans, orient orr, pos poss){

    Trans = Matrix4f::Zero();

    Matrix3f Rot;
    getRPY_matrix(Rot,orr);

    Trans(0,0) = Rot(0,0); Trans(0,1) = Rot(0,1); Trans(0,2) = Rot(0,2); Trans(0,3) = poss.Xpos;
    Trans(1,0) = Rot(1,0); Trans(1,1) = Rot(1,1); Trans(1,2) = Rot(1,2); Trans(1,3) = poss.Ypos;
    Trans(2,0) = Rot(2,0); Trans(2,1) = Rot(2,1); Trans(2,2) = Rot(2,2); Trans(2,3) = poss.Zpos;
    Trans(3,0) = 0;        Trans(3,1) = 0;        Trans(3,2) = 0;        Trans(3,3) = 1;

}


float CommonFunctions::getNorm(pos poss){
    return sqrt(pow((poss.Xpos),2)+pow((poss.Ypos),2)+pow((poss.Zpos),2));
}


void CommonFunctions::getRotAxis(std::vector<float>& xt, orient orr, int id){

    Matrix3f Rot;
    getRPY_matrix(Rot,orr);
    Vector3f v = Rot.col(id);

    // get the components of the axis
    xt.push_back(v(0)); // x
    xt.push_back(v(1)); // y
    xt.push_back(v(2)); // z
}


void CommonFunctions::getRPY(Matrix4f Trans, std::vector<float>& rpy){

    rpy = std::vector<float>(3);

    if(abs(Trans(0,0) < 1e-10) && (abs(Trans(1,0) < 1e-10))){

        rpy.at(0) = 0; // [rad]
        rpy.at(1) = atan2(-Trans(2,0),Trans(0,0)); // [rad]
        rpy.at(2) = atan2(-Trans(1,2),Trans(1,1)); // [rad]

    }else{

        rpy.at(0) = atan2(Trans(1,0),Trans(0,0)); // [rad]
        float sp = sin(rpy.at(0));
        float cp = cos(rpy.at(0));
        rpy.at(1) = atan2(-Trans(2,0), cp*Trans(0,0)+sp*Trans(1,0)); // [rad]
        rpy.at(2) = atan2(sp*Trans(0,2)-cp*Trans(1,2),cp*Trans(1,1)-sp*Trans(0,1)); // [rad]

    }


}


