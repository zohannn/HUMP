#include "../include/humplanner.hpp"


namespace HUMotion {

unsigned HUMPlanner::joints_arm = 7;
unsigned HUMPlanner::joints_hand = 4;
unsigned HUMPlanner::hand_fingers = 3;
unsigned HUMPlanner::n_phalange = 3;

HUMPlanner::HUMPlanner(string name = string ("Default Planner"))
{
    // planner settings
    this->name = name;

    // humanoid settings
    this->matWorldToRightArm = Matrix4d::Identity(4,4);
    this->matWorldToLeftArm = Matrix4d::Identity(4,4);
    this->matRightHand = Matrix4d::Identity(4,4);
    this->matLeftHand = Matrix4d::Identity(4,4);

    this->minRightLimits = vector<double>(joints_arm+joints_hand);
    this->minLeftLimits = vector<double>(joints_arm+joints_hand);
    this->maxRightLimits = vector<double>(joints_arm+joints_hand);
    this->maxLeftLimits= vector<double>(joints_arm+joints_hand);

    this->torso_size = {0,0,0};

}



HUMPlanner::HUMPlanner(const HUMPlanner &hp)
{
    // planner settings
    this->name=hp.name;

    //humanoid settings
    this->matWorldToRightArm = hp.matWorldToRightArm;
    this->matWorldToLeftArm = hp.matWorldToLeftArm;
    this->matRightHand = hp.matRightHand;
    this->matLeftHand = hp.matLeftHand;

    this->minRightLimits = hp.minRightLimits;
    this->minLeftLimits = hp.minLeftLimits;
    this->maxRightLimits = hp.maxRightLimits;
    this->maxLeftLimits = hp.maxLeftLimits;

    this->torso_size = hp.torso_size;

    this->DH_rightArm = hp.DH_rightArm;
    this->DH_leftArm = hp.DH_leftArm;

    this->hhand = hp.hhand;
    this->bhand = hp.bhand;

    // scenario settings
    if(!hp.obstacles.empty()){this->obstacles=hp.obstacles;}
    //if(hp.obj_tar!=NULL){this->obj_tar=objectPtr(new Object(*(hp.obj_tar.get())));}
}

HUMPlanner::~HUMPlanner()
{

}

void HUMPlanner::setName(string name)
{
    this->name = name;
}

string HUMPlanner::getName()
{
    return this->name;
}

void HUMPlanner::addObstacle(objectPtr obs)
{
    this->obstacles.push_back(objectPtr(new Object(*obs.get())));
}

bool HUMPlanner::setObstacle(objectPtr obs, unsigned pos)
{
    if(this->obstacles.size() > pos){
        this->obstacles.at(pos) = objectPtr(new Object(*obs.get()));
        return true;
    }else{
        return false;
    }
}

bool HUMPlanner::getObstacles(std::vector<objectPtr> &obs)
{
    obs.clear();
    if(!this->obstacles.empty()){
        obs = std::vector<objectPtr>(this->obstacles.size());
        std::copy(this->obstacles.begin(),this->obstacles.end(),obs.begin());
        return true;
    }else{
        return false;
    }
}

objectPtr HUMPlanner::getObstacle(unsigned pos)
{
    if(this->obstacles.size() > pos){
        std::vector<objectPtr>::iterator ii = this->obstacles.begin();
        advance(ii,pos);
        return (*ii);
    }else{
        return NULL;
    }
}

objectPtr HUMPlanner::getObstacle(std::string name)
{
    objectPtr obj = NULL;

    for(std::size_t i=0; i<this->obstacles.size();++i){
        string n = this->obstacles.at(i)->getName();
        if(boost::iequals(n,name)){
            obj=this->obstacles.at(i);
            break;
        }
    }
    return obj;
}
/*
void HUMPlanner::setObjTarget(objectPtr obj)
{
    this->obj_tar = objectPtr(new Object(*obj.get()));
}

objectPtr HUMPlanner::getObjTarget()
{
    return this->obj_tar;
}
*/
void HUMPlanner::clearScenario()
{
    this->obstacles.clear();
    //this->obj_tar = NULL;
}

void HUMPlanner::setMatRightArm(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->matWorldToRightArm(i, j) = m(i,j);
        }
    }
}

void HUMPlanner::getMatRightArm(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->matWorldToRightArm(i, j);
        }
    }
}

void HUMPlanner::setMatLeftArm(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->matWorldToLeftArm(i, j) = m(i,j);
        }
    }
}

void HUMPlanner::getMatLeftArm(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->matWorldToLeftArm(i, j);
        }
    }
}

void HUMPlanner::setMatRightHand(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->matRightHand(i, j) = m(i,j);
        }
    }
}

void HUMPlanner::getMatRightHand(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->matRightHand(i, j);
        }
    }
}

void HUMPlanner::setMatLeftHand(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            this->matLeftHand(i, j) = m(i,j);
        }
    }
}

void HUMPlanner::getMatLeftHand(Matrix4d &m)
{
    for (unsigned i = 0; i < m.rows(); ++ i){
        for (unsigned j = 0; j < m.cols(); ++ j){
            m (i, j) = this->matLeftHand(i, j);
        }
    }
}

void HUMPlanner::setRightMinLimits(vector<double> &min_rl)
{
    if(!min_rl.empty()){
        std::copy(min_rl.begin(),min_rl.end(),this->minRightLimits.begin());
    }
}

void HUMPlanner::getRightMinLimits(vector<double> &min_rl)
{
   if(!this->minRightLimits.empty()){
    min_rl = vector<double>(joints_arm+joints_hand);
    std::copy(this->minRightLimits.begin(),this->minRightLimits.end(),min_rl.begin());
   }
}

void HUMPlanner::setRightMaxLimits(vector<double> &max_rl)
{
    if(!max_rl.empty()){
        std::copy(max_rl.begin(),max_rl.end(),this->maxRightLimits.begin());
    }
}

void HUMPlanner::getRightMaxLimits(vector<double> &max_rl)
{
    if(!this->maxRightLimits.empty()){
     max_rl = vector<double>(joints_arm+joints_hand);
     std::copy(this->maxRightLimits.begin(),this->maxRightLimits.end(),max_rl.begin());
    }
}

void HUMPlanner::setLeftMinLimits(vector<double> &min_ll)
{
    if(!min_ll.empty()){
        std::copy(min_ll.begin(),min_ll.end(),this->minLeftLimits.begin());
    }
}

void HUMPlanner::getLeftMinLimits(vector<double> &min_ll)
{
    if(!this->minLeftLimits.empty()){
     min_ll = vector<double>(joints_arm+joints_hand);
     std::copy(this->minLeftLimits.begin(),this->minLeftLimits.end(),min_ll.begin());
    }
}

void HUMPlanner::setLeftMaxLimits(vector<double> &max_ll)
{
    if(!max_ll.empty()){
        std::copy(max_ll.begin(),max_ll.end(),this->maxLeftLimits.begin());
    }
}

void HUMPlanner::getLeftMaxLimits(vector<double> &max_ll)
{
    if(!this->maxLeftLimits.empty()){
     max_ll = vector<double>(joints_arm+joints_hand);
     std::copy(this->maxLeftLimits.begin(),this->maxLeftLimits.end(),max_ll.begin());
    }
}

void HUMPlanner::setTorsoSize(vector<double> &tsize)
{
    this->torso_size = tsize;
}

void HUMPlanner::getTorsoSize(vector<double> &tsize)
{    
    tsize = this->torso_size;
}

void HUMPlanner::setDH_rightArm(DHparameters &p)
{
    this->DH_rightArm = p;
}

DHparameters HUMPlanner::getDH_rightArm()
{
    return this->DH_rightArm;
}

void HUMPlanner::setDH_leftArm(DHparameters &p)
{
    this->DH_leftArm = p;
}

DHparameters HUMPlanner::getDH_leftArm()
{
    return this->DH_leftArm;
}

void HUMPlanner::setBarrettHand(BarrettHand &bhand)
{
    this->bhand = bhand;
}

BarrettHand HUMPlanner::getBarrettHand()
{
    return this->bhand;
}

void HUMPlanner::setHumanHand(HumanHand &hhand)
{
    this->hhand = hhand;
}

HumanHand HUMPlanner::getHumanHand()
{
    return this->hhand;
}

void HUMPlanner::setShpos(std::vector<double> &shPos)
{
    this->shPos = shPos;
}

void HUMPlanner::getShpos(std::vector<double> &shPos)
{
    shPos = this->shPos;
}


void HUMPlanner::writeBodyDim(double h_xsize,double h_ysize, ofstream &stream)
{

    string bodyxsize=  boost::str(boost::format("%.2f") % (h_xsize/2));
    boost::replace_all(bodyxsize,",",".");
    string bodyysize=  boost::str(boost::format("%.2f") % (h_ysize/2));
    boost::replace_all(bodyysize,",",".");

    stream << string("# BODY INFO \n");
    stream << string("param body := \n");
    stream << to_string(1)+string(" ")+bodyxsize+string("\n");
    stream << to_string(2)+string(" ")+bodyysize+string(";\n");
}

void HUMPlanner::writeArmDHParams(DHparameters dh, ofstream &stream, int k)
{
    // D-H Parameters of the Arm
    stream << string("# D-H PARAMETERS OF THE ARM \n");
    stream << string("param alpha := \n");
    string alpha_str;
    alpha_str =  boost::str(boost::format("%.2f") % (dh.alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_str+string("\n");
    for(std::size_t i=1; i< joints_arm; ++i){
        alpha_str =  boost::str(boost::format("%.2f") % (k*dh.alpha.at(i)));

        if (i == dh.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_str+string("\n");
        }
    }
    stream << string("param a := \n");
    string a_str;
    for(std::size_t i=0; i<joints_arm; ++i){
        a_str =  boost::str(boost::format("%.2f") % (dh.a.at(i)));
        if (i == dh.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_str+string("\n");
        }
    }
    stream << string("param d := \n");
    string d_str;
    for(std::size_t i=0; i<joints_arm; ++i){
        d_str =  boost::str(boost::format("%.2f") % (k*dh.d.at(i)));
        if (i == dh.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_str+string("\n");
        }
    }
}

void HUMPlanner::write_dHO(std::ofstream& stream, double dHO)
{

    string dHOstr=  boost::str(boost::format("%.2f") % (dHO));
    boost::replace_all(dHOstr,",",".");
    stream << string("# DISTANCE HAND - TARGET\n");
    stream << string("param dFH := ")+dHOstr+string(";\n");
}

void HUMPlanner::writeArmLimits(ofstream &stream, std::vector<double> &minArmLimits, std::vector<double> &maxArmLimits)
{

    stream << string("# JOINT LIMITS \n");
    stream << string("# Lower Bound \n");
    stream << string("param llim := \n");

    for (std::size_t i=0; i < minArmLimits.size(); ++i){
        string minLim=  boost::str(boost::format("%.2f") % (minArmLimits.at(i)));
        boost::replace_all(minLim,",",".");
        if (i == minArmLimits.size()-1){
            stream << to_string(i+1)+string(" ")+minLim+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+minLim+string("\n");
        }
    }

    stream << string("# Upper Bound \n");
    stream << string("param ulim := \n");

    for (std::size_t i=0; i < maxArmLimits.size(); ++i){
        string maxLim=  boost::str(boost::format("%.2f") % (maxArmLimits.at(i)));
        boost::replace_all(maxLim,",",".");

        if (i == maxArmLimits.size()-1){
            stream << to_string(i+1)+string(" ")+maxLim+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+maxLim+string("\n");
        }
    }

}

void HUMPlanner::writeArmInitPose(ofstream &stream, std::vector<double> &initArmPosture)
{

    stream << string("# INITIAL POSE \n");
    stream << string("param thet_init := \n");

    for (std::size_t i=0; i < initArmPosture.size(); ++i){
        string initArmstr =  boost::str(boost::format("%.2f") % (initArmPosture.at(i)));
        boost::replace_all(initArmstr,",",".");
        if (i == initArmPosture.size()-1){
            stream << to_string(i+1)+string(" ")+initArmstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+initArmstr+string("\n");
        }
    }

}

void HUMPlanner::writeFingerFinalPose(ofstream &stream, std::vector<double> &finalHand)
{

    stream << string("# FINAL FINGER JOINTS \n");
    stream << string("param joint_fingers := \n");

    for (std::size_t i=0; i < finalHand.size(); ++i){
        string finalHandstr =  boost::str(boost::format("%.2f") % (finalHand.at(i)));
        boost::replace_all(finalHandstr,",",".");
        if (i == finalHand.size()-1){
            stream << to_string(i+1)+string(" ")+finalHandstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+finalHandstr+string("\n");
        }
    }

}

void HUMPlanner::writeLambda(ofstream &stream, std::vector<double> &lambda)
{

    stream << string("# JOINT EXPENSE FACTORS \n");
    stream << string("param lambda := \n");

    for (std::size_t i=0; i < lambda.size(); ++i){
        string lambdastr =  boost::str(boost::format("%.2f") % (lambda.at(i)));
        boost::replace_all(lambdastr,",",".");
        if (i == lambda.size()-1){
            stream << to_string(i+1)+string(" ")+lambdastr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+lambdastr+string("\n");
        }
    }
}

void HUMPlanner::writeHumanHandParams(HumanHand& hhand, std::ofstream& stream, int k)
{
    stream << string("# PARAMETERS OF THE HAND \n");

    // Index finger
    stream << string("# Index Finger \n");
    HumanFinger finger_1 = hhand.fingers.at(0);

    stream << string("param u1x := ");
    string ux_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.ux));
    stream << ux_fing1_str+string(";\n");

    stream << string("param u1y := ");
    string uy_fing1_str =  boost::str(boost::format("%.2f") % (k*finger_1.uy));
    stream << uy_fing1_str+string(";\n");

    stream << string("param u1z := ");
    string uz_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.uz));
    stream << uz_fing1_str+string(";\n");

    stream << string("param alpha_fing1 := \n");
    string alpha_fing1_str;
    alpha_fing1_str =  boost::str(boost::format("%.2f") % (k*finger_1.finger_specs.alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_fing1_str+string("\n");
    for(std::size_t i=1; i<n_phalange+1; ++i){
        alpha_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.alpha.at(i)));
        if (i == finger_1.finger_specs.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_fing1_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_fing1_str+string("\n");
        }
    }
    stream << string("param a_fing1 := \n");
    string a_fing1_str;
    for(std::size_t i=0; i<n_phalange+1; ++i){
        a_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.a.at(i)));
        if (i == finger_1.finger_specs.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_fing1_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_fing1_str+string("\n");
        }
    }
    stream << string("param d_fing1 := \n");
    string d_fing1_str;
    for(std::size_t i=0; i<n_phalange+1; ++i){
        d_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.d.at(i)));
        if (i == finger_1.finger_specs.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_fing1_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_fing1_str+string("\n");
        }
    }
    stream << string("param theta0_fing1 := ");
    string theta0_fing1_str;
    theta0_fing1_str =  boost::str(boost::format("%.2f") % (k*finger_1.finger_specs.theta.at(0)));
    stream << theta0_fing1_str+string(";\n");

    // Ring finger
    stream << string("# Ring Finger \n");
    HumanFinger finger_3 = hhand.fingers.at(2);

    stream << string("param u3x := ");
    string ux_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.ux));
    stream << ux_fing3_str+string(";\n");

    stream << string("param u3y := ");
    string uy_fing3_str =  boost::str(boost::format("%.2f") % (k*finger_3.uy));
    stream << uy_fing3_str+string(";\n");

    stream << string("param u3z := ");
    string uz_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.uz));
    stream << uz_fing3_str+string(";\n");

    stream << string("param alpha_fing3 := \n");
    string alpha_fing3_str;
    alpha_fing3_str =  boost::str(boost::format("%.2f") % (k*finger_3.finger_specs.alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_fing3_str+string("\n");
    for(std::size_t i=1; i<n_phalange+1; ++i){
        alpha_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.alpha.at(i)));
        if (i == finger_3.finger_specs.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_fing3_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_fing3_str+string("\n");
        }
    }
    stream << string("param a_fing3 := \n");
    string a_fing3_str;
    for(std::size_t i=0; i<n_phalange+1; ++i){
        a_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.a.at(i)));
        if (i == finger_3.finger_specs.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_fing3_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_fing3_str+string("\n");
        }
    }
    stream << string("param d_fing3 := \n");
    string d_fing3_str;
    for(std::size_t i=0; i<n_phalange+1; ++i){
        d_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.d.at(i)));
        if (i == finger_3.finger_specs.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_fing3_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_fing3_str+string("\n");
        }
    }
    stream << string("param theta0_fing3 := ");
    string theta0_fing3_str;
    theta0_fing3_str =  boost::str(boost::format("%.2f") % (k*finger_3.finger_specs.theta.at(0)));
    stream << theta0_fing3_str+string(";\n");

    // Thumb finger
    stream << string("# Thumb Finger \n");
    HumanThumb thumb_finger = hhand.thumb;

    stream << string("param uTx := ");
    string uTx_fing2_str =  boost::str(boost::format("%.2f") % (thumb_finger.uTx));
    stream << uTx_fing2_str+string(";\n");

    stream << string("param uTy := ");
    string uTy_fing2_str =  boost::str(boost::format("%.2f") % (k*thumb_finger.uTy));
    stream << uTy_fing2_str+string(";\n");

    stream << string("param uTz := ");
    string uTz_fing2_str =  boost::str(boost::format("%.2f") % (thumb_finger.uTz));
    stream << uTz_fing2_str+string(";\n");

    stream << string("param alpha_thumb := \n");
    string alpha_thumb_str;
    if (k == 1){ // right hand
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(0)));
    }else if(k==-1){// left hand
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(0)-M_PI/2));
    }
    stream << to_string(1)+string(" ")+alpha_thumb_str+string("\n");
    for(std::size_t i=1; i<n_phalange+2; ++i){
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(i)));
        if (i == thumb_finger.thumb_specs.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_thumb_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_thumb_str+string("\n");
        }
    }

    stream << string("param a_thumb := \n");
    string a_thumb_str;
    for(std::size_t i=0; i<n_phalange+2; ++i){
        a_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.a.at(i)));
        if (i == thumb_finger.thumb_specs.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_thumb_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_thumb_str+string("\n");
        }
    }

    stream << string("param d_thumb := \n");
    string d_thumb_str;
    for(std::size_t i=0; i<n_phalange+2; ++i){
        d_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.d.at(i)));
        if (i == thumb_finger.thumb_specs.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_thumb_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_thumb_str+string("\n");
        }
    }

    stream << string("param theta0_thumb:= ");
    string theta0_thumb_str;
    theta0_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.theta.at(0)));
    stream << theta0_thumb_str+string(";\n");
}

void HUMPlanner::writeHumanHandParamsMod(ofstream &stream)
{
    stream << string("# Parameters of the Hand \n");

    stream << string("# Index finger \n");
    stream << string("param u1x; \n");
    stream << string("param u1y; \n");
    stream << string("param u1z; \n");
    stream << string("param alpha_fing1 {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
    stream << string("param a_fing1 {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
    stream << string("param d_fing1 {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
    stream << string("param theta0_fing1; \n");

    stream << string("# Ring finger \n");
    stream << string("param u3x; \n");
    stream << string("param u3y; \n");
    stream << string("param u3z; \n");
    stream << string("param alpha_fing3 {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
    stream << string("param a_fing3 {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
    stream << string("param d_fing3 {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
    stream << string("param theta0_fing3; \n");

    stream << string("# Thumb finger \n");
    stream << string("param uTx; \n");
    stream << string("param uTy; \n");
    stream << string("param uTz; \n");
    stream << string("param alpha_thumb {i in 1..")+to_string(n_phalange+2)+string("} ; \n");
    stream << string("param a_thumb {i in 1..")+to_string(n_phalange+2)+string("} ; \n");
    stream << string("param d_thumb {i in 1..")+to_string(n_phalange+2)+string("} ; \n");
    stream << string("param theta0_thumb; \n");

}

void HUMPlanner::writeBarrettHandParams(BarrettHand& bhand, std::ofstream& stream)
{


    // rk and jk parameters
    stream << string("# R and J parameters \n");
    stream << string("param rk := \n");

    for (size_t i=0; i < bhand.rk.size(); ++i){
        string rkstr =  boost::str(boost::format("%.2f") % (bhand.rk.at(i)));
        boost::replace_all(rkstr,",",".");
        if (i == bhand.rk.size()-1){
            stream << to_string(i+1)+string(" ")+rkstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+rkstr+string("\n");
        }
    }

    stream << string("param jk := \n");

    for (size_t i=0; i < bhand.jk.size(); ++i){
        string jkstr =  boost::str(boost::format("%.2f") % (bhand.jk.at(i)));
        boost::replace_all(jkstr,",",".");
        if (i == bhand.jk.size()-1){
            stream << to_string(i+1)+string(" ")+jkstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+jkstr+string("\n");
        }

    }


    string Awstr =  boost::str(boost::format("%.2f") % (bhand.Aw));
    boost::replace_all(Awstr,",",".");
    stream << string("param Aw :=")+Awstr+string(";\n");

    string A1str =  boost::str(boost::format("%.2f") % (bhand.A1));
    boost::replace_all(A1str,",",".");
    stream << string("param A1 :=")+A1str+string(";\n");

    string A2str =  boost::str(boost::format("%.2f") % (bhand.A2));
    boost::replace_all(A2str,",",".");
    stream << string("param A2 :=")+A2str+string(";\n");

    string A3str =  boost::str(boost::format("%.2f") % (bhand.A3));
    boost::replace_all(A3str,",",".");
    stream << string("param A3 :=")+A3str+string(";\n");

    string D3str =  boost::str(boost::format("%.2f") % (bhand.D3));
    boost::replace_all(D3str,",",".");
    stream << string("param D3 :=")+D3str+string(";\n");

    string phi2str =  boost::str(boost::format("%.2f") % (bhand.phi2));
    boost::replace_all(phi2str,",",".");
    stream << string("param phi_2 :=")+phi2str+string(";\n");

    string phi3str =  boost::str(boost::format("%.2f") % (bhand.phi3));
    boost::replace_all(phi3str,",",".");
    stream << string("param phi_3 :=")+phi3str+string(";\n");

}

void HUMPlanner::writeBarrettHandParamsMod(ofstream &stream)
{
    stream << string("param rk {i in 1..3} ; \n");
    stream << string("param jk {i in 1..3} ; \n");
    stream << string("param Aw; \n");
    stream << string("param A1; \n");
    stream << string("param A2; \n");
    stream << string("param A3; \n");
    stream << string("param D3; \n");
    stream << string("param phi_2; \n");
    stream << string("param phi_3; \n");
}

void HUMPlanner::writeInfoTarget(ofstream &stream, std::vector<double> tar)
{
    stream << string("# TARGET POSITION \n");
    stream << string("param Tar_pos := \n");

    string tarx =  boost::str(boost::format("%.2f") % (tar.at(0)));
    boost::replace_all(tarx,",",".");
    stream << to_string(1)+string(" ")+tarx+string("\n");

    string tary =  boost::str(boost::format("%.2f") % (tar.at(1)));
    boost::replace_all(tary,",",".");
    stream << to_string(2)+string(" ")+tary+string("\n");

    string tarz =  boost::str(boost::format("%.2f") % (tar.at(2)));
    boost::replace_all(tarz,",",".");
    stream << to_string(3)+string(" ")+tarz+string(";\n");

    stream << string("# TARGET ORIENTATION \n");
    std::vector<double> rpy = {tar.at(3),tar.at(4),tar.at(5)};
    std::vector<double> xt; std::vector<double> yt; std::vector<double> zt;
    this->getRotAxis(xt,0,rpy);
    this->getRotAxis(yt,1,rpy);
    this->getRotAxis(zt,2,rpy);

    stream << string("param x_t := \n");
    string tarxt0 =  boost::str(boost::format("%.2f") % (xt[0]));
    boost::replace_all(tarxt0,",",".");
    stream << to_string(1)+string(" ")+tarxt0+string("\n");

    string tarxt1 =  boost::str(boost::format("%.2f") % (xt[1]));
    boost::replace_all(tarxt1,",",".");
    stream << to_string(2)+string(" ")+tarxt1+string("\n");

    string tarxt2 =  boost::str(boost::format("%.2f") % (xt[2]));
    boost::replace_all(tarxt2,",",".");
    stream << to_string(3)+string(" ")+tarxt2+string(";\n");

    stream << string("param y_t := \n");
    string taryt0 =  boost::str(boost::format("%.2f") % (yt[0]));
    boost::replace_all(taryt0,",",".");
    stream << to_string(1)+string(" ")+taryt0+string("\n");

    string taryt1 =  boost::str(boost::format("%.2f") % (yt[1]));
    boost::replace_all(taryt1,",",".");
    stream << to_string(2)+string(" ")+taryt1+string("\n");

    string taryt2 =  boost::str(boost::format("%.2f") % (yt[2]));
    boost::replace_all(taryt2,",",".");
    stream << to_string(3)+string(" ")+taryt2+string(";\n");

    stream << string("param z_t := \n");
    string tarzt0 =  boost::str(boost::format("%.2f") % (zt[0]));
    boost::replace_all(tarzt0,",",".");
    stream << to_string(1)+string(" ")+tarzt0+string("\n");

    string tarzt1 =  boost::str(boost::format("%.2f") % (zt[1]));
    boost::replace_all(tarzt1,",",".");
    stream << to_string(2)+string(" ")+tarzt1+string("\n");

    string tarzt2 =  boost::str(boost::format("%.2f") % (zt[2]));
    boost::replace_all(tarzt2,",",".");
    stream << to_string(3)+string(" ")+tarzt2+string(";\n");

}


void HUMPlanner::writeInfoApproachRetreat(ofstream &stream, std::vector<double> tar, std::vector<double> approach_retreat)
{
    double dist = approach_retreat.at(3);
    string dist_str = boost::str(boost::format("%.2f") % dist); boost::replace_all(dist_str,",",".");

    std::vector<double> rpy = {tar.at(3),tar.at(4),tar.at(5)};
    Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
    Vector3d v(approach_retreat.at(0),approach_retreat.at(1),approach_retreat.at(2));
    Vector3d vv =Rot_tar*v;

    stream << string("# VECTOR APPROACH/RETREAT DISTANCE \n");
    stream << string("param dist :=")+dist_str+string(";\n");

    stream << string("# VECTOR APPROACH/RETREAT ORIENTATION \n");

    stream << string("param v_t := \n");
    string tarxt0 =  boost::str(boost::format("%.2f") % (vv(0)));
    boost::replace_all(tarxt0,",",".");
    stream << to_string(1)+string(" ")+tarxt0+string("\n");

    string tarxt1 =  boost::str(boost::format("%.2f") % (vv(1)));
    boost::replace_all(tarxt1,",",".");
    stream << to_string(2)+string(" ")+tarxt1+string("\n");

    string tarxt2 =  boost::str(boost::format("%.2f") % (vv(2)));
    boost::replace_all(tarxt2,",",".");
    stream << to_string(3)+string(" ")+tarxt2+string(";\n");

}


void HUMPlanner::writeInfoObstacles(ofstream &stream, std::vector<objectPtr> &obstacles)
{
    // obstacles
    stream << string("# OBSTACLES POSITION+RADIUS+ORIENTATION \n");
    stream << string("param Obstacles : 1 2 3 4 5 6 7 8 9 := \n");

    for (std::size_t i = 0; i < obstacles.size(); ++i){

        objectPtr obs = obstacles.at(i);
        std::vector<double> position; obs->getPos(position);
        std::vector<double> orientation; obs->getOr(orientation);
        std::vector<double> dimension; obs->getSize(dimension);

        string obsx =  boost::str(boost::format("%.2f") % (position.at(0))); boost::replace_all(obsx,",",".");
        string obsy =  boost::str(boost::format("%.2f") % (position.at(1))); boost::replace_all(obsy,",",".");
        string obsz =  boost::str(boost::format("%.2f") % (position.at(2))); boost::replace_all(obsz,",",".");
        string obsxsize =  boost::str(boost::format("%.2f") % (dimension.at(0)/2)); boost::replace_all(obsxsize,",",".");
        string obsysize =  boost::str(boost::format("%.2f") % (dimension.at(1)/2)); boost::replace_all(obsysize,",",".");
        string obszsize =  boost::str(boost::format("%.2f") % (dimension.at(2)/2)); boost::replace_all(obszsize,",",".");
        string obsroll =  boost::str(boost::format("%.2f") % (orientation.at(0))); boost::replace_all(obsroll,",",".");
        string obspitch =  boost::str(boost::format("%.2f") % (orientation.at(1))); boost::replace_all(obspitch,",",".");
        string obsyaw =  boost::str(boost::format("%.2f") % (orientation.at(2))); boost::replace_all(obsyaw,",",".");

        stream << to_string(i+1)+string(" ")+
                           obsx+string(" ")+
                           obsy+string(" ")+
                           obsz+string(" ")+
                           obsxsize+string(" ")+
                           obsysize+string(" ")+
                           obszsize+string(" ")+
                           obsroll+string(" ")+
                           obspitch+string(" ")+
                           obsyaw+string(" ")+
                           string(" #")+obs->getName()+
                           string("\n");



        if (i == obstacles.size()-1){ stream << string("  ;\n");}

    }
    if (obstacles.empty()){
         stream << string(" param n_Obstacles := ")+to_string(0)+string(";\n");
    }else{
        stream << string(" param n_Obstacles := ")+to_string(obstacles.size())+string(";\n");
    }

}

void HUMPlanner::writeInfoObjectTarget(ofstream &stream, objectPtr obj)
{
    std::vector<double> position; obj->getPos(position);
    std::vector<double> orientation; obj->getOr(orientation);
    std::vector<double> dimension; obj->getSize(dimension);

    stream << string("# OBJECT OF THE TARGET POSITION+RADIUS+ORIENTATION \n");
    stream << string("param ObjTar : 1 2 3 4 5 6 7 8 9 := \n");

    string objx =  boost::str(boost::format("%.2f") % (position.at(0)));
    boost::replace_all(objx,",",".");
    string objy =  boost::str(boost::format("%.2f") % (position.at(1)));
    boost::replace_all(objy,",",".");
    string objz =  boost::str(boost::format("%.2f") % (position.at(2)));
    boost::replace_all(objz,",",".");
    string objxsize =  boost::str(boost::format("%.2f") % (dimension.at(0)/2));
    boost::replace_all(objxsize,",",".");
    string objysize =  boost::str(boost::format("%.2f") % (dimension.at(1)/2));
    boost::replace_all(objysize,",",".");
    string objzsize =  boost::str(boost::format("%.2f") % (dimension.at(2)/2));
    boost::replace_all(objzsize,",",".");
    string objroll =  boost::str(boost::format("%.2f") % (orientation.at(0)));
    boost::replace_all(objroll,",",".");
    string objpitch =  boost::str(boost::format("%.2f") % (orientation.at(1)));
    boost::replace_all(objpitch,",",".");
    string objyaw =  boost::str(boost::format("%.2f") % (orientation.at(2)));
    boost::replace_all(objyaw,",",".");

    stream << to_string(1)+string(" ")+
                       objx+string(" ")+
                       objy+string(" ")+
                       objz+string(" ")+
                       objxsize+string(" ")+
                       objysize+string(" ")+
                       objzsize+string(" ")+
                       objroll+string(" ")+
                       objpitch+string(" ")+
                       objyaw+string(" ")+
                       string(" #")+obj->getName()+
                       string("\n");
    stream << string("  ;\n");

    stream << string(" param n_ObjTar := ")+to_string(1)+string(";\n");
}

void HUMPlanner::writePI(ofstream &stream)
{
    stream << string("param pi := 4*atan(1); \n");
}

void HUMPlanner::writeBodyDimMod(ofstream &stream)
{
    stream << string("# Body info \n");
    stream << string("param body {i in 1..2}; \n");
}

void HUMPlanner::writeArmDHParamsMod(ofstream &stream)
{
    stream << string("# D-H parameters of the arm \n");
    stream << string("param alpha {i in 1..")+to_string(joints_arm)+string("} ; \n");
    stream << string("param a {i in 1..")+to_string(joints_arm)+string("} ; \n");
    stream << string("param d {i in 1..")+to_string(joints_arm)+string("} ; \n");
}

void HUMPlanner::write_dHOMod(ofstream &stream)
{
    stream << string("# Distance hand - target  \n");
    stream << string("param dFH; \n");
}

void HUMPlanner::writeInfoObjectsMod(ofstream &stream,bool vec)
{

    stream << string("# Target Position \n");
    stream << string("param Tar_pos {i in 1..3}; \n");
    stream << string("# Target orientation \n");
    stream << string("param x_t {i in 1..3}; \n");
    stream << string("param y_t {i in 1..3}; \n");
    stream << string("param z_t {i in 1..3}; \n");

    if(vec){
        stream << string("# Vector approach/retreat distance \n");
        stream << string("param dist; \n");
        stream << string("# Vector approach/retreat orientation \n");
        stream << string("param v_t {i in 1..3}; \n");
    }

    stream << string("# Obstacles \n");
    stream << string("param n_Obstacles; \n");
    stream << string("param Obstacles {i in 1..n_Obstacles, j in 1..9}; \n");

    stream << string("# Object of the target \n");
    stream << string("param n_ObjTar; \n");
    stream << string("param ObjTar {i in 1..n_ObjTar, j in 1..9}; \n");
}

void HUMPlanner::writeRotMatObsts(ofstream &stream)
{
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("# Rotation matrix of the obstacles \n");
    stream << string("param c_roll {i in 1..n_Obstacles} := cos(Obstacles[i,7]); \n");
    stream << string("param s_roll {i in 1..n_Obstacles} := sin(Obstacles[i,7]); \n");
    stream << string("param c_pitch {i in 1..n_Obstacles} := cos(Obstacles[i,8]); \n");
    stream << string("param s_pitch {i in 1..n_Obstacles} := sin(Obstacles[i,8]); \n");
    stream << string("param c_yaw {i in 1..n_Obstacles} := cos(Obstacles[i,9]); \n");
    stream << string("param s_yaw {i in 1..n_Obstacles} := sin(Obstacles[i,9]); \n");

    stream << string("param Rot {i1 in 1..4, i2 in 1..4,i in 1..n_Obstacles} :=  \n");
    stream << string("# 1st row \n");
    stream << string("if 		   ( i1=1 && i2=1 ) then 	c_roll[i]*c_pitch[i] \n");
    stream << string("else	if ( i1=1 && i2=2 ) then   -s_roll[i]*c_yaw[i]+c_roll[i]*s_pitch[i]*s_yaw[i] \n");
    stream << string("else	if ( i1=1 && i2=3 ) then 	s_roll[i]*s_yaw[i]+c_roll[i]*s_pitch[i]*c_yaw[i] \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then 	s_roll[i]*c_pitch[i] \n");
    stream << string("else	if ( i1=2 && i2=2 ) then 	c_roll[i]*c_yaw[i]+s_roll[i]*s_pitch[i]*s_yaw[i] \n");
    stream << string("else	if ( i1=2 && i2=3 ) then   -c_roll[i]*s_yaw[i]+s_roll[i]*s_pitch[i]*c_yaw[i] \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then   -s_pitch[i] \n");
    stream << string("else	if ( i1=3 && i2=2 ) then	c_pitch[i]*s_yaw[i] \n");
    stream << string("else	if ( i1=3 && i2=3 ) then	c_pitch[i]*c_yaw[i] \n");
    stream << string("   ; \n");
}

void HUMPlanner::writeArmDirKin(ofstream &stream, Matrix4d &matWorldToArm, Matrix4d &matHand, std::vector<double> &tolsArm, bool final)
{
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  Direct Kinematics model of the arm \n\n");

    stream << string("param c_alpha {i in 1..7} := cos(alpha[i]); \n");
    stream << string("param s_alpha {i in 1..7} := sin(alpha[i]); \n");
    stream << string("\n");

    string mat00 =  boost::str(boost::format("%.2f") % (matWorldToArm(0,0))); boost::replace_all(mat00,",",".");
    string mat01 =  boost::str(boost::format("%.2f") % (matWorldToArm(0,1))); boost::replace_all(mat01,",",".");
    string mat02 =  boost::str(boost::format("%.2f") % (matWorldToArm(0,2))); boost::replace_all(mat02,",",".");
    string mat03 =  boost::str(boost::format("%.2f") % (matWorldToArm(0,3))); boost::replace_all(mat03,",",".");
    string mat10 =  boost::str(boost::format("%.2f") % (matWorldToArm(1,0))); boost::replace_all(mat10,",",".");
    string mat11 =  boost::str(boost::format("%.2f") % (matWorldToArm(1,1))); boost::replace_all(mat11,",",".");
    string mat12 =  boost::str(boost::format("%.2f") % (matWorldToArm(1,2))); boost::replace_all(mat12,",",".");
    string mat13 =  boost::str(boost::format("%.2f") % (matWorldToArm(1,3))); boost::replace_all(mat13,",",".");
    string mat20 =  boost::str(boost::format("%.2f") % (matWorldToArm(2,0))); boost::replace_all(mat20,",",".");
    string mat21 =  boost::str(boost::format("%.2f") % (matWorldToArm(2,1))); boost::replace_all(mat21,",",".");
    string mat22 =  boost::str(boost::format("%.2f") % (matWorldToArm(2,2))); boost::replace_all(mat22,",",".");
    string mat23 =  boost::str(boost::format("%.2f") % (matWorldToArm(2,3))); boost::replace_all(mat23,",",".");
    string mat30 =  boost::str(boost::format("%.2f") % (matWorldToArm(3,0))); boost::replace_all(mat30,",",".");
    string mat31 =  boost::str(boost::format("%.2f") % (matWorldToArm(3,1))); boost::replace_all(mat31,",",".");
    string mat32 =  boost::str(boost::format("%.2f") % (matWorldToArm(3,2))); boost::replace_all(mat32,",",".");
    string mat33 =  boost::str(boost::format("%.2f") % (matWorldToArm(3,3))); boost::replace_all(mat33,",",".");

    stream << string("param T_WorldToArm {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then ")+mat00+string("  \n");
    stream << string("else	if ( i1=1 && i2=2 ) then ")+mat01+string("  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then ")+mat02+string("  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then ")+mat03+string("  \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then ")+mat10+string("  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then ")+mat11+string("  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then ")+mat12+string("  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then ")+mat13+string("  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then ")+mat20+string("  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then ")+mat21+string("  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then ")+mat22+string("  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then ")+mat23+string("  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then ")+mat30+string("  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then ")+mat31+string("  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then ")+mat32+string("  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then ")+mat33+string("  \n");
    stream << string(";  \n");

    string idx;
    string idx1;
    for (unsigned i = 0 ; i < joints_arm; ++i){

        idx = to_string(i);
        idx1 = to_string(i+1);

        if (final){
            stream << string("var T_")+idx+string("_")+idx1+string(" {i1 in 1..4, i2 in 1..4} =  \n");
        }else{
            stream << string("var T_")+idx+string("_")+idx1+string(" {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
        }
        stream << string("# 1st row \n");
        if(final){
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[")+idx1+string("]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[")+idx1+string("])  \n");
        }else{
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,")+idx1+string("]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,")+idx1+string("])  \n");
        }
        stream << string("else	if ( i1=1 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a[")+idx1+string("]  \n");
        stream << string("# 2st row \n");
        if(final){
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[")+idx1+string("])*c_alpha[")+idx1+string("] \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[")+idx1+string("])*c_alpha[")+idx1+string("] \n");
        }else{
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,")+idx1+string("])*c_alpha[")+idx1+string("] \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,")+idx1+string("])*c_alpha[")+idx1+string("] \n");
        }
        stream << string("else	if ( i1=2 && i2=3 ) then -s_alpha[")+idx1+string("] \n");
        stream << string("else	if ( i1=2 && i2=4 ) then -s_alpha[")+idx1+string("]*d[")+idx1+string("] \n");
        stream << string("# 3rd row \n");
        if(final){
            stream << string("else	if ( i1=3 && i2=1 ) then sin(theta[")+idx1+string("])*s_alpha[")+idx1+string("] \n");
            stream << string("else	if ( i1=3 && i2=2 ) then cos(theta[")+idx1+string("])*s_alpha[")+idx1+string("] \n");
        }else{
            stream << string("else	if ( i1=3 && i2=1 ) then sin(theta[i,")+idx1+string("])*s_alpha[")+idx1+string("] \n");
            stream << string("else	if ( i1=3 && i2=2 ) then cos(theta[i,")+idx1+string("])*s_alpha[")+idx1+string("] \n");
        }
        stream << string("else	if ( i1=3 && i2=3 ) then c_alpha[")+idx1+string("] \n");
        stream << string("else	if ( i1=3 && i2=4 ) then c_alpha[")+idx1+string("]*d[")+idx1+string("] \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0 \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

    }

    string matHand00 =  boost::str(boost::format("%.2f") % (matHand(0,0))); boost::replace_all(matHand00,",",".");
    string matHand01 =  boost::str(boost::format("%.2f") % (matHand(0,1))); boost::replace_all(matHand01,",",".");
    string matHand02 =  boost::str(boost::format("%.2f") % (matHand(0,2))); boost::replace_all(matHand02,",",".");
    string matHand03 =  boost::str(boost::format("%.2f") % (matHand(0,3))); boost::replace_all(matHand03,",",".");
    string matHand10 =  boost::str(boost::format("%.2f") % (matHand(1,0))); boost::replace_all(matHand10,",",".");
    string matHand11 =  boost::str(boost::format("%.2f") % (matHand(1,1))); boost::replace_all(matHand11,",",".");
    string matHand12 =  boost::str(boost::format("%.2f") % (matHand(1,2))); boost::replace_all(matHand12,",",".");
    string matHand13 =  boost::str(boost::format("%.2f") % (matHand(1,3))); boost::replace_all(matHand13,",",".");
    string matHand20 =  boost::str(boost::format("%.2f") % (matHand(2,0))); boost::replace_all(matHand20,",",".");
    string matHand21 =  boost::str(boost::format("%.2f") % (matHand(2,1))); boost::replace_all(matHand21,",",".");
    string matHand22 =  boost::str(boost::format("%.2f") % (matHand(2,2))); boost::replace_all(matHand22,",",".");
    string matHand23 =  boost::str(boost::format("%.2f") % (matHand(2,3))); boost::replace_all(matHand23,",",".");
    string matHand30 =  boost::str(boost::format("%.2f") % (matHand(3,0))); boost::replace_all(matHand30,",",".");
    string matHand31 =  boost::str(boost::format("%.2f") % (matHand(3,1))); boost::replace_all(matHand31,",",".");
    string matHand32 =  boost::str(boost::format("%.2f") % (matHand(3,2))); boost::replace_all(matHand32,",",".");
    string matHand33 =  boost::str(boost::format("%.2f") % (matHand(3,3))); boost::replace_all(matHand33,",",".");

    stream << string("param T_")+idx1+string("_H {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then ")+matHand00+string("  \n");
    stream << string("else	if ( i1=1 && i2=2 ) then ")+matHand01+string("  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then ")+matHand02+string("  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then ")+matHand03+string("  \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then ")+matHand10+string("  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then ")+matHand11+string("  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then ")+matHand12+string("  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then ")+matHand13+string("  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then ")+matHand20+string("  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then ")+matHand21+string("  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then ")+matHand22+string("  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then ")+matHand23+string("  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then ")+matHand30+string("  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then ")+matHand31+string("  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then ")+matHand32+string("  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then ")+matHand33+string("  \n");
    stream << string(";  \n");

    // positions on the arm
    if(final){
        stream << string("var T_W_1 {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_WorldToArm[i1,j]*T_0_1[j,i2];\n");
    }else{
        stream << string("var T_W_1 {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_WorldToArm[i1,j]*T_0_1[j,i2,i];\n");
    }
    for (unsigned i = 1 ; i < joints_arm; ++i){
        idx = to_string(i);
        idx1 = to_string(i+1);
        if(final){
            stream << string("var T_W_")+idx1+string(" {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx+string("[i1,j]*T_")+idx+string("_")+idx1+string("[j,i2];\n");
        }else{
            stream << string("var T_W_")+idx1+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_W_")+idx+string("[i1,j,i]*T_")+idx+string("_")+idx1+string("[j,i2,i];\n");
        }
    }
    if(final){
        stream << string("var T_W_H {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx1+string("[i1,j]*T_")+idx1+string("_H[j,i2];\n\n");
    }else{
        stream << string("var T_W_H {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_W_")+idx1+string("[i1,j,i]*T_")+idx1+string("_H[j,i2];\n\n");
    }

    string tolArm1 =  boost::str(boost::format("%.2f") % tolsArm.at(0)); boost::replace_all(tolArm1,",",".");
    string tolArm2 =  boost::str(boost::format("%.2f") % tolsArm.at(1)); boost::replace_all(tolArm2,",",".");
    string tolArm3 =  boost::str(boost::format("%.2f") % tolsArm.at(2)); boost::replace_all(tolArm3,",",".");
    string tolArm4 =  boost::str(boost::format("%.2f") % tolsArm.at(3)); boost::replace_all(tolArm4,",",".");

    if(final){
        stream << string("var Shoulder {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_1[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm1+string("\n");
        stream << string(";  \n");

        stream << string("var Elbow {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_3[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm2+string("\n");
        stream << string(";  \n");

        stream << string("var Wrist {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_5[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm3+string("\n");
        stream << string(";  \n");

        stream << string("var Hand {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_H[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm4+string("\n");
        stream << string(";  \n");

        stream << string("# Hand orientation \n");
        stream << string("var x_H {j in 1..3} = T_W_H [j,1]; \n");
        stream << string("var y_H {j in 1..3} = T_W_H [j,2]; \n");
        stream << string("var z_H {j in 1..3} = T_W_H [j,3]; \n");

    }else{
        stream << string("var Shoulder {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_1[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm1+string("\n");
        stream << string(";  \n");

        stream << string("var Elbow {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_3[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm2+string("\n");
        stream << string(";  \n");

        stream << string("var Wrist {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_5[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm3+string("\n");
        stream << string(";  \n");

        stream << string("var Hand {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_H[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm4+string("\n");
        stream << string(";  \n");

        stream << string("# Hand orientation \n");
        stream << string("var x_H {j in 1..3,i in Iterations} = T_W_H [j,1,i]; \n");
        stream << string("var y_H {j in 1..3,i in Iterations} = T_W_H [j,2,i]; \n");
        stream << string("var z_H {j in 1..3,i in Iterations} = T_W_H [j,3,i]; \n");
    }
}

void HUMPlanner::writeHumanHandDirKin(ofstream &stream, MatrixXd &tolsHand, bool final, bool transport)
{
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  Direct Kinematics model of the fingers \n\n");

    string tolHand1;
    string tolHand2;
    string tolHand3;
    string tolHand4;

    stream << string("# Index finger \n\n");

    stream << string("param TF1_H_0 {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then cos(theta0_fing1) \n");
    stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_fing1)*cos(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_fing1)*sin(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then u1x \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_fing1)  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_fing1)*cos(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_fing1)*sin(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then u1y  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1[1])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then u1z  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");


    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF1_0_1 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[2]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[2])*cos(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[2])*sin(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[2]*cos(joint_fingers[2]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[2])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[2])*cos(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[2])*sin(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[2]*sin(joint_fingers[2])  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF1_0_1 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9])*cos(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9])*sin(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[2]*cos(theta[i,9]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9])*cos(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9])*sin(alpha_fing1[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[2]*sin(theta[i,9])  \n");

    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1[2])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1[2])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing1[2]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");


    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF1_1_2 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(2*joint_fingers[2]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*joint_fingers[2]/3)*cos(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(2*joint_fingers[2]/3)*sin(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[3]*cos(2*joint_fingers[2]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(2*joint_fingers[2]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(2*joint_fingers[2]/3)*cos(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*joint_fingers[2]/3)*sin(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[3]*sin(2*joint_fingers[2]/3)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF1_1_2 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(2*theta[i,9]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*theta[i,9]/3)*cos(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(2*theta[i,9]/3)*sin(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[3]*cos(2*theta[i,9]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(2*theta[i,9]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(2*theta[i,9]/3)*cos(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*theta[i,9]/3)*sin(alpha_fing1[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[3]*sin(2*theta[i,9]/3)  \n");

    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1[3])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1[3])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing1[3]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF1_2_3 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[2]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[2]/3)*cos(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[2]/3)*sin(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[4]*cos(joint_fingers[2]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[2]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[2]/3)*cos(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[2]/3)*sin(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[4]*sin(joint_fingers[2]/3)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF1_2_3 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9]/3)*cos(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9]/3)*sin(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing1[4]*cos(theta[i,9]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9]/3)*cos(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9]/3)*sin(alpha_fing1[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[4]*sin(theta[i,9]/3)  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1[4])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1[4])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing1[4]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");


    // position of the fingers
    tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,0)); boost::replace_all(tolHand1,",",".");
    tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,0)); boost::replace_all(tolHand2,",",".");
    tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,0)); boost::replace_all(tolHand3,",",".");
    tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,0)); boost::replace_all(tolHand4,",",".");

    if(final){
        // final posture selection
        stream << string("var F1_0   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H[i1,j]*TF1_H_0[j,i2]; \n");
        stream << string("var F1_1   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_0[i1,j]*TF1_0_1[j,i2]; \n");
        stream << string("var F1_2   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_1[i1,j]*TF1_1_2[j,i2]; \n");
        stream << string("var F1_tip {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_2[i1,j]*TF1_2_3[j,i2]; \n\n");

        stream << string("var Finger1_0   {i1 in 1..4} =  if i1<4 then F1_0[i1,4] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger1_1   {i1 in 1..4} =  if i1<4 then F1_1[i1,4] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger1_2   {i1 in 1..4} =  if i1<4 then F1_2[i1,4] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger1_tip {i1 in 1..4} =  if i1<4 then F1_tip[i1,4] else ")+tolHand4+string("; \n\n");
    }else{
        //bounce posture selection
        stream << string("var F1_0   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H[i1,j,i]*TF1_H_0[j,i2]; \n");
        if(transport){
            stream << string("var F1_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_0[i1,j,i]*TF1_0_1[j,i2]; \n");
            stream << string("var F1_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_1[i1,j,i]*TF1_1_2[j,i2]; \n");
            stream << string("var F1_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_2[i1,j,i]*TF1_2_3[j,i2]; \n\n");
        }else{
            stream << string("var F1_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_0[i1,j,i]*TF1_0_1[j,i2,i]; \n");
            stream << string("var F1_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_1[i1,j,i]*TF1_1_2[j,i2,i]; \n");
            stream << string("var F1_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_2[i1,j,i]*TF1_2_3[j,i2,i]; \n\n");
        }
        stream << string("var Finger1_0   {i1 in 1..4,i in Iterations} =  if i1<4 then F1_0[i1,4,i] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger1_1   {i1 in 1..4,i in Iterations} =  if i1<4 then F1_1[i1,4,i] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger1_2   {i1 in 1..4,i in Iterations} =  if i1<4 then F1_2[i1,4,i] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger1_tip {i1 in 1..4,i in Iterations} =  if i1<4 then F1_tip[i1,4,i] else ")+tolHand4+string("; \n\n");

    }

    stream << string("# Ring finger \n\n");

    stream << string("param TF2_H_0 {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then cos(theta0_fing3) \n");
    stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_fing3)*cos(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_fing3)*sin(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then u3x \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_fing3)  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_fing3)*cos(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_fing3)*sin(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then u3y  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3[1])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then u3z  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");


    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF2_0_1 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[3]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[3])*cos(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[3])*sin(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[2]*cos(joint_fingers[3]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[3])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[3])*cos(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[3])*sin(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing3[2]*sin(joint_fingers[3])  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF2_0_1 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9])*cos(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9])*sin(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[2]*cos(theta[i,9]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9])*cos(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9])*sin(alpha_fing3[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing3[2]*sin(theta[i,9])  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3[2])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3[2])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing3[2]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF2_1_2 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(2*joint_fingers[3]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*joint_fingers[3]/3)*cos(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(2*joint_fingers[3]/3)*sin(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[3]*cos(2*joint_fingers[3]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(2*joint_fingers[3]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(2*joint_fingers[3]/3)*cos(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*joint_fingers[3]/3)*sin(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[3]*sin(2*joint_fingers[3]/3)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF2_1_2 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(2*theta[i,9]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*theta[i,9]/3)*cos(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(2*theta[i,9]/3)*sin(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[3]*cos(2*theta[i,9]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(2*theta[i,9]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(2*theta[i,9]/3)*cos(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*theta[i,9]/3)*sin(alpha_fing3[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing1[3]*sin(2*theta[i,9]/3)  \n");

    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3[3])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3[3])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing3[3]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF2_2_3 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[3]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[3]/3)*cos(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[3]/3)*sin(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[4]*cos(joint_fingers[3]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[3]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[3]/3)*cos(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[3]/3)*sin(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing3[4]*sin(joint_fingers[3]/3)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF2_2_3 {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]/3) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9]/3)*cos(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9]/3)*sin(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_fing3[4]*cos(theta[i,9]/3) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9]/3)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9]/3)*cos(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9]/3)*sin(alpha_fing3[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_fing3[4]*sin(theta[i,9]/3)  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3[4])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3[4])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_fing3[4]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    // position of the fingers
    tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,1)); boost::replace_all(tolHand1,",",".");
    tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,1)); boost::replace_all(tolHand2,",",".");
    tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,1)); boost::replace_all(tolHand3,",",".");
    tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,1)); boost::replace_all(tolHand4,",",".");

    if(final){
        // final posture selection
        stream << string("var F2_0   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H[i1,j]*TF2_H_0[j,i2]; \n");
        stream << string("var F2_1   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F2_0[i1,j]*TF2_0_1[j,i2]; \n");
        stream << string("var F2_2   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F2_1[i1,j]*TF2_1_2[j,i2]; \n");
        stream << string("var F2_tip {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F2_2[i1,j]*TF2_2_3[j,i2]; \n\n");

        stream << string("var Finger2_0   {i1 in 1..4} =  if i1<4 then F2_0[i1,4] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger2_1   {i1 in 1..4} =  if i1<4 then F2_1[i1,4] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger2_2   {i1 in 1..4} =  if i1<4 then F2_2[i1,4] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger2_tip {i1 in 1..4} =  if i1<4 then F2_tip[i1,4] else ")+tolHand4+string("; \n\n");
    }else{
        //bounce posture selection
        stream << string("var F2_0   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H[i1,j,i]*TF2_H_0[j,i2]; \n");
        if (transport){
            stream << string("var F2_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_0[i1,j,i]*TF2_0_1[j,i2]; \n");
            stream << string("var F2_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_1[i1,j,i]*TF2_1_2[j,i2]; \n");
            stream << string("var F2_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_2[i1,j,i]*TF2_2_3[j,i2]; \n\n");
        }else{
            stream << string("var F2_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_0[i1,j,i]*TF2_0_1[j,i2,i]; \n");
            stream << string("var F2_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_1[i1,j,i]*TF2_1_2[j,i2,i]; \n");
            stream << string("var F2_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_2[i1,j,i]*TF2_2_3[j,i2,i]; \n\n");
        }
        stream << string("var Finger2_0   {i1 in 1..4,i in Iterations} =  if i1<4 then F2_0[i1,4,i] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger2_1   {i1 in 1..4,i in Iterations} =  if i1<4 then F2_1[i1,4,i] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger2_2   {i1 in 1..4,i in Iterations} =  if i1<4 then F2_2[i1,4,i] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger2_tip {i1 in 1..4,i in Iterations} =  if i1<4 then F2_tip[i1,4,i] else ")+tolHand4+string("; \n\n");

    }

    stream << string("# Thumb finger \n\n");

    stream << string("param TF3_H_0 {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then cos(theta0_thumb) \n");
    stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_thumb)*cos(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_thumb)*sin(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then uTx \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_thumb)  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_thumb)*cos(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_thumb)*sin(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then uTy  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb[1])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then uTz  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF3_0_1 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[1]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[1])*cos(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[1])*sin(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[2]*cos(joint_fingers[1]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[1])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[1])*cos(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[1])*sin(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[2]*sin(joint_fingers[1])  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF3_0_1 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,8]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,8])*cos(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,8])*sin(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[2]*cos(theta[i,8]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,8])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,8])*cos(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,8])*sin(alpha_thumb[2])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[2]*sin(theta[i,8])  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb[2])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb[2])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_thumb[2]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF3_1_2 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers[4]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers[4])*cos(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers[4])*sin(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[3]*cos(joint_fingers[4]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers[4])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers[4])*cos(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers[4])*sin(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[3]*sin(joint_fingers[4])  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF3_1_2 {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,10]) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,10])*cos(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,10])*sin(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[3]*cos(theta[i,10]) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,10])  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,10])*cos(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,10])*sin(alpha_thumb[3])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[3]*sin(theta[i,10])  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb[3])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb[3])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_thumb[3]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF3_2_3 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(11*joint_fingers[4]/10) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*joint_fingers[4]/10)*cos(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(11*joint_fingers[4]/10)*sin(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[4]*cos(11*joint_fingers[4]/10) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(11*joint_fingers[4]/10)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(11*joint_fingers[4]/10)*cos(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*joint_fingers[4]/10)*sin(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[4]*sin(11*joint_fingers[4]/10)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF3_2_3 {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(11*theta[i,10]/10) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*theta[i,10]/10)*cos(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(11*theta[i,10]/10)*sin(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[4]*cos(11*theta[i,10]/10) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(11*theta[i,10]/10)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(11*theta[i,10]/10)*cos(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*theta[i,10]/10)*sin(alpha_thumb[4])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[4]*sin(11*theta[i,10]/10)  \n");

    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb[4])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb[4])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_thumb[4]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    if (final || transport){
        // final posture selection or bounce posture selection for transporting movements
        stream << string("param TF3_3_4 {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(11*joint_fingers[4]/12) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*joint_fingers[4]/12)*cos(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(11*joint_fingers[4]/12)*sin(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[5]*cos(11*joint_fingers[4]/12) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(11*joint_fingers[4]/12)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(11*joint_fingers[4]/12)*cos(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*joint_fingers[4]/12)*sin(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[5]*sin(11*joint_fingers[4]/12)  \n");
    }else{
        // bounce posture selection for reaching movements
        stream << string("var TF3_3_4 {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(11*theta[i,10]/12) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*theta[i,10]/12)*cos(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(11*theta[i,10]/12)*sin(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_thumb[5]*cos(11*theta[i,10]/12) \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(11*theta[i,10]/12)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(11*theta[i,10]/12)*cos(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*theta[i,10]/12)*sin(alpha_thumb[5])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then a_thumb[5]*sin(11*theta[i,10]/12)  \n");
    }
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb[5])  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb[5])  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then d_thumb[5]  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
    stream << string(";  \n");

    // position of the fingers
    tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,2)); boost::replace_all(tolHand1,",",".");
    tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,2)); boost::replace_all(tolHand2,",",".");
    tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,2)); boost::replace_all(tolHand3,",",".");
    tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,2)); boost::replace_all(tolHand4,",",".");

    if(final){
        // final posture selection
        stream << string("var F3_0   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H[i1,j]*TF3_H_0[j,i2]; \n");
        stream << string("var F3_1   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_0[i1,j]*TF3_0_1[j,i2]; \n");
        stream << string("var F3_2   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_1[i1,j]*TF3_1_2[j,i2]; \n");
        stream << string("var F3_3   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_2[i1,j]*TF3_2_3[j,i2]; \n");
        stream << string("var F3_tip {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_3[i1,j]*TF3_3_4[j,i2]; \n\n");

        //PostureMod << string("var Finger3_0   {i1 in 1..4} =  if i1<4 then F3_0[i1,4] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger3_1   {i1 in 1..4} =  if i1<4 then F3_1[i1,4] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger3_2   {i1 in 1..4} =  if i1<4 then F3_2[i1,4] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger3_3   {i1 in 1..4} =  if i1<4 then F3_3[i1,4] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger3_tip {i1 in 1..4} =  if i1<4 then F3_tip[i1,4] else ")+tolHand4+string("; \n\n");

    }else{
        //bounce posture selection
        stream << string("var F3_0   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4} T_W_H[i1,j,i]*TF3_H_0[j,i2]; \n");
        if(transport){
            stream << string("var F3_1   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_0[i1,j,i]*TF3_0_1[j,i2]; \n");
            stream << string("var F3_2   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_1[i1,j,i]*TF3_1_2[j,i2]; \n");
            stream << string("var F3_3   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_2[i1,j,i]*TF3_2_3[j,i2]; \n");
            stream << string("var F3_tip {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_3[i1,j,i]*TF3_3_4[j,i2]; \n\n");
        }else{
            stream << string("var F3_1   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_0[i1,j,i]*TF3_0_1[j,i2,i]; \n");
            stream << string("var F3_2   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_1[i1,j,i]*TF3_1_2[j,i2,i]; \n");
            stream << string("var F3_3   {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_2[i1,j,i]*TF3_2_3[j,i2,i]; \n");
            stream << string("var F3_tip {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_3[i1,j,i]*TF3_3_4[j,i2,i]; \n\n");
        }
        stream << string("var Finger3_1   {i1 in 1..4, i in Iterations} =  if i1<4 then F3_1[i1,4,i] 	else ")+tolHand1+string("; \n");
        stream << string("var Finger3_2   {i1 in 1..4, i in Iterations} =  if i1<4 then F3_2[i1,4,i] 	else ")+tolHand2+string("; \n");
        stream << string("var Finger3_3   {i1 in 1..4, i in Iterations} =  if i1<4 then F3_3[i1,4,i] 	else ")+tolHand3+string("; \n");
        stream << string("var Finger3_tip {i1 in 1..4, i in Iterations} =  if i1<4 then F3_tip[i1,4,i] else ")+tolHand4+string("; \n\n");

    }

}

void HUMPlanner::writeBarrettHandDirKin(ofstream &stream, MatrixXd &tolsHand, bool final, bool transport)
{
    std::vector<int> rk; std::vector<int> jk;
    rk = this->bhand.rk; jk = this->bhand.jk;
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  Direct Kinematics model of the fingers \n\n");

    if(final){
        // final posture selection
        for (unsigned i = 0 ; i < hand_fingers; ++i){
            //for (int j = 0; j <N_PHALANGE; ++j){

            string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
            string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                 stream << string("# Finger ")+to_string(i+1)+string(" \n\n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("{i1 in 1..4, i2 in 1..4} :=   \n");
                 stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*joint_fingers[")+to_string(1)+string("]-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*joint_fingers[")+to_string(1)+string("]-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*joint_fingers[")+to_string(1)+string("]-(pi/2)*(")+jkk+string("))  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw 	 				 \n");
                 stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(2)+string("{i1 in 1..4, i2 in 1..4} :=   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=3&&i2=1)					then    sin(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                 stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                 stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A1 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(3)+string("{i1 in 1..4, i2 in 1..4} :=   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then    sin((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                 stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 														1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A2 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("{i1 in 1..4, i2 in 1..4} :=   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                 stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                 stream << string("else 	if (i1=1&&i2=4)					              then    A3 \n");
                 stream << string("else 	if (i1=2&&i2=4)					              then    D3 \n");
                 stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string(";\n\n");


                 // position of the fingers
                 stream << string("var F")+to_string(i+1)+string("_0   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_6[i1,j]*TF")+to_string(i+1)+string("_1[j,i2]; \n");
                 stream << string("var F")+to_string(i+1)+string("_1   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0[i1,j]*TF")+to_string(i+1)+string("_2[j,i2]; \n");
                 stream << string("var F")+to_string(i+1)+string("_2   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1[i1,j]*TF")+to_string(i+1)+string("_3[j,i2]; \n");
                 stream << string("var F")+to_string(i+1)+string("_tip {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2[i1,j]*TF")+to_string(i+1)+string("_4[j,i2]; \n\n");

                 string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                 string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                 string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                 string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                 stream << string("var Finger")+to_string(i+1)+string("_0   {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_0[i1,4] 	else ")+tolHand1+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_1   {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_1[i1,4] 	else ")+tolHand2+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_2   {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_2[i1,4] 	else ")+tolHand3+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_tip {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_tip[i1,4] else ")+tolHand4+string("; \n\n");

           // }

        }
    }else{
        // bounce poncure selection


        if (transport){
            // transport or engage movement
            for (unsigned i = 0 ; i < hand_fingers; ++i){
                //for (int j = 0; j <N_PHALANGE; ++j){
                /*
                int k;
                if (i == 2){
                    k = 8;
                }else{
                    k = 9;
                }
                */

                string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
                string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                 stream << string("# Finger ")+to_string(i+1)+string(" \n\n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string("))  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw 	 				 \n");
                 stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(2)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=3&&i2=1)					then    sin(joint_fingers[")+to_string(i+2)+string("]+phi_2)  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                 stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                 stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A1 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(3)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then    sin((joint_fingers[")+to_string(i+2)+string("])/3+phi_3) \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                 stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 														1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A2 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} :=   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                 stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                 stream << string("else 	if (i1=1&&i2=4)					              then    A3 \n");
                 stream << string("else 	if (i1=2&&i2=4)					              then    D3 \n");
                 stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string(";\n\n");


                 // position of the fingers
                 stream << string("var F")+to_string(i+1)+string("_0   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_6[i1,j,i]*TF")+to_string(i+1)+string("_1[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0[i1,j,i]*TF")+to_string(i+1)+string("_2[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1[i1,j,i]*TF")+to_string(i+1)+string("_3[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2[i1,j,i]*TF")+to_string(i+1)+string("_4[j,i2,i]; \n\n");


                 string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                 string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                 string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                 string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                 stream << string("var Finger")+to_string(i+1)+string("_0   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_0[i1,4,i] 	else ")+tolHand1+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_1   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_1[i1,4,i] 	else ")+tolHand2+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_2   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_2[i1,4,i] 	else ")+tolHand3+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_tip {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_tip[i1,4,i] else ")+tolHand4+string("; \n\n");


               // }


            }

        }else{
            // reaching movements
            for (unsigned i = 0 ; i < hand_fingers; ++i){
                //for (int j = 0; j <N_PHALANGE; ++j){
                int k;
                if (i == 2){
                    k = 8;
                }else{
                    k = 9;
                }
                string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
                string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                 stream << string("# Finger ")+to_string(i+1)+string(" \n\n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string("))  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw 	 				 \n");
                 stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                 stream << string("; \n");

                 stream << string("var TF")+to_string(i+1)+string("_")+to_string(2)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(theta[i,")+to_string(k)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin(theta[i,")+to_string(k)+string("]+phi_2)  \n");
                 stream << string("else 	if (i1=3&&i2=1)					then    sin(theta[i,")+to_string(k)+string("]+phi_2)  \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                 stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                 stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A1 \n");
                 stream << string("; \n");

                 stream << string("var TF")+to_string(i+1)+string("_")+to_string(3)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((theta[i,")+to_string(k)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=1&&i2=2)					then   -sin((theta[i,")+to_string(k)+string("])/3+phi_3) \n");
                 stream << string("else 	if (i1=2&&i2=1)					then    sin((theta[i,")+to_string(k)+string("])/3+phi_3) \n");
                 stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                 stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 														1 \n");
                 stream << string("else	if ( i1=1 && i2=4 ) then 								A2 \n");
                 stream << string("; \n");

                 stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("{i1 in 1..4, i2 in 1..4,i in Iterations} :=   \n");
                 stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                 stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                 stream << string("else 	if (i1=1&&i2=4)					              then    A3 \n");
                 stream << string("else 	if (i1=2&&i2=4)					              then    D3 \n");
                 stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                 stream << string(";\n\n");


                 // position of the fingers
                 stream << string("var F")+to_string(i+1)+string("_0   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_6[i1,j,i]*TF")+to_string(i+1)+string("_1[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_1   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0[i1,j,i]*TF")+to_string(i+1)+string("_2[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_2   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1[i1,j,i]*TF")+to_string(i+1)+string("_3[j,i2,i]; \n");
                 stream << string("var F")+to_string(i+1)+string("_tip {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2[i1,j,i]*TF")+to_string(i+1)+string("_4[j,i2,i]; \n\n");

                 string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                 string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                 string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                 string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                 stream << string("var Finger")+to_string(i+1)+string("_0   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_0[i1,4,i] 	else ")+tolHand1+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_1   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_1[i1,4,i] 	else ")+tolHand2+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_2   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_2[i1,4,i] 	else ")+tolHand3+string("; \n");
                 stream << string("var Finger")+to_string(i+1)+string("_tip {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_tip[i1,4,i] else ")+tolHand4+string("; \n\n");


               // }


            }

        }


    }

}

void HUMPlanner::writeObjective(ofstream &stream, bool final)
{
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  \n");
    stream << string("#		       Objective function          # \n");
    stream << string("#  \n");
    if(final){
        stream << string("minimize z: sum {j in nJoints} (lambda[j]*(thet_init[j]-theta[j])^2); \n");
    }else{
        stream << string("minimize z: sum {j in nJoints} (lambda[j]*(thet_init[j]-theta_b[j])^2); \n");
    }
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");

}

void HUMPlanner::writeBodyConstraints(ofstream &stream, bool final)
{
    stream << string("# Constraints with the body: the body is modeled as a cylinder \n");
    if (final){

        stream << string("subject to BodyArm_Elbow: (Elbow[1]/body[1])^2 + (Elbow[2]/body[2])^2 >= 1; \n");
        stream << string("subject to BodyArm_Wrist: (Wrist[1]/body[1])^2 + (Wrist[2]/body[2])^2 >= 1; \n");
        stream << string("subject to BodyArm_Hand:  (Hand[1]/body[1])^2  + (Hand[2]/body[2])^2  >= 1; \n\n");
    }else{
        stream << string("subject to BodyArm_Elbow{l in Iterations}: (Elbow[1,l]/body[1])^2 + (Elbow[2,l]/body[2])^2 >= 1; \n");
        stream << string("subject to BodyArm_Wrist{l in Iterations}: (Wrist[1,l]/body[1])^2 + (Wrist[2,l]/body[2])^2 >= 1; \n");
        stream << string("subject to BodyArm_Hand{l in Iterations}:  (Hand[1,l]/body[1])^2  + (Hand[2,l]/body[2])^2  >= 1; \n\n");

    }
}

void HUMPlanner::RPY_matrix(std::vector<double> rpy, Matrix3d &Rot)
{
    Rot = Matrix3d::Zero();

    if(!rpy.empty()){
        double roll = rpy.at(0); // around z
        double pitch = rpy.at(1); // around y
        double yaw = rpy.at(2); // around x

        // Rot = Rot_z * Rot_y * Rot_x

        Rot(0,0) = cos(roll)*cos(pitch);  Rot(0,1) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw); Rot(0,2) = sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
        Rot(1,0) = sin(roll)*cos(pitch);  Rot(1,1) = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw); Rot(1,2) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
        Rot(2,0) = -sin(pitch);           Rot(2,1) = cos(pitch)*sin(yaw);                              Rot(2,2) = cos(pitch)*cos(yaw);

    }
}

void HUMPlanner::getRotAxis(vector<double> &xt, int id, std::vector<double> rpy)
{
    Matrix3d Rot;
    this->RPY_matrix(rpy,Rot);
    Vector3d v = Rot.col(id);

    // get the components of the axis
    xt.push_back(v(0)); // x
    xt.push_back(v(1)); // y
    xt.push_back(v(2)); // z
}

void HUMPlanner::Trans_matrix(std::vector<double> xyz, std::vector<double> rpy, Matrix4d &Trans)
{
    Trans = Matrix4d::Zero();

    Matrix3d Rot;
    this->RPY_matrix(rpy,Rot);

    Trans(0,0) = Rot(0,0); Trans(0,1) = Rot(0,1); Trans(0,2) = Rot(0,2); Trans(0,3) = xyz.at(0);
    Trans(1,0) = Rot(1,0); Trans(1,1) = Rot(1,1); Trans(1,2) = Rot(1,2); Trans(1,3) = xyz.at(1);
    Trans(2,0) = Rot(2,0); Trans(2,1) = Rot(2,1); Trans(2,2) = Rot(2,2); Trans(2,3) = xyz.at(2);
    Trans(3,0) = 0;        Trans(3,1) = 0;        Trans(3,2) = 0;        Trans(3,3) = 1;

}

bool HUMPlanner::writeFilesFinalPosture(huml_params& params,int mov_type, int pre_post, std::vector<double> initArmPosture, std::vector<double> initialGuess,std::vector<objectPtr> obsts)
{
    //  --- create the "Models" directory if it does not exist ---
    struct stat st = {0};
    if (stat("Models", &st) == -1) {
        mkdir("Models", 0700);
    }
    string path("Models/");

    // movement settings
    int arm_code = params.mov_specs.arm_code;
    int hand_code = params.mov_specs.hand_code;
    int griptype = params.mov_specs.griptype;
    std::vector<double> tar = params.mov_specs.target;
    int dHO = params.mov_specs.dHO;
    std::vector<double> finalHand = params.mov_specs.finalHand;
    std::string mov_infoLine = params.mov_specs.mov_infoline;
    objectPtr obj_tar;
    bool approach = params.mov_specs.approach;
    bool retreat = params.mov_specs.retreat;
    std::vector<double> pre_grasp_approach;
    std::vector<double> post_grasp_retreat;
    std::vector<double> pre_place_approach;
    std::vector<double> post_place_retreat;
    switch(mov_type){
    case 0: // pick
        obj_tar = params.mov_specs.obj;
        if(approach){pre_grasp_approach = params.mov_specs.pre_grasp_approach;}
        if(retreat){post_grasp_retreat = params.mov_specs.post_grasp_retreat;}
        break;

    }


    // tolerances
    std::vector<double> lambda(params.lambda_final.begin(),params.lambda_final.begin()+joints_arm);
    std::vector<double> tolsArm = params.tolsArm;
    MatrixXd tolsHand = params.tolsHand;
    MatrixXd tolsObstacles = params.final_tolsObstacles;
    double tolTarPos = params.tolTarPos;
    double tolTarOr = params.tolTarOr;
    bool obstacle_avoidance = params.obstacle_avoidance;

    Matrix4d matWorldToArm;
    Matrix4d matHand;
    std::vector<double> minLimits;
    std::vector<double> maxLimits;
    DHparameters dh_arm;

    int k;
    switch(arm_code){
    case 0: // dual arm
        //TO DO
        break;
    case 1: // right arm
        k=1;
        matWorldToArm = this->matWorldToRightArm;
        matHand = this->matRightHand;
        minLimits = this->minRightLimits;
        maxLimits = this->maxRightLimits;
        dh_arm = this->DH_rightArm;
        break;
    case 2: // left arm
        k=-1;
        matWorldToArm = this->matWorldToLeftArm;
        matHand = this->matLeftHand;
        minLimits = this->minLeftLimits;
        maxLimits = this->maxLeftLimits;
        dh_arm = this->DH_leftArm;
        break;
    }
    std::vector<double> minArmLimits(minLimits.begin(),minLimits.begin()+joints_arm);
    std::vector<double> maxArmLimits(maxLimits.begin(),maxLimits.begin()+joints_arm);

   //------------------------- Write the dat file --------------------------------------------------
    string filename("FinalPosture.dat");
    ofstream PostureDat;
    // open the file
    PostureDat.open(path+filename);

    PostureDat << string("# FINAL POSTURE DATA FILE \n");
    PostureDat << string("# Units of measure: [rad], [mm] \n\n");

    PostureDat << string("data; \n");

    //PostureDat << string("param pi := 4*atan(1); \n");

    // Body dimension
    this->writeBodyDim(this->torso_size.at(0),this->torso_size.at(1),PostureDat);
    // D-H Parameters of the Arm
    this->writeArmDHParams(dh_arm,PostureDat,k);
    // distance between the hand and the object
    this->write_dHO(PostureDat,dHO);
    // joint limits
    this->writeArmLimits(PostureDat,minArmLimits,maxArmLimits);
    // initial pose of the arm
    this->writeArmInitPose(PostureDat,initArmPosture);
    // final posture of the fingers
    this->writeFingerFinalPose(PostureDat,finalHand);
    // joint expense factors of the arm
    this->writeLambda(PostureDat,lambda);
    // initial guess
    PostureDat << string("# INITIAL GUESS \n");
    PostureDat << string("var theta := \n");
    for (std::size_t i=0; i < initialGuess.size(); ++i){
        string guess =  boost::str(boost::format("%.2f") % (initialGuess.at(i)));
        boost::replace_all(guess,",",".");
        if (i == initialGuess.size()-1){
            PostureDat << to_string(i+1)+string(" ")+guess+string(";\n");
        }else{
            PostureDat << to_string(i+1)+string(" ")+guess+string("\n");
        }
    }
    // Parameters of the Fingers
    switch(hand_code){
    case 0: // human hand
        this->writeHumanHandParams(this->hhand,PostureDat,k);
        break;
    case 1: // barrett hand
        this->writeBarrettHandParams(this->bhand,PostureDat);
        break;
    }
    // info of the target to reach
    this->writeInfoTarget(PostureDat,tar);
    // info approach/retreat
    switch(mov_type){
    case 0: //pick
        switch(pre_post){
        case 0: // no approach, no retreat
            break;
        case 1: // approach
            if(approach){this->writeInfoApproachRetreat(PostureDat,tar,pre_grasp_approach);}
            break;
        case 2: // retreat
            if(retreat){this->writeInfoApproachRetreat(PostureDat,tar,post_grasp_retreat);}
            break;
        }
        break;
    }
    //info objects
    this->writeInfoObstacles(PostureDat,obsts);
    // object that has the target
    switch(mov_type){
    case 0: //pick
        this->writeInfoObjectTarget(PostureDat,obj_tar);
        break;
    }
    //close the file
    PostureDat.close();

    // ------------- Write the mod file ------------------------- //

    string filenamemod("FinalPosture.mod");
    ofstream PostureMod;
    // open the file
    PostureMod.open(path+filenamemod);

    PostureMod << string("# FINAL POSTURE MODEL FILE \n");
    PostureMod << string("# Movement to plan: \n");
    PostureMod << string("# ")+mov_infoLine+string("\n\n");

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("# PARAMETERS \n\n");
    PostureMod << string("set nJoints := 1..")+to_string(initialGuess.size())+string(";\n");

    this->writePI(PostureMod);
    this->writeBodyDimMod(PostureMod);
    this->writeArmDHParamsMod(PostureMod);
    this->write_dHOMod(PostureMod);

    PostureMod << string("# Joint Limits \n");
    PostureMod << string("param llim {i in 1..")+to_string(joints_arm)+string("} ; \n");
    PostureMod << string("param ulim {i in 1..")+to_string(joints_arm)+string("} ; \n");

    PostureMod << string("# Initial posture \n");
    PostureMod << string("param thet_init {i in 1..")+to_string(joints_arm)+string("} ; \n");

    PostureMod << string("# Final finger posture \n");
    PostureMod << string("param joint_fingers {i in 1..")+to_string(joints_arm)+string("} ; \n");

    PostureMod << string("# Joint Expense Factors \n");
    PostureMod << string("param lambda {i in 1..")+to_string(joints_arm)+string("} ; \n");

    switch(hand_code){
    case 0: // human hand
        this->writeHumanHandParamsMod(PostureMod);
        break;
    case 1: // barrett hand
        this->writeBarrettHandParamsMod(PostureMod);
        break;
    }
    // info objects
    bool vec=false;// true if there is some pre or post operation
    if((approach || retreat) && pre_post!=0){vec=true;}
    this->writeInfoObjectsMod(PostureMod,vec);

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("# DECISION VARIABLES \n");
    PostureMod << string("var theta {i in 1..")+to_string(joints_arm)+string("} >= llim[i], <= ulim[i]; \n");

    // Rotation matrix of the obstacles
    this->writeRotMatObsts(PostureMod);
    // Direct Kinematics of the arm
    this->writeArmDirKin(PostureMod,matWorldToArm,matHand,tolsArm,true);

    switch(hand_code){
    case 0: // human hand
        this->writeHumanHandDirKin(PostureMod,tolsHand,true,false);
        break;
    case 1: // barrett hand
        this->writeBarrettHandDirKin(PostureMod,tolsHand,true,false);
        break;
    }

    // Points of the arm
    PostureMod << string("var Points_Arm {j in 1..21, i in 1..4} = \n");
    PostureMod << string("if ( j=1 ) then 	(Shoulder[i]+Elbow[i])/2  \n");
    PostureMod << string("else	if ( j=2 ) then 	Elbow[i] \n");
    PostureMod << string("else    if ( j=3 ) then 	(Wrist[i]+Elbow[i])/2  \n");
    PostureMod << string("else	if ( j=4 ) then 	Wrist[i] \n");
    PostureMod << string("else	if ( j=5 ) then 	Wrist[i]+0.45*(Hand[i]-Wrist[i]) \n");
    PostureMod << string("else	if ( j=6 ) then 	Wrist[i]+0.75*(Hand[i]-Wrist[i]) \n");
    PostureMod << string("else	if ( j=7 ) then 	Finger1_1[i] \n");
    PostureMod << string("else	if ( j=8 ) then 	Finger2_1[i] \n");
    PostureMod << string("else	if ( j=9 ) then 	Finger3_1[i]\n");
    PostureMod << string("else	if ( j=10 ) then 	(Finger1_1[i]+Finger1_2[i])/2 \n");
    PostureMod << string("else	if ( j=11 ) then 	(Finger2_1[i]+Finger2_2[i])/2 \n");
    PostureMod << string("else	if ( j=12 ) then 	(Finger3_1[i]+Finger3_2[i])/2 \n");
    PostureMod << string("else	if ( j=13 ) then 	 Finger1_2[i] \n");
    PostureMod << string("else	if ( j=14 ) then 	 Finger2_2[i] \n");
    PostureMod << string("else	if ( j=15 ) then 	 Finger3_2[i] \n");
    PostureMod << string("else	if ( j=16 ) then 	(Finger1_2[i]+Finger1_tip[i])/2	 \n");
    PostureMod << string("else	if ( j=17 ) then 	(Finger2_2[i]+Finger2_tip[i])/2 \n");
    PostureMod << string("else	if ( j=18 ) then 	(Finger3_2[i]+Finger3_tip[i])/2 \n");
    PostureMod << string("else	if ( j=19 ) then 	Finger1_tip[i]\n");
    PostureMod << string("else	if ( j=20 ) then 	Finger2_tip[i] \n");
    PostureMod << string("else	if ( j=21 ) then 	Finger3_tip[i] \n");
    PostureMod << string("; \n\n");

    // objective function
    this->writeObjective(PostureMod,true);

    // constraints
    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("#  \n");
    PostureMod << string("#		      Constraints                  # \n");
    PostureMod << string("#  \n");
    string tarpos = boost::str(boost::format("%.2f") % tolTarPos); boost::replace_all(tarpos,",",".");
    string taror = boost::str(boost::format("%.4f") % tolTarOr); boost::replace_all(taror,",",".");
    PostureMod << string("# Hand position \n");
    switch (mov_type){
    case 0: //pick
        if(pre_post == 0){
            PostureMod << string("# subject to contr_hand_pos  {i in 1..3}: Hand[i] + dFH*z_H[i] - Tar_pos[i] = 0; \n");
            PostureMod << string("subject to contr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        }else{
            PostureMod << string("# subject to contr_hand_pos  {i in 1..3}: Hand[i] + (dFH+dist)*z_H[i] - Tar_pos[i] = 0; \n");
            PostureMod << string("subject to contr_hand_pos: (sum{i in 1..3} (Hand[i] + (dFH+dist)*z_H[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        }
        break;
    }

/*
    switch(mov_type){
    case 0: // pick
        PostureMod << string("subject to contr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        break;
    case 1: // reaching
        break;
    case 2:// transport
        break;
    case 3: // engage
        // the target here has been computed according the new position of the target
        // (the constraint is equal to that one for reaching and grasping)
        PostureMod << string("subject to contr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        break;
    case 4: // disengage
        break;
    case 5: // Go park
        break;
    }
    */

    PostureMod << string("# Hand orientation\n");
    switch(griptype){
    case 111: case 211: // side thumb left
        switch (mov_type){
        case 0: //pick
            // hand constraints for approaching and retreating direction setting
            if(pre_post==0){
                // do not use approach/retreat options
                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 )<= ")+taror+string("; #  x_H = z_t \n");
            }else{
                // use approach/retreat options
                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 + sum{i in 1..3} (z_H[i] + v_t[i])^2 )<= ")+taror+string("; #  x_H = z_t and z_H = -v_t \n");
            }
            break;
        case 1:// reaching
            break;
        case 2: // transport
            break;
        case 3: // engage
/*

            // hand constraints for approaching direction setting
            // (TO REVIEW)
            switch (targetAxis) {

            case 0: // none

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 )<= ")+taror+string("; #  x_H = z_t \n");

                break;
            case 1: // x target

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 + sum{i in 1..3} (z_H[i] + x_t[i])^2 )<= ")+taror+string("; #  x_H = z_t and z_H = -x_t \n");
                break;

            case 2: // y target

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 + sum{i in 1..3} (z_H[i] + y_t[i])^2 )<= ")+taror+string("; #  x_H = z_t and z_H = -y_t \n");

                break;

            case 3: // z target

                PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 )<= ")+taror+string("; #  x_H = z_t \n");

                break;

            }
            */

            break;


        case 4: // disengage
            break;
        case 5: // Go park
            break;

        }



        break;

    case 112: case 212: // side thumb right

        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] + z_t[i])^2) <= ")+taror+string("; #  x_H = -z_t \n");

        break;

    case 113: case 213: // side thumb up

        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (y_H[i] + z_t[i])^2) <= ")+taror+string("; #  y_H = -z_t \n");

        break;

    case 114: case 214: // side thumb down

        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (y_H[i] - z_t[i])^2) <= ")+taror+string("; #  y_H = z_t \n");

        break;

    case 121: case 221: // above

        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (z_H[i] + z_t[i])^2) <= ")+to_string(tolTarOr)+string("; #  z_H = -z_t \n");

        break;

    case 122: case 222: // below

        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (z_H[i] - z_t[i])^2) <= ")+to_string(tolTarOr)+string("; #  z_H = z_t \n");

        break;
    }


    if(obstacle_avoidance){
        // obstacles
        //xx
        string txx1 = boost::str(boost::format("%.2f") % tolsObstacles(0,0)); boost::replace_all(txx1,",",".");
        string txx2 = boost::str(boost::format("%.2f") % tolsObstacles(1,0)); boost::replace_all(txx2,",",".");
        string txx3 = boost::str(boost::format("%.2f") % tolsObstacles(2,0)); boost::replace_all(txx3,",",".");
        //yy
        string tyy1 = boost::str(boost::format("%.2f") % tolsObstacles(0,1)); boost::replace_all(tyy1,",",".");
        string tyy2 = boost::str(boost::format("%.2f") % tolsObstacles(1,1)); boost::replace_all(tyy2,",",".");
        string tyy3 = boost::str(boost::format("%.2f") % tolsObstacles(2,1)); boost::replace_all(tyy3,",",".");
        //zz
        string tzz1 = boost::str(boost::format("%.2f") % tolsObstacles(0,2)); boost::replace_all(tzz1,",",".");
        string tzz2 = boost::str(boost::format("%.2f") % tolsObstacles(1,2)); boost::replace_all(tzz2,",",".");
        string tzz3 = boost::str(boost::format("%.2f") % tolsObstacles(2,2)); boost::replace_all(tzz3,",",".");
        //xy
        string txy1 = boost::str(boost::format("%.2f") % tolsObstacles(0,3)); boost::replace_all(txy1,",",".");
        string txy2 = boost::str(boost::format("%.2f") % tolsObstacles(1,3)); boost::replace_all(txy2,",",".");
        string txy3 = boost::str(boost::format("%.2f") % tolsObstacles(2,3)); boost::replace_all(txy3,",",".");
        //xz
        string txz1 = boost::str(boost::format("%.2f") % tolsObstacles(0,4)); boost::replace_all(txz1,",",".");
        string txz2 = boost::str(boost::format("%.2f") % tolsObstacles(1,4)); boost::replace_all(txz2,",",".");
        string txz3 = boost::str(boost::format("%.2f") % tolsObstacles(2,4)); boost::replace_all(txz3,",",".");
        //yz
        string tyz1 = boost::str(boost::format("%.2f") % tolsObstacles(0,5)); boost::replace_all(tyz1,",",".");
        string tyz2 = boost::str(boost::format("%.2f") % tolsObstacles(1,5)); boost::replace_all(tyz2,",",".");
        string tyz3 = boost::str(boost::format("%.2f") % tolsObstacles(2,5)); boost::replace_all(tyz3,",",".");


        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("# \n");
        PostureMod << string("subject to obst_Arm{j in 1..21, i in 1..n_Obstacles}:  \n");
        PostureMod << string("((Points_Arm[j,1]-Obstacles[i,1])^2)*(  \n");
        PostureMod << string("(Rot[1,1,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4]+")+txx1+string(")^2) + \n");
        PostureMod << string("(Rot[2,1,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4]+")+txx2+string(")^2) + \n");
        PostureMod << string("(Rot[3,1,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4]+")+txx3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm[j,2]-Obstacles[i,2])^2)*(  \n");
        PostureMod << string("(Rot[1,2,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4]+")+tyy1+string(")^2) + \n");
        PostureMod << string("(Rot[2,2,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4]+")+tyy2+string(")^2) + \n");
        PostureMod << string("(Rot[3,2,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4]+")+tyy3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm[j,3]-Obstacles[i,3])^2)*( \n");
        PostureMod << string("(Rot[1,3,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4]+")+tzz1+string(")^2) + \n");
        PostureMod << string("(Rot[2,3,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4]+")+tzz2+string(")^2) +  \n");
        PostureMod << string("(Rot[3,3,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4]+")+tzz3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,1]-Obstacles[i,1])*(Points_Arm[j,2]-Obstacles[i,2])* ( \n");
        PostureMod << string("(Rot[1,1,i]*Rot[1,2,i])/((Obstacles[i,4]+Points_Arm[j,4]+")+txy1+string(")^2) + \n");
        PostureMod << string("(Rot[2,1,i]*Rot[2,2,i])/((Obstacles[i,5]+Points_Arm[j,4]+")+txy2+string(")^2) + \n");
        PostureMod << string("(Rot[3,1,i]*Rot[3,2,i])/((Obstacles[i,6]+Points_Arm[j,4]+")+txy3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,1]-Obstacles[i,1])*(Points_Arm[j,3]-Obstacles[i,3])* ( \n");
        PostureMod << string("(Rot[1,1,i]*Rot[1,3,i])/((Obstacles[i,4]+Points_Arm[j,4]+")+txz1+string(")^2) + \n");
        PostureMod << string("(Rot[2,1,i]*Rot[2,3,i])/((Obstacles[i,5]+Points_Arm[j,4]+")+txz2+string(")^2) + \n");
        PostureMod << string("(Rot[3,1,i]*Rot[3,3,i])/((Obstacles[i,6]+Points_Arm[j,4]+")+txz3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,2]-Obstacles[i,2])*(Points_Arm[j,3]-Obstacles[i,3])* ( \n");
        PostureMod << string("(Rot[1,2,i]*Rot[1,3,i])/((Obstacles[i,4]+Points_Arm[j,4]+")+tyz1+string(")^2) + \n");
        PostureMod << string("(Rot[2,2,i]*Rot[2,3,i])/((Obstacles[i,5]+Points_Arm[j,4]+")+tyz2+string(")^2) + \n");
        PostureMod << string("(Rot[3,2,i]*Rot[3,3,i])/((Obstacles[i,6]+Points_Arm[j,4]+")+tyz3+string(")^2)) \n");
        PostureMod << string("-1 >=0; \n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("#  \n");

    }

    // constraints with the body
    this->writeBodyConstraints(PostureMod,true);

    // constraints with the table
    // this->writeTableConstraints(PostureMod,true,griptype,tols_table);

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n\n\n");

    // close the file
    PostureMod.close();

    // ----- write run file for options --------

    string filenamerun("options.run");
    ofstream optionsrun;
    // open the file
    optionsrun.open(path+filenamerun);

    optionsrun << string("option presolve 0; \n");

    //close the file
    optionsrun.close();


    return true;

}



bool HUMPlanner::writeFilesBouncePosture(huml_params& params,int mov_type, int pre_post,std::vector<double> minAuxLimits, std::vector<double> maxAuxLimits,std::vector<double> initAuxPosture, std::vector<double> finalAuxPosture,
                                         std::vector<double> initialGuess, std::vector<objectPtr> objs,boundaryConditions b)
{
    //  --- create the "Models" directory if it does not exist ---
    struct stat st = {0};
    if (stat("Models", &st) == -1) {
        mkdir("Models", 0700);
    }
    string path("Models/");

    Matrix4d matWorldToArm;
    Matrix4d matHand;
    DHparameters dh;

    // movement setting
    int arm_code = params.mov_specs.arm_code;
    int hand_code = params.mov_specs.hand_code;
    int griptype = params.mov_specs.griptype;
    double dHO = params.mov_specs.dHO;
    std::vector<double> finalHand = params.mov_specs.finalHand;
    std::vector<double> tar = params.mov_specs.target;
    objectPtr obj_tar = params.mov_specs.obj;
    string mov_infoLine = params.mov_specs.mov_infoline;
    bool approach = params.mov_specs.approach;
    bool retreat = params.mov_specs.retreat;
    std::vector<double> pre_grasp_approach;
    std::vector<double> post_grasp_retreat;
    std::vector<double> pre_place_approach;
    std::vector<double> post_place_retreat;
    std::vector<double> vel_approach = params.vel_approach;
    std::vector<double> acc_approach = params.acc_approach;
    switch(mov_type){
    case 0: // pick
        obj_tar = params.mov_specs.obj;
        if(approach){pre_grasp_approach = params.mov_specs.pre_grasp_approach;}
        if(retreat){post_grasp_retreat = params.mov_specs.post_grasp_retreat;}
        break;

    }

    // tolerances
    int steps = params.steps;
    double totalTime = params.totalTime;
    std::vector<double> lambda = params.lambda_bounce;
    std::vector<double> tolsArm = params.tolsArm;
    MatrixXd tolsHand = params.tolsHand;
    vector< MatrixXd > tolsTarget = params.singleArm_tolsTarget;
    vector< MatrixXd > tolsObstacles = params.singleArm_tolsObstacles;
    bool obstacle_avoidance = params.obstacle_avoidance;
    bool target_avoidance = params.target_avoidance;
    int k;
    switch(arm_code){
    case 0: // dual arm
        //TO DO
        break;
    case 1: // right arm
        k=1;
        matWorldToArm=this->matWorldToRightArm;
        matHand = this->matRightHand;
        dh=this->DH_rightArm;
        break;
    case 2: // left arm
        k=-1;
        matWorldToArm=this->matWorldToLeftArm;
        matHand = this->matLeftHand;
        dh=this->DH_leftArm;
        break;
    }
    //------------------------- Write the dat file --------------------------------------------------
     string filename("BouncePosture.dat");
     ofstream PostureDat;
     // open the file
     PostureDat.open(path+filename);

     PostureDat << string("# BOUNCE POSTURE DATA FILE \n");
     PostureDat << string("# Units of measure: [rad], [mm] \n\n");

     PostureDat << string("data; \n");

     // number of steps
     PostureDat << string("param Nsteps :=")+to_string(steps)+string(";\n");

     // total time
     string time =  boost::str(boost::format("%.2f") % (totalTime));
     boost::replace_all(time,",",".");
     PostureDat << string("param TotalTime :=")+time+string(";\n");
     // Body dimension
     this->writeBodyDim(this->torso_size.at(0),this->torso_size.at(1),PostureDat);
     // D-H Parameters of the Arm
     this->writeArmDHParams(dh,PostureDat,k);
     // distance between the hand and the object
     this->write_dHO(PostureDat,dHO);
     // joint limits
     this->writeArmLimits(PostureDat,minAuxLimits,maxAuxLimits);
     // initial pose of the arm
     this->writeArmInitPose(PostureDat,initAuxPosture);
     // final pose of the arm
     PostureDat << string("# FINAL POSE \n");
     PostureDat << string("param thet_final := \n");
     for (std::size_t i=0; i < finalAuxPosture.size(); ++i){
         string finalAuxstr =  boost::str(boost::format("%.2f") % (finalAuxPosture.at(i)));
         boost::replace_all(finalAuxstr,",",".");
         if (i == finalAuxPosture.size()-1){
             PostureDat << to_string(i+1)+string(" ")+finalAuxstr+string(";\n");
         }else{
             PostureDat << to_string(i+1)+string(" ")+finalAuxstr+string("\n");
         }
     }
     // final posture of the fingers
     this->writeFingerFinalPose(PostureDat,finalHand);
     // joint expense factors of the arm
     this->writeLambda(PostureDat,lambda);
     // initial guess
     PostureDat << string("# INITIAL GUESS \n");
     PostureDat << string("var theta_b := \n");
     for (std::size_t i=0; i < initialGuess.size(); ++i){
         string guess =  boost::str(boost::format("%.2f") % (initialGuess.at(i)));
         boost::replace_all(guess,",",".");
         if (i == initialGuess.size()-1){
             PostureDat << to_string(i+1)+string(" ")+guess+string(";\n");
         }else{
             PostureDat << to_string(i+1)+string(" ")+guess+string("\n");
         }
     }

     // boundary conditions
     if(pre_post==0 || !approach){
         // boundary conditions initial velocity
         PostureDat << string("# INITIAL VELOCITY \n");
         PostureDat << string("param vel_0 := \n");
         for (std::size_t i=0; i < b.vel_0.size(); ++i){
             string vel_0 =  boost::str(boost::format("%.2f") % (b.vel_0.at(i)));
             boost::replace_all(vel_0,",",".");
             if (i == b.vel_0.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+vel_0+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+vel_0+string("\n");
             }
         }
         // boundary conditions initial acceleration
         PostureDat << string("# INITIAL ACCELERATION \n");
         PostureDat << string("param acc_0 := \n");

         for (std::size_t i=0; i < b.acc_0.size(); ++i){
             string acc_0 =  boost::str(boost::format("%.2f") % (b.acc_0.at(i)));
             boost::replace_all(acc_0,",",".");
             if (i == b.acc_0.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+acc_0+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+acc_0+string("\n");
             }
         }
         // boundary conditions final velocity
         PostureDat << string("# FINAL VELOCITY \n");
         PostureDat << string("param vel_f := \n");
         for (std::size_t i=0; i < b.vel_f.size(); ++i){
             string vel_f =  boost::str(boost::format("%.2f") % (b.vel_f.at(i)));
             boost::replace_all(vel_f,",",".");
             if (i == b.vel_f.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+vel_f+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+vel_f+string("\n");
             }
         }
         // boundary conditions final acceleration
         PostureDat << string("# FINAL ACCELERATION \n");
         PostureDat << string("param acc_f := \n");
         for (std::size_t i=0; i < b.acc_f.size(); ++i){
             string acc_f =  boost::str(boost::format("%.2f") % (b.acc_f.at(i)));
             boost::replace_all(acc_f,",",".");

             if (i == b.acc_f.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+acc_f+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+acc_f+string("\n");
             }
         }
     }else if (approach && pre_post==1){ // use approach options
         // boundary conditions initial velocity
         PostureDat << string("# INITIAL VELOCITY \n");
         PostureDat << string("param vel_0 := \n");
         for (std::size_t i=0; i < b.vel_0.size(); ++i){
             string vel_0 =  boost::str(boost::format("%.2f") % (b.vel_0.at(i)));
             boost::replace_all(vel_0,",",".");
             if (i == b.vel_0.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+vel_0+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+vel_0+string("\n");
             }
         }
         // boundary conditions initial acceleration
         PostureDat << string("# INITIAL ACCELERATION \n");
         PostureDat << string("param acc_0 := \n");

         for (std::size_t i=0; i < b.acc_0.size(); ++i){
             string acc_0 =  boost::str(boost::format("%.2f") % (b.acc_0.at(i)));
             boost::replace_all(acc_0,",",".");
             if (i == b.acc_0.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+acc_0+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+acc_0+string("\n");
             }
         }
         // boundary conditions final velocity
         PostureDat << string("# FINAL VELOCITY \n");
         PostureDat << string("param vel_f := \n");
         for (std::size_t i=0; i < b.vel_f.size(); ++i){
             string vel_f =  boost::str(boost::format("%.2f") % (vel_approach.at(i)));
             boost::replace_all(vel_f,",",".");
             if (i == b.vel_f.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+vel_f+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+vel_f+string("\n");
             }
         }
         // boundary conditions final acceleration
         PostureDat << string("# FINAL ACCELERATION \n");
         PostureDat << string("param acc_f := \n");
         for (std::size_t i=0; i < b.acc_f.size(); ++i){
             string acc_f =  boost::str(boost::format("%.2f") % (acc_approach.at(i)));
             boost::replace_all(acc_f,",",".");

             if (i == b.acc_f.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+acc_f+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+acc_f+string("\n");
             }
         }

     }
     // tb and phi
     PostureDat << string("# TB and PHI \n");
     PostureDat << string("param TB := ");
     string tb_str =  boost::str(boost::format("%.2f") % (TB));
     PostureDat << tb_str+string(";\n");
     PostureDat << string("param PHI := ");
     string phi_str =  boost::str(boost::format("%.2f") % (PHI));
     PostureDat << phi_str+string(";\n");

     // Parameters of the Fingers
     switch(hand_code){
     case 0: // human hand
         this->writeHumanHandParams(this->hhand,PostureDat,k);
         break;
     case 1: // barrett hand
         this->writeBarrettHandParams(this->bhand,PostureDat);
         break;
     }
     // info of the target to reach
     this->writeInfoTarget(PostureDat,tar);
     // info approach/retreat
     switch(mov_type){
     case 0: //pick         
         if (pre_post!=0){this->writeInfoApproachRetreat(PostureDat,tar,pre_grasp_approach);}
         break;
     }

     //info objects
     this->writeInfoObstacles(PostureDat,objs);
     // object that has the target
     switch(mov_type){
     case 0: //pick
         this->writeInfoObjectTarget(PostureDat,obj_tar);
         break;
     }
     //close the file
     PostureDat.close();

     // ------------- Write the mod file ------------------------- //

     string filenamemod("BouncePosture.mod");
     ofstream PostureMod;
     // open the file
     PostureMod.open(path+filenamemod);

     PostureMod << string("# BOUNCE POSTURE MODEL FILE \n");
     PostureMod << string("# Movement to plan: \n");
     PostureMod << string("# ")+mov_infoLine+string("\n\n");

     PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
     PostureMod << string("# PARAMETERS \n\n");

     this->writePI(PostureMod);
     this->writeBodyDimMod(PostureMod);
     this->writeArmDHParamsMod(PostureMod);
     this->write_dHOMod(PostureMod);

     PostureMod << string("# Joint Limits \n");
     PostureMod << string("param llim {i in 1..")+to_string(minAuxLimits.size())+string("} ; \n");
     PostureMod << string("param ulim {i in 1..")+to_string(maxAuxLimits.size())+string("} ; \n");
     PostureMod << string("# Initial posture \n");
     PostureMod << string("param thet_init {i in 1..")+to_string(initAuxPosture.size())+string("} ; \n");
     PostureMod << string("# Final posture \n");
     PostureMod << string("param thet_final {i in 1..")+to_string(finalAuxPosture.size())+string("} ; \n");
     PostureMod << string("# Final finger posture \n");
     PostureMod << string("param joint_fingers {i in 1..")+to_string(joints_hand)+string("} ; \n");
     PostureMod << string("# Joint Expense Factors \n");
     PostureMod << string("param lambda {i in 1..")+to_string(lambda.size())+string("} ; \n");

     switch(hand_code){
     case 0: // human hand
         this->writeHumanHandParamsMod(PostureMod);
         break;
     case 1: // barrett hand
         this->writeBarrettHandParamsMod(PostureMod);
         break;
     }
     // info objects
     bool vec=false;// true if there is some pre or post operation
     if(approach && pre_post!=0){vec=true;}
     this->writeInfoObjectsMod(PostureMod,vec);

     PostureMod << string("# Boundary Conditions \n");
     PostureMod << string("param vel_0 {i in 1..")+to_string(b.vel_0.size())+string("} ; \n");
     PostureMod << string("param vel_f {i in 1..")+to_string(b.vel_f.size())+string("} ; \n");
     PostureMod << string("param acc_0 {i in 1..")+to_string(b.acc_0.size())+string("} ; \n");
     PostureMod << string("param acc_f {i in 1..")+to_string(b.acc_f.size())+string("} ; \n");

     PostureMod << string("# Time and iterations\n");
     PostureMod << string("param Nsteps;\n");
     PostureMod << string("param TB;\n");
     PostureMod << string("param PHI;\n");
     PostureMod << string("param TotalTime;\n");
     PostureMod << string("set Iterations := 1..(Nsteps+1);\n");
     PostureMod << string("set nJoints := 1..")+to_string(initialGuess.size())+string(";\n");
     PostureMod << string("set Iterations_nJoints := Iterations cross nJoints;\n");
     PostureMod << string("param time {i in Iterations} = ((i-1)*TotalTime)/Nsteps;\n");

     PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
     PostureMod << string("# DECISION VARIABLES \n");
     PostureMod << string("# Bounce Posture \n");
     PostureMod << string("var theta_b {i in 1..")+to_string(initialGuess.size())+string("} >= llim[i], <= ulim[i]; \n");

     PostureMod << string("# Direct Movement \n");
     PostureMod << string("param the_direct {(i,j) in Iterations_nJoints} := thet_init[j]+ \n");
     PostureMod << string("( thet_final[j] - thet_init[j] ) * (10*(time[i])^3 -15*(time[i])^4 +6*(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("vel_0[j] *TotalTime* (time[i] - 6 *(time[i])^3 +8*(time[i])^4 -3*(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("vel_f[j] *TotalTime* (- 4 *(time[i])^3 +7*(time[i])^4 -3*(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("acc_0[j]/2 *TotalTime^2* (time[i]^2 - 3 *(time[i])^3 +3*(time[i])^4 -(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("acc_f[j]/2 *TotalTime^2* ((time[i])^3 -2*(time[i])^4 +(time[i])^5); \n");

     PostureMod << string("# Back and forth Movement \n");
     PostureMod << string("var the_bf 	{(i,j) in Iterations_nJoints} =  \n");
     PostureMod << string("((time[i]*(1-time[i]))/(TB*(1-TB)))*(theta_b[j] - thet_init[j])*(sin(pi*(time[i])^PHI))^2; \n");

     PostureMod << string("# Composite Movement \n");
     PostureMod << string("var theta 	{(i,j) in Iterations_nJoints} = \n");
     PostureMod << string("the_direct[i,j] + the_bf[i,j];\n");

     // Rotation matrix of the obstacles
     this->writeRotMatObsts(PostureMod);
     // Direct Kinematics of the arm
     this->writeArmDirKin(PostureMod,matWorldToArm,matHand,tolsArm,false);

     std::vector<double> obj_tar_size; obj_tar->getSize(obj_tar_size);
     string obj_radius =  boost::str(boost::format("%.2f") % std::max(obj_tar_size.at(0),obj_tar_size.at(1))); boost::replace_all(obj_radius,",",".");
     string obj_size_z =  boost::str(boost::format("%.2f") % obj_tar_size.at(2)); boost::replace_all(obj_size_z,",",".");

     bool engage = false; // true when there is an object to engage
     bool transport = false; // true when there is an object to transport
     bool gopark = false; // true when the movement is Go park
     bool release_object = false; // true when Go park and release an object

     /*
     // object to transport (place movements)
     switch (mov_type) {
     case 0: // pick
         break;
     case 1: // reaching
         break;
     case 2: // transport
         transport = true;
         break;
     case 3: // engage

         engage = true;

         PostureMod << string("var Obj2Transp {j in 1..4, i in Iterations} = if j<3 then Hand[j,i] + dFH * z_H[j,i] \n");
         PostureMod << string("else 	if (j=4) then ")+obj_radius+string("; \n");

         switch (griptype) {

         case 111: case 211: case 112: case 212: // Side thumb right, Side thumb left


             PostureMod << string("var Obj2Transp_1 {j in 1..4, i in Iterations} = if j<3 then Obj2Transp[j,i] + x_H[j,i] *(")+obj_size_z+string("/2)\n");
             PostureMod << string("else 	if (j=4) then ")+obj_radius+string("; \n");

             PostureMod << string("var Obj2Transp_2 {j in 1..4, i in Iterations} = if j<3 then Obj2Transp[j,i] - x_H[j,i] *(")+obj_size_z+string("/2)\n");
             PostureMod << string("else 	if (j=4) then ")+obj_radius+string("; \n");

             break;

         case 113: case 213: case 114: case 214: // Side thumb up, Side thumb down

             // TO DO
             break;

         case 121: case 221: case 122: case 222: // Above, Below

             // TO DO
             break;
         }

         break;

     case 4: // disengage

         break;

     case 5: // Go park
         gopark=true;
         release_object=std::strcmp(obj_tar->getName().c_str(),"")!=0;
         break;
     }
     */

     switch(hand_code){
     case 0: // human hand
         this->writeHumanHandDirKin(PostureMod,tolsHand,false,engage || transport);
         break;
     case 1: // barrett hand
         this->writeBarrettHandDirKin(PostureMod,tolsHand,false,engage || transport);
         break;
     }
     // Points of the arm
     if (engage){
      PostureMod << string("var Points_Arm {j in 1..18, i in 1..4,k in Iterations} = \n");
     }else{
      PostureMod << string("var Points_Arm {j in 1..15, i in 1..4,k in Iterations} = \n");
     }
     PostureMod << string("if ( j=1 ) then 	(Shoulder[i,k]+Elbow[i,k])/2  \n");
     PostureMod << string("else	if ( j=2 ) then 	Elbow[i,k] \n");
     PostureMod << string("else    if ( j=3 ) then 	(Wrist[i,k]+Elbow[i,k])/2  \n");
     PostureMod << string("else	if ( j=4 ) then 	Wrist[i,k] \n");
     PostureMod << string("else	if ( j=5 ) then 	Wrist[i,k]+0.45*(Hand[i,k]-Wrist[i,k]) \n");
     PostureMod << string("else	if ( j=6 ) then 	Wrist[i,k]+0.75*(Hand[i,k]-Wrist[i,k]) \n");
     PostureMod << string("else	if ( j=7 ) then 	Finger1_1[i,k] \n");
     PostureMod << string("else	if ( j=8 ) then 	Finger2_1[i,k] \n");
     PostureMod << string("else	if ( j=9 ) then 	Finger3_1[i,k]\n");
     PostureMod << string("#else	if ( j=10 ) then 	(Finger1_1[i,k]+Finger1_2[i,k])/2 \n");
     PostureMod << string("#else	if ( j=11 ) then 	(Finger2_1[i,k]+Finger2_2[i,k])/2 \n");
     PostureMod << string("#else	if ( j=12 ) then 	(Finger3_1[i,k]+Finger3_2[i,k])/2 \n");
     PostureMod << string("else	if ( j=10 ) then 	 Finger1_2[i,k] \n");
     PostureMod << string("else	if ( j=11 ) then 	 Finger2_2[i,k] \n");
     PostureMod << string("else	if ( j=12 ) then 	 Finger3_2[i,k] \n");
     PostureMod << string("#else	if ( j=16 ) then 	(Finger1_2[i,k]+Finger1_tip[i,k])/2	 \n");
     PostureMod << string("#else	if ( j=17 ) then 	(Finger2_2[i,k]+Finger2_tip[i,k])/2 \n");
     PostureMod << string("#else	if ( j=18 ) then 	(Finger3_2[i,k]+Finger3_tip[i,k])/2 \n");
     PostureMod << string("else	if ( j=13 ) then 	Finger1_tip[i,k]\n");
     PostureMod << string("else	if ( j=14 ) then 	Finger2_tip[i,k] \n");
     PostureMod << string("else	if ( j=15 ) then 	Finger3_tip[i,k] \n");
     if (engage){
         PostureMod << string("else    if ( j=16 ) then 	Obj2Transp[i,k] \n");
         PostureMod << string("else    if ( j=17 ) then 	Obj2Transp_1[i,k] \n");
         PostureMod << string("else    if ( j=18 ) then 	Obj2Transp_2[i,k] \n");
     }
     PostureMod << string("; \n\n");
     // objective function
     this->writeObjective(PostureMod,false);
     // constraints
     PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
     PostureMod << string("#  \n");
     PostureMod << string("#		     Constraints                  # \n");
     PostureMod << string("#  \n");
     PostureMod << string("# joint limits for all the trajectory \n");
     PostureMod << string("subject to co_JointLimits {i in Iterations, j in nJoints}: llim[j] <= theta[i,j]  <= ulim[j]; \n\n");

     // hand constraints for approaching direction setting
     switch (griptype) {
     case 111: case 211: // side thumb left
         switch (mov_type) {
         case 0: // pick
             // hand constraints for approaching and retreating direction settings
             if(approach && pre_post!=0){
                 PostureMod << string("# Hand approach orientation\n");
                 PostureMod << string("subject to constr_hand_or {k in (Nsteps-5)..(Nsteps+1)}: ( sum{i in 1..3} (z_H[i,k] + v_t[i])^2 )<= 0.010; #  z_H = -v_t  \n\n");
             }
         case 1: // reaching
             break;
         case 2: // transport
             break;
         case 3: // engage
             /*

             switch (targetAxis) {

             case 0: // none

                 break;

             case 1: // x target

                 //PostureMod << string("# Hand approach orientation\n");
                 //PostureMod << string("subject to constr_hand_or {k in (Nsteps-5)..(Nsteps+1)}: ( sum{i in 1..3} (z_H[i,k] + x_t[i])^2 )<= 0.010; #  z_H = -x_t  \n\n");

                 break;

             case 2: // y target

                 //PostureMod << string("# Hand approach orientation\n");
                 //PostureMod << string("subject to constr_hand_or {k in (Nsteps-5)..(Nsteps+1)}: ( sum{i in 1..3} (z_H[i,k] + y_t[i])^2 )<= 0.010; #  z_H = -y_t  \n\n");

                 break;

             case 3: // z target

                 //PostureMod << string("# Hand approach orientation\n");
                 //PostureMod << string("subject to const_hand_or {k in (Nsteps-5)..(Nsteps+1)}: ( sum{i in 1..3} (x_H[i,k] - z_t[i])^2 )<= 0.010; #  z_H = -z_t  \n\n");

                 //PostureMod << string("# Hand approach orientation at the beginning\n");
                 //PostureMod << string("subject to constr_hand_or {k in 1..5}: ( sum{i in 1..3} (z_H[i,k] + z_w[i])^2 )<= 0.010; #  z_H = -y_t  \n\n");

                 break;
             }
             */

             break;

         case 4: // disengage
             break;
         case 5: // Go park

             /*
             switch (targetAxis) {
             case 0: // none

                 break;

             case 1: // x target

                 //PostureMod << string("# Hand approach orientation\n");
                 //PostureMod << string("subject to constr_hand_or {k in 1..5}: ( sum{i in 1..3} (z_H[i,k] + x_t[i])^2 )<= 0.010; #  z_H = -x_t  \n\n");

                 break;

             case 2: // y target


                 //PostureMod << string("# Hand approach orientation\n");
                 //PostureMod << string("subject to constr_hand_or {k in 1..5}: ( sum{i in 1..3} (z_H[i,k] + y_t[i])^2 )<= 0.010; #  z_H = -y_t  \n\n");

                 break;

             case 3: // z target

                 //PostureMod << string("# Hand approach orientation\n");
                 //PostureMod << string("subject to const_hand_or {k in 1..5}: ( sum{i in 1..3} (z_H[i,k] + z_t[i])^2 )<= 0.010; #  z_H = -z_t  \n\n");

                 break;
             }
             */

             break;
         }
         break;

     case 112: case 212: // side thumb right

         break;

     case 113: case 213: // side thumb up

         break;

     case 114: case 214: // side thumb down

         break;

     case 121: case 221: // above

         break;

     case 122: case 222: // below

         break;
     }
     if((gopark && release_object) || (!gopark && !release_object) ){

           if(target_avoidance){

                // constraints with the targets
                MatrixXd tols_0 = tolsTarget.at(0);
                MatrixXd tols_1 = tolsTarget.at(1);
                MatrixXd tols_2 = tolsTarget.at(2);

                //xx1
                string txx1_0 = boost::str(boost::format("%.2f") % tols_0(0,0)); boost::replace_all(txx1_0,",",".");
                string txx1_1 = boost::str(boost::format("%.2f") % tols_1(0,0)); boost::replace_all(txx1_1,",",".");
                string txx1_2 = boost::str(boost::format("%.2f") % tols_2(0,0)); boost::replace_all(txx1_2,",",".");

                PostureMod << string("param tol_target_xx1 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx1_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx1_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx1_2+string("; \n");

                //xx2
                string txx2_0 = boost::str(boost::format("%.2f") % tols_0(1,0)); boost::replace_all(txx2_0,",",".");
                string txx2_1 = boost::str(boost::format("%.2f") % tols_1(1,0)); boost::replace_all(txx2_1,",",".");
                string txx2_2 = boost::str(boost::format("%.2f") % tols_2(1,0)); boost::replace_all(txx2_2,",",".");

                PostureMod << string("param tol_target_xx2 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx2_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx2_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx2_2+string("; \n");


                // xx3
                string txx3_0 = boost::str(boost::format("%.2f") % tols_0(2,0)); boost::replace_all(txx3_0,",",".");
                string txx3_1 = boost::str(boost::format("%.2f") % tols_1(2,0)); boost::replace_all(txx3_1,",",".");
                string txx3_2 = boost::str(boost::format("%.2f") % tols_2(2,0)); boost::replace_all(txx3_2,",",".");

                PostureMod << string("param tol_target_xx3 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx3_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx3_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx3_2+string("; \n");

                // yy1
                string tyy1_0 = boost::str(boost::format("%.2f") % tols_0(0,1)); boost::replace_all(tyy1_0,",",".");
                string tyy1_1 = boost::str(boost::format("%.2f") % tols_1(0,1)); boost::replace_all(tyy1_1,",",".");
                string tyy1_2 = boost::str(boost::format("%.2f") % tols_2(0,1)); boost::replace_all(tyy1_2,",",".");

                PostureMod << string("param tol_target_yy1 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy1_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy1_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy1_2+string("; \n");

                // yy2
                string tyy2_0 = boost::str(boost::format("%.2f") % tols_0(1,1)); boost::replace_all(tyy2_0,",",".");
                string tyy2_1 = boost::str(boost::format("%.2f") % tols_1(1,1)); boost::replace_all(tyy2_1,",",".");
                string tyy2_2 = boost::str(boost::format("%.2f") % tols_2(1,1)); boost::replace_all(tyy2_2,",",".");

                PostureMod << string("param tol_target_yy2 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy2_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy2_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy2_2+string("; \n");


                // yy3
                string tyy3_0 = boost::str(boost::format("%.2f") % tols_0(2,1)); boost::replace_all(tyy3_0,",",".");
                string tyy3_1 = boost::str(boost::format("%.2f") % tols_1(2,1)); boost::replace_all(tyy3_1,",",".");
                string tyy3_2 = boost::str(boost::format("%.2f") % tols_2(2,1)); boost::replace_all(tyy3_2,",",".");

                PostureMod << string("param tol_target_yy3 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy3_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy3_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy3_2+string("; \n");

                // zz1
                string tzz1_0 = boost::str(boost::format("%.2f") % tols_0(0,2)); boost::replace_all(tzz1_0,",",".");
                string tzz1_1 = boost::str(boost::format("%.2f") % tols_1(0,2)); boost::replace_all(tzz1_1,",",".");
                string tzz1_2 = boost::str(boost::format("%.2f") % tols_2(0,2)); boost::replace_all(tzz1_2,",",".");

                PostureMod << string("param tol_target_zz1 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz1_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz1_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz1_2+string("; \n");

                // zz2
                string tzz2_0 = boost::str(boost::format("%.2f") % tols_0(1,2)); boost::replace_all(tzz2_0,",",".");
                string tzz2_1 = boost::str(boost::format("%.2f") % tols_1(1,2)); boost::replace_all(tzz2_1,",",".");
                string tzz2_2 = boost::str(boost::format("%.2f") % tols_2(1,2)); boost::replace_all(tzz2_2,",",".");

                PostureMod << string("param tol_target_zz2 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz2_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz2_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz2_2+string("; \n");

                // zz3
                string tzz3_0 = boost::str(boost::format("%.2f") % tols_0(2,2)); boost::replace_all(tzz3_0,",",".");
                string tzz3_1 = boost::str(boost::format("%.2f") % tols_1(2,2)); boost::replace_all(tzz3_1,",",".");
                string tzz3_2 = boost::str(boost::format("%.2f") % tols_2(2,2)); boost::replace_all(tzz3_2,",",".");

                PostureMod << string("param tol_target_zz3 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz3_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz3_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz3_2+string("; \n");


                // xy1
                string txy1_0 = boost::str(boost::format("%.2f") % tols_0(0,3)); boost::replace_all(txy1_0,",",".");
                string txy1_1 = boost::str(boost::format("%.2f") % tols_1(0,3)); boost::replace_all(txy1_1,",",".");
                string txy1_2 = boost::str(boost::format("%.2f") % tols_2(0,3)); boost::replace_all(txy1_2,",",".");

                PostureMod << string("param tol_target_xy1 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy1_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy1_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy1_2+string("; \n");

                // xy2
                string txy2_0 = boost::str(boost::format("%.2f") % tols_0(1,3)); boost::replace_all(txy2_0,",",".");
                string txy2_1 = boost::str(boost::format("%.2f") % tols_1(1,3)); boost::replace_all(txy2_1,",",".");
                string txy2_2 = boost::str(boost::format("%.2f") % tols_2(1,3)); boost::replace_all(txy2_2,",",".");

                PostureMod << string("param tol_target_xy2 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy2_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy2_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy2_2+string("; \n");

                // xy3
                string txy3_0 = boost::str(boost::format("%.2f") % tols_0(2,3)); boost::replace_all(txy3_0,",",".");
                string txy3_1 = boost::str(boost::format("%.2f") % tols_1(2,3)); boost::replace_all(txy3_1,",",".");
                string txy3_2 = boost::str(boost::format("%.2f") % tols_2(2,3)); boost::replace_all(txy3_2,",",".");

                PostureMod << string("param tol_target_xy3 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy3_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy3_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy3_2+string("; \n");


                // xz1
                string txz1_0 = boost::str(boost::format("%.2f") % tols_0(0,4)); boost::replace_all(txz1_0,",",".");
                string txz1_1 = boost::str(boost::format("%.2f") % tols_1(0,4)); boost::replace_all(txz1_1,",",".");
                string txz1_2 = boost::str(boost::format("%.2f") % tols_2(0,4)); boost::replace_all(txz1_2,",",".");

                PostureMod << string("param tol_target_xz1 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz1_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz1_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz1_2+string("; \n");

                // xz2
                string txz2_0 = boost::str(boost::format("%.2f") % tols_0(1,4)); boost::replace_all(txz2_0,",",".");
                string txz2_1 = boost::str(boost::format("%.2f") % tols_1(1,4)); boost::replace_all(txz2_1,",",".");
                string txz2_2 = boost::str(boost::format("%.2f") % tols_2(1,4)); boost::replace_all(txz2_2,",",".");

                PostureMod << string("param tol_target_xz2 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz2_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz2_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz2_2+string("; \n");

                // xz3
                string txz3_0 = boost::str(boost::format("%.2f") % tols_0(2,4)); boost::replace_all(txz3_0,",",".");
                string txz3_1 = boost::str(boost::format("%.2f") % tols_1(2,4)); boost::replace_all(txz3_1,",",".");
                string txz3_2 = boost::str(boost::format("%.2f") % tols_2(2,4)); boost::replace_all(txz3_2,",",".");

                PostureMod << string("param tol_target_xz3 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz3_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz3_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz3_2+string("; \n");

                // yz1
                string tyz1_0 = boost::str(boost::format("%.2f") % tols_0(0,5)); boost::replace_all(tyz1_0,",",".");
                string tyz1_1 = boost::str(boost::format("%.2f") % tols_1(0,5)); boost::replace_all(tyz1_1,",",".");
                string tyz1_2 = boost::str(boost::format("%.2f") % tols_2(0,5)); boost::replace_all(tyz1_2,",",".");

                PostureMod << string("param tol_target_yz1 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz1_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz1_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz1_2+string("; \n");

                // yz2
                string tyz2_0 = boost::str(boost::format("%.2f") % tols_0(1,5)); boost::replace_all(tyz2_0,",",".");
                string tyz2_1 = boost::str(boost::format("%.2f") % tols_1(1,5)); boost::replace_all(tyz2_1,",",".");
                string tyz2_2 = boost::str(boost::format("%.2f") % tols_2(1,5)); boost::replace_all(tyz2_2,",",".");

                PostureMod << string("param tol_target_yz2 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz2_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz2_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz2_2+string("; \n");


                // yz3
                string tyz3_0 = boost::str(boost::format("%.2f") % tols_0(2,5)); boost::replace_all(tyz3_0,",",".");
                string tyz3_1 = boost::str(boost::format("%.2f") % tols_1(2,5)); boost::replace_all(tyz3_1,",",".");
                string tyz3_2 = boost::str(boost::format("%.2f") % tols_2(2,5)); boost::replace_all(tyz3_2,",",".");

                PostureMod << string("param tol_target_yz3 {i in 1..Nsteps+1} := \n");
                PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz3_0+string("\n");
                PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz3_1+string("\n");
                PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz3_2+string("; \n");


                PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
                PostureMod << string("# \n");
                if (engage){
                    PostureMod << string("subject to target_Arm{j in 1..18, l in 1..Nsteps-2}:   \n");

                }else if (gopark && release_object){
                    PostureMod << string("subject to target_Arm{j in 1..15, l in 5..Nsteps+1}:   \n");
                }else if(!gopark && !release_object){
                    PostureMod << string("subject to target_Arm{j in 1..15, l in 1..Nsteps-2}:   \n");

                }
                if((gopark && release_object) || (!gopark && !release_object) ){
                    PostureMod << string("((Points_Arm[j,1,l]-ObjTar[1,1])^2)*( \n");
                    PostureMod << string("(x_t[1])^2 / ((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_xx1[l])^2) + \n");
                    PostureMod << string("(x_t[2])^2 / ((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_xx2[l])^2) + \n");
                    PostureMod << string("(x_t[3])^2 / ((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_xx3[l])^2)) \n");
                    PostureMod << string("+ \n");
                    PostureMod << string("((Points_Arm[j,2,l]-ObjTar[1,2])^2)*(  \n");
                    PostureMod << string("(y_t[1])^2 / ((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_yy1[l])^2) + \n");
                    PostureMod << string("(y_t[2])^2 / ((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_yy2[l])^2) + \n");
                    PostureMod << string("(y_t[3])^2 / ((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_yy3[l])^2)) \n");
                    PostureMod << string("+ \n");
                    PostureMod << string("((Points_Arm[j,3,l]-ObjTar[1,3])^2)*( \n");
                    PostureMod << string("(z_t[1])^2 / ((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_zz1[l])^2) + \n");
                    PostureMod << string("(z_t[2])^2 / ((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_zz2[l])^2) +  \n");
                    PostureMod << string("(z_t[3])^2 / ((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_zz3[l])^2)) \n");
                    PostureMod << string("+ \n");
                    PostureMod << string("2*(Points_Arm[j,1,l]-ObjTar[1,1])*(Points_Arm[j,2,l]-ObjTar[1,2])* ( \n");
                    PostureMod << string("(x_t[1]*y_t[1])/((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_xy1[l])^2) + \n");
                    PostureMod << string("(x_t[2]*y_t[2])/((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_xy2[l])^2) + \n");
                    PostureMod << string("(x_t[3]*y_t[3])/((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_xy3[l])^2)) \n");
                    PostureMod << string("+ \n");
                    PostureMod << string("2*(Points_Arm[j,1,l]-ObjTar[1,1])*(Points_Arm[j,3,l]-ObjTar[1,3])* ( \n");
                    PostureMod << string("(x_t[1]*z_t[1])/((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_xz1[l])^2) + \n");
                    PostureMod << string("(x_t[2]*z_t[2])/((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_xz2[l])^2) + \n");
                    PostureMod << string("(x_t[3]*z_t[3])/((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_xz3[l])^2)) \n");
                    PostureMod << string("+ \n");
                    PostureMod << string("2*(Points_Arm[j,2,l]-ObjTar[1,2])*(Points_Arm[j,3,l]-ObjTar[1,3])* (\n");
                    PostureMod << string("(y_t[1]*z_t[1])/((ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_yz1[l])^2) + \n");
                    PostureMod << string("(y_t[2]*z_t[2])/((ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_yz2[l])^2) + \n");
                    PostureMod << string("(y_t[3]*z_t[3])/((ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_yz3[l])^2)) \n");
                    PostureMod << string("-1 >=0; \n");
                    PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
                    PostureMod << string("# \n");
                }

           }

        }

     if(obstacle_avoidance){
        // coinstraints with the obstacles
        MatrixXd tolsObs_0 = tolsObstacles.at(0);
        MatrixXd tolsObs_1 = tolsObstacles.at(1);
        //xx1
        string tbxx1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,0)); boost::replace_all(tbxx1_0,",",".");
        string tbxx1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,0)); boost::replace_all(tbxx1_1,",",".");
        PostureMod << string("param tol_obs_xx1 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx1_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx1_1+string("; \n");

        //xx2
        string tbxx2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,0)); boost::replace_all(tbxx2_0,",",".");
        string tbxx2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,0)); boost::replace_all(tbxx2_1,",",".");

        PostureMod << string("param tol_obs_xx2 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx2_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx2_1+string("; \n");


        // xx3
        string tbxx3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,0)); boost::replace_all(tbxx3_0,",",".");
        string tbxx3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,0)); boost::replace_all(tbxx3_1,",",".");

        PostureMod << string("param tol_obs_xx3 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx3_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx3_1+string("; \n");

        // yy1
        string tbyy1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,1)); boost::replace_all(tbyy1_0,",",".");
        string tbyy1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,1)); boost::replace_all(tbyy1_1,",",".");

        PostureMod << string("param tol_obs_yy1 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy1_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy1_1+string("; \n");

        // yy2
        string tbyy2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,1)); boost::replace_all(tbyy2_0,",",".");
        string tbyy2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,1)); boost::replace_all(tbyy2_1,",",".");

        PostureMod << string("param tol_obs_yy2 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy2_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy2_1+string("; \n");


        // yy3
        string tbyy3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,1)); boost::replace_all(tbyy3_0,",",".");
        string tbyy3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,1)); boost::replace_all(tbyy3_1,",",".");

        PostureMod << string("param tol_obs_yy3 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy3_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy3_1+string("; \n");


        // zz1
        string tbzz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,2)); boost::replace_all(tbzz1_0,",",".");
        string tbzz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,2)); boost::replace_all(tbzz1_1,",",".");

        PostureMod << string("param tol_obs_zz1 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz1_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz1_1+string("; \n");

        // zz2
        string tbzz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,2)); boost::replace_all(tbzz2_0,",",".");
        string tbzz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,2)); boost::replace_all(tbzz2_1,",",".");

        PostureMod << string("param tol_obs_zz2 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz2_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz2_1+string("; \n");

        // zz3
        string tbzz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,2)); boost::replace_all(tbzz3_0,",",".");
        string tbzz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,2)); boost::replace_all(tbzz3_1,",",".");

        PostureMod << string("param tol_obs_zz3 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz3_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz3_1+string("; \n");


        // xy1
        string tbxy1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,3)); boost::replace_all(tbxy1_0,",",".");
        string tbxy1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,3)); boost::replace_all(tbxy1_1,",",".");

        PostureMod << string("param tol_obs_xy1 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy1_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy1_1+string("; \n");

        // xy2
        string tbxy2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,3)); boost::replace_all(tbxy2_0,",",".");
        string tbxy2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,3)); boost::replace_all(tbxy2_1,",",".");

        PostureMod << string("param tol_obs_xy2 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy2_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy2_1+string("; \n");

        // xy3
        string tbxy3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,3)); boost::replace_all(tbxy3_0,",",".");
        string tbxy3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,3)); boost::replace_all(tbxy3_1,",",".");

        PostureMod << string("param tol_obs_xy3 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy3_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy3_1+string("; \n");


        // xz1
        string tbxz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,4)); boost::replace_all(tbxz1_0,",",".");
        string tbxz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,4)); boost::replace_all(tbxz1_1,",",".");

        PostureMod << string("param tol_obs_xz1 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz1_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz1_1+string("; \n");

        // xz2
        string tbxz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,4)); boost::replace_all(tbxz2_0,",",".");
        string tbxz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,4)); boost::replace_all(tbxz2_1,",",".");

        PostureMod << string("param tol_obs_xz2 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz2_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz2_1+string("; \n");

        // xz3
        string tbxz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,4)); boost::replace_all(tbxz3_0,",",".");
        string tbxz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,4)); boost::replace_all(tbxz3_1,",",".");

        PostureMod << string("param tol_obs_xz3 {i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz3_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz3_1+string("; \n");

        // yz1
        string tbyz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,5)); boost::replace_all(tbyz1_0,",",".");
        string tbyz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,5)); boost::replace_all(tbyz1_1,",",".");

        PostureMod << string("param tol_obs_yz1{i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz1_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz1_1+string("; \n");

        // yz2
        string tbyz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,5)); boost::replace_all(tbyz2_0,",",".");
        string tbyz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,5)); boost::replace_all(tbyz2_1,",",".");

        PostureMod << string("param tol_obs_yz2{i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz2_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz2_1+string("; \n");


        // yz3
        string tbyz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,5)); boost::replace_all(tbyz3_0,",",".");
        string tbyz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,5)); boost::replace_all(tbyz3_1,",",".");

        PostureMod << string("param tol_obs_yz3{i in 1..Nsteps+1} :=  \n");
        PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz3_0+string("\n");
        PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz3_1+string("; \n");

        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("# \n");
        if(engage || transport){
             PostureMod << string("subject to obst_Arm{j in 1..18, i in 1..n_Obstacles, l in 1..Nsteps+1}:\n");
        }else if(gopark){
            PostureMod << string("subject to obst_Arm{j in 1..15, i in 1..(n_Obstacles), l in 1..Nsteps+1}:\n");
        }else{
             PostureMod << string("subject to obst_Arm{j in 1..15, i in 1..(n_Obstacles), l in 1..Nsteps+1}:\n");
        }
        PostureMod << string("((Points_Arm[j,1,l]-Obstacles[i,1])^2)*(\n");
        PostureMod << string("(Rot[1,1,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_xx1[l])^2) +\n");
        PostureMod << string("(Rot[2,1,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_xx2[l])^2) + \n");
        PostureMod << string("(Rot[3,1,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_xx3[l])^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm[j,2,l]-Obstacles[i,2])^2)*( \n");
        PostureMod << string("(Rot[1,2,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_yy1[l])^2) + \n");
        PostureMod << string("(Rot[2,2,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_yy2[l])^2) + \n");
        PostureMod << string("(Rot[3,2,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_yy3[l])^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm[j,3,l]-Obstacles[i,3])^2)*( \n");
        PostureMod << string("(Rot[1,3,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_zz1[l])^2) + \n");
        PostureMod << string("(Rot[2,3,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_zz2[l])^2) + \n");
        PostureMod << string("(Rot[3,3,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_zz3[l])^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,1,l]-Obstacles[i,1])*(Points_Arm[j,2,l]-Obstacles[i,2])* ( \n");
        PostureMod << string("(Rot[1,1,i]*Rot[1,2,i])/((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_xy1[l])^2) + \n");
        PostureMod << string("(Rot[2,1,i]*Rot[2,2,i])/((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_xy2[l])^2) + \n");
        PostureMod << string("(Rot[3,1,i]*Rot[3,2,i])/((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_xy3[l])^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,1,l]-Obstacles[i,1])*(Points_Arm[j,3,l]-Obstacles[i,3])* ( \n");
        PostureMod << string("(Rot[1,1,i]*Rot[1,3,i])/((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_xz1[l])^2) + \n");
        PostureMod << string("(Rot[2,1,i]*Rot[2,3,i])/((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_xz2[l])^2) + \n");
        PostureMod << string("(Rot[3,1,i]*Rot[3,3,i])/((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_xz3[l])^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,2,l]-Obstacles[i,2])*(Points_Arm[j,3,l]-Obstacles[i,3])* ( \n");
        PostureMod << string("(Rot[1,2,i]*Rot[1,3,i])/((Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_yz1[l])^2) + \n");
        PostureMod << string("(Rot[2,2,i]*Rot[2,3,i])/((Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_yz2[l])^2) + \n");
        PostureMod << string("(Rot[3,2,i]*Rot[3,3,i])/((Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_yz3[l])^2)) \n");
        PostureMod << string("-1 >=0; \n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
        PostureMod << string("# \n");

     }
     // constraints with the body
     this->writeBodyConstraints(PostureMod,false);

     // constraints with the table
     //this->writeTableConstraints(PostureMod,false,griptype,tols_table);

     PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n\n\n");

     // close the file
     PostureMod.close();

     // ----- write run file for options --------
     string filenamerun("options.run");
     ofstream optionsrun;
     // open the file
     optionsrun.open(path+filenamerun);

     optionsrun << string("option presolve 0; \n");


     //close the file
     optionsrun.close();

     return true;

}



void HUMPlanner::getObstaclesSingleArm(std::vector<double> center, double radius, std::vector<objectPtr> &obsts, int hand_code)
{
    double tol;
    switch (hand_code){
    case 0: // human hand
        tol = 50.0;
        break;
    case 1://barrett hand
        tol = 5;
        break;
    }

    for (std::size_t i =0; i < this->obstacles.size(); ++i){
        objectPtr obj = obstacles.at(i);
        // get the RPY matrix
        Matrix3d Rot; std::vector<double> rpy; obj->getOr(rpy); this->RPY_matrix(rpy,Rot);
        // get the position of the object
        std::vector<double> pos; obj->getPos(pos);
        // get the size of the object
        std::vector<double> dim; obj->getSize(dim);
        // distance vector
        Vector3d diff;
        diff(0) = center.at(0) - pos.at(0);
        diff(1) = center.at(1) - pos.at(1);
        diff(2) = center.at(2) - pos.at(2);
        // A matrix
        Matrix3d A;
        A(0,0) = 1/pow(radius+dim.at(0)+tol,2); A(0,1) = 0; A(0,2) = 0;
        A(1,0) = 0; A(1,1) = 1/pow(radius+dim.at(1)+tol,2); A(1,2) = 0;
        A(2,0) = 0; A(2,1) = 0; A(2,2) = 1/pow(radius+dim.at(2)+tol,2);

        MatrixXd diff1 = MatrixXd::Zero(3,1);
        diff1(0,0) = diff(0); diff1(1,0) = diff(1); diff1(2,0) = diff(2);

        MatrixXd diffT = diff1.transpose();
        MatrixXd RotT = Rot.transpose();

        MatrixXd to_check = diffT*RotT;
        to_check = to_check * A;
        to_check = to_check * Rot;
        to_check = to_check * diff;

        if (to_check(0,0) < 1){
            // the object is a real obstacle
            obsts.push_back(obj);
        }

    }

}

/*
std::string HUMPlanner::exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}
*/

bool HUMPlanner::amplRead(string &datFile, string &modFile, string &nlFile)
{
    string cmdLine;

#if AMPL==0
    cmdLine = string("wine ")+AMPL_PATH+string("/ampl.exe -ogModels/")+nlFile+string(" Models/")+modFile+string(".mod")+
            string(" Models/")+datFile+string(".dat")+string(" Models/options.run");
#elif AMPL==1
    cmdLine = AMPL_PATH+string("/./ampl -ogModels/")+nlFile+string(" Models/")+modFile+string(".mod")+
            string(" Models/")+datFile+string(".dat")+string(" Models/options.run");
#endif
//#ifdef DEBUG
    int status = system(cmdLine.c_str());
    return(status==0);
//#else
//    std::string result = this->exec(cmdLine.c_str());
//    bool status = (std::strcmp(result.c_str(),"")==0);
//    return (status);
//#endif
}

bool HUMPlanner::optimize(string &nlfile, std::vector<Number> &x, double tol, double acc_tol)
{
    // Create a new instance of IpoptApplication
    //  (use a SmartPtr, not raw)
    // We are using the factory, since this allows us to compile this
    // example with an Ipopt Windows DLL
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    app->RethrowNonIpoptException(true);

    app->Options()->SetNumericValue("tol", tol);
    app->Options()->SetNumericValue("acceptable_tol", acc_tol);
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetIntegerValue("print_level",3);

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
      return (int) status;
    }

    // Add the suffix handler for scaling
    SmartPtr<AmplSuffixHandler> suffix_handler = new AmplSuffixHandler();
    suffix_handler->AddAvailableSuffix("scaling_factor", AmplSuffixHandler::Variable_Source, AmplSuffixHandler::Number_Type);
    suffix_handler->AddAvailableSuffix("scaling_factor", AmplSuffixHandler::Constraint_Source, AmplSuffixHandler::Number_Type);
    suffix_handler->AddAvailableSuffix("scaling_factor", AmplSuffixHandler::Objective_Source, AmplSuffixHandler::Number_Type);
    // Modified for warm-start from AMPL
    suffix_handler->AddAvailableSuffix("ipopt_zL_out", AmplSuffixHandler::Variable_Source, AmplSuffixHandler::Number_Type);
    suffix_handler->AddAvailableSuffix("ipopt_zU_out", AmplSuffixHandler::Variable_Source, AmplSuffixHandler::Number_Type);
    suffix_handler->AddAvailableSuffix("ipopt_zL_in", AmplSuffixHandler::Variable_Source, AmplSuffixHandler::Number_Type);
    suffix_handler->AddAvailableSuffix("ipopt_zU_in", AmplSuffixHandler::Variable_Source, AmplSuffixHandler::Number_Type);

    char *cstr = new char[nlfile.length() + 1];
    strcpy(cstr, nlfile.c_str());

    SmartPtr<AmplInterface> ampl_tnlp = new AmplInterface(ConstPtr(app->Jnlst()),
                                            app->Options(),
                                            cstr, suffix_handler);


    delete [] cstr;

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(ampl_tnlp);
    std::vector<Number> x_sol;
    std::vector<Number> z_L_sol;
    std::vector<Number> z_U_sol;
    std::vector<Number> lambda_sol;
    Number obj_sol;

    if (ampl_tnlp->get_status() == SolverReturn::SUCCESS){
        ampl_tnlp->get_solutions(x_sol,
                                 z_L_sol,
                                 z_U_sol,
                                 lambda_sol,
                                 obj_sol);

        x=x_sol;

        return true;

    }else{
        x=x_sol;

        return false;

    }
}


bool HUMPlanner::singleArmFinalPosture(int mov_type,int pre_post,huml_params& params, std::vector<double> initPosture, std::vector<double>& finalPosture)

{
    // movement settings
    int arm_code = params.mov_specs.arm_code;
    int hand_code = params.mov_specs.hand_code;
    std::vector<double> target = params.mov_specs.target;

    std::vector<double> initArmPosture(initPosture.begin(),initPosture.begin()+joints_arm);

    // initial guess choice
    std::vector<double> initialGuess = initArmPosture; // arbitrary choice

    double Lu; double Ll; double Lh;
    switch(arm_code){
    case 0: // dual arm
        //TO DO
        break;
    case 1: // right arm
        Lu = this->DH_rightArm.d.at(2);
        Ll = this->DH_rightArm.d.at(4);
        Lh = this->DH_rightArm.d.at(6);
        break;
    case 2: // left arm
        Lu = this->DH_leftArm.d.at(2);
        Ll = this->DH_leftArm.d.at(4);
        Lh = this->DH_leftArm.d.at(6);
        break;
    }
    double max_ext = Lh+Ll+Lu;
    if(sqrt(pow(target.at(0) -this->shPos.at(0),2)+
            pow(target.at(1) -this->shPos.at(1),2)+
            pow(target.at(2) -this->shPos.at(2),2))>= max_ext){
        throw string("The object to grasp is too far away");
    }
    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    this->getObstaclesSingleArm(this->shPos,max_ext,obsts,hand_code);

    // write the files for the final posture selection
    bool written = this->writeFilesFinalPosture(params,mov_type,pre_post,initArmPosture,initialGuess,obsts);

    if(written){
        // call AMPL the produce the .nl file
        string fn = string("FinalPosture");
        bool nlwritten = this->amplRead(fn,fn,fn);
        if(nlwritten){
            // call ipopt for optimization
            string nlfile = string("Models/")+fn+string(".nl");
            std::vector<Number> x_sol;
            try
            {
                double tol_stop = 1e-2;
                if (this->optimize(nlfile,x_sol,tol_stop,tol_stop)){
                    finalPosture = std::vector<double>(x_sol.size());
                    for (std::size_t i=0; i < x_sol.size(); ++i){
                        finalPosture.at(i)=x_sol[i];
                    }
                    return true;
                }else{return false;}
            }catch(const std::exception &exc){throw string(exc.what());}
        }else{throw string("Error in reading the files for optimization");}
    }else{throw string("Error in writing the files for optimization");}

}


bool HUMPlanner::singleArmBouncePosture(int mov_type,int pre_post,huml_params& params,std::vector<double> initPosture,std::vector<double> finalPosture,std::vector<double>& bouncePosture)
{


    std::vector<double> initialGuess;
    std::vector<double> minLimits;
    std::vector<double> maxLimits;
    DHparameters dh;
    // movement settings
    int arm_code = params.mov_specs.arm_code;
    int hand_code = params.mov_specs.hand_code;
    std::vector<double> finalHand = params.mov_specs.finalHand;
    //tolerances
    boundaryConditions b = params.bounds;
    boundaryConditions bAux;
    std::vector<double> lambda = params.lambda_bounce;

    switch(arm_code){
    case 1: // right arm
        minLimits = this->minRightLimits;
        maxLimits = this->maxRightLimits;
        dh = this->DH_rightArm;
        break;
    case 2: // left arm
        minLimits = this->minLeftLimits;
        maxLimits = this->maxLeftLimits;
        dh = this->DH_leftArm;
        break;
    }
    std::vector<double> initAuxPosture(initPosture.begin(),initPosture.begin()+joints_arm);
    std::vector<double> finalAuxPosture(finalPosture.begin(),finalPosture.begin()+joints_arm);
    std::vector<double> minAuxLimits(minLimits.begin(),minLimits.begin()+joints_arm);
    std::vector<double> maxAuxLimits(maxLimits.begin(),maxLimits.begin()+joints_arm);
    std::vector<double> lambdaAux(lambda.begin(),lambda.begin()+joints_arm);
    std::vector<double> vel0Aux(b.vel_0.begin(),b.vel_0.begin()+joints_arm);
    std::vector<double> velfAux(b.vel_f.begin(),b.vel_f.begin()+joints_arm);
    std::vector<double> acc0Aux(b.acc_0.begin(),b.acc_0.begin()+joints_arm);
    std::vector<double> accfAux(b.acc_f.begin(),b.acc_f.begin()+joints_arm);
    switch(hand_code){
    case 0:// human hand
        initAuxPosture.push_back(initPosture.at(7));
        initAuxPosture.push_back(initPosture.at(8));
        initAuxPosture.push_back(initPosture.at(10));
        finalAuxPosture.push_back(finalHand.at(0));
        finalAuxPosture.push_back(finalHand.at(1));
        finalAuxPosture.push_back(finalHand.at(3));
        minAuxLimits.push_back(minLimits.at(7));
        minAuxLimits.push_back(minLimits.at(8));
        minAuxLimits.push_back(minLimits.at(10));
        maxAuxLimits.push_back(maxLimits.at(7));
        maxAuxLimits.push_back(maxLimits.at(8));
        maxAuxLimits.push_back(maxLimits.at(10));
        lambdaAux.push_back(lambda.at(7));
        lambdaAux.push_back(lambda.at(8));
        lambdaAux.push_back(lambda.at(10));
        vel0Aux.push_back(b.vel_0.at(7));
        vel0Aux.push_back(b.vel_0.at(8));
        vel0Aux.push_back(b.vel_0.at(10));
        velfAux.push_back(b.vel_f.at(7));
        velfAux.push_back(b.vel_f.at(8));
        velfAux.push_back(b.vel_f.at(10));
        acc0Aux.push_back(b.acc_0.at(7));
        acc0Aux.push_back(b.acc_0.at(8));
        acc0Aux.push_back(b.acc_0.at(10));
        accfAux.push_back(b.acc_f.at(7));
        accfAux.push_back(b.acc_f.at(8));
        accfAux.push_back(b.acc_f.at(10));
        break;
    case 1:// barrett hand
        initAuxPosture.push_back(initPosture.at(8));
        initAuxPosture.push_back(initPosture.at(10));
        finalAuxPosture.push_back(finalHand.at(1));
        finalAuxPosture.push_back(finalHand.at(3));
        minAuxLimits.push_back(minLimits.at(8));
        minAuxLimits.push_back(minLimits.at(10));
        maxAuxLimits.push_back(maxLimits.at(8));
        maxAuxLimits.push_back(maxLimits.at(10));
        lambdaAux.push_back(lambda.at(8));
        lambdaAux.push_back(lambda.at(10));
        vel0Aux.push_back(b.vel_0.at(8));
        vel0Aux.push_back(b.vel_0.at(10));
        velfAux.push_back(b.vel_f.at(8));
        velfAux.push_back(b.vel_f.at(10));
        acc0Aux.push_back(b.acc_0.at(8));
        acc0Aux.push_back(b.acc_0.at(10));
        accfAux.push_back(b.acc_f.at(8));
        accfAux.push_back(b.acc_f.at(10));
        break;
    }
    bAux.vel_0=vel0Aux;
    bAux.vel_f=velfAux;
    bAux.acc_0=acc0Aux;
    bAux.acc_f=accfAux;
    // initial guess choice
    initialGuess = initAuxPosture;

    double Lu = dh.d.at(2);
    double Ll = dh.d.at(4);
    double Lh = dh.d.at(6);
    double max_ext = Lh+Ll+Lu;
    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    this->getObstaclesSingleArm(this->shPos,max_ext,obsts,hand_code);

    bool written = this->writeFilesBouncePosture(params,mov_type,pre_post,minAuxLimits,maxAuxLimits,initAuxPosture,finalAuxPosture,initialGuess,obsts,bAux);

    if (written){
        // call AMPL the produce the .nl file
        string fn = string("BouncePosture");
        bool nlwritten = this->amplRead(fn,fn,fn);
        if(nlwritten){
            // call ipopt for optimization
            string nlfile = string("Models/")+fn+string(".nl");
            std::vector<Number> x_sol;
            try
            {
                double tol_stop = 1e-6;
                if (this->optimize(nlfile,x_sol,tol_stop,tol_stop)){
                    bouncePosture = std::vector<double>(joints_arm+joints_hand);
                    for (std::size_t i=0; i < x_sol.size()-2; ++i){
                       // printf("x[%i] = %f\n", i, x_sol[i]);
                        //switch(arm_code){
                        //case 1: //right arm
                            //this->rightBouncePosture.at(i) = x_sol[i];
                            //break;
                        //case 2: // left arm
                            //this->leftBouncePosture.at(i) = x_sol[i];
                            //break;
                        //}
                        bouncePosture.at(i) = x_sol[i];

                    }
                    switch(hand_code){
                    case 0://human hand
                        bouncePosture.at(7) = x_sol[7];
                        bouncePosture.at(8) = x_sol[8];
                        bouncePosture.at(9) = x_sol[8];
                        bouncePosture.at(10) = x_sol[9];
                        break;
                    case 1:// barrett hand
                        bouncePosture.at(7) = 0.0;
                        bouncePosture.at(8) = x_sol[8];
                        bouncePosture.at(9) = x_sol[8];
                        bouncePosture.at(10) = x_sol[7];
                        break;

                    }
                    return true;
                }else{return false;}
            }catch(const std::exception &exc){throw string(exc.what());}
        }else{throw string("Error in writing the files for optimization");}
    }else{throw string("Error in writing the files for optimization");}


}


double HUMPlanner::getTimeStep(huml_params &tols, MatrixXd &jointTraj)
{
    int steps1 = jointTraj.rows();
    int n_joints = jointTraj.cols();
    double timestep;

    std::vector<double> w_max = tols.w_max;
    std::vector<double> lambda = tols.lambda_bounce;


    double num = 0;
    double den = 0;

    for (int k =0; k < n_joints; ++k){

        double time_k;
        double deltaTheta = 0;
        std::vector<double> diffs;
        for (int i = 1; i < steps1; ++i){
            double diff = abs(jointTraj(i,k) - jointTraj(i-1,k))*180/M_PI;
            diffs.push_back(diff);
            deltaTheta = deltaTheta + diff;
        }
        std::vector<double>::iterator res = std::max_element(diffs.begin(),diffs.end());
        int poss = std::distance(diffs.begin(),res);
        double deltaThetaMax = diffs.at(poss);
        double tol = -3.0; // tolerance in [deg/sec]
        time_k = ((steps1-1)*deltaThetaMax/(w_max.at(k)+tol)) + (lambda.at(k)*log(1+deltaTheta));

        num = num + lambda.at(k)*deltaTheta*time_k;
        den = den + lambda.at(k)*deltaTheta;

    }

    double totalTime = num/den;
    timestep = (totalTime/(steps1-1));
    return timestep;
}

/*
void HUMPlanner::getDelta(VectorXd& jointTraj, std::vector<double> &delta)
{
    // Formula of the numarical differentiation with 5 points
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)

       delta = std::vector<double>(jointTraj.size());

       int h = 1;
       int tnsample;
       double f0;
       double f1;
       double f2;
       double f3;
       double f4;

       // 1st point
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       tnsample = 0;
       f0 = jointTraj(tnsample);
       f1 = jointTraj(tnsample+1);
       f2 = jointTraj(tnsample+2);
       f3 = jointTraj(tnsample+3);
       f4 = jointTraj(tnsample+4);
       delta.at(tnsample) = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h);

       // 2nd point
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       tnsample = 1;
       f0 = jointTraj(tnsample-1);
       f1 = jointTraj(tnsample);
       f2 = jointTraj(tnsample+1);
       f3 = jointTraj(tnsample+2);
       f4 = jointTraj(tnsample+3);
       delta.at(tnsample) = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h);

       // 3rd point
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       for (int i=2; i< jointTraj.size() -2;++i){     // centered
           f0 = jointTraj(i-2);
           f1 = jointTraj(i-1);
           f2 = jointTraj(i);
           f3 = jointTraj(i+1);
           f4 = jointTraj(i+2);
           delta.at(i) = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h);
       }

       // 4th point
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       tnsample = jointTraj.size()-2;
       f0 = jointTraj(tnsample-3);
       f1 = jointTraj(tnsample-2);
       f2 = jointTraj(tnsample-1);
       f3 = jointTraj(tnsample);
       f4 = jointTraj(tnsample+1);
       delta.at(tnsample) = ( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h);

       // 5th point
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)
       tnsample = jointTraj.size()-1;
       f0 = jointTraj(tnsample-4);
       f1 = jointTraj(tnsample-3);
       f2 = jointTraj(tnsample-2);
       f3 = jointTraj(tnsample-1);
       f4 = jointTraj(tnsample);
       delta.at(tnsample) = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h);

}

*/
void HUMPlanner::directTrajectory(huml_params &tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, MatrixXd &Traj, int mod)
{
    int steps = tols.steps;
    std::vector<double> time = std::vector<double>(steps+1); // time
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time
    std::vector<double> vel_0;
    std::vector<double> vel_f;
    std::vector<double> acc_0;
    std::vector<double> acc_f;

    switch(mod){
    case 0: // move
        vel_0 = tols.bounds.vel_0;
        vel_f = tols.bounds.vel_f;
        acc_0 = tols.bounds.acc_0;
        acc_f = tols.bounds.acc_f;
        break;
    case 1:// pre_approach
        vel_0 = tols.bounds.vel_0;
        vel_f = tols.vel_approach;
        acc_0 = tols.bounds.acc_0;
        acc_f = tols.acc_approach;
        break;
    case 2: // approach
        vel_0 = tols.vel_approach;
        vel_f = std::vector<double>(tols.vel_approach.size(),0.0);
        acc_0 = tols.acc_approach;
        acc_f = std::vector<double>(tols.acc_approach.size(),0.0);
        steps = 5;
        break;
    case 3:// retreat
        vel_0 = std::vector<double>(tols.bounds.vel_0.size(),0.0);
        vel_f = tols.bounds.vel_f;
        acc_0 = std::vector<double>(tols.bounds.acc_0.size(),0.0);
        acc_f = tols.bounds.acc_f;
        steps = 5;
        break;
    }



    double T = tols.totalTime;
    double delta = T/steps;

    time.at(0)=0.0; tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        time.at(i) = time.at(i-1)+delta;
        tau.at(i) = time.at(i)/T;
    }
    Traj = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i <= steps;++i){
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            Traj(i,j) = initPosture.at(j) +
                    (finalPosture.at(j) - initPosture.at(j))*
                    (10*pow(tau.at(i),3)-15*pow(tau.at(i),4)+6*pow(tau.at(i),5))+
                    vel_0.at(j)*T*(tau.at(i)-6*pow(tau.at(i),3)+8*pow(tau.at(i),4)-3*pow(tau.at(i),5))+
                    vel_f.at(j)*T*(-4*pow(tau.at(i),3)+7*pow(tau.at(i),4)-3*pow(tau.at(i),5))+
                    0.5*acc_0.at(j)*pow(T,2)*(pow(tau.at(i),2)-3*pow(tau.at(i),3)+3*pow(tau.at(i),4)-pow(tau.at(i),5))+
                    0.5*acc_f.at(j)*pow(T,2)*(pow(tau.at(i),3)-2*pow(tau.at(i),4)+pow(tau.at(i),5));

        }
    }

}

void HUMPlanner::directVelocity(huml_params &tols, std::vector<double> &initPosture, std::vector<double> &finalPosture, MatrixXd &Vel, int mod)
{
    int steps = tols.steps;
    std::vector<double> time = std::vector<double>(steps+1); // time
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time
    std::vector<double> vel_0;
    std::vector<double> vel_f;
    std::vector<double> acc_0;
    std::vector<double> acc_f;

    switch(mod){
    case 0: // move
        vel_0 = tols.bounds.vel_0;
        vel_f = tols.bounds.vel_f;
        acc_0 = tols.bounds.acc_0;
        acc_f = tols.bounds.acc_f;
        break;
    case 1:// pre_approach
        vel_0 = tols.bounds.vel_0;
        vel_f = tols.vel_approach;
        acc_0 = tols.bounds.acc_0;
        acc_f = tols.acc_approach;
        break;
    case 2: // approach
        vel_0 = tols.vel_approach;
        vel_f = std::vector<double>(tols.vel_approach.size(),0.0);
        acc_0 = tols.acc_approach;
        acc_f = std::vector<double>(tols.acc_approach.size(),0.0);
        steps = 5;
        break;
    case 3:// retreat
        vel_0 = std::vector<double>(tols.bounds.vel_0.size(),0.0);
        vel_f = tols.bounds.vel_f;
        acc_0 = std::vector<double>(tols.bounds.acc_0.size(),0.0);
        acc_f = tols.bounds.acc_f;
        steps = 5;
        break;
    }



    double T = tols.totalTime;
    double delta = T/steps;

    time.at(0)=0.0; tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        time.at(i) = time.at(i-1)+delta;
        tau.at(i) = time.at(i)/T;
    }
    Vel = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i <= steps;++i){
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            Vel(i,j) = (30/T)*(finalPosture.at(j) - initPosture.at(j))*
                    (pow(tau.at(i),2)-2*pow(tau.at(i),3)+pow(tau.at(i),4))+
                    vel_0.at(j)*(1-18*pow(tau.at(i),2)+32*pow(tau.at(i),3)-15*pow(tau.at(i),4))+
                    vel_f.at(j)*(-12*pow(tau.at(i),2)+28*pow(tau.at(i),3)-15*pow(tau.at(i),4))+
                    0.5*acc_0.at(j)*T*(2*tau.at(i)-9*pow(tau.at(i),2)+12*pow(tau.at(i),3)-5*pow(tau.at(i),4))+
                    0.5*acc_f.at(j)*T*(3*pow(tau.at(i),2)-8*pow(tau.at(i),3)+5*pow(tau.at(i),4));

        }
    }


}

void HUMPlanner::directAcceleration(huml_params &tols, std::vector<double> &initPosture, std::vector<double> &finalPosture, MatrixXd &Acc, int mod)
{
    int steps = tols.steps;
    std::vector<double> time = std::vector<double>(steps+1); // time
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time
    std::vector<double> vel_0;
    std::vector<double> vel_f;
    std::vector<double> acc_0;
    std::vector<double> acc_f;

    switch(mod){
    case 0: // move
        vel_0 = tols.bounds.vel_0;
        vel_f = tols.bounds.vel_f;
        acc_0 = tols.bounds.acc_0;
        acc_f = tols.bounds.acc_f;
        break;
    case 1:// pre_approach
        vel_0 = tols.bounds.vel_0;
        vel_f = tols.vel_approach;
        acc_0 = tols.bounds.acc_0;
        acc_f = tols.acc_approach;
        break;
    case 2: // approach
        vel_0 = tols.vel_approach;
        vel_f = std::vector<double>(tols.vel_approach.size(),0.0);
        acc_0 = tols.acc_approach;
        acc_f = std::vector<double>(tols.acc_approach.size(),0.0);
        steps = 5;
        break;
    case 3:// retreat
        vel_0 = std::vector<double>(tols.bounds.vel_0.size(),0.0);
        vel_f = tols.bounds.vel_f;
        acc_0 = std::vector<double>(tols.bounds.acc_0.size(),0.0);
        acc_f = tols.bounds.acc_f;
        steps = 5;
        break;
    }



    double T = tols.totalTime;
    double delta = T/steps;

    time.at(0)=0.0; tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        time.at(i) = time.at(i-1)+delta;
        tau.at(i) = time.at(i)/T;
    }
    Acc = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i <= steps;++i){
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            Acc(i,j) = (60/pow(T,2))*(finalPosture.at(j) - initPosture.at(j))*
                    (tau.at(i)-3*pow(tau.at(i),2)+2*pow(tau.at(i),3))+
                    (12*vel_0.at(j)/T)*(-3*tau.at(i)+8*pow(tau.at(i),2)-5*pow(tau.at(i),3))+
                    (12*vel_f.at(j)/T)*(-2*tau.at(i)+7*pow(tau.at(i),2)-5*pow(tau.at(i),3))+
                    acc_0.at(j)*(1-9*tau.at(i)+18*pow(tau.at(i),2)-10*pow(tau.at(i),3))+
                    acc_f.at(j)*(3*tau.at(i)-12*pow(tau.at(i),2)+10*pow(tau.at(i),3));

        }
    }
}

void HUMPlanner::backForthTrajectory(huml_params &tols, std::vector<double> &initPosture, std::vector<double> &bouncePosture, MatrixXd &Traj)
{

    int steps = tols.steps;
    std::vector<double> time = std::vector<double>(steps+1); // time
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time


    double T = tols.totalTime;
    double delta = T/steps;

    time.at(0)=0.0; tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        time.at(i) = time.at(i-1)+delta;
        tau.at(i) = time.at(i)/T;
    }
    Traj = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i<=steps;++i){
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            double ttau = tau.at(i);
            Traj(i,j) = ((ttau*(1-ttau))/(TB*(1-TB)))*(bouncePosture.at(j) - initPosture.at(j))* pow(sin(M_PI*pow(ttau,PHI)),2);
        }
    }
}

void HUMPlanner::backForthVelocity(huml_params &tols, std::vector<double> &initPosture, std::vector<double> &bouncePosture, MatrixXd &Vel)
{
    int steps = tols.steps;
    std::vector<double> time = std::vector<double>(steps+1); // time
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time


    double T = tols.totalTime;
    double delta = T/steps;

    time.at(0)=0.0; tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        time.at(i) = time.at(i-1)+delta;
        tau.at(i) = time.at(i)/T;
    }
    Vel = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i<=steps;++i){
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            double ttau = tau.at(i);
            Vel(i,j) = ((ttau*(1-ttau))/(TB*(1-TB)))*(bouncePosture.at(j) - initPosture.at(j))*((1-2*ttau)*pow(sin(M_PI*pow(ttau,PHI)),2)+
                                                                                                (1-ttau)*M_PI*PHI*pow(ttau,PHI)*sin(2*M_PI*pow(ttau,PHI)));
        }
    }
}

void HUMPlanner::backForthAcceleration(huml_params &tols, std::vector<double> &initPosture, std::vector<double> &bouncePosture, MatrixXd &Acc)
{
    int steps = tols.steps;
    std::vector<double> time = std::vector<double>(steps+1); // time
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time


    double T = tols.totalTime;
    double delta = T/steps;

    time.at(0)=0.0; tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        time.at(i) = time.at(i-1)+delta;
        tau.at(i) = time.at(i)/T;
    }
    Acc = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i<=steps;++i){
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            double ttau = tau.at(i);
            Acc(i,j) = ((ttau*(1-ttau))/(TB*(1-TB)))*(bouncePosture.at(j) - initPosture.at(j))*(2*pow(M_PI,2)*pow(PHI,2)*pow(ttau,(2*PHI-1))*(1-ttau)*cos(2*M_PI*pow(ttau,PHI))
                                                                                                -2*pow(sin(M_PI*pow(ttau,2)),2)
                                                                                                -2*M_PI*PHI*pow(ttau,(PHI-1))*(2*ttau-1)*sin(2*M_PI*pow(ttau,PHI)));
        }
    }
}

void HUMPlanner::computeMovement(const MatrixXd &direct, const MatrixXd &back, MatrixXd& tot)
{
    tot = MatrixXd::Constant(direct.rows(),direct.cols(),0);

    for (int i = 0; i<direct.rows();++i){
        for (int j = 0; j<direct.cols(); ++j){
            tot(i,j) = direct(i,j)+back(i,j);
        }
    }
}

double HUMPlanner::getTrajectory(huml_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj,int mod)
{
    double timestep;
    this->directTrajectory(tols,initPosture,finalPosture,traj,mod);
    timestep = this->getTimeStep(tols,traj);

    return timestep;
}

double HUMPlanner::getTrajectory(huml_params &tols,std::vector<double> initPosture,
                                 std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj,int mod)
{

    double timestep;
    MatrixXd dTraj;
    MatrixXd bTraj;

    this->directTrajectory(tols,initPosture,finalPosture,dTraj,mod);
    this->backForthTrajectory(tols,initPosture,bouncePosture,bTraj);
    this->computeMovement(dTraj,bTraj,traj);
    timestep = this->getTimeStep(tols,traj);

    /*
    switch (mov_type) {
        case 0: // pick
            this->directMovement(tols,initPosture,finalPosture,dTraj);
            this->backForthMovement(tols,initPosture,bouncePosture,bTraj);
            this->computeTraj(dTraj,bTraj,totTraj);
            timestep = this->getTimeStep(tols,traj);
            break;
        case 1: // reaching
            break;
        case 2: // transport
            break;
        case 3: // engage
*/
    /*
            switch(arm_code){

            case 0: // both arms

                // TO DO
                break;

            case 1: // right arm
                this->scene->getHumanoid()->getRightPosture(initPosture);
                finalPosture=this->rightFinalPosture;
                fPosture_diseng=this->rightFinalPosture_diseng;
                fPosture_eng=this->rightFinalPosture_eng;
                bouncePosture=this->rightBouncePosture;
                break;
            case 2: // left arm
                this->scene->getHumanoid()->getLeftPosture(initPosture);
                finalPosture=this->leftFinalPosture;
                fPosture_diseng=this->leftFinalPosture_diseng;
                fPosture_eng=this->leftFinalPosture_eng;
                bouncePosture=this->leftBouncePosture;
                break;
            }

            this->directMovement(initPosture,fPosture_diseng,4,dTraj_diseng);
            this->directMovement(fPosture_diseng,fPosture_eng,steps,dTraj);
            this->backForthMovement(fPosture_diseng,bouncePosture,steps,bTraj);
            this->computeTraj(dTraj,bTraj);
            this->directMovement(fPosture_eng,finalPosture,4,dTraj_eng);

            // compose the trajectory
            totTraj.resize(this->optimalTraj.rows()+10,this->optimalTraj.cols());

            totTraj.row(0) = dTraj_diseng.row(0);
            totTraj.row(1) = dTraj_diseng.row(1);
            totTraj.row(2) = dTraj_diseng.row(2);
            totTraj.row(3) = dTraj_diseng.row(3);
            totTraj.row(4) = dTraj_diseng.row(4);

            for(int i =0; i < this->optimalTraj.rows(); ++i){
                totTraj.row(i+5) = this->optimalTraj.row(i);
            }

            totTraj.row(totTraj.rows()-5) = dTraj_eng.row(0);
            totTraj.row(totTraj.rows()-4) = dTraj_eng.row(1);
            totTraj.row(totTraj.rows()-3) = dTraj_eng.row(2);
            totTraj.row(totTraj.rows()-2) = dTraj_eng.row(3);
            totTraj.row(totTraj.rows()-1) = dTraj_eng.row(4);

            this->optimalTraj=totTraj;
            traj = this->optimalTraj;

            this->getTimeStep(traj,timeStep);

            break;

        case 4: // disengage
            break;
        case 5: // Go park

            if(std::strcmp(obj_tar->getName().c_str(),"")!=0){
                // an object has to be released
                initPosture.at(7) = 0.0;
                initPosture.at(8) = 0.0;
                initPosture.at(9) = 0.0;
                initPosture.at(10) = 0.0;
            }
            this->directMovement(tols,initPosture,finalPosture,dTraj);
            this->backForthMovement(tols,initPosture,bouncePosture,bTraj);
            this->computeTraj(dTraj,bTraj,totTraj);
            timestep = this->getTimeStep(tols,traj);

            break;

        }
         */
    return timestep;
}

double HUMPlanner::getVelocity(huml_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, int mod)
{

    double timestep = this->getTrajectory(tols,initPosture,finalPosture,traj,mod);

    this->directVelocity(tols,initPosture,finalPosture,vel,mod);

    return timestep;

    /*
    double timestep = this->getTrajectory(tols,initPosture,finalPosture,traj,mod);
    vel.resize(traj.rows(),traj.cols());

    std::vector<double> delta;
    VectorXd posture;
    for (int k = 0; k < traj.cols(); ++k){
        posture = traj.col(k);
        this->getDelta(posture,delta);
        for (int i =0; i < traj.rows(); ++i){
            vel(i,k) = delta.at(i)/timestep;
        }
    }

    return timestep;

    */
}

double HUMPlanner::getVelocity(huml_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj, MatrixXd &vel,int mod)
{

    double timestep = this->getTrajectory(tols,initPosture,finalPosture,bouncePosture,traj,mod);

    MatrixXd dVel;
    MatrixXd bVel;

    this->directVelocity(tols,initPosture,finalPosture,dVel,mod);
    this->backForthVelocity(tols,initPosture,bouncePosture,bVel);
    this->computeMovement(dVel,bVel,vel);

    return timestep;

    /*
    double timestep = this->getTrajectory(tols,initPosture,finalPosture,bouncePosture,traj,mod);
    vel.resize(traj.rows(),traj.cols());

    std::vector<double> delta;
    VectorXd posture;
    for (int k = 0; k < traj.cols(); ++k){
        posture = traj.col(k);
        this->getDelta(posture,delta);
        for (int i =0; i < traj.rows(); ++i){
            vel(i,k) = delta.at(i)/timestep;
        }
    }

    return timestep;

    */
}

double HUMPlanner::getAcceleration(huml_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, int mod)
{

    double timestep = this->getVelocity(tols,initPosture,finalPosture,traj,vel,mod);

    this->directAcceleration(tols,initPosture,finalPosture,acc,mod);

    return timestep;

    /*
    double timestep = this->getVelocity(tols,initPosture,finalPosture,traj,vel,mod);
    acc.resize(traj.rows(),traj.cols());

    std::vector<double> delta;
    VectorXd vel_posture;
    for (int k = 0; k < vel.cols(); ++k){
        vel_posture = vel.col(k);
        this->getDelta(vel_posture,delta);
        for (int i =0; i < vel.rows(); ++i){
            acc(i,k) = delta.at(i)/timestep;
        }
    }

    return timestep;
    */
}

double HUMPlanner::getAcceleration(huml_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc,int mod)
{
    double timestep = this->getVelocity(tols,initPosture,finalPosture,bouncePosture,traj,vel,mod);

    MatrixXd dAcc;
    MatrixXd bAcc;

    this->directAcceleration(tols,initPosture,finalPosture,dAcc,mod);
    this->backForthAcceleration(tols,initPosture,bouncePosture,bAcc);
    this->computeMovement(dAcc,bAcc,acc);

    return timestep;

    /*

    double timestep = this->getVelocity(tols,initPosture,finalPosture,bouncePosture,traj,vel,mod);
    acc.resize(traj.rows(),traj.cols());

    std::vector<double> delta;
    VectorXd vel_posture;
    for (int k = 0; k < vel.cols(); ++k){
        vel_posture = vel.col(k);
        this->getDelta(vel_posture,delta);
        for (int i =0; i < vel.rows(); ++i){
            acc(i,k) = delta.at(i)/timestep;
        }
    }

    return timestep;

    */
}


planning_result HUMPlanner::plan_pick(huml_params &params, std::vector<double> initPosture)
{
    planning_result res;

    int mov_type = 0; // pick
    res.mov_type = mov_type;
    std::vector<double> finalHand = params.mov_specs.finalHand;
    int arm_code = params.mov_specs.hand_code;
    std::vector<double> minLimits; std::vector<double> maxLimits;
    switch(arm_code){
    case 1: // right arm
        minLimits = this->minRightLimits;
        maxLimits = this->maxRightLimits;
        break;
    case 2: // left arm
        minLimits = this->minLeftLimits;
        maxLimits = this->maxLeftLimits;
        break;
    }

    res.object_id = params.mov_specs.obj->getName();
    bool approach = params.mov_specs.approach;
    bool retreat = params.mov_specs.retreat;
    int pre_post = 0; // 0 = use no options, 1 = use approach options, 2 = use retreat options

    try
    {
        if(approach){
            pre_post = 1;
            std::vector<double> finalPosture_pre_grasp;
            bool FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,initPosture,finalPosture_pre_grasp);
            if (FPosture){
                std::vector<double> bouncePosture_pre_grasp;
                bool BPosture = this->singleArmBouncePosture(mov_type,pre_post,params,initPosture,finalPosture_pre_grasp,bouncePosture_pre_grasp);
                if(BPosture){
                    pre_post = 0;
                    std::vector<double> finalPosture_grasp;
                    bool FPosture_grasp = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture_pre_grasp,finalPosture_grasp);
                    if(FPosture_grasp){
                        res.status = 0; res.status_msg = string("HUML: trajectory planned successfully ");
                        res.time_steps.clear();
                        res.trajectory_stages.clear(); res.trajectory_descriptions.clear();
                        res.velocity_stages.clear();
                        res.acceleration_stages.clear();
                        int mod;
                        // extend the final postures
                        std::vector<double> finalPosture_pre_grasp_ext = finalPosture_pre_grasp;
                        std::vector<double> finalPosture_grasp_ext = finalPosture_grasp;
                        finalPosture_grasp_ext.push_back(finalHand.at(0));
                        finalPosture_pre_grasp_ext.push_back(finalHand.at(0));
                        for(size_t i=1;i<finalHand.size();++i){
                            finalPosture_grasp_ext.push_back(finalHand.at(i));
                            if(((finalHand.at(i) -AP) > minLimits.at(i+7))){
                                finalPosture_pre_grasp_ext.push_back(finalHand.at(i)-AP);
                            }else{
                               finalPosture_pre_grasp_ext.push_back(minLimits.at(i+7));
                            }
                        }
                        // pre-approach stage
                        MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 1;
                        timestep = this->getAcceleration(params,initPosture,finalPosture_pre_grasp_ext,bouncePosture_pre_grasp,traj,vel,acc,mod);
                        res.time_steps.push_back(timestep);
                        res.trajectory_stages.push_back(traj); res.trajectory_descriptions.push_back(string("HUML: pick of the object, pre-approach ")+res.object_id);
                        res.velocity_stages.push_back(vel);
                        res.acceleration_stages.push_back(acc);
                        // approach stage
                        mod = 2;
                        timestep = this->getAcceleration(params,finalPosture_pre_grasp_ext,finalPosture_grasp_ext,traj,vel,acc,mod);
                        res.time_steps.push_back(timestep);
                        res.trajectory_stages.push_back(traj); res.trajectory_descriptions.push_back(string("HUML: pick of the object, approach ")+res.object_id);
                        res.velocity_stages.push_back(vel);
                        res.acceleration_stages.push_back(acc);
                    }else{res.status = 30; res.status_msg = string("HUML: final posture grasp selection failed ");}
                }else{ res.status = 20; res.status_msg = string("HUML: bounce posture pre grasp selection failed ");}
            }else{res.status = 10; res.status_msg = string("HUML: final posture pre grasp selection failed ");}
        }else{
            pre_post = 0;
            std::vector<double> finalPosture;
            bool FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,initPosture,finalPosture);
            if (FPosture){
                std::vector<double> bouncePosture;
                bool BPosture = this->singleArmBouncePosture(mov_type,pre_post,params,initPosture,finalPosture,bouncePosture);
                if(BPosture){
                    res.status = 0; res.status_msg = string("HUML: trajectory planned successfully ");
                    res.time_steps.clear();
                    res.trajectory_stages.clear(); res.trajectory_descriptions.clear();
                    res.velocity_stages.clear();
                    res.acceleration_stages.clear();
                    int mod;
                    // extend the final postures
                    std::vector<double> finalPosture_ext = finalPosture;
                    for(size_t i=0;i<finalHand.size();++i){
                        finalPosture_ext.push_back(finalHand.at(i));
                    }
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; mod = 0;
                    double timestep = this->getAcceleration(params,initPosture,finalPosture_ext,bouncePosture,traj,vel,acc,mod);
                    res.time_steps.push_back(timestep);
                    res.trajectory_stages.push_back(traj); res.trajectory_descriptions.push_back(string("HUML: pick of the object, grasp ")+res.object_id);
                    res.velocity_stages.push_back(vel);
                    res.acceleration_stages.push_back(acc);
                }else{ res.status = 20; res.status_msg = string("HUML: bounce posture selection failed ");}
            }else{res.status = 10; res.status_msg = string("HUML: final posture selection failed ");}

        }
        if(retreat){
            // TO DO
        }
    }catch (const string message){throw message;
}catch( ... ){throw string ("HUML: error in optimizing the trajecory");}

    return res;


}

} // namespace HUMotion
