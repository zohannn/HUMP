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
    if(!hp.obstacles_right.empty()){this->obstacles_right=hp.obstacles_right;}
    if(!hp.obstacles_left.empty()){this->obstacles_left=hp.obstacles_left;}
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

void HUMPlanner::addObstacleRight(objectPtr obs)
{
    this->obstacles_right.push_back(objectPtr(new Object(*obs.get())));
}

void HUMPlanner::addObstacleLeft(objectPtr obs)
{
    this->obstacles_left.push_back(objectPtr(new Object(*obs.get())));
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

void HUMPlanner::writeDualArmDHParams(DHparameters dh_right, DHparameters dh_left, std::ofstream& stream)
{
    // D-H Parameters of the Right Arm
    stream << string("# D-H PARAMETERS OF THE RIGHT ARM \n");
    stream << string("param alpha_right := \n");
    string alpha_right_str;
    alpha_right_str =  boost::str(boost::format("%.2f") % (dh_right.alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_right_str+string("\n");
    for(std::size_t i=1; i< joints_arm; ++i){
        alpha_right_str =  boost::str(boost::format("%.2f") % (dh_right.alpha.at(i)));

        if (i == dh_right.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_right_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_right_str+string("\n");
        }
    }
    stream << string("param a_right := \n");
    string a_right_str;
    for(std::size_t i=0; i<joints_arm; ++i){
        a_right_str =  boost::str(boost::format("%.2f") % (dh_right.a.at(i)));
        if (i == dh_right.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_right_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_right_str+string("\n");
        }
    }
    stream << string("param d_right := \n");
    string d_right_str;
    for(std::size_t i=0; i<joints_arm; ++i){
        d_right_str =  boost::str(boost::format("%.2f") % (dh_right.d.at(i)));
        if (i == dh_right.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_right_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_right_str+string("\n");
        }
    }

    // D-H Parameters of the Left Arm
    stream << string("# D-H PARAMETERS OF THE LEFT ARM \n");
    stream << string("param alpha_left := \n");
    string alpha_left_str;
    alpha_left_str =  boost::str(boost::format("%.2f") % (dh_left.alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_left_str+string("\n");
    for(std::size_t i=1; i< joints_arm; ++i){
        alpha_left_str =  boost::str(boost::format("%.2f") % (dh_left.alpha.at(i)));

        if (i == dh_left.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_left_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_left_str+string("\n");
        }
    }
    stream << string("param a_left := \n");
    string a_left_str;
    for(std::size_t i=0; i<joints_arm; ++i){
        a_left_str =  boost::str(boost::format("%.2f") % (dh_left.a.at(i)));
        if (i == dh_left.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_left_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_left_str+string("\n");
        }
    }
    stream << string("param d_left := \n");
    string d_left_str;
    for(std::size_t i=0; i<joints_arm; ++i){
        d_left_str =  boost::str(boost::format("%.2f") % (dh_left.d.at(i)));
        if (i == dh_left.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_left_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_left_str+string("\n");
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

void HUMPlanner::write_dual_dHO(std::ofstream& stream, double dHO_right, double dHO_left)
{
    string dHOstr_right=  boost::str(boost::format("%.2f") % (dHO_right));
    boost::replace_all(dHOstr_right,",",".");
    stream << string("# DISTANCE RIGHT HAND - TARGET\n");
    stream << string("param dFH_right := ")+dHOstr_right+string(";\n");

    string dHOstr_left=  boost::str(boost::format("%.2f") % (dHO_left));
    boost::replace_all(dHOstr_left,",",".");
    stream << string("# DISTANCE LEFT HAND - TARGET\n");
    stream << string("param dFH_left := ")+dHOstr_left+string(";\n");
}

void HUMPlanner::writeArmLimits(ofstream &stream, std::vector<double> &minArmLimits, std::vector<double> &maxArmLimits,bool final)
{

    stream << string("# JOINT LIMITS \n");
    stream << string("# Lower Bound \n");
    stream << string("param llim := \n");

    double joint_spacer;
    if(final)
    {
        joint_spacer = SPACER;
    }else{
        joint_spacer = SPACER_BOUNCE;
    }

    for (std::size_t i=0; i < minArmLimits.size(); ++i){
        string minLim=  boost::str(boost::format("%.2f") % (minArmLimits.at(i)+joint_spacer));
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
        string maxLim=  boost::str(boost::format("%.2f") % (maxArmLimits.at(i)-joint_spacer));
        boost::replace_all(maxLim,",",".");

        if (i == maxArmLimits.size()-1){
            stream << to_string(i+1)+string(" ")+maxLim+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+maxLim+string("\n");
        }
    }

}

void HUMPlanner::writeDualArmLimits(std::ofstream& stream, std::vector<double>& minRightArmLimits,std::vector<double>& maxRightArmLimits,
                        std::vector<double>& minLeftArmLimits,std::vector<double>& maxLeftArmLimits,bool final)
{

    std::vector<double> minArmLimits; std::vector<double> maxArmLimits;
    minArmLimits = minRightArmLimits;
    minArmLimits.insert(minArmLimits.end(),minLeftArmLimits.begin(),minLeftArmLimits.end());
    maxArmLimits = maxRightArmLimits;
    maxArmLimits.insert(maxArmLimits.end(),maxLeftArmLimits.begin(),maxLeftArmLimits.end());


    stream << string("# JOINT LIMITS \n");
    stream << string("# Lower Bound \n");
    stream << string("param llim := \n");

    double joint_spacer;
    if(final){
        joint_spacer = SPACER;
    }else{
        joint_spacer = SPACER_BOUNCE;
    }

    for (std::size_t i=0; i < minArmLimits.size(); ++i){
        string minLim=  boost::str(boost::format("%.2f") % (minArmLimits.at(i)+joint_spacer));
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
        string maxLim=  boost::str(boost::format("%.2f") % (maxArmLimits.at(i)-joint_spacer));
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

void HUMPlanner::writeDualArmInitPose(std::ofstream& stream,std::vector<double>& initRightArmPosture,std::vector<double>& initLeftArmPosture)
{
    std::vector<double> initArmPosture = initRightArmPosture;
    initArmPosture.insert(initArmPosture.end(),initLeftArmPosture.begin(),initLeftArmPosture.end());

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

void HUMPlanner::writeFingerDualFinalPose(std::ofstream& stream,std::vector<double>& finalHand_right,std::vector<double>& finalHand_left)
{
    stream << string("# FINAL FINGER JOINTS RIGHT \n");
    stream << string("param joint_fingers_right := \n");

    for (std::size_t i=0; i < finalHand_right.size(); ++i){
        string finalHandstr =  boost::str(boost::format("%.2f") % (finalHand_right.at(i)));
        boost::replace_all(finalHandstr,",",".");
        if (i == finalHand_right.size()-1){
            stream << to_string(i+1)+string(" ")+finalHandstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+finalHandstr+string("\n");
        }
    }

    stream << string("# FINAL FINGER JOINTS LEFT \n");
    stream << string("param joint_fingers_left := \n");

    for (std::size_t i=0; i < finalHand_left.size(); ++i){
        string finalHandstr =  boost::str(boost::format("%.2f") % (finalHand_left.at(i)));
        boost::replace_all(finalHandstr,",",".");
        if (i == finalHand_left.size()-1){
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

void HUMPlanner::writeDualLambda(std::ofstream& stream,std::vector<double>& lambda_right,std::vector<double>& lambda_left)
{
    std::vector<double> lambda = lambda_right;
    lambda.insert(lambda.end(),lambda_left.begin(),lambda_left.end());

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

void HUMPlanner::writeDualHumanHandParams(HumanHand& hhand, std::ofstream& stream, bool right)
{
    if(right){
        stream << string("# PARAMETERS OF THE RIGHT HAND \n");
        // Index finger
        stream << string("# Index Finger \n");
        HumanFinger finger_1 = hhand.fingers.at(0);
        stream << string("param u1x_right := ");
        string ux_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.ux));
        stream << ux_fing1_str+string(";\n");
        stream << string("param u1y_right := ");
        string uy_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.uy));
        stream << uy_fing1_str+string(";\n");
        stream << string("param u1z_right := ");
        string uz_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.uz));
        stream << uz_fing1_str+string(";\n");
        stream << string("param alpha_fing1_right := \n");
        string alpha_fing1_str;
        alpha_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.alpha.at(0)));
        stream << to_string(1)+string(" ")+alpha_fing1_str+string("\n");
        for(std::size_t i=1; i<n_phalange+1; ++i){
            alpha_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.alpha.at(i)));
            if (i == finger_1.finger_specs.alpha.size()-1){
                stream << to_string(i+1)+string(" ")+alpha_fing1_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+alpha_fing1_str+string("\n");
            }
        }
        stream << string("param a_fing1_right := \n");
        string a_fing1_str;
        for(std::size_t i=0; i<n_phalange+1; ++i){
            a_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.a.at(i)));
            if (i == finger_1.finger_specs.a.size()-1){
                stream << to_string(i+1)+string(" ")+a_fing1_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+a_fing1_str+string("\n");
            }
        }
        stream << string("param d_fing1_right := \n");
        string d_fing1_str;
        for(std::size_t i=0; i<n_phalange+1; ++i){
            d_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.d.at(i)));
            if (i == finger_1.finger_specs.d.size()-1){
                stream << to_string(i+1)+string(" ")+d_fing1_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+d_fing1_str+string("\n");
            }
        }
        stream << string("param theta0_fing1_right := ");
        string theta0_fing1_str;
        theta0_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.theta.at(0)));
        stream << theta0_fing1_str+string(";\n");
        // Ring finger
        stream << string("# Ring Finger \n");
        HumanFinger finger_3 = hhand.fingers.at(2);
        stream << string("param u3x_right := ");
        string ux_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.ux));
        stream << ux_fing3_str+string(";\n");
        stream << string("param u3y_right := ");
        string uy_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.uy));
        stream << uy_fing3_str+string(";\n");
        stream << string("param u3z_right := ");
        string uz_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.uz));
        stream << uz_fing3_str+string(";\n");
        stream << string("param alpha_fing3_right := \n");
        string alpha_fing3_str;
        alpha_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.alpha.at(0)));
        stream << to_string(1)+string(" ")+alpha_fing3_str+string("\n");
        for(std::size_t i=1; i<n_phalange+1; ++i){
            alpha_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.alpha.at(i)));
            if (i == finger_3.finger_specs.alpha.size()-1){
                stream << to_string(i+1)+string(" ")+alpha_fing3_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+alpha_fing3_str+string("\n");
            }
        }
        stream << string("param a_fing3_right := \n");
        string a_fing3_str;
        for(std::size_t i=0; i<n_phalange+1; ++i){
            a_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.a.at(i)));
            if (i == finger_3.finger_specs.a.size()-1){
                stream << to_string(i+1)+string(" ")+a_fing3_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+a_fing3_str+string("\n");
            }
        }
        stream << string("param d_fing3_right := \n");
        string d_fing3_str;
        for(std::size_t i=0; i<n_phalange+1; ++i){
            d_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.d.at(i)));
            if (i == finger_3.finger_specs.d.size()-1){
                stream << to_string(i+1)+string(" ")+d_fing3_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+d_fing3_str+string("\n");
            }
        }
        stream << string("param theta0_fing3_right := ");
        string theta0_fing3_str;
        theta0_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.theta.at(0)));
        stream << theta0_fing3_str+string(";\n");
        // Thumb finger
        stream << string("# Thumb Finger \n");
        HumanThumb thumb_finger = hhand.thumb;
        stream << string("param uTx_right := ");
        string uTx_fing2_str =  boost::str(boost::format("%.2f") % (thumb_finger.uTx));
        stream << uTx_fing2_str+string(";\n");
        stream << string("param uTy_right := ");
        string uTy_fing2_str =  boost::str(boost::format("%.2f") % (thumb_finger.uTy));
        stream << uTy_fing2_str+string(";\n");
        stream << string("param uTz_right := ");
        string uTz_fing2_str =  boost::str(boost::format("%.2f") % (thumb_finger.uTz));
        stream << uTz_fing2_str+string(";\n");
        stream << string("param alpha_thumb_right := \n");
        string alpha_thumb_str;
        //if (k == 1){ // right hand
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(0)));
        //}else if(k==-1){// left hand
        //alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(0)-M_PI/2));
        //}
        stream << to_string(1)+string(" ")+alpha_thumb_str+string("\n");
        for(std::size_t i=1; i<n_phalange+2; ++i){
            alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(i)));
            if (i == thumb_finger.thumb_specs.alpha.size()-1){
                stream << to_string(i+1)+string(" ")+alpha_thumb_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+alpha_thumb_str+string("\n");
            }
        }
        stream << string("param a_thumb_right := \n");
        string a_thumb_str;
        for(std::size_t i=0; i<n_phalange+2; ++i){
            a_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.a.at(i)));
            if (i == thumb_finger.thumb_specs.a.size()-1){
                stream << to_string(i+1)+string(" ")+a_thumb_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+a_thumb_str+string("\n");
            }
        }
        stream << string("param d_thumb_right := \n");
        string d_thumb_str;
        for(std::size_t i=0; i<n_phalange+2; ++i){
            d_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.d.at(i)));
            if (i == thumb_finger.thumb_specs.d.size()-1){
                stream << to_string(i+1)+string(" ")+d_thumb_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+d_thumb_str+string("\n");
            }
        }
        stream << string("param theta0_thumb_right:= ");
        string theta0_thumb_str;
        theta0_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.theta.at(0)));
        stream << theta0_thumb_str+string(";\n");
    }else{
        stream << string("# PARAMETERS OF THE LEFT HAND \n");
        // Index finger
        stream << string("# Index Finger \n");
        HumanFinger finger_1 = hhand.fingers.at(0);
        stream << string("param u1x_left := ");
        string ux_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.ux));
        stream << ux_fing1_str+string(";\n");
        stream << string("param u1y_left := ");
        string uy_fing1_str =  boost::str(boost::format("%.2f") % (-1*finger_1.uy));
        stream << uy_fing1_str+string(";\n");
        stream << string("param u1z_left := ");
        string uz_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.uz));
        stream << uz_fing1_str+string(";\n");
        stream << string("param alpha_fing1_left := \n");
        string alpha_fing1_str;
        alpha_fing1_str =  boost::str(boost::format("%.2f") % (-1*finger_1.finger_specs.alpha.at(0)));
        stream << to_string(1)+string(" ")+alpha_fing1_str+string("\n");
        for(std::size_t i=1; i<n_phalange+1; ++i){
            alpha_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.alpha.at(i)));
            if (i == finger_1.finger_specs.alpha.size()-1){
                stream << to_string(i+1)+string(" ")+alpha_fing1_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+alpha_fing1_str+string("\n");
            }
        }
        stream << string("param a_fing1_left := \n");
        string a_fing1_str;
        for(std::size_t i=0; i<n_phalange+1; ++i){
            a_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.a.at(i)));
            if (i == finger_1.finger_specs.a.size()-1){
                stream << to_string(i+1)+string(" ")+a_fing1_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+a_fing1_str+string("\n");
            }
        }
        stream << string("param d_fing1_left := \n");
        string d_fing1_str;
        for(std::size_t i=0; i<n_phalange+1; ++i){
            d_fing1_str =  boost::str(boost::format("%.2f") % (finger_1.finger_specs.d.at(i)));
            if (i == finger_1.finger_specs.d.size()-1){
                stream << to_string(i+1)+string(" ")+d_fing1_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+d_fing1_str+string("\n");
            }
        }
        stream << string("param theta0_fing1_left := ");
        string theta0_fing1_str;
        theta0_fing1_str =  boost::str(boost::format("%.2f") % (-1*finger_1.finger_specs.theta.at(0)));
        stream << theta0_fing1_str+string(";\n");
        // Ring finger
        stream << string("# Ring Finger \n");
        HumanFinger finger_3 = hhand.fingers.at(2);
        stream << string("param u3x_left := ");
        string ux_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.ux));
        stream << ux_fing3_str+string(";\n");
        stream << string("param u3y_left := ");
        string uy_fing3_str =  boost::str(boost::format("%.2f") % (-1*finger_3.uy));
        stream << uy_fing3_str+string(";\n");
        stream << string("param u3z_left := ");
        string uz_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.uz));
        stream << uz_fing3_str+string(";\n");
        stream << string("param alpha_fing3_left := \n");
        string alpha_fing3_str;
        alpha_fing3_str =  boost::str(boost::format("%.2f") % (-1*finger_3.finger_specs.alpha.at(0)));
        stream << to_string(1)+string(" ")+alpha_fing3_str+string("\n");
        for(std::size_t i=1; i<n_phalange+1; ++i){
            alpha_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.alpha.at(i)));
            if (i == finger_3.finger_specs.alpha.size()-1){
                stream << to_string(i+1)+string(" ")+alpha_fing3_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+alpha_fing3_str+string("\n");
            }
        }
        stream << string("param a_fing3_left := \n");
        string a_fing3_str;
        for(std::size_t i=0; i<n_phalange+1; ++i){
            a_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.a.at(i)));
            if (i == finger_3.finger_specs.a.size()-1){
                stream << to_string(i+1)+string(" ")+a_fing3_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+a_fing3_str+string("\n");
            }
        }
        stream << string("param d_fing3_left := \n");
        string d_fing3_str;
        for(std::size_t i=0; i<n_phalange+1; ++i){
            d_fing3_str =  boost::str(boost::format("%.2f") % (finger_3.finger_specs.d.at(i)));
            if (i == finger_3.finger_specs.d.size()-1){
                stream << to_string(i+1)+string(" ")+d_fing3_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+d_fing3_str+string("\n");
            }
        }
        stream << string("param theta0_fing3_left := ");
        string theta0_fing3_str;
        theta0_fing3_str =  boost::str(boost::format("%.2f") % (-1*finger_3.finger_specs.theta.at(0)));
        stream << theta0_fing3_str+string(";\n");
        // Thumb finger
        stream << string("# Thumb Finger \n");
        HumanThumb thumb_finger = hhand.thumb;
        stream << string("param uTx_left := ");
        string uTx_fing2_str =  boost::str(boost::format("%.2f") % (thumb_finger.uTx));
        stream << uTx_fing2_str+string(";\n");
        stream << string("param uTy_left := ");
        string uTy_fing2_str =  boost::str(boost::format("%.2f") % (-1*thumb_finger.uTy));
        stream << uTy_fing2_str+string(";\n");
        stream << string("param uTz_left := ");
        string uTz_fing2_str =  boost::str(boost::format("%.2f") % (thumb_finger.uTz));
        stream << uTz_fing2_str+string(";\n");
        stream << string("param alpha_thumb_left := \n");
        string alpha_thumb_str;
        //if (k == 1){ // right hand
        //alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(0)));
        //}else if(k==-1){// left hand
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(0)-M_PI/2));
        //}
        stream << to_string(1)+string(" ")+alpha_thumb_str+string("\n");
        for(std::size_t i=1; i<n_phalange+2; ++i){
            alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.alpha.at(i)));
            if (i == thumb_finger.thumb_specs.alpha.size()-1){
                stream << to_string(i+1)+string(" ")+alpha_thumb_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+alpha_thumb_str+string("\n");
            }
        }
        stream << string("param a_thumb_left := \n");
        string a_thumb_str;
        for(std::size_t i=0; i<n_phalange+2; ++i){
            a_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.a.at(i)));
            if (i == thumb_finger.thumb_specs.a.size()-1){
                stream << to_string(i+1)+string(" ")+a_thumb_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+a_thumb_str+string("\n");
            }
        }
        stream << string("param d_thumb_left := \n");
        string d_thumb_str;
        for(std::size_t i=0; i<n_phalange+2; ++i){
            d_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.d.at(i)));
            if (i == thumb_finger.thumb_specs.d.size()-1){
                stream << to_string(i+1)+string(" ")+d_thumb_str+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+d_thumb_str+string("\n");
            }
        }
        stream << string("param theta0_thumb_left:= ");
        string theta0_thumb_str;
        theta0_thumb_str =  boost::str(boost::format("%.2f") % (thumb_finger.thumb_specs.theta.at(0)));
        stream << theta0_thumb_str+string(";\n");

    }
}

void HUMPlanner::writeHumanHandParamsMod(ofstream &stream)
{
    stream << string("# PARAMETERS OF THE HAND \n");

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

void HUMPlanner::writeDualHumanHandParamsMod(std::ofstream& stream,bool right)
{
    if(right)
    {
        stream << string("# PARAMETERS OF THE RIGHT HAND \n");

        stream << string("# Index finger \n");
        stream << string("param u1x_right; \n");
        stream << string("param u1y_right; \n");
        stream << string("param u1z_right; \n");
        stream << string("param alpha_fing1_right {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param a_fing1_right {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param d_fing1_right {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param theta0_fing1_right; \n");

        stream << string("# Ring finger \n");
        stream << string("param u3x_right; \n");
        stream << string("param u3y_right; \n");
        stream << string("param u3z_right; \n");
        stream << string("param alpha_fing3_right {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param a_fing3_right {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param d_fing3_right {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param theta0_fing3_right; \n");

        stream << string("# Thumb finger \n");
        stream << string("param uTx_right; \n");
        stream << string("param uTy_right; \n");
        stream << string("param uTz_right; \n");
        stream << string("param alpha_thumb_right {i in 1..")+to_string(n_phalange+2)+string("} ; \n");
        stream << string("param a_thumb_right {i in 1..")+to_string(n_phalange+2)+string("} ; \n");
        stream << string("param d_thumb_right {i in 1..")+to_string(n_phalange+2)+string("} ; \n");
        stream << string("param theta0_thumb_right; \n");
    }else{
        stream << string("# PARAMETERS OF THE LEFT HAND \n");

        stream << string("# Index finger \n");
        stream << string("param u1x_left; \n");
        stream << string("param u1y_left; \n");
        stream << string("param u1z_left; \n");
        stream << string("param alpha_fing1_left {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param a_fing1_left {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param d_fing1_left {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param theta0_fing1_left; \n");

        stream << string("# Ring finger \n");
        stream << string("param u3x_left; \n");
        stream << string("param u3y_left; \n");
        stream << string("param u3z_left; \n");
        stream << string("param alpha_fing3_left {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param a_fing3_left {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param d_fing3_left {i in 1..")+to_string(n_phalange+1)+string("} ; \n");
        stream << string("param theta0_fing3_left; \n");

        stream << string("# Thumb finger \n");
        stream << string("param uTx_left; \n");
        stream << string("param uTy_left; \n");
        stream << string("param uTz_left; \n");
        stream << string("param alpha_thumb_left {i in 1..")+to_string(n_phalange+2)+string("} ; \n");
        stream << string("param a_thumb_left {i in 1..")+to_string(n_phalange+2)+string("} ; \n");
        stream << string("param d_thumb_left {i in 1..")+to_string(n_phalange+2)+string("} ; \n");
        stream << string("param theta0_thumb_left; \n");
    }
}

void HUMPlanner::writeBarrettHandParams(BarrettHand& bhand, std::ofstream& stream)
{

    stream << string("# PARAMETERS OF THE HAND \n");
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

void HUMPlanner::writeDualBarrettHandParams(BarrettHand& bhand, std::ofstream& stream,bool right)
{
    if(right){
        stream << string("# PARAMETERS OF THE RIGHT HAND \n");
        // rk and jk parameters
        stream << string("# R and J parameters (right) \n");
        stream << string("param rk_right := \n");
        for (size_t i=0; i < bhand.rk.size(); ++i){
            string rkstr =  boost::str(boost::format("%.2f") % (bhand.rk.at(i)));
            boost::replace_all(rkstr,",",".");
            if (i == bhand.rk.size()-1){
                stream << to_string(i+1)+string(" ")+rkstr+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+rkstr+string("\n");
            }
        }
        stream << string("param jk_right := \n");
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
        stream << string("param Aw_right :=")+Awstr+string(";\n");
        string A1str =  boost::str(boost::format("%.2f") % (bhand.A1));
        boost::replace_all(A1str,",",".");
        stream << string("param A1_right :=")+A1str+string(";\n");
        string A2str =  boost::str(boost::format("%.2f") % (bhand.A2));
        boost::replace_all(A2str,",",".");
        stream << string("param A2_right :=")+A2str+string(";\n");
        string A3str =  boost::str(boost::format("%.2f") % (bhand.A3));
        boost::replace_all(A3str,",",".");
        stream << string("param A3_right :=")+A3str+string(";\n");
        string D3str =  boost::str(boost::format("%.2f") % (bhand.D3));
        boost::replace_all(D3str,",",".");
        stream << string("param D3_right :=")+D3str+string(";\n");
        string phi2str =  boost::str(boost::format("%.2f") % (bhand.phi2));
        boost::replace_all(phi2str,",",".");
        stream << string("param phi_2_right :=")+phi2str+string(";\n");
        string phi3str =  boost::str(boost::format("%.2f") % (bhand.phi3));
        boost::replace_all(phi3str,",",".");
        stream << string("param phi_3_right :=")+phi3str+string(";\n");
    }else{
        stream << string("# PARAMETERS OF THE LEFT HAND \n");
        // rk and jk parameters
        stream << string("# R and J parameters (left) \n");
        stream << string("param rk_left := \n");
        for (size_t i=0; i < bhand.rk.size(); ++i){
            string rkstr =  boost::str(boost::format("%.2f") % (bhand.rk.at(i)));
            boost::replace_all(rkstr,",",".");
            if (i == bhand.rk.size()-1){
                stream << to_string(i+1)+string(" ")+rkstr+string(";\n");
            }else{
                stream << to_string(i+1)+string(" ")+rkstr+string("\n");
            }
        }
        stream << string("param jk_left := \n");
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
        stream << string("param Aw_left :=")+Awstr+string(";\n");
        string A1str =  boost::str(boost::format("%.2f") % (bhand.A1));
        boost::replace_all(A1str,",",".");
        stream << string("param A1_left :=")+A1str+string(";\n");
        string A2str =  boost::str(boost::format("%.2f") % (bhand.A2));
        boost::replace_all(A2str,",",".");
        stream << string("param A2_left :=")+A2str+string(";\n");
        string A3str =  boost::str(boost::format("%.2f") % (bhand.A3));
        boost::replace_all(A3str,",",".");
        stream << string("param A3_left :=")+A3str+string(";\n");
        string D3str =  boost::str(boost::format("%.2f") % (bhand.D3));
        boost::replace_all(D3str,",",".");
        stream << string("param D3_left :=")+D3str+string(";\n");
        string phi2str =  boost::str(boost::format("%.2f") % (bhand.phi2));
        boost::replace_all(phi2str,",",".");
        stream << string("param phi_2_left :=")+phi2str+string(";\n");
        string phi3str =  boost::str(boost::format("%.2f") % (bhand.phi3));
        boost::replace_all(phi3str,",",".");
        stream << string("param phi_3_left :=")+phi3str+string(";\n");
    }
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

void HUMPlanner::writeDualBarrettHandParamsMod(std::ofstream& stream, bool right)
{
    if(right){
        stream << string("# PARAMETERS OF THE RIGHT HAND \n");
        stream << string("param rk_right {i in 1..3} ; \n");
        stream << string("param jk_right {i in 1..3} ; \n");
        stream << string("param Aw_right; \n");
        stream << string("param A1_right; \n");
        stream << string("param A2_right; \n");
        stream << string("param A3_right; \n");
        stream << string("param D3_right; \n");
        stream << string("param phi_2_right; \n");
        stream << string("param phi_3_right; \n");
    }else{
        stream << string("# PARAMETERS OF THE LEFT HAND \n");
        stream << string("param rk_left {i in 1..3} ; \n");
        stream << string("param jk_left {i in 1..3} ; \n");
        stream << string("param Aw_left; \n");
        stream << string("param A1_left; \n");
        stream << string("param A2_left; \n");
        stream << string("param A3_left; \n");
        stream << string("param D3_left; \n");
        stream << string("param phi_2_left; \n");
        stream << string("param phi_3_left; \n");
    }
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

void HUMPlanner::writeDualInfoTarget(ofstream &stream,std::vector<double> tar_right,std::vector<double> tar_left)
{
    string tarx; string tary; string tarz;
    string tarxt0; string taryt0; string tarzt0;
    string tarxt1; string taryt1; string tarzt1;
    string tarxt2; string taryt2; string tarzt2;
    std::vector<double> rpy_r; Matrix3d Rot_tar_r; Matrix3d Rot_tar_r_inv; std::vector<double> xt_r; std::vector<double> yt_r; std::vector<double> zt_r;
    std::vector<double> rpy_l; Matrix3d Rot_tar_l; Matrix3d Rot_tar_l_inv; std::vector<double> xt_l; std::vector<double> yt_l; std::vector<double> zt_l;

    stream << string("# TARGET RIGHT POSITION \n");
    stream << string("param Tar_pos_right := \n");
    tarx =  boost::str(boost::format("%.2f") % (tar_right.at(0)));
    boost::replace_all(tarx,",",".");
    stream << to_string(1)+string(" ")+tarx+string("\n");
    tary =  boost::str(boost::format("%.2f") % (tar_right.at(1)));
    boost::replace_all(tary,",",".");
    stream << to_string(2)+string(" ")+tary+string("\n");
    tarz =  boost::str(boost::format("%.2f") % (tar_right.at(2)));
    boost::replace_all(tarz,",",".");
    stream << to_string(3)+string(" ")+tarz+string(";\n");

    stream << string("# TARGET RIGHT ORIENTATION \n");
    rpy_r = {tar_right.at(3),tar_right.at(4),tar_right.at(5)}; this->RPY_matrix(rpy_r,Rot_tar_r); Rot_tar_r_inv = Rot_tar_r.transpose();
    this->getRotAxis(xt_r,0,rpy_r);
    this->getRotAxis(yt_r,1,rpy_r);
    this->getRotAxis(zt_r,2,rpy_r);
    stream << string("param x_t_right := \n");
    tarxt0 =  boost::str(boost::format("%.2f") % (xt_r[0]));
    boost::replace_all(tarxt0,",",".");
    stream << to_string(1)+string(" ")+tarxt0+string("\n");
    tarxt1 =  boost::str(boost::format("%.2f") % (xt_r[1]));
    boost::replace_all(tarxt1,",",".");
    stream << to_string(2)+string(" ")+tarxt1+string("\n");
    tarxt2 =  boost::str(boost::format("%.2f") % (xt_r[2]));
    boost::replace_all(tarxt2,",",".");
    stream << to_string(3)+string(" ")+tarxt2+string(";\n");
    stream << string("param y_t_right := \n");
    taryt0 =  boost::str(boost::format("%.2f") % (yt_r[0]));
    boost::replace_all(taryt0,",",".");
    stream << to_string(1)+string(" ")+taryt0+string("\n");
    taryt1 =  boost::str(boost::format("%.2f") % (yt_r[1]));
    boost::replace_all(taryt1,",",".");
    stream << to_string(2)+string(" ")+taryt1+string("\n");
    taryt2 =  boost::str(boost::format("%.2f") % (yt_r[2]));
    boost::replace_all(taryt2,",",".");
    stream << to_string(3)+string(" ")+taryt2+string(";\n");
    stream << string("param z_t_right := \n");
    tarzt0 =  boost::str(boost::format("%.2f") % (zt_r[0]));
    boost::replace_all(tarzt0,",",".");
    stream << to_string(1)+string(" ")+tarzt0+string("\n");
    tarzt1 =  boost::str(boost::format("%.2f") % (zt_r[1]));
    boost::replace_all(tarzt1,",",".");
    stream << to_string(2)+string(" ")+tarzt1+string("\n");
    tarzt2 =  boost::str(boost::format("%.2f") % (zt_r[2]));
    boost::replace_all(tarzt2,",",".");
    stream << to_string(3)+string(" ")+tarzt2+string(";\n");

    /*
    stream << string("param Rot_tar_right_inv : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_tar_r_inv(0,0))+string(" ")+to_string(Rot_tar_r_inv(0,1))+string(" ")+to_string(Rot_tar_r_inv(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_tar_r_inv(1,0))+string(" ")+to_string(Rot_tar_r_inv(1,1))+string(" ")+to_string(Rot_tar_r_inv(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_tar_r_inv(2,0))+string(" ")+to_string(Rot_tar_r_inv(2,1))+string(" ")+to_string(Rot_tar_r_inv(2,2))+string(" \n");
    stream << string("; \n");
    */

    stream << string("# TARGET LEFT POSITION \n");
    stream << string("param Tar_pos_left := \n");
    tarx =  boost::str(boost::format("%.2f") % (tar_left.at(0)));
    boost::replace_all(tarx,",",".");
    stream << to_string(1)+string(" ")+tarx+string("\n");
    tary =  boost::str(boost::format("%.2f") % (tar_left.at(1)));
    boost::replace_all(tary,",",".");
    stream << to_string(2)+string(" ")+tary+string("\n");
    tarz =  boost::str(boost::format("%.2f") % (tar_left.at(2)));
    boost::replace_all(tarz,",",".");
    stream << to_string(3)+string(" ")+tarz+string(";\n");

    stream << string("# TARGET LEFT ORIENTATION \n");
    rpy_l = {tar_left.at(3),tar_left.at(4),tar_left.at(5)}; this->RPY_matrix(rpy_l,Rot_tar_l); Rot_tar_l_inv = Rot_tar_l.transpose();
    this->getRotAxis(xt_l,0,rpy_l);
    this->getRotAxis(yt_l,1,rpy_l);
    this->getRotAxis(zt_l,2,rpy_l);
    stream << string("param x_t_left := \n");
    tarxt0 =  boost::str(boost::format("%.2f") % (xt_l[0]));
    boost::replace_all(tarxt0,",",".");
    stream << to_string(1)+string(" ")+tarxt0+string("\n");
    tarxt1 =  boost::str(boost::format("%.2f") % (xt_l[1]));
    boost::replace_all(tarxt1,",",".");
    stream << to_string(2)+string(" ")+tarxt1+string("\n");
    tarxt2 =  boost::str(boost::format("%.2f") % (xt_l[2]));
    boost::replace_all(tarxt2,",",".");
    stream << to_string(3)+string(" ")+tarxt2+string(";\n");
    stream << string("param y_t_left := \n");
    taryt0 =  boost::str(boost::format("%.2f") % (yt_l[0]));
    boost::replace_all(taryt0,",",".");
    stream << to_string(1)+string(" ")+taryt0+string("\n");
    taryt1 =  boost::str(boost::format("%.2f") % (yt_l[1]));
    boost::replace_all(taryt1,",",".");
    stream << to_string(2)+string(" ")+taryt1+string("\n");
    taryt2 =  boost::str(boost::format("%.2f") % (yt_l[2]));
    boost::replace_all(taryt2,",",".");
    stream << to_string(3)+string(" ")+taryt2+string(";\n");
    stream << string("param z_t_left := \n");
    tarzt0 =  boost::str(boost::format("%.2f") % (zt_l[0]));
    boost::replace_all(tarzt0,",",".");
    stream << to_string(1)+string(" ")+tarzt0+string("\n");
    tarzt1 =  boost::str(boost::format("%.2f") % (zt_l[1]));
    boost::replace_all(tarzt1,",",".");
    stream << to_string(2)+string(" ")+tarzt1+string("\n");
    tarzt2 =  boost::str(boost::format("%.2f") % (zt_l[2]));
    boost::replace_all(tarzt2,",",".");
    stream << to_string(3)+string(" ")+tarzt2+string(";\n");

    /*
    stream << string("param Rot_tar_left_inv : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_tar_l_inv(0,0))+string(" ")+to_string(Rot_tar_l_inv(0,1))+string(" ")+to_string(Rot_tar_l_inv(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_tar_l_inv(1,0))+string(" ")+to_string(Rot_tar_l_inv(1,1))+string(" ")+to_string(Rot_tar_l_inv(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_tar_l_inv(2,0))+string(" ")+to_string(Rot_tar_l_inv(2,1))+string(" ")+to_string(Rot_tar_l_inv(2,2))+string(" \n");
    stream << string("; \n");
    */
}


void HUMPlanner::writeInfoApproachRetreat(ofstream &stream, std::vector<double> tar, std::vector<double> approach_retreat)
{
    double dist = approach_retreat.at(3);
    string dist_str = boost::str(boost::format("%.2f") % dist); boost::replace_all(dist_str,",",".");

    std::vector<double> rpy = {tar.at(3),tar.at(4),tar.at(5)};
    Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
    Vector3d v(approach_retreat.at(0),approach_retreat.at(1),approach_retreat.at(2));
    Vector3d vv = Rot_tar*v;


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

void HUMPlanner::writeDualInfoApproachRetreat(ofstream &stream, std::vector<double> tar, std::vector<double> approach_retreat, bool right)
{
    double dist; string dist_str; std::vector<double> rpy; Matrix3d Rot_tar;
    string tarxt0; string tarxt1; string tarxt2;

    if(right){
        dist = approach_retreat.at(3);
        dist_str = boost::str(boost::format("%.2f") % dist); boost::replace_all(dist_str,",",".");
        rpy = {tar.at(3),tar.at(4),tar.at(5)};
        this->RPY_matrix(rpy,Rot_tar);
        Vector3d v_r(approach_retreat.at(0),approach_retreat.at(1),approach_retreat.at(2));
        Vector3d vv_r = Rot_tar*v_r;
        stream << string("# VECTOR APPROACH/RETREAT RIGHT DISTANCE \n");
        stream << string("param dist_right :=")+dist_str+string(";\n");
        stream << string("# VECTOR APPROACH/RETREAT RIGHT ORIENTATION \n");
        stream << string("param v_t_right := \n");
        tarxt0 =  boost::str(boost::format("%.2f") % (vv_r(0)));
        boost::replace_all(tarxt0,",",".");
        stream << to_string(1)+string(" ")+tarxt0+string("\n");
        tarxt1 =  boost::str(boost::format("%.2f") % (vv_r(1)));
        boost::replace_all(tarxt1,",",".");
        stream << to_string(2)+string(" ")+tarxt1+string("\n");
        tarxt2 =  boost::str(boost::format("%.2f") % (vv_r(2)));
        boost::replace_all(tarxt2,",",".");
        stream << to_string(3)+string(" ")+tarxt2+string(";\n");
    }else{
        dist = approach_retreat.at(3);
        dist_str = boost::str(boost::format("%.2f") % dist); boost::replace_all(dist_str,",",".");
        rpy = {tar.at(3),tar.at(4),tar.at(5)};
        this->RPY_matrix(rpy,Rot_tar);
        Vector3d v_l(approach_retreat.at(0),approach_retreat.at(1),approach_retreat.at(2));
        Vector3d vv_l = Rot_tar*v_l;
        stream << string("# VECTOR APPROACH/RETREAT LEFT DISTANCE \n");
        stream << string("param dist_left :=")+dist_str+string(";\n");
        stream << string("# VECTOR APPROACH/RETREAT LEFT ORIENTATION \n");
        stream << string("param v_t_left := \n");
        tarxt0 =  boost::str(boost::format("%.2f") % (vv_l(0)));
        boost::replace_all(tarxt0,",",".");
        stream << to_string(1)+string(" ")+tarxt0+string("\n");
        tarxt1 =  boost::str(boost::format("%.2f") % (vv_l(1)));
        boost::replace_all(tarxt1,",",".");
        stream << to_string(2)+string(" ")+tarxt1+string("\n");
        tarxt2 =  boost::str(boost::format("%.2f") % (vv_l(2)));
        boost::replace_all(tarxt2,",",".");
        stream << to_string(3)+string(" ")+tarxt2+string(";\n");
    }
}


void HUMPlanner::writeInfoApproachRetreat_place(ofstream &stream, std::vector<double> tar, std::vector<double> approach, std::vector<double> retreat)
{
    double dist_app = approach.at(3);
    string dist_app_str = boost::str(boost::format("%.2f") % dist_app); boost::replace_all(dist_app_str,",",".");

    double dist_ret = retreat.at(3);
    string dist_ret_str = boost::str(boost::format("%.2f") % dist_ret); boost::replace_all(dist_ret_str,",",".");

    std::vector<double> rpy = {tar.at(3),tar.at(4),tar.at(5)};
    Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
    Vector3d v_app(approach.at(0),approach.at(1),approach.at(2));
    Vector3d vv_app =Rot_tar*v_app;
    Vector3d v_ret(retreat.at(0),retreat.at(1),retreat.at(2));
    Vector3d vv_ret =Rot_tar*v_ret;

    stream << string("# VECTOR APPROACH DISTANCE \n");
    stream << string("param dist_app :=")+dist_app_str+string(";\n");

    stream << string("# VECTOR APPROACH ORIENTATION \n");

    stream << string("param v_app := \n");
    string tarxt0_app =  boost::str(boost::format("%.2f") % (vv_app(0)));
    boost::replace_all(tarxt0_app,",",".");
    stream << to_string(1)+string(" ")+tarxt0_app+string("\n");

    string tarxt1_app =  boost::str(boost::format("%.2f") % (vv_app(1)));
    boost::replace_all(tarxt1_app,",",".");
    stream << to_string(2)+string(" ")+tarxt1_app+string("\n");

    string tarxt2_app =  boost::str(boost::format("%.2f") % (vv_app(2)));
    boost::replace_all(tarxt2_app,",",".");
    stream << to_string(3)+string(" ")+tarxt2_app+string(";\n");

    stream << string("# VECTOR RETREAT DISTANCE \n");
    stream << string("param dist_ret :=")+dist_ret_str+string(";\n");

    stream << string("# VECTOR RETREAT ORIENTATION \n");

    stream << string("param v_ret := \n");
    string tarxt0_ret =  boost::str(boost::format("%.2f") % (vv_ret(0)));
    boost::replace_all(tarxt0_ret,",",".");
    stream << to_string(1)+string(" ")+tarxt0_ret+string("\n");

    string tarxt1_ret =  boost::str(boost::format("%.2f") % (vv_ret(1)));
    boost::replace_all(tarxt1_ret,",",".");
    stream << to_string(2)+string(" ")+tarxt1_ret+string("\n");

    string tarxt2_ret =  boost::str(boost::format("%.2f") % (vv_ret(2)));
    boost::replace_all(tarxt2_ret,",",".");
    stream << to_string(3)+string(" ")+tarxt2_ret+string(";\n");



}


void HUMPlanner::writeInfoObstacles(ofstream &stream, std::vector<objectPtr> &obstacles)
{
    // obstacles
    stream << string("# OBSTACLES POSITION+RADIUS+ORIENTATION \n");

    if(!obstacles.empty()){
        stream << string("param Obstacles : 1 2 3 4 5 6 7 8 9 := \n");
    }

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

void HUMPlanner::writeDualInfoObstacles(ofstream &stream, std::vector<objectPtr> &obstacles, bool right)
{
    if(right)
    {
        // right arm-hand obstacles
        stream << string("# OBSTACLES OF THE RIGHT ARM-HAND POSITION+RADIUS+ORIENTATION \n");

        if(!obstacles.empty()){
            stream << string("param Obstacles_right : 1 2 3 4 5 6 7 8 9 := \n");
        }

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
             stream << string(" param n_Obstacles_right := ")+to_string(0)+string(";\n");
        }else{
            stream << string(" param n_Obstacles_right := ")+to_string(obstacles.size())+string(";\n");
        }
    }else{

        // left arm-hand obstacles
        stream << string("# OBSTACLES OF THE LEFT ARM-HAND POSITION+RADIUS+ORIENTATION \n");

        if(!obstacles.empty()){
            stream << string("param Obstacles_left : 1 2 3 4 5 6 7 8 9 := \n");
        }

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
             stream << string(" param n_Obstacles_left := ")+to_string(0)+string(";\n");
        }else{
            stream << string(" param n_Obstacles_left := ")+to_string(obstacles.size())+string(";\n");
        }

    }
}

void HUMPlanner::writeInfoObjectTarget(ofstream &stream, objectPtr obj)
{
    std::vector<double> position; obj->getPos(position);
    std::vector<double> orientation; obj->getOr(orientation);
    std::vector<double> dimension; obj->getSize(dimension);
    Matrix3d Rot_obj; this->RPY_matrix(orientation,Rot_obj);

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

    stream << string("param Rot_obj : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_obj(0,0))+string(" ")+to_string(Rot_obj(0,1))+string(" ")+to_string(Rot_obj(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_obj(1,0))+string(" ")+to_string(Rot_obj(1,1))+string(" ")+to_string(Rot_obj(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_obj(2,0))+string(" ")+to_string(Rot_obj(2,1))+string(" ")+to_string(Rot_obj(2,2))+string(" \n");
    stream << string("; \n");
}

void HUMPlanner::writeInfoObjectTargetPlaceRetreat(ofstream &stream,std::vector<double> tar, Matrix4d T_tar_to_obj,std::vector<double> dim, std::string name)
{
    string objx; string objy; string objz;
    string objroll; string objpitch; string objyaw;
    string objxsize; string objysize; string objzsize;

    Matrix4d T_tar; Matrix3d Rot_tar;
    std::vector<double> rpy = {tar.at(3),tar.at(4),tar.at(5)};
    this->RPY_matrix(rpy,Rot_tar);
    T_tar(0,0) = Rot_tar(0,0); T_tar(0,1) = Rot_tar(0,1); T_tar(0,2) = Rot_tar(0,2); T_tar(0,3) = tar.at(0);
    T_tar(1,0) = Rot_tar(1,0); T_tar(1,1) = Rot_tar(1,1); T_tar(1,2) = Rot_tar(1,2); T_tar(1,3) = tar.at(1);
    T_tar(2,0) = Rot_tar(2,0); T_tar(2,1) = Rot_tar(2,1); T_tar(2,2) = Rot_tar(2,2); T_tar(2,3) = tar.at(2);
    T_tar(3,0) = 0; T_tar(3,1) = 0; T_tar(3,2) = 0; T_tar(3,3) = 1;
    Matrix4d T_obj = T_tar * T_tar_to_obj;
    Matrix3d Rot_obj = T_obj.block<3,3>(0,0);
    Matrix3d Rot_tar_to_obj = T_tar_to_obj.block<3,3>(0,0);
    Vector3d tar_to_obj = T_tar_to_obj.block<3,1>(0,3);
    std::vector<double> rpy_obj; this->getRPY(rpy_obj,Rot_obj);
    std::vector<double> pos_obj = {T_obj(0,3),T_obj(1,3),T_obj(2,3)};

    stream << string("# OBJECT OF THE TARGET POSITION+RADIUS+ORIENTATION \n");
    stream << string("param ObjTar : 1 2 3 4 5 6 7 8 9 := \n");
    objx =  boost::str(boost::format("%.2f") % (pos_obj.at(0)));
    boost::replace_all(objx,",",".");
    objy =  boost::str(boost::format("%.2f") % (pos_obj.at(1)));
    boost::replace_all(objy,",",".");
    objz =  boost::str(boost::format("%.2f") % (pos_obj.at(2)));
    boost::replace_all(objz,",",".");
    objxsize =  boost::str(boost::format("%.2f") % (dim.at(0)/2));
    boost::replace_all(objxsize,",",".");
    objysize =  boost::str(boost::format("%.2f") % (dim.at(1)/2));
    boost::replace_all(objysize,",",".");
    objzsize =  boost::str(boost::format("%.2f") % (dim.at(2)/2));
    boost::replace_all(objzsize,",",".");
    objroll =  boost::str(boost::format("%.2f") % (rpy_obj.at(0)));
    boost::replace_all(objroll,",",".");
    objpitch =  boost::str(boost::format("%.2f") % (rpy_obj.at(1)));
    boost::replace_all(objpitch,",",".");
    objyaw =  boost::str(boost::format("%.2f") % (rpy_obj.at(2)));
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
                       string(" #")+name+
                       string("\n");
    stream << string(";\n");
    stream << string("param n_ObjTar := ")+to_string(1)+string(";\n");

    stream << string("param Rot_obj : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_obj(0,0))+string(" ")+to_string(Rot_obj(0,1))+string(" ")+to_string(Rot_obj(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_obj(1,0))+string(" ")+to_string(Rot_obj(1,1))+string(" ")+to_string(Rot_obj(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_obj(2,0))+string(" ")+to_string(Rot_obj(2,1))+string(" ")+to_string(Rot_obj(2,2))+string(" \n");
    stream << string("; \n");

}

 void HUMPlanner::writeInfoObjectTarget(ofstream &stream,std::vector<double> tar, Matrix4d T_tar_to_obj, std::vector<double> dim, std::string name)
 {
     string objx; string objy; string objz;
     string objroll; string objpitch; string objyaw;
     string objxsize; string objysize; string objzsize;

     Matrix4d T_tar; Matrix3d Rot_tar;
     std::vector<double> rpy = {tar.at(3),tar.at(4),tar.at(5)};
     this->RPY_matrix(rpy,Rot_tar);
     T_tar(0,0) = Rot_tar(0,0); T_tar(0,1) = Rot_tar(0,1); T_tar(0,2) = Rot_tar(0,2); T_tar(0,3) = tar.at(0);
     T_tar(1,0) = Rot_tar(1,0); T_tar(1,1) = Rot_tar(1,1); T_tar(1,2) = Rot_tar(1,2); T_tar(1,3) = tar.at(1);
     T_tar(2,0) = Rot_tar(2,0); T_tar(2,1) = Rot_tar(2,1); T_tar(2,2) = Rot_tar(2,2); T_tar(2,3) = tar.at(2);
     T_tar(3,0) = 0; T_tar(3,1) = 0; T_tar(3,2) = 0; T_tar(3,3) = 1;
     Matrix4d T_obj = T_tar * T_tar_to_obj;
     Matrix3d Rot_obj = T_obj.block<3,3>(0,0);
     Matrix3d Rot_tar_to_obj = T_tar_to_obj.block<3,3>(0,0);
     Vector3d tar_to_obj = T_tar_to_obj.block<3,1>(0,3);
     std::vector<double> rpy_obj; this->getRPY(rpy_obj,Rot_obj);
     std::vector<double> pos_obj = {T_obj(0,3),T_obj(1,3),T_obj(2,3)};

     stream << string("# OBJECT OF THE TARGET POSITION+RADIUS+ORIENTATION \n");
     stream << string("param ObjTar : 1 2 3 4 5 6 7 8 9 := \n");
     objx =  boost::str(boost::format("%.2f") % (pos_obj.at(0)));
     boost::replace_all(objx,",",".");
     objy =  boost::str(boost::format("%.2f") % (pos_obj.at(1)));
     boost::replace_all(objy,",",".");
     objz =  boost::str(boost::format("%.2f") % (pos_obj.at(2)));
     boost::replace_all(objz,",",".");
     objxsize =  boost::str(boost::format("%.2f") % (dim.at(0)/2));
     boost::replace_all(objxsize,",",".");
     objysize =  boost::str(boost::format("%.2f") % (dim.at(1)/2));
     boost::replace_all(objysize,",",".");
     objzsize =  boost::str(boost::format("%.2f") % (dim.at(2)/2));
     boost::replace_all(objzsize,",",".");
     objroll =  boost::str(boost::format("%.2f") % (rpy_obj.at(0)));
     boost::replace_all(objroll,",",".");
     objpitch =  boost::str(boost::format("%.2f") % (rpy_obj.at(1)));
     boost::replace_all(objpitch,",",".");
     objyaw =  boost::str(boost::format("%.2f") % (rpy_obj.at(2)));
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
                        string(" #")+name+
                        string("\n");
     stream << string(";\n");
     stream << string("param n_ObjTar := ")+to_string(1)+string(";\n");

     stream << string("param Rot_tar_obj : 1 2 3 :=  \n");
     stream << string("1 ")+to_string(Rot_tar_to_obj(0,0))+string(" ")+to_string(Rot_tar_to_obj(0,1))+string(" ")+to_string(Rot_tar_to_obj(0,2))+string(" \n");
     stream << string("2 ")+to_string(Rot_tar_to_obj(1,0))+string(" ")+to_string(Rot_tar_to_obj(1,1))+string(" ")+to_string(Rot_tar_to_obj(1,2))+string(" \n");
     stream << string("3 ")+to_string(Rot_tar_to_obj(2,0))+string(" ")+to_string(Rot_tar_to_obj(2,1))+string(" ")+to_string(Rot_tar_to_obj(2,2))+string(" \n");
     stream << string("; \n");

     stream << string("param tar_to_obj :=  \n");
     stream << string("1 ")+to_string(tar_to_obj(0))+string(" \n");
     stream << string("2 ")+to_string(tar_to_obj(1))+string(" \n");
     stream << string("3 ")+to_string(tar_to_obj(2))+string(" \n");
     stream << string("; \n");

     stream << string("param Rot_obj : 1 2 3 :=  \n");
     stream << string("1 ")+to_string(Rot_obj(0,0))+string(" ")+to_string(Rot_obj(0,1))+string(" ")+to_string(Rot_obj(0,2))+string(" \n");
     stream << string("2 ")+to_string(Rot_obj(1,0))+string(" ")+to_string(Rot_obj(1,1))+string(" ")+to_string(Rot_obj(1,2))+string(" \n");
     stream << string("3 ")+to_string(Rot_obj(2,0))+string(" ")+to_string(Rot_obj(2,1))+string(" ")+to_string(Rot_obj(2,2))+string(" \n");
     stream << string("; \n");

 }

void HUMPlanner::writeDualInfoObjectTarget(ofstream &stream, objectPtr obj_right, objectPtr obj_left)
{
    std::vector<double> position; string objx; string objy; string objz;
    std::vector<double> orientation; Matrix3d Rot_obj; string objroll; string objpitch; string objyaw;
    std::vector<double> dimension; string objxsize; string objysize; string objzsize;

    obj_right->getPos(position);
    obj_right->getOr(orientation); this->RPY_matrix(orientation,Rot_obj);
    obj_right->getSize(dimension);
    stream << string("# OBJECT OF THE RIGHT TARGET POSITION+RADIUS+ORIENTATION \n");
    stream << string("param ObjTar_right : 1 2 3 4 5 6 7 8 9 := \n");
    objx =  boost::str(boost::format("%.2f") % (position.at(0)));
    boost::replace_all(objx,",",".");
    objy =  boost::str(boost::format("%.2f") % (position.at(1)));
    boost::replace_all(objy,",",".");
    objz =  boost::str(boost::format("%.2f") % (position.at(2)));
    boost::replace_all(objz,",",".");
    objxsize =  boost::str(boost::format("%.2f") % (dimension.at(0)/2));
    boost::replace_all(objxsize,",",".");
    objysize =  boost::str(boost::format("%.2f") % (dimension.at(1)/2));
    boost::replace_all(objysize,",",".");
    objzsize =  boost::str(boost::format("%.2f") % (dimension.at(2)/2));
    boost::replace_all(objzsize,",",".");
    objroll =  boost::str(boost::format("%.2f") % (orientation.at(0)));
    boost::replace_all(objroll,",",".");
    objpitch =  boost::str(boost::format("%.2f") % (orientation.at(1)));
    boost::replace_all(objpitch,",",".");
    objyaw =  boost::str(boost::format("%.2f") % (orientation.at(2)));
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
                       string(" #")+obj_right->getName()+
                       string("\n");
    stream << string(";\n");
    stream << string("param n_ObjTar_right := ")+to_string(1)+string(";\n");


    stream << string("param Rot_obj_right : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_obj(0,0))+string(" ")+to_string(Rot_obj(0,1))+string(" ")+to_string(Rot_obj(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_obj(1,0))+string(" ")+to_string(Rot_obj(1,1))+string(" ")+to_string(Rot_obj(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_obj(2,0))+string(" ")+to_string(Rot_obj(2,1))+string(" ")+to_string(Rot_obj(2,2))+string(" \n");
    stream << string("; \n");


    obj_left->getPos(position);
    obj_left->getOr(orientation); this->RPY_matrix(orientation,Rot_obj);
    obj_left->getSize(dimension);
    stream << string("# OBJECT OF THE LEFT TARGET POSITION+RADIUS+ORIENTATION \n");
    stream << string("param ObjTar_left : 1 2 3 4 5 6 7 8 9 := \n");
    objx =  boost::str(boost::format("%.2f") % (position.at(0)));
    boost::replace_all(objx,",",".");
    objy =  boost::str(boost::format("%.2f") % (position.at(1)));
    boost::replace_all(objy,",",".");
    objz =  boost::str(boost::format("%.2f") % (position.at(2)));
    boost::replace_all(objz,",",".");
    objxsize =  boost::str(boost::format("%.2f") % (dimension.at(0)/2));
    boost::replace_all(objxsize,",",".");
    objysize =  boost::str(boost::format("%.2f") % (dimension.at(1)/2));
    boost::replace_all(objysize,",",".");
    objzsize =  boost::str(boost::format("%.2f") % (dimension.at(2)/2));
    boost::replace_all(objzsize,",",".");
    objroll =  boost::str(boost::format("%.2f") % (orientation.at(0)));
    boost::replace_all(objroll,",",".");
    objpitch =  boost::str(boost::format("%.2f") % (orientation.at(1)));
    boost::replace_all(objpitch,",",".");
    objyaw =  boost::str(boost::format("%.2f") % (orientation.at(2)));
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
                       string(" #")+obj_left->getName()+
                       string("\n");
    stream << string(";\n");
    stream << string("param n_ObjTar_left := ")+to_string(1)+string(";\n");


    stream << string("param Rot_obj_left : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_obj(0,0))+string(" ")+to_string(Rot_obj(0,1))+string(" ")+to_string(Rot_obj(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_obj(1,0))+string(" ")+to_string(Rot_obj(1,1))+string(" ")+to_string(Rot_obj(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_obj(2,0))+string(" ")+to_string(Rot_obj(2,1))+string(" ")+to_string(Rot_obj(2,2))+string(" \n");
    stream << string("; \n");


}

void HUMPlanner::writeDualInfoObjectTarget(ofstream &stream, std::vector<double> tar_right, Matrix4d T_tar_to_obj_right, std::vector<double> dim_right, std::string name_right, std::vector<double> tar_left, Matrix4d T_tar_to_obj_left,std::vector<double> dim_left,std::string name_left)
{
    string objx; string objy; string objz;
    string objroll; string objpitch; string objyaw;
    string objxsize; string objysize; string objzsize;

    Matrix4d T_tar_right; Matrix3d Rot_tar_right;
    std::vector<double> rpy_r = {tar_right.at(3),tar_right.at(4),tar_right.at(5)};
    this->RPY_matrix(rpy_r,Rot_tar_right);
    T_tar_right(0,0) = Rot_tar_right(0,0); T_tar_right(0,1) = Rot_tar_right(0,1); T_tar_right(0,2) = Rot_tar_right(0,2); T_tar_right(0,3) = tar_right.at(0);
    T_tar_right(1,0) = Rot_tar_right(1,0); T_tar_right(1,1) = Rot_tar_right(1,1); T_tar_right(1,2) = Rot_tar_right(1,2); T_tar_right(1,3) = tar_right.at(1);
    T_tar_right(2,0) = Rot_tar_right(2,0); T_tar_right(2,1) = Rot_tar_right(2,1); T_tar_right(2,2) = Rot_tar_right(2,2); T_tar_right(2,3) = tar_right.at(2);
    T_tar_right(3,0) = 0; T_tar_right(3,1) = 0; T_tar_right(3,2) = 0; T_tar_right(3,3) = 1;
    Matrix4d T_obj_right = T_tar_right * T_tar_to_obj_right;
    Matrix3d Rot_obj_right = T_obj_right.block<3,3>(0,0);
    Matrix3d Rot_tar_to_obj_right = T_tar_to_obj_right.block<3,3>(0,0);
    Vector3d tar_to_obj_right = T_tar_to_obj_right.block<3,1>(0,3);
    std::vector<double> rpy_obj_right; this->getRPY(rpy_obj_right,Rot_obj_right);
    std::vector<double> pos_obj_right = {T_obj_right(0,3),T_obj_right(1,3),T_obj_right(2,3)};

    stream << string("# OBJECT OF THE RIGHT TARGET POSITION+RADIUS+ORIENTATION \n");
    stream << string("param ObjTar_right : 1 2 3 4 5 6 7 8 9 := \n");
    objx =  boost::str(boost::format("%.2f") % (pos_obj_right.at(0)));
    boost::replace_all(objx,",",".");
    objy =  boost::str(boost::format("%.2f") % (pos_obj_right.at(1)));
    boost::replace_all(objy,",",".");
    objz =  boost::str(boost::format("%.2f") % (pos_obj_right.at(2)));
    boost::replace_all(objz,",",".");
    objxsize =  boost::str(boost::format("%.2f") % (dim_right.at(0)/2));
    boost::replace_all(objxsize,",",".");
    objysize =  boost::str(boost::format("%.2f") % (dim_right.at(1)/2));
    boost::replace_all(objysize,",",".");
    objzsize =  boost::str(boost::format("%.2f") % (dim_right.at(2)/2));
    boost::replace_all(objzsize,",",".");
    objroll =  boost::str(boost::format("%.2f") % (rpy_obj_right.at(0)));
    boost::replace_all(objroll,",",".");
    objpitch =  boost::str(boost::format("%.2f") % (rpy_obj_right.at(1)));
    boost::replace_all(objpitch,",",".");
    objyaw =  boost::str(boost::format("%.2f") % (rpy_obj_right.at(2)));
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
                       string(" #")+name_right+
                       string("\n");
    stream << string(";\n");
    stream << string("param n_ObjTar_right := ")+to_string(1)+string(";\n");

    stream << string("param Rot_tar_obj_right : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_tar_to_obj_right(0,0))+string(" ")+to_string(Rot_tar_to_obj_right(0,1))+string(" ")+to_string(Rot_tar_to_obj_right(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_tar_to_obj_right(1,0))+string(" ")+to_string(Rot_tar_to_obj_right(1,1))+string(" ")+to_string(Rot_tar_to_obj_right(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_tar_to_obj_right(2,0))+string(" ")+to_string(Rot_tar_to_obj_right(2,1))+string(" ")+to_string(Rot_tar_to_obj_right(2,2))+string(" \n");
    stream << string("; \n");

    stream << string("param tar_to_obj_right :=  \n");
    stream << string("1 ")+to_string(tar_to_obj_right(0))+string(" \n");
    stream << string("2 ")+to_string(tar_to_obj_right(1))+string(" \n");
    stream << string("3 ")+to_string(tar_to_obj_right(2))+string(" \n");
    stream << string("; \n");

    stream << string("param Rot_obj_right : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_obj_right(0,0))+string(" ")+to_string(Rot_obj_right(0,1))+string(" ")+to_string(Rot_obj_right(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_obj_right(1,0))+string(" ")+to_string(Rot_obj_right(1,1))+string(" ")+to_string(Rot_obj_right(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_obj_right(2,0))+string(" ")+to_string(Rot_obj_right(2,1))+string(" ")+to_string(Rot_obj_right(2,2))+string(" \n");
    stream << string("; \n");

    Matrix4d T_tar_left; Matrix3d Rot_tar_left;
    std::vector<double> rpy_l = {tar_left.at(3),tar_left.at(4),tar_left.at(5)};
    this->RPY_matrix(rpy_l,Rot_tar_left);
    T_tar_left(0,0) = Rot_tar_left(0,0); T_tar_left(0,1) = Rot_tar_left(0,1); T_tar_left(0,2) = Rot_tar_left(0,2); T_tar_left(0,3) = tar_left.at(0);
    T_tar_left(1,0) = Rot_tar_left(1,0); T_tar_left(1,1) = Rot_tar_left(1,1); T_tar_left(1,2) = Rot_tar_left(1,2); T_tar_left(1,3) = tar_left.at(1);
    T_tar_left(2,0) = Rot_tar_left(2,0); T_tar_left(2,1) = Rot_tar_left(2,1); T_tar_left(2,2) = Rot_tar_left(2,2); T_tar_left(2,3) = tar_left.at(2);
    T_tar_left(3,0) = 0; T_tar_left(3,1) = 0; T_tar_left(3,2) = 0; T_tar_left(3,3) = 1;
    Matrix4d T_obj_left = T_tar_left * T_tar_to_obj_left;
    Matrix3d Rot_obj_left = T_obj_left.block<3,3>(0,0);
    Matrix3d Rot_tar_to_obj_left = T_tar_to_obj_left.block<3,3>(0,0);
    Vector3d tar_to_obj_left = T_tar_to_obj_left.block<3,1>(0,3);
    std::vector<double> rpy_obj_left; this->getRPY(rpy_obj_left,Rot_obj_left);
    std::vector<double> pos_obj_left = {T_obj_left(0,3),T_obj_left(1,3),T_obj_left(2,3)};

    stream << string("# OBJECT OF THE LEFT TARGET POSITION+RADIUS+ORIENTATION \n");
    stream << string("param ObjTar_left : 1 2 3 4 5 6 7 8 9 := \n");
    objx =  boost::str(boost::format("%.2f") % (pos_obj_left.at(0)));
    boost::replace_all(objx,",",".");
    objy =  boost::str(boost::format("%.2f") % (pos_obj_left.at(1)));
    boost::replace_all(objy,",",".");
    objz =  boost::str(boost::format("%.2f") % (pos_obj_left.at(2)));
    boost::replace_all(objz,",",".");
    objxsize =  boost::str(boost::format("%.2f") % (dim_left.at(0)/2));
    boost::replace_all(objxsize,",",".");
    objysize =  boost::str(boost::format("%.2f") % (dim_left.at(1)/2));
    boost::replace_all(objysize,",",".");
    objzsize =  boost::str(boost::format("%.2f") % (dim_left.at(2)/2));
    boost::replace_all(objzsize,",",".");
    objroll =  boost::str(boost::format("%.2f") % (rpy_obj_left.at(0)));
    boost::replace_all(objroll,",",".");
    objpitch =  boost::str(boost::format("%.2f") % (rpy_obj_left.at(1)));
    boost::replace_all(objpitch,",",".");
    objyaw =  boost::str(boost::format("%.2f") % (rpy_obj_left.at(2)));
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
                       string(" #")+name_left+
                       string("\n");
    stream << string(";\n");
    stream << string("param n_ObjTar_left := ")+to_string(1)+string(";\n");

    stream << string("param Rot_tar_obj_left : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_tar_to_obj_left(0,0))+string(" ")+to_string(Rot_tar_to_obj_left(0,1))+string(" ")+to_string(Rot_tar_to_obj_left(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_tar_to_obj_left(1,0))+string(" ")+to_string(Rot_tar_to_obj_left(1,1))+string(" ")+to_string(Rot_tar_to_obj_left(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_tar_to_obj_left(2,0))+string(" ")+to_string(Rot_tar_to_obj_left(2,1))+string(" ")+to_string(Rot_tar_to_obj_left(2,2))+string(" \n");
    stream << string("; \n");

    stream << string("param tar_to_obj_left :=  \n");
    stream << string("1 ")+to_string(tar_to_obj_left(0))+string(" \n");
    stream << string("2 ")+to_string(tar_to_obj_left(1))+string(" \n");
    stream << string("3 ")+to_string(tar_to_obj_left(2))+string(" \n");
    stream << string("; \n");

    stream << string("param Rot_obj_left : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_obj_left(0,0))+string(" ")+to_string(Rot_obj_left(0,1))+string(" ")+to_string(Rot_obj_left(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_obj_left(1,0))+string(" ")+to_string(Rot_obj_left(1,1))+string(" ")+to_string(Rot_obj_left(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_obj_left(2,0))+string(" ")+to_string(Rot_obj_left(2,1))+string(" ")+to_string(Rot_obj_left(2,2))+string(" \n");
    stream << string("; \n");
}

void HUMPlanner::writeDualInfoObjectTargetPlaceRetreat(ofstream &stream, std::vector<double> tar_right, Matrix4d T_tar_to_obj_right, std::vector<double> dim_right, std::string name_right, std::vector<double> tar_left, Matrix4d T_tar_to_obj_left,std::vector<double> dim_left,std::string name_left)
{
    string objx; string objy; string objz;
    string objroll; string objpitch; string objyaw;
    string objxsize; string objysize; string objzsize;

    Matrix4d T_tar_right; Matrix3d Rot_tar_right;
    std::vector<double> rpy_r = {tar_right.at(3),tar_right.at(4),tar_right.at(5)};
    this->RPY_matrix(rpy_r,Rot_tar_right);
    T_tar_right(0,0) = Rot_tar_right(0,0); T_tar_right(0,1) = Rot_tar_right(0,1); T_tar_right(0,2) = Rot_tar_right(0,2); T_tar_right(0,3) = tar_right.at(0);
    T_tar_right(1,0) = Rot_tar_right(1,0); T_tar_right(1,1) = Rot_tar_right(1,1); T_tar_right(1,2) = Rot_tar_right(1,2); T_tar_right(1,3) = tar_right.at(1);
    T_tar_right(2,0) = Rot_tar_right(2,0); T_tar_right(2,1) = Rot_tar_right(2,1); T_tar_right(2,2) = Rot_tar_right(2,2); T_tar_right(2,3) = tar_right.at(2);
    T_tar_right(3,0) = 0; T_tar_right(3,1) = 0; T_tar_right(3,2) = 0; T_tar_right(3,3) = 1;
    Matrix4d T_obj_right = T_tar_right * T_tar_to_obj_right;
    Matrix3d Rot_obj_right = T_obj_right.block<3,3>(0,0);
    Matrix3d Rot_tar_to_obj_right = T_tar_to_obj_right.block<3,3>(0,0);
    Vector3d tar_to_obj_right = T_tar_to_obj_right.block<3,1>(0,3);
    std::vector<double> rpy_obj_right; this->getRPY(rpy_obj_right,Rot_obj_right);
    std::vector<double> pos_obj_right = {T_obj_right(0,3),T_obj_right(1,3),T_obj_right(2,3)};

    stream << string("# OBJECT OF THE RIGHT TARGET POSITION+RADIUS+ORIENTATION \n");
    stream << string("param ObjTar_right : 1 2 3 4 5 6 7 8 9 := \n");
    objx =  boost::str(boost::format("%.2f") % (pos_obj_right.at(0)));
    boost::replace_all(objx,",",".");
    objy =  boost::str(boost::format("%.2f") % (pos_obj_right.at(1)));
    boost::replace_all(objy,",",".");
    objz =  boost::str(boost::format("%.2f") % (pos_obj_right.at(2)));
    boost::replace_all(objz,",",".");
    objxsize =  boost::str(boost::format("%.2f") % (dim_right.at(0)/2));
    boost::replace_all(objxsize,",",".");
    objysize =  boost::str(boost::format("%.2f") % (dim_right.at(1)/2));
    boost::replace_all(objysize,",",".");
    objzsize =  boost::str(boost::format("%.2f") % (dim_right.at(2)/2));
    boost::replace_all(objzsize,",",".");
    objroll =  boost::str(boost::format("%.2f") % (rpy_obj_right.at(0)));
    boost::replace_all(objroll,",",".");
    objpitch =  boost::str(boost::format("%.2f") % (rpy_obj_right.at(1)));
    boost::replace_all(objpitch,",",".");
    objyaw =  boost::str(boost::format("%.2f") % (rpy_obj_right.at(2)));
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
                       string(" #")+name_right+
                       string("\n");
    stream << string(";\n");
    stream << string("param n_ObjTar_right := ")+to_string(1)+string(";\n");

    stream << string("param Rot_obj_right : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_obj_right(0,0))+string(" ")+to_string(Rot_obj_right(0,1))+string(" ")+to_string(Rot_obj_right(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_obj_right(1,0))+string(" ")+to_string(Rot_obj_right(1,1))+string(" ")+to_string(Rot_obj_right(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_obj_right(2,0))+string(" ")+to_string(Rot_obj_right(2,1))+string(" ")+to_string(Rot_obj_right(2,2))+string(" \n");
    stream << string("; \n");

    Matrix4d T_tar_left; Matrix3d Rot_tar_left;
    std::vector<double> rpy_l = {tar_left.at(3),tar_left.at(4),tar_left.at(5)};
    this->RPY_matrix(rpy_l,Rot_tar_left);
    T_tar_left(0,0) = Rot_tar_left(0,0); T_tar_left(0,1) = Rot_tar_left(0,1); T_tar_left(0,2) = Rot_tar_left(0,2); T_tar_left(0,3) = tar_left.at(0);
    T_tar_left(1,0) = Rot_tar_left(1,0); T_tar_left(1,1) = Rot_tar_left(1,1); T_tar_left(1,2) = Rot_tar_left(1,2); T_tar_left(1,3) = tar_left.at(1);
    T_tar_left(2,0) = Rot_tar_left(2,0); T_tar_left(2,1) = Rot_tar_left(2,1); T_tar_left(2,2) = Rot_tar_left(2,2); T_tar_left(2,3) = tar_left.at(2);
    T_tar_left(3,0) = 0; T_tar_left(3,1) = 0; T_tar_left(3,2) = 0; T_tar_left(3,3) = 1;
    Matrix4d T_obj_left = T_tar_left * T_tar_to_obj_left;
    Matrix3d Rot_obj_left = T_obj_left.block<3,3>(0,0);
    Matrix3d Rot_tar_to_obj_left = T_tar_to_obj_left.block<3,3>(0,0);
    Vector3d tar_to_obj_left = T_tar_to_obj_left.block<3,1>(0,3);
    std::vector<double> rpy_obj_left; this->getRPY(rpy_obj_left,Rot_obj_left);
    std::vector<double> pos_obj_left = {T_obj_left(0,3),T_obj_left(1,3),T_obj_left(2,3)};

    stream << string("# OBJECT OF THE LEFT TARGET POSITION+RADIUS+ORIENTATION \n");
    stream << string("param ObjTar_left : 1 2 3 4 5 6 7 8 9 := \n");
    objx =  boost::str(boost::format("%.2f") % (pos_obj_left.at(0)));
    boost::replace_all(objx,",",".");
    objy =  boost::str(boost::format("%.2f") % (pos_obj_left.at(1)));
    boost::replace_all(objy,",",".");
    objz =  boost::str(boost::format("%.2f") % (pos_obj_left.at(2)));
    boost::replace_all(objz,",",".");
    objxsize =  boost::str(boost::format("%.2f") % (dim_left.at(0)/2));
    boost::replace_all(objxsize,",",".");
    objysize =  boost::str(boost::format("%.2f") % (dim_left.at(1)/2));
    boost::replace_all(objysize,",",".");
    objzsize =  boost::str(boost::format("%.2f") % (dim_left.at(2)/2));
    boost::replace_all(objzsize,",",".");
    objroll =  boost::str(boost::format("%.2f") % (rpy_obj_left.at(0)));
    boost::replace_all(objroll,",",".");
    objpitch =  boost::str(boost::format("%.2f") % (rpy_obj_left.at(1)));
    boost::replace_all(objpitch,",",".");
    objyaw =  boost::str(boost::format("%.2f") % (rpy_obj_left.at(2)));
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
                       string(" #")+name_left+
                       string("\n");
    stream << string(";\n");
    stream << string("param n_ObjTar_left := ")+to_string(1)+string(";\n");

    stream << string("param Rot_obj_left : 1 2 3 :=  \n");
    stream << string("1 ")+to_string(Rot_obj_left(0,0))+string(" ")+to_string(Rot_obj_left(0,1))+string(" ")+to_string(Rot_obj_left(0,2))+string(" \n");
    stream << string("2 ")+to_string(Rot_obj_left(1,0))+string(" ")+to_string(Rot_obj_left(1,1))+string(" ")+to_string(Rot_obj_left(1,2))+string(" \n");
    stream << string("3 ")+to_string(Rot_obj_left(2,0))+string(" ")+to_string(Rot_obj_left(2,1))+string(" ")+to_string(Rot_obj_left(2,2))+string(" \n");
    stream << string("; \n");
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

void HUMPlanner::writeDualArmDHParamsMod(ofstream &stream)
{
    stream << string("# D-H parameters of the right arm \n");
    stream << string("param alpha_right {i in 1..")+to_string(joints_arm)+string("} ; \n");
    stream << string("param a_right {i in 1..")+to_string(joints_arm)+string("} ; \n");
    stream << string("param d_right {i in 1..")+to_string(joints_arm)+string("} ; \n");

    stream << string("# D-H parameters of the left arm \n");
    stream << string("param alpha_left {i in 1..")+to_string(joints_arm)+string("} ; \n");
    stream << string("param a_left {i in 1..")+to_string(joints_arm)+string("} ; \n");
    stream << string("param d_left {i in 1..")+to_string(joints_arm)+string("} ; \n");
}

void HUMPlanner::write_dHOMod(ofstream &stream)
{
    stream << string("# Distance hand - target  \n");
    stream << string("param dFH; \n");
}

void HUMPlanner::write_dual_dHOMod(ofstream &stream)
{
    stream << string("# Distance right hand - target  \n");
    stream << string("param dFH_right; \n");

    stream << string("# Distance left hand - target  \n");
    stream << string("param dFH_left; \n");
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
    stream << string("param Rot_obj {i1 in 1..3, i2 in 1..3};\n");
}

void HUMPlanner::writeDualInfoObjectsMod(ofstream &stream,bool vec_right,bool vec_left)
{

    stream << string("# Right Target Position \n");
    stream << string("param Tar_pos_right {i in 1..3}; \n");
    stream << string("# Right Target orientation \n");
    stream << string("param x_t_right {i in 1..3}; \n");
    stream << string("param y_t_right {i in 1..3}; \n");
    stream << string("param z_t_right {i in 1..3}; \n");

    stream << string("# Left Target Position \n");
    stream << string("param Tar_pos_left {i in 1..3}; \n");
    stream << string("# Left Target orientation \n");
    stream << string("param x_t_left {i in 1..3}; \n");
    stream << string("param y_t_left {i in 1..3}; \n");
    stream << string("param z_t_left {i in 1..3}; \n");

    if(vec_right){
        stream << string("# Vector approach/retreat right distance \n");
        stream << string("param dist_right; \n");
        stream << string("# Vector approach/retreat right orientation \n");
        stream << string("param v_t_right {i in 1..3}; \n");
    }
    if(vec_left){
        stream << string("# Vector approach/retreat left distance \n");
        stream << string("param dist_left; \n");
        stream << string("# Vector approach/retreat left orientation \n");
        stream << string("param v_t_left {i in 1..3}; \n");
    }

    stream << string("# Obstacles for the right arm-hand \n");
    stream << string("param n_Obstacles_right; \n");
    stream << string("param Obstacles_right {i in 1..n_Obstacles_right, j in 1..9}; \n");

    stream << string("# Obstacles for the left arm-hand \n");
    stream << string("param n_Obstacles_left; \n");
    stream << string("param Obstacles_left {i in 1..n_Obstacles_left, j in 1..9}; \n");

    stream << string("# Object of the right target \n");
    stream << string("param n_ObjTar_right; \n");
    stream << string("param ObjTar_right {i in 1..n_ObjTar_right, j in 1..9}; \n");
    stream << string("param Rot_obj_right {i1 in 1..3, i2 in 1..3};\n");

    stream << string("# Object of the left target \n");
    stream << string("param n_ObjTar_left; \n");
    stream << string("param ObjTar_left {i in 1..n_ObjTar_left, j in 1..9}; \n");
    stream << string("param Rot_obj_left {i1 in 1..3, i2 in 1..3};\n");

}

void HUMPlanner::writeInfoObjectsMod_place(ofstream &stream, bool vec)
{
    stream << string("# Target Position \n");
    stream << string("param Tar_pos {i in 1..3}; \n");
    stream << string("# Target orientation \n");
    stream << string("param x_t {i in 1..3}; \n");
    stream << string("param y_t {i in 1..3}; \n");
    stream << string("param z_t {i in 1..3}; \n");

    if(vec){
        stream << string("# Vector approach distance \n");
        stream << string("param dist_app; \n");
        stream << string("# Vector approach orientation \n");
        stream << string("param v_app {i in 1..3}; \n");
        stream << string("# Vector retreat distance \n");
        stream << string("param dist_ret; \n");
        stream << string("# Vector retreat orientation \n");
        stream << string("param v_ret {i in 1..3}; \n");
    }

    stream << string("# Obstacles \n");
    stream << string("param n_Obstacles; \n");
    stream << string("param Obstacles {i in 1..n_Obstacles, j in 1..9}; \n");
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

    stream << string("param Rot {i1 in 1..3, i2 in 1..3,i in 1..n_Obstacles} :=  \n");
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

void HUMPlanner::writeDualRotMatObsts(ofstream &stream)
{
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("# Rotation matrix of the obstacles of the right arm-hand \n");
    stream << string("param c_roll_right {i in 1..n_Obstacles_right} := cos(Obstacles_right[i,7]); \n");
    stream << string("param s_roll_right {i in 1..n_Obstacles_right} := sin(Obstacles_right[i,7]); \n");
    stream << string("param c_pitch_right {i in 1..n_Obstacles_right} := cos(Obstacles_right[i,8]); \n");
    stream << string("param s_pitch_right {i in 1..n_Obstacles_right} := sin(Obstacles_right[i,8]); \n");
    stream << string("param c_yaw_right {i in 1..n_Obstacles_right} := cos(Obstacles_right[i,9]); \n");
    stream << string("param s_yaw_right {i in 1..n_Obstacles_right} := sin(Obstacles_right[i,9]); \n");

    stream << string("param Rot_right {i1 in 1..3, i2 in 1..3,i in 1..n_Obstacles_right} :=  \n");
    stream << string("# 1st row \n");
    stream << string("if 		   ( i1=1 && i2=1 ) then 	c_roll_right[i]*c_pitch_right[i] \n");
    stream << string("else	if ( i1=1 && i2=2 ) then   -s_roll_right[i]*c_yaw_right[i]+c_roll_right[i]*s_pitch_right[i]*s_yaw_right[i] \n");
    stream << string("else	if ( i1=1 && i2=3 ) then 	s_roll_right[i]*s_yaw_right[i]+c_roll_right[i]*s_pitch_right[i]*c_yaw_right[i] \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then 	s_roll_right[i]*c_pitch_right[i] \n");
    stream << string("else	if ( i1=2 && i2=2 ) then 	c_roll_right[i]*c_yaw_right[i]+s_roll_right[i]*s_pitch_right[i]*s_yaw_right[i] \n");
    stream << string("else	if ( i1=2 && i2=3 ) then   -c_roll_right[i]*s_yaw_right[i]+s_roll_right[i]*s_pitch_right[i]*c_yaw_right[i] \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then   -s_pitch_right[i] \n");
    stream << string("else	if ( i1=3 && i2=2 ) then	c_pitch_right[i]*s_yaw_right[i] \n");
    stream << string("else	if ( i1=3 && i2=3 ) then	c_pitch_right[i]*c_yaw_right[i] \n");
    stream << string("   ; \n");

    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("# Rotation matrix of the obstacles of the left arm-hand \n");
    stream << string("param c_roll_left {i in 1..n_Obstacles_left} := cos(Obstacles_left[i,7]); \n");
    stream << string("param s_roll_left {i in 1..n_Obstacles_left} := sin(Obstacles_left[i,7]); \n");
    stream << string("param c_pitch_left {i in 1..n_Obstacles_left} := cos(Obstacles_left[i,8]); \n");
    stream << string("param s_pitch_left {i in 1..n_Obstacles_left} := sin(Obstacles_left[i,8]); \n");
    stream << string("param c_yaw_left {i in 1..n_Obstacles_left} := cos(Obstacles_left[i,9]); \n");
    stream << string("param s_yaw_left {i in 1..n_Obstacles_left} := sin(Obstacles_left[i,9]); \n");

    stream << string("param Rot_left {i1 in 1..3, i2 in 1..3,i in 1..n_Obstacles_left} :=  \n");
    stream << string("# 1st row \n");
    stream << string("if 		   ( i1=1 && i2=1 ) then 	c_roll_left[i]*c_pitch_left[i] \n");
    stream << string("else	if ( i1=1 && i2=2 ) then   -s_roll_left[i]*c_yaw_left[i]+c_roll_left[i]*s_pitch_left[i]*s_yaw_left[i] \n");
    stream << string("else	if ( i1=1 && i2=3 ) then 	s_roll_left[i]*s_yaw_left[i]+c_roll_left[i]*s_pitch_left[i]*c_yaw_left[i] \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then 	s_roll_left[i]*c_pitch_left[i] \n");
    stream << string("else	if ( i1=2 && i2=2 ) then 	c_roll_left[i]*c_yaw_left[i]+s_roll_left[i]*s_pitch_left[i]*s_yaw_left[i] \n");
    stream << string("else	if ( i1=2 && i2=3 ) then   -c_roll_left[i]*s_yaw_left[i]+s_roll_left[i]*s_pitch_left[i]*c_yaw_left[i] \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then   -s_pitch_left[i] \n");
    stream << string("else	if ( i1=3 && i2=2 ) then	c_pitch_left[i]*s_yaw_left[i] \n");
    stream << string("else	if ( i1=3 && i2=3 ) then	c_pitch_left[i]*c_yaw_left[i] \n");
    stream << string("   ; \n");

}

void HUMPlanner::writeRotMatObjTar(ofstream &stream)
{
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("# Rotation matrix target to object to place by the arm \n");
    stream << string("param Rot_tar_obj {i1 in 1..3, i2 in 1..3}; \n");
    stream << string("param tar_to_obj {i in 1..3}; \n");
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
}

void HUMPlanner::writeRotMatObjTarDual(ofstream &stream)
{
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("# Rotation matrix target to object to place by the right arm \n");
    stream << string("param Rot_tar_obj_right {i1 in 1..3, i2 in 1..3}; \n");
    stream << string("param tar_to_obj_right {i in 1..3}; \n");
    stream << string("# Rotation matrix target to object to place by the left arm \n");
    stream << string("param Rot_tar_obj_left {i1 in 1..3, i2 in 1..3}; \n");
    stream << string("param tar_to_obj_left {i in 1..3}; \n");
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");    
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
        stream << string("var Rot_H {j in 1..3,k in 1..3} = T_W_H [j,k]; \n");

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
        stream << string("var Rot_H {j in 1..3,k in 1..3,i in Iterations} = T_W_H [j,k,i]; \n");
    }
}

void HUMPlanner::writeDualArmDirKin(ofstream &stream, int dual_mov_type,Matrix4d &matWorldToRightArm, Matrix4d &matRightHand, std::vector<double>& tolsRightArm, Matrix4d &matWorldToLeftArm, Matrix4d &matLeftHand, std::vector<double>& tolsLeftArm,bool final)
{
    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  Direct Kinematics model of the right arm \n\n");

    stream << string("param c_alpha_right {i in 1..7} := cos(alpha_right[i]); \n");
    stream << string("param s_alpha_right {i in 1..7} := sin(alpha_right[i]); \n");
    stream << string("\n");

    string mat00_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(0,0))); boost::replace_all(mat00_right,",",".");
    string mat01_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(0,1))); boost::replace_all(mat01_right,",",".");
    string mat02_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(0,2))); boost::replace_all(mat02_right,",",".");
    string mat03_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(0,3))); boost::replace_all(mat03_right,",",".");
    string mat10_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(1,0))); boost::replace_all(mat10_right,",",".");
    string mat11_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(1,1))); boost::replace_all(mat11_right,",",".");
    string mat12_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(1,2))); boost::replace_all(mat12_right,",",".");
    string mat13_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(1,3))); boost::replace_all(mat13_right,",",".");
    string mat20_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(2,0))); boost::replace_all(mat20_right,",",".");
    string mat21_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(2,1))); boost::replace_all(mat21_right,",",".");
    string mat22_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(2,2))); boost::replace_all(mat22_right,",",".");
    string mat23_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(2,3))); boost::replace_all(mat23_right,",",".");
    string mat30_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(3,0))); boost::replace_all(mat30_right,",",".");
    string mat31_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(3,1))); boost::replace_all(mat31_right,",",".");
    string mat32_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(3,2))); boost::replace_all(mat32_right,",",".");
    string mat33_right =  boost::str(boost::format("%.2f") % (matWorldToRightArm(3,3))); boost::replace_all(mat33_right,",",".");

    stream << string("param T_WorldToRightArm {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then ")+mat00_right+string("  \n");
    stream << string("else	if ( i1=1 && i2=2 ) then ")+mat01_right+string("  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then ")+mat02_right+string("  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then ")+mat03_right+string("  \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then ")+mat10_right+string("  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then ")+mat11_right+string("  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then ")+mat12_right+string("  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then ")+mat13_right+string("  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then ")+mat20_right+string("  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then ")+mat21_right+string("  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then ")+mat22_right+string("  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then ")+mat23_right+string("  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then ")+mat30_right+string("  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then ")+mat31_right+string("  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then ")+mat32_right+string("  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then ")+mat33_right+string("  \n");
    stream << string(";  \n");

    string idx_r;
    string idx1_r;
    for (unsigned i = 0 ; i < joints_arm; ++i){
        idx_r = to_string(i);
        idx1_r = to_string(i+1);
        if (final){
            stream << string("var T_")+idx_r+string("_")+idx1_r+string("_right")+string(" {i1 in 1..4, i2 in 1..4} =  \n");
        }else{
            stream << string("var T_")+idx_r+string("_")+idx1_r+string("_right")+string(" {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
        }
        stream << string("# 1st row \n");
        if(final){
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[")+idx1_r+string("]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[")+idx1_r+string("])  \n");
        }else{
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,")+idx1_r+string("]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,")+idx1_r+string("])  \n");
        }
        stream << string("else	if ( i1=1 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_right[")+idx1_r+string("]  \n");
        stream << string("# 2st row \n");
        if(final){
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[")+idx1_r+string("])*c_alpha_right[")+idx1_r+string("] \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[")+idx1_r+string("])*c_alpha_right[")+idx1_r+string("] \n");
        }else{
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,")+idx1_r+string("])*c_alpha_right[")+idx1_r+string("] \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,")+idx1_r+string("])*c_alpha_right[")+idx1_r+string("] \n");
        }
        stream << string("else	if ( i1=2 && i2=3 ) then -s_alpha_right[")+idx1_r+string("] \n");
        stream << string("else	if ( i1=2 && i2=4 ) then -s_alpha_right[")+idx1_r+string("]*d_right[")+idx1_r+string("] \n");
        stream << string("# 3rd row \n");
        if(final){
            stream << string("else	if ( i1=3 && i2=1 ) then sin(theta[")+idx1_r+string("])*s_alpha_right[")+idx1_r+string("] \n");
            stream << string("else	if ( i1=3 && i2=2 ) then cos(theta[")+idx1_r+string("])*s_alpha_right[")+idx1_r+string("] \n");
        }else{
            stream << string("else	if ( i1=3 && i2=1 ) then sin(theta[i,")+idx1_r+string("])*s_alpha_right[")+idx1_r+string("] \n");
            stream << string("else	if ( i1=3 && i2=2 ) then cos(theta[i,")+idx1_r+string("])*s_alpha_right[")+idx1_r+string("] \n");
        }
        stream << string("else	if ( i1=3 && i2=3 ) then c_alpha_right[")+idx1_r+string("] \n");
        stream << string("else	if ( i1=3 && i2=4 ) then c_alpha_right[")+idx1_r+string("]*d_right[")+idx1_r+string("] \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0 \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

    }

    string matHand00_right =  boost::str(boost::format("%.2f") % (matRightHand(0,0))); boost::replace_all(matHand00_right,",",".");
    string matHand01_right =  boost::str(boost::format("%.2f") % (matRightHand(0,1))); boost::replace_all(matHand01_right,",",".");
    string matHand02_right =  boost::str(boost::format("%.2f") % (matRightHand(0,2))); boost::replace_all(matHand02_right,",",".");
    string matHand03_right =  boost::str(boost::format("%.2f") % (matRightHand(0,3))); boost::replace_all(matHand03_right,",",".");
    string matHand10_right =  boost::str(boost::format("%.2f") % (matRightHand(1,0))); boost::replace_all(matHand10_right,",",".");
    string matHand11_right =  boost::str(boost::format("%.2f") % (matRightHand(1,1))); boost::replace_all(matHand11_right,",",".");
    string matHand12_right =  boost::str(boost::format("%.2f") % (matRightHand(1,2))); boost::replace_all(matHand12_right,",",".");
    string matHand13_right =  boost::str(boost::format("%.2f") % (matRightHand(1,3))); boost::replace_all(matHand13_right,",",".");
    string matHand20_right =  boost::str(boost::format("%.2f") % (matRightHand(2,0))); boost::replace_all(matHand20_right,",",".");
    string matHand21_right =  boost::str(boost::format("%.2f") % (matRightHand(2,1))); boost::replace_all(matHand21_right,",",".");
    string matHand22_right =  boost::str(boost::format("%.2f") % (matRightHand(2,2))); boost::replace_all(matHand22_right,",",".");
    string matHand23_right =  boost::str(boost::format("%.2f") % (matRightHand(2,3))); boost::replace_all(matHand23_right,",",".");
    string matHand30_right =  boost::str(boost::format("%.2f") % (matRightHand(3,0))); boost::replace_all(matHand30_right,",",".");
    string matHand31_right =  boost::str(boost::format("%.2f") % (matRightHand(3,1))); boost::replace_all(matHand31_right,",",".");
    string matHand32_right =  boost::str(boost::format("%.2f") % (matRightHand(3,2))); boost::replace_all(matHand32_right,",",".");
    string matHand33_right =  boost::str(boost::format("%.2f") % (matRightHand(3,3))); boost::replace_all(matHand33_right,",",".");

    stream << string("param T_")+idx1_r+string("_H_right {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then ")+matHand00_right+string("  \n");
    stream << string("else	if ( i1=1 && i2=2 ) then ")+matHand01_right+string("  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then ")+matHand02_right+string("  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then ")+matHand03_right+string("  \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then ")+matHand10_right+string("  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then ")+matHand11_right+string("  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then ")+matHand12_right+string("  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then ")+matHand13_right+string("  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then ")+matHand20_right+string("  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then ")+matHand21_right+string("  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then ")+matHand22_right+string("  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then ")+matHand23_right+string("  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then ")+matHand30_right+string("  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then ")+matHand31_right+string("  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then ")+matHand32_right+string("  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then ")+matHand33_right+string("  \n");
    stream << string(";  \n");

    // --------------------------- positions on the right arm ------------------------------------------ //
    if(final){
        stream << string("var T_W_1_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_WorldToRightArm[i1,j]*T_0_1_right[j,i2];\n");
    }else{
        stream << string("var T_W_1_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_WorldToRightArm[i1,j]*T_0_1_right[j,i2,i];\n");
    }
    for (unsigned i = 1 ; i < joints_arm; ++i){
        idx_r = to_string(i);
        idx1_r = to_string(i+1);
        if(final){
            stream << string("var T_W_")+idx1_r+string("_right")+string(" {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx_r+string("_right")+string("[i1,j]*T_")+idx_r+string("_")+idx1_r+string("_right")+string("[j,i2];\n");
        }else{
            stream << string("var T_W_")+idx1_r+string("_right")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_W_")+idx_r+string("_right")+string("[i1,j,i]*T_")+idx_r+string("_")+idx1_r+string("_right")+string("[j,i2,i];\n");
        }
    }
    if(final){
        stream << string("var T_W_H_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx1_r+string("_right")+string("[i1,j]*T_")+idx1_r+string("_H_right[j,i2];\n\n");
    }else{
        stream << string("var T_W_H_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_W_")+idx1_r+string("_right")+string("[i1,j,i]*T_")+idx1_r+string("_H_right[j,i2];\n\n");
    }

    string tolArm1_right =  boost::str(boost::format("%.2f") % tolsRightArm.at(0)); boost::replace_all(tolArm1_right,",",".");
    string tolArm2_right =  boost::str(boost::format("%.2f") % tolsRightArm.at(1)); boost::replace_all(tolArm2_right,",",".");
    string tolArm3_right =  boost::str(boost::format("%.2f") % tolsRightArm.at(2)); boost::replace_all(tolArm3_right,",",".");
    string tolArm4_right =  boost::str(boost::format("%.2f") % tolsRightArm.at(3)); boost::replace_all(tolArm4_right,",",".");

    if(final){
        stream << string("var Shoulder_right {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_1_right[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm1_right+string("\n");
        stream << string(";  \n");
        stream << string("var Elbow_right {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_3_right[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm2_right+string("\n");
        stream << string(";  \n");
        stream << string("var Wrist_right {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_5_right[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm3_right+string("\n");
        stream << string(";  \n");
        stream << string("var Hand_right {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_H_right[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm4_right+string("\n");
        stream << string(";  \n");
        stream << string("# Right Hand orientation \n");
        stream << string("var x_H_right {j in 1..3} = T_W_H_right [j,1]; \n");
        stream << string("var y_H_right {j in 1..3} = T_W_H_right [j,2]; \n");
        stream << string("var z_H_right {j in 1..3} = T_W_H_right [j,3]; \n");
        stream << string("var Rot_H_right {j in 1..3,k in 1..3} = T_W_H_right [j,k]; \n");

    }else{
        stream << string("var Shoulder_right {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_1_right[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm1_right+string("\n");
        stream << string(";  \n");
        stream << string("var Elbow_right {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_3_right[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm2_right+string("\n");
        stream << string(";  \n");
        stream << string("var Wrist_right {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_5_right[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm3_right+string("\n");
        stream << string(";  \n");
        stream << string("var Hand_right {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_H_right[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm4_right+string("\n");
        stream << string(";  \n");
        stream << string("# Right Hand orientation \n");
        stream << string("var x_H_right {j in 1..3,i in Iterations} = T_W_H_right [j,1,i]; \n");
        stream << string("var y_H_right {j in 1..3,i in Iterations} = T_W_H_right [j,2,i]; \n");
        stream << string("var z_H_right {j in 1..3,i in Iterations} = T_W_H_right [j,3,i]; \n");
        stream << string("var Rot_H_right {j in 1..3,k in 1..3,i in Iterations} = T_W_H_right [j,k,i]; \n");
    }

    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  Direct Kinematics model of the left arm \n\n");

    stream << string("param c_alpha_left {i in 1..7} := cos(alpha_left[i]); \n");
    stream << string("param s_alpha_left {i in 1..7} := sin(alpha_left[i]); \n");
    stream << string("\n");

    string mat00_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(0,0))); boost::replace_all(mat00_left,",",".");
    string mat01_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(0,1))); boost::replace_all(mat01_left,",",".");
    string mat02_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(0,2))); boost::replace_all(mat02_left,",",".");
    string mat03_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(0,3))); boost::replace_all(mat03_left,",",".");
    string mat10_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(1,0))); boost::replace_all(mat10_left,",",".");
    string mat11_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(1,1))); boost::replace_all(mat11_left,",",".");
    string mat12_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(1,2))); boost::replace_all(mat12_left,",",".");
    string mat13_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(1,3))); boost::replace_all(mat13_left,",",".");
    string mat20_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(2,0))); boost::replace_all(mat20_left,",",".");
    string mat21_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(2,1))); boost::replace_all(mat21_left,",",".");
    string mat22_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(2,2))); boost::replace_all(mat22_left,",",".");
    string mat23_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(2,3))); boost::replace_all(mat23_left,",",".");
    string mat30_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(3,0))); boost::replace_all(mat30_left,",",".");
    string mat31_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(3,1))); boost::replace_all(mat31_left,",",".");
    string mat32_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(3,2))); boost::replace_all(mat32_left,",",".");
    string mat33_left =  boost::str(boost::format("%.2f") % (matWorldToLeftArm(3,3))); boost::replace_all(mat33_left,",",".");

    stream << string("param T_WorldToLeftArm {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then ")+mat00_left+string("  \n");
    stream << string("else	if ( i1=1 && i2=2 ) then ")+mat01_left+string("  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then ")+mat02_left+string("  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then ")+mat03_left+string("  \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then ")+mat10_left+string("  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then ")+mat11_left+string("  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then ")+mat12_left+string("  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then ")+mat13_left+string("  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then ")+mat20_left+string("  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then ")+mat21_left+string("  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then ")+mat22_left+string("  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then ")+mat23_left+string("  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then ")+mat30_left+string("  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then ")+mat31_left+string("  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then ")+mat32_left+string("  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then ")+mat33_left+string("  \n");
    stream << string(";  \n");

    string idx_l; string idx_ll;
    string idx1_l; string idx1_ll;
    for (unsigned i = 0 ; i < joints_arm; ++i){
        idx_l = to_string(i); idx_ll = to_string(i+joints_arm);
        idx1_l = to_string(i+1); //idx1_ll = to_string(i+1+joints_arm);

        if(final || dual_mov_type==1){
            idx1_ll = to_string(i+1+joints_arm);
        }else{
            // dual pick or dual move
            idx1_ll = to_string(i+3+joints_arm);
        }

        if (final){
            stream << string("var T_")+idx_l+string("_")+idx1_l+string("_left")+string(" {i1 in 1..4, i2 in 1..4} =  \n");
        }else{
            stream << string("var T_")+idx_l+string("_")+idx1_l+string("_left")+string(" {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
        }
        stream << string("# 1st row \n");
        if(final){
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[")+idx1_ll+string("]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[")+idx1_ll+string("])  \n");
        }else{
            // dual place
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,")+idx1_ll+string("]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,")+idx1_ll+string("])  \n");
        }
        stream << string("else	if ( i1=1 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_left[")+idx1_l+string("]  \n");
        stream << string("# 2st row \n");
        if(final){
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[")+idx1_ll+string("])*c_alpha_left[")+idx1_l+string("] \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[")+idx1_ll+string("])*c_alpha_left[")+idx1_l+string("] \n");
        }else{
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,")+idx1_ll+string("])*c_alpha_left[")+idx1_l+string("] \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,")+idx1_ll+string("])*c_alpha_left[")+idx1_l+string("] \n");
        }
        stream << string("else	if ( i1=2 && i2=3 ) then -s_alpha_left[")+idx1_l+string("] \n");
        stream << string("else	if ( i1=2 && i2=4 ) then -s_alpha_left[")+idx1_l+string("]*d_left[")+idx1_l+string("] \n");
        stream << string("# 3rd row \n");
        if(final){
            stream << string("else	if ( i1=3 && i2=1 ) then sin(theta[")+idx1_ll+string("])*s_alpha_left[")+idx1_l+string("] \n");
            stream << string("else	if ( i1=3 && i2=2 ) then cos(theta[")+idx1_ll+string("])*s_alpha_left[")+idx1_l+string("] \n");
        }else{
            stream << string("else	if ( i1=3 && i2=1 ) then sin(theta[i,")+idx1_ll+string("])*s_alpha_left[")+idx1_l+string("] \n");
            stream << string("else	if ( i1=3 && i2=2 ) then cos(theta[i,")+idx1_ll+string("])*s_alpha_left[")+idx1_l+string("] \n");
        }
        stream << string("else	if ( i1=3 && i2=3 ) then c_alpha_left[")+idx1_l+string("] \n");
        stream << string("else	if ( i1=3 && i2=4 ) then c_alpha_left[")+idx1_l+string("]*d_left[")+idx1_l+string("] \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0 \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

    }

    string matHand00_left =  boost::str(boost::format("%.2f") % (matLeftHand(0,0))); boost::replace_all(matHand00_left,",",".");
    string matHand01_left =  boost::str(boost::format("%.2f") % (matLeftHand(0,1))); boost::replace_all(matHand01_left,",",".");
    string matHand02_left =  boost::str(boost::format("%.2f") % (matLeftHand(0,2))); boost::replace_all(matHand02_left,",",".");
    string matHand03_left =  boost::str(boost::format("%.2f") % (matLeftHand(0,3))); boost::replace_all(matHand03_left,",",".");
    string matHand10_left =  boost::str(boost::format("%.2f") % (matLeftHand(1,0))); boost::replace_all(matHand10_left,",",".");
    string matHand11_left =  boost::str(boost::format("%.2f") % (matLeftHand(1,1))); boost::replace_all(matHand11_left,",",".");
    string matHand12_left =  boost::str(boost::format("%.2f") % (matLeftHand(1,2))); boost::replace_all(matHand12_left,",",".");
    string matHand13_left =  boost::str(boost::format("%.2f") % (matLeftHand(1,3))); boost::replace_all(matHand13_left,",",".");
    string matHand20_left =  boost::str(boost::format("%.2f") % (matLeftHand(2,0))); boost::replace_all(matHand20_left,",",".");
    string matHand21_left =  boost::str(boost::format("%.2f") % (matLeftHand(2,1))); boost::replace_all(matHand21_left,",",".");
    string matHand22_left =  boost::str(boost::format("%.2f") % (matLeftHand(2,2))); boost::replace_all(matHand22_left,",",".");
    string matHand23_left =  boost::str(boost::format("%.2f") % (matLeftHand(2,3))); boost::replace_all(matHand23_left,",",".");
    string matHand30_left =  boost::str(boost::format("%.2f") % (matLeftHand(3,0))); boost::replace_all(matHand30_left,",",".");
    string matHand31_left =  boost::str(boost::format("%.2f") % (matLeftHand(3,1))); boost::replace_all(matHand31_left,",",".");
    string matHand32_left =  boost::str(boost::format("%.2f") % (matLeftHand(3,2))); boost::replace_all(matHand32_left,",",".");
    string matHand33_left =  boost::str(boost::format("%.2f") % (matLeftHand(3,3))); boost::replace_all(matHand33_left,",",".");

    stream << string("param T_")+idx1_l+string("_H_left {i1 in 1..4, i2 in 1..4} =  \n");
    stream << string("# 1st row \n");
    stream << string("if ( i1=1 && i2=1 ) then ")+matHand00_left+string("  \n");
    stream << string("else	if ( i1=1 && i2=2 ) then ")+matHand01_left+string("  \n");
    stream << string("else	if ( i1=1 && i2=3 ) then ")+matHand02_left+string("  \n");
    stream << string("else	if ( i1=1 && i2=4 ) then ")+matHand03_left+string("  \n");
    stream << string("# 2nd row \n");
    stream << string("else	if ( i1=2 && i2=1 ) then ")+matHand10_left+string("  \n");
    stream << string("else	if ( i1=2 && i2=2 ) then ")+matHand11_left+string("  \n");
    stream << string("else	if ( i1=2 && i2=3 ) then ")+matHand12_left+string("  \n");
    stream << string("else	if ( i1=2 && i2=4 ) then ")+matHand13_left+string("  \n");
    stream << string("# 3rd row \n");
    stream << string("else	if ( i1=3 && i2=1 ) then ")+matHand20_left+string("  \n");
    stream << string("else	if ( i1=3 && i2=2 ) then ")+matHand21_left+string("  \n");
    stream << string("else	if ( i1=3 && i2=3 ) then ")+matHand22_left+string("  \n");
    stream << string("else	if ( i1=3 && i2=4 ) then ")+matHand23_left+string("  \n");
    stream << string("# 4th row \n");
    stream << string("else	if ( i1=4 && i2=1 ) then ")+matHand30_left+string("  \n");
    stream << string("else	if ( i1=4 && i2=2 ) then ")+matHand31_left+string("  \n");
    stream << string("else	if ( i1=4 && i2=3 ) then ")+matHand32_left+string("  \n");
    stream << string("else	if ( i1=4 && i2=4 ) then ")+matHand33_left+string("  \n");
    stream << string(";  \n");

    // --------------------------- positions on the left arm ------------------------------------------ //
    if(final){
        stream << string("var T_W_1_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_WorldToLeftArm[i1,j]*T_0_1_left[j,i2];\n");
    }else{
        stream << string("var T_W_1_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_WorldToLeftArm[i1,j]*T_0_1_left[j,i2,i];\n");
    }
    for (unsigned i = 1 ; i < joints_arm; ++i){
        idx_l = to_string(i);
        idx1_l = to_string(i+1);
        if(final){
            stream << string("var T_W_")+idx1_l+string("_left")+string(" {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx_l+string("_left")+string("[i1,j]*T_")+idx_l+string("_")+idx1_l+string("_left")+string("[j,i2];\n");
        }else{
            stream << string("var T_W_")+idx1_l+string("_left")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_W_")+idx_l+string("_left")+string("[i1,j,i]*T_")+idx_l+string("_")+idx1_l+string("_left")+string("[j,i2,i];\n");
        }
    }
    if(final){
        stream << string("var T_W_H_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx1_l+string("_left")+string("[i1,j]*T_")+idx1_l+string("_H_left[j,i2];\n\n");
    }else{
        stream << string("var T_W_H_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}   T_W_")+idx1_l+string("_left")+string("[i1,j,i]*T_")+idx1_l+string("_H_left[j,i2];\n\n");
    }

    string tolArm1_left =  boost::str(boost::format("%.2f") % tolsLeftArm.at(0)); boost::replace_all(tolArm1_left,",",".");
    string tolArm2_left =  boost::str(boost::format("%.2f") % tolsLeftArm.at(1)); boost::replace_all(tolArm2_left,",",".");
    string tolArm3_left =  boost::str(boost::format("%.2f") % tolsLeftArm.at(2)); boost::replace_all(tolArm3_left,",",".");
    string tolArm4_left =  boost::str(boost::format("%.2f") % tolsLeftArm.at(3)); boost::replace_all(tolArm4_left,",",".");

    if(final){
        stream << string("var Shoulder_left {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_1_left[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm1_left+string("\n");
        stream << string(";  \n");
        stream << string("var Elbow_left {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_3_left[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm2_left+string("\n");
        stream << string(";  \n");
        stream << string("var Wrist_left {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_5_left[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm3_left+string("\n");
        stream << string(";  \n");
        stream << string("var Hand_left {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_H_left[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm4_left+string("\n");
        stream << string(";  \n");
        stream << string("# Left Hand orientation \n");
        stream << string("var x_H_left {j in 1..3} = T_W_H_left [j,1]; \n");
        stream << string("var y_H_left {j in 1..3} = T_W_H_left [j,2]; \n");
        stream << string("var z_H_left {j in 1..3} = T_W_H_left [j,3]; \n");
        stream << string("var Rot_H_left {j in 1..3,k in 1..3} = T_W_H_left [j,k]; \n");

    }else{
        stream << string("var Shoulder_left {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_1_left[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm1_left+string("\n");
        stream << string(";  \n");
        stream << string("var Elbow_left {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_3_left[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm2_left+string("\n");
        stream << string(";  \n");
        stream << string("var Wrist_left {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_5_left[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm3_left+string("\n");
        stream << string(";  \n");
        stream << string("var Hand_left {i in 1..4,j in Iterations} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_H_left[i,4,j] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm4_left+string("\n");
        stream << string(";  \n");
        stream << string("# Left Hand orientation \n");
        stream << string("var x_H_left {j in 1..3,i in Iterations} = T_W_H_left [j,1,i]; \n");
        stream << string("var y_H_left {j in 1..3,i in Iterations} = T_W_H_left [j,2,i]; \n");
        stream << string("var z_H_left {j in 1..3,i in Iterations} = T_W_H_left [j,3,i]; \n");
        stream << string("var Rot_H_left {j in 1..3,k in 1..3,i in Iterations} = T_W_H_left [j,k,i]; \n");
    }

}

void HUMPlanner::writeInitDualArmDirKin(ofstream &stream, std::vector<double>& tolsRightArm,std::vector<double>& tolsLeftArm)
{

    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  Direct Kinematics model of the right arm init \n\n");

    string idx_r;
    string idx1_r;
    for (unsigned i = 0 ; i < joints_arm; ++i){
        idx_r = to_string(i);
        idx1_r = to_string(i+1);

        stream << string("param T_")+idx_r+string("_")+idx1_r+string("_init_right")+string(" {i1 in 1..4, i2 in 1..4} =  \n");

        stream << string("# 1st row \n");

            stream << string("if ( i1=1 && i2=1 ) then cos(thet_init[")+idx1_r+string("]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(thet_init[")+idx1_r+string("])  \n");

        stream << string("else	if ( i1=1 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_right[")+idx1_r+string("]  \n");
        stream << string("# 2st row \n");

            stream << string("else	if ( i1=2 && i2=1 ) then sin(thet_init[")+idx1_r+string("])*c_alpha_right[")+idx1_r+string("] \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(thet_init[")+idx1_r+string("])*c_alpha_right[")+idx1_r+string("] \n");

        stream << string("else	if ( i1=2 && i2=3 ) then -s_alpha_right[")+idx1_r+string("] \n");
        stream << string("else	if ( i1=2 && i2=4 ) then -s_alpha_right[")+idx1_r+string("]*d_right[")+idx1_r+string("] \n");
        stream << string("# 3rd row \n");

            stream << string("else	if ( i1=3 && i2=1 ) then sin(thet_init[")+idx1_r+string("])*s_alpha_right[")+idx1_r+string("] \n");
            stream << string("else	if ( i1=3 && i2=2 ) then cos(thet_init[")+idx1_r+string("])*s_alpha_right[")+idx1_r+string("] \n");

        stream << string("else	if ( i1=3 && i2=3 ) then c_alpha_right[")+idx1_r+string("] \n");
        stream << string("else	if ( i1=3 && i2=4 ) then c_alpha_right[")+idx1_r+string("]*d_right[")+idx1_r+string("] \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0 \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

    }


    // --------------------------- positions on the right arm ------------------------------------------ //

        stream << string("param T_W_1_init_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_WorldToRightArm[i1,j]*T_0_1_init_right[j,i2];\n");

    for (unsigned i = 1 ; i < joints_arm; ++i){
        idx_r = to_string(i);
        idx1_r = to_string(i+1);
            stream << string("param T_W_")+idx1_r+string("_init_right")+string(" {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx_r+string("_init_right")+string("[i1,j]*T_")+idx_r+string("_")+idx1_r+string("_init_right")+string("[j,i2];\n");

    }
        stream << string("param T_W_H_init_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx1_r+string("_init_right")+string("[i1,j]*T_")+idx1_r+string("_H_right[j,i2];\n\n");


    string tolArm1_right =  boost::str(boost::format("%.2f") % tolsRightArm.at(0)); boost::replace_all(tolArm1_right,",",".");
    string tolArm2_right =  boost::str(boost::format("%.2f") % tolsRightArm.at(1)); boost::replace_all(tolArm2_right,",",".");
    string tolArm3_right =  boost::str(boost::format("%.2f") % tolsRightArm.at(2)); boost::replace_all(tolArm3_right,",",".");
    string tolArm4_right =  boost::str(boost::format("%.2f") % tolsRightArm.at(3)); boost::replace_all(tolArm4_right,",",".");

        stream << string("param Shoulder_init_right {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_1_init_right[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm1_right+string("\n");
        stream << string(";  \n");
        stream << string("param Elbow_init_right {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_3_init_right[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm2_right+string("\n");
        stream << string(";  \n");
        stream << string("param Wrist_init_right {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_5_init_right[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm3_right+string("\n");
        stream << string(";  \n");
        stream << string("param Hand_init_right {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_H_init_right[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm4_right+string("\n");
        stream << string(";  \n");
        stream << string("# Right Hand init orientation \n");
        stream << string("param x_H_init_right {j in 1..3} = T_W_H_init_right [j,1]; \n");
        stream << string("param y_H_init_right {j in 1..3} = T_W_H_init_right [j,2]; \n");
        stream << string("param z_H_init_right {j in 1..3} = T_W_H_init_right [j,3]; \n");
        stream << string("param Rot_H_init_right {j in 1..3,k in 1..3} = T_W_H_init_right [j,k]; \n");



    stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    stream << string("#  Direct Kinematics model of the left arm init \n\n");

    string idx_l; string idx_ll;
    string idx1_l; string idx1_ll;
    for (unsigned i = 0 ; i < joints_arm; ++i){
        idx_l = to_string(i); idx_ll = to_string(i+joints_arm);
        idx1_l = to_string(i+1); idx1_ll = to_string(i+1+joints_arm);

            stream << string("param T_")+idx_l+string("_")+idx1_l+string("_init_left")+string(" {i1 in 1..4, i2 in 1..4} =  \n");

        stream << string("# 1st row \n");

            stream << string("if ( i1=1 && i2=1 ) then cos(thet_init[")+idx1_ll+string("]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(thet_init[")+idx1_ll+string("])  \n");

        stream << string("else	if ( i1=1 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then a_left[")+idx1_l+string("]  \n");
        stream << string("# 2st row \n");

            stream << string("else	if ( i1=2 && i2=1 ) then sin(thet_init[")+idx1_ll+string("])*c_alpha_left[")+idx1_l+string("] \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(thet_init[")+idx1_ll+string("])*c_alpha_left[")+idx1_l+string("] \n");

        stream << string("else	if ( i1=2 && i2=3 ) then -s_alpha_left[")+idx1_l+string("] \n");
        stream << string("else	if ( i1=2 && i2=4 ) then -s_alpha_left[")+idx1_l+string("]*d_left[")+idx1_l+string("] \n");
        stream << string("# 3rd row \n");

            stream << string("else	if ( i1=3 && i2=1 ) then sin(thet_init[")+idx1_ll+string("])*s_alpha_left[")+idx1_l+string("] \n");
            stream << string("else	if ( i1=3 && i2=2 ) then cos(thet_init[")+idx1_ll+string("])*s_alpha_left[")+idx1_l+string("] \n");

        stream << string("else	if ( i1=3 && i2=3 ) then c_alpha_left[")+idx1_l+string("] \n");
        stream << string("else	if ( i1=3 && i2=4 ) then c_alpha_left[")+idx1_l+string("]*d_left[")+idx1_l+string("] \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0 \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

    }


    // --------------------------- positions on the left arm ------------------------------------------ //

        stream << string("param T_W_1_init_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_WorldToLeftArm[i1,j]*T_0_1_init_left[j,i2];\n");

    for (unsigned i = 1 ; i < joints_arm; ++i){
        idx_l = to_string(i);
        idx1_l = to_string(i+1);
            stream << string("param T_W_")+idx1_l+string("_init_left")+string(" {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx_l+string("_init_left")+string("[i1,j]*T_")+idx_l+string("_")+idx1_l+string("_init_left")+string("[j,i2];\n");

    }

        stream << string("param T_W_H_init_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}   T_W_")+idx1_l+string("_init_left")+string("[i1,j]*T_")+idx1_l+string("_H_left[j,i2];\n\n");


    string tolArm1_left =  boost::str(boost::format("%.2f") % tolsLeftArm.at(0)); boost::replace_all(tolArm1_left,",",".");
    string tolArm2_left =  boost::str(boost::format("%.2f") % tolsLeftArm.at(1)); boost::replace_all(tolArm2_left,",",".");
    string tolArm3_left =  boost::str(boost::format("%.2f") % tolsLeftArm.at(2)); boost::replace_all(tolArm3_left,",",".");
    string tolArm4_left =  boost::str(boost::format("%.2f") % tolsLeftArm.at(3)); boost::replace_all(tolArm4_left,",",".");


        stream << string("param Shoulder_init_left {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_1_init_left[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm1_left+string("\n");
        stream << string(";  \n");
        stream << string("param Elbow_init_left {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_3_init_left[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm2_left+string("\n");
        stream << string(";  \n");
        stream << string("param Wrist_init_left {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_5_init_left[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm3_left+string("\n");
        stream << string(";  \n");
        stream << string("param Hand_init_left {i in 1..4} = #xyz+radius \n");
        stream << string("if ( i<4 ) then 	T_W_H_init_left[i,4] \n");
        stream << string("else	if ( i=4 ) then  ")+tolArm4_left+string("\n");
        stream << string(";  \n");
        stream << string("# Left Hand init orientation \n");
        stream << string("param x_H_init_left {j in 1..3} = T_W_H_init_left [j,1]; \n");
        stream << string("param y_H_init_left {j in 1..3} = T_W_H_init_left [j,2]; \n");
        stream << string("param z_H_init_left {j in 1..3} = T_W_H_init_left [j,3]; \n");
        stream << string("param Rot_H_init_left {j in 1..3,k in 1..3} = T_W_H_init_left [j,k]; \n");


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

void HUMPlanner::writeDualHumanHandDirKin(std::ofstream& stream,MatrixXd& tolsHand, bool final, bool transport_right, bool transport_left,bool right)
{

    string tolHand1; string tolHand2; string tolHand3; string tolHand4;

    if (right)
    {
        // right hand
        stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
        stream << string("#  Direct Kinematics model of the fingers in the RIGHT HAND \n\n");

        stream << string("# Index finger right \n\n");

        stream << string("param TF1_H_0_right {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta0_fing1_right) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_fing1_right)*cos(alpha_fing1_right[1])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_fing1_right)*sin(alpha_fing1_right[1])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then u1x_right \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_fing1_right)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_fing1_right)*cos(alpha_fing1_right[1])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_fing1_right)*sin(alpha_fing1_right[1])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then u1y_right  \n");
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1_right[1])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1_right[1])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then u1z_right  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_right){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF1_0_1_right {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_right[2]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_right[2])*cos(alpha_fing1_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_right[2])*sin(alpha_fing1_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_right[2]*cos(joint_fingers_right[2]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_right[2])*cos(alpha_fing1_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_right[2])*sin(alpha_fing1_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_right[2]*sin(joint_fingers_right[2])  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF1_0_1_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9])*cos(alpha_fing1_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9])*sin(alpha_fing1_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_right[2]*cos(theta[i,9]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9])*cos(alpha_fing1_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9])*sin(alpha_fing1_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_right[2]*sin(theta[i,9])  \n");

        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1_right[2])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1_right[2])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing1_right[2]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_right){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF1_1_2_right {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(2*joint_fingers_right[2]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*joint_fingers_right[2]/3)*cos(alpha_fing1_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(2*joint_fingers_right[2]/3)*sin(alpha_fing1_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_right[3]*cos(2*joint_fingers_right[2]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(2*joint_fingers_right[2]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(2*joint_fingers_right[2]/3)*cos(alpha_fing1_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*joint_fingers_right[2]/3)*sin(alpha_fing1_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_right[3]*sin(2*joint_fingers_right[2]/3)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF1_1_2_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(2*theta[i,9]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*theta[i,9]/3)*cos(alpha_fing1_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(2*theta[i,9]/3)*sin(alpha_fing1_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_right[3]*cos(2*theta[i,9]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(2*theta[i,9]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(2*theta[i,9]/3)*cos(alpha_fing1_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*theta[i,9]/3)*sin(alpha_fing1_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_right[3]*sin(2*theta[i,9]/3)  \n");

        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1_right[3])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1_right[3])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing1_right[3]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_right){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF1_2_3_right {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_right[2]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_right[2]/3)*cos(alpha_fing1_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_right[2]/3)*sin(alpha_fing1_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_right[4]*cos(joint_fingers_right[2]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_right[2]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_right[2]/3)*cos(alpha_fing1_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_right[2]/3)*sin(alpha_fing1_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_right[4]*sin(joint_fingers_right[2]/3)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF1_2_3_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9]/3)*cos(alpha_fing1_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9]/3)*sin(alpha_fing1_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_right[4]*cos(theta[i,9]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9]/3)*cos(alpha_fing1_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9]/3)*sin(alpha_fing1_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_right[4]*sin(theta[i,9]/3)  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1_right[4])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1_right[4])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing1_right[4]  \n");
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
            stream << string("var F1_0_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H_right[i1,j]*TF1_H_0_right[j,i2]; \n");
            stream << string("var F1_1_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_0_right[i1,j]*TF1_0_1_right[j,i2]; \n");
            stream << string("var F1_2_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_1_right[i1,j]*TF1_1_2_right[j,i2]; \n");
            stream << string("var F1_tip_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_2_right[i1,j]*TF1_2_3_right[j,i2]; \n\n");

            stream << string("var Finger1_0_right {i1 in 1..4} =  if i1<4 then F1_0_right[i1,4] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger1_1_right {i1 in 1..4} =  if i1<4 then F1_1_right[i1,4] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger1_2_right {i1 in 1..4} =  if i1<4 then F1_2_right[i1,4] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger1_tip_right {i1 in 1..4} =  if i1<4 then F1_tip_right[i1,4] else ")+tolHand4+string("; \n\n");
        }else{
            //bounce posture selection
            stream << string("var F1_0_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H_right[i1,j,i]*TF1_H_0_right[j,i2]; \n");
            if(transport_right){
                stream << string("var F1_1_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_0_right[i1,j,i]*TF1_0_1_right[j,i2]; \n");
                stream << string("var F1_2_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_1_right[i1,j,i]*TF1_1_2_right[j,i2]; \n");
                stream << string("var F1_tip_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_2_right[i1,j,i]*TF1_2_3_right[j,i2]; \n\n");
            }else{
                stream << string("var F1_1_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_0_right[i1,j,i]*TF1_0_1_right[j,i2,i]; \n");
                stream << string("var F1_2_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_1_right[i1,j,i]*TF1_1_2_right[j,i2,i]; \n");
                stream << string("var F1_tip_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_2_right[i1,j,i]*TF1_2_3_right[j,i2,i]; \n\n");
            }
            stream << string("var Finger1_0_right {i1 in 1..4,i in Iterations} =  if i1<4 then F1_0_right[i1,4,i] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger1_1_right {i1 in 1..4,i in Iterations} =  if i1<4 then F1_1_right[i1,4,i] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger1_2_right {i1 in 1..4,i in Iterations} =  if i1<4 then F1_2_right[i1,4,i] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger1_tip_right {i1 in 1..4,i in Iterations} =  if i1<4 then F1_tip_right[i1,4,i] else ")+tolHand4+string("; \n\n");

        }

        stream << string("# Ring finger right \n\n");

        stream << string("param TF2_H_0_right {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta0_fing3_right) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_fing3_right)*cos(alpha_fing3_right[1])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_fing3_right)*sin(alpha_fing3_right[1])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then u3x_right \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_fing3_right)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_fing3_right)*cos(alpha_fing3_right[1])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_fing3_right)*sin(alpha_fing3_right[1])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then u3y_right  \n");
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3_right[1])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3_right[1])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then u3z_right  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_right){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF2_0_1_right {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_right[3]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_right[3])*cos(alpha_fing3_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_right[3])*sin(alpha_fing3_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_right[2]*cos(joint_fingers_right[3]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_right[3])*cos(alpha_fing3_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_right[3])*sin(alpha_fing3_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing3_right[2]*sin(joint_fingers_right[3])  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF2_0_1_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9])*cos(alpha_fing3_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9])*sin(alpha_fing3_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_right[2]*cos(theta[i,9]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9])*cos(alpha_fing3_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9])*sin(alpha_fing3_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing3_right[2]*sin(theta[i,9])  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3_right[2])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3_right[2])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing3_right[2]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_right){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF2_1_2_right {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(2*joint_fingers_right[3]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*joint_fingers_right[3]/3)*cos(alpha_fing3_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(2*joint_fingers_right[3]/3)*sin(alpha_fing3_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_right[3]*cos(2*joint_fingers_right[3]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(2*joint_fingers_right[3]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(2*joint_fingers_right[3]/3)*cos(alpha_fing3_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*joint_fingers_right[3]/3)*sin(alpha_fing3_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_right[3]*sin(2*joint_fingers_right[3]/3)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF2_1_2_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(2*theta[i,9]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*theta[i,9]/3)*cos(alpha_fing3_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(2*theta[i,9]/3)*sin(alpha_fing3_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_right[3]*cos(2*theta[i,9]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(2*theta[i,9]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(2*theta[i,9]/3)*cos(alpha_fing3_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*theta[i,9]/3)*sin(alpha_fing3_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_right[3]*sin(2*theta[i,9]/3)  \n");

        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3_right[3])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3_right[3])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing3_right[3]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_right){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF2_2_3_right {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_right[3]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_right[3]/3)*cos(alpha_fing3_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_right[3]/3)*sin(alpha_fing3_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_right[4]*cos(joint_fingers_right[3]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_right[3]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_right[3]/3)*cos(alpha_fing3_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_right[3]/3)*sin(alpha_fing3_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing3_right[4]*sin(joint_fingers_right[3]/3)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF2_2_3_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,9]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,9]/3)*cos(alpha_fing3_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,9]/3)*sin(alpha_fing3_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_right[4]*cos(theta[i,9]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,9]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,9]/3)*cos(alpha_fing3_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,9]/3)*sin(alpha_fing3_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing3_right[4]*sin(theta[i,9]/3)  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3_right[4])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3_right[4])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing3_right[4]  \n");
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
            stream << string("var F2_0_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H_right[i1,j]*TF2_H_0_right[j,i2]; \n");
            stream << string("var F2_1_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} F2_0_right[i1,j]*TF2_0_1_right[j,i2]; \n");
            stream << string("var F2_2_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} F2_1_right[i1,j]*TF2_1_2_right[j,i2]; \n");
            stream << string("var F2_tip_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} F2_2_right[i1,j]*TF2_2_3_right[j,i2]; \n\n");

            stream << string("var Finger2_0_right {i1 in 1..4} =  if i1<4 then F2_0_right[i1,4] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger2_1_right {i1 in 1..4} =  if i1<4 then F2_1_right[i1,4] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger2_2_right {i1 in 1..4} =  if i1<4 then F2_2_right[i1,4] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger2_tip_right {i1 in 1..4} =  if i1<4 then F2_tip_right[i1,4] else ")+tolHand4+string("; \n\n");
        }else{
            //bounce posture selection
            stream << string("var F2_0_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H_right[i1,j,i]*TF2_H_0_right[j,i2]; \n");
            if (transport_right){
                stream << string("var F2_1_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_0_right[i1,j,i]*TF2_0_1_right[j,i2]; \n");
                stream << string("var F2_2_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_1_right[i1,j,i]*TF2_1_2_right[j,i2]; \n");
                stream << string("var F2_tip_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_2_right[i1,j,i]*TF2_2_3_right[j,i2]; \n\n");
            }else{
                stream << string("var F2_1_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_0_right[i1,j,i]*TF2_0_1_right[j,i2,i]; \n");
                stream << string("var F2_2_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_1_right[i1,j,i]*TF2_1_2_right[j,i2,i]; \n");
                stream << string("var F2_tip_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_2_right[i1,j,i]*TF2_2_3_right[j,i2,i]; \n\n");
            }
            stream << string("var Finger2_0_right {i1 in 1..4,i in Iterations} =  if i1<4 then F2_0_right[i1,4,i] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger2_1_right {i1 in 1..4,i in Iterations} =  if i1<4 then F2_1_right[i1,4,i] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger2_2_right {i1 in 1..4,i in Iterations} =  if i1<4 then F2_2_right[i1,4,i] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger2_tip_right {i1 in 1..4,i in Iterations} =  if i1<4 then F2_tip_right[i1,4,i] else ")+tolHand4+string("; \n\n");

        }

        stream << string("# Thumb finger right \n\n");

        stream << string("param TF3_H_0_right {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta0_thumb_right) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_thumb_right)*cos(alpha_thumb_right[1])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_thumb_right)*sin(alpha_thumb_right[1])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then uTx_right \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_thumb_right)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_thumb_right)*cos(alpha_thumb_right[1])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_thumb_right)*sin(alpha_thumb_right[1])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then uTy_right  \n");
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb_right[1])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb_right[1])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then uTz_right  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_right){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF3_0_1_right {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_right[1]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_right[1])*cos(alpha_thumb_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_right[1])*sin(alpha_thumb_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_right[2]*cos(joint_fingers_right[1]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_right[1])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_right[1])*cos(alpha_thumb_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_right[1])*sin(alpha_thumb_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_right[2]*sin(joint_fingers_right[1])  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF3_0_1_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,8]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,8])*cos(alpha_thumb_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,8])*sin(alpha_thumb_right[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_right[2]*cos(theta[i,8]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,8])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,8])*cos(alpha_thumb_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,8])*sin(alpha_thumb_right[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_right[2]*sin(theta[i,8])  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb_right[2])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb_right[2])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_thumb_right[2]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_right){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF3_1_2_right {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_right[4]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_right[4])*cos(alpha_thumb_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_right[4])*sin(alpha_thumb_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_right[3]*cos(joint_fingers_right[4]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_right[4])*cos(alpha_thumb_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_right[4])*sin(alpha_thumb_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_right[3]*sin(joint_fingers_right[4])  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF3_1_2_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,10]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,10])*cos(alpha_thumb_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,10])*sin(alpha_thumb_right[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_right[3]*cos(theta[i,10]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,10])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,10])*cos(alpha_thumb_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,10])*sin(alpha_thumb_right[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_right[3]*sin(theta[i,10])  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb_right[3])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb_right[3])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_thumb_right[3]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_right){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF3_2_3_right {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(11*joint_fingers_right[4]/10) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*joint_fingers_right[4]/10)*cos(alpha_thumb_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(11*joint_fingers_right[4]/10)*sin(alpha_thumb_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_right[4]*cos(11*joint_fingers_right[4]/10) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(11*joint_fingers_right[4]/10)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(11*joint_fingers_right[4]/10)*cos(alpha_thumb_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*joint_fingers_right[4]/10)*sin(alpha_thumb_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_right[4]*sin(11*joint_fingers_right[4]/10)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF3_2_3_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(11*theta[i,10]/10) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*theta[i,10]/10)*cos(alpha_thumb_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(11*theta[i,10]/10)*sin(alpha_thumb_right[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_right[4]*cos(11*theta[i,10]/10) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(11*theta[i,10]/10)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(11*theta[i,10]/10)*cos(alpha_thumb_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*theta[i,10]/10)*sin(alpha_thumb_right[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_right[4]*sin(11*theta[i,10]/10)  \n");

        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb_right[4])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb_right[4])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_thumb_right[4]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_right){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF3_3_4_right {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(11*joint_fingers_right[4]/12) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*joint_fingers_right[4]/12)*cos(alpha_thumb_right[5])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(11*joint_fingers_right[4]/12)*sin(alpha_thumb_right[5])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_right[5]*cos(11*joint_fingers_right[4]/12) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(11*joint_fingers_right[4]/12)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(11*joint_fingers_right[4]/12)*cos(alpha_thumb_right[5])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*joint_fingers_right[4]/12)*sin(alpha_thumb_right[5])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_right[5]*sin(11*joint_fingers_right[4]/12)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF3_3_4_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(11*theta[i,10]/12) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*theta[i,10]/12)*cos(alpha_thumb_right[5])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(11*theta[i,10]/12)*sin(alpha_thumb_right[5])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_right[5]*cos(11*theta[i,10]/12) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(11*theta[i,10]/12)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(11*theta[i,10]/12)*cos(alpha_thumb_right[5])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*theta[i,10]/12)*sin(alpha_thumb_right[5])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_right[5]*sin(11*theta[i,10]/12)  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb_right[5])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb_right[5])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_thumb_right[5]  \n");
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
            stream << string("var F3_0_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H_right[i1,j]*TF3_H_0_right[j,i2]; \n");
            stream << string("var F3_1_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_0_right[i1,j]*TF3_0_1_right[j,i2]; \n");
            stream << string("var F3_2_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_1_right[i1,j]*TF3_1_2_right[j,i2]; \n");
            stream << string("var F3_3_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_2_right[i1,j]*TF3_2_3_right[j,i2]; \n");
            stream << string("var F3_tip_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_3_right[i1,j]*TF3_3_4_right[j,i2]; \n\n");

            //PostureMod << string("var Finger3_0   {i1 in 1..4} =  if i1<4 then F3_0[i1,4] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger3_1_right {i1 in 1..4} =  if i1<4 then F3_1_right[i1,4] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger3_2_right {i1 in 1..4} =  if i1<4 then F3_2_right[i1,4] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger3_3_right {i1 in 1..4} =  if i1<4 then F3_3_right[i1,4] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger3_tip_right {i1 in 1..4} =  if i1<4 then F3_tip_right[i1,4] else ")+tolHand4+string("; \n\n");

        }else{
            //bounce posture selection
            stream << string("var F3_0_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4} T_W_H_right[i1,j,i]*TF3_H_0_right[j,i2]; \n");
            if(transport_right){
                stream << string("var F3_1_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_0_right[i1,j,i]*TF3_0_1_right[j,i2]; \n");
                stream << string("var F3_2_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_1_right[i1,j,i]*TF3_1_2_right[j,i2]; \n");
                stream << string("var F3_3_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_2_right[i1,j,i]*TF3_2_3_right[j,i2]; \n");
                stream << string("var F3_tip_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_3_right[i1,j,i]*TF3_3_4_right[j,i2]; \n\n");
            }else{
                stream << string("var F3_1_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_0_right[i1,j,i]*TF3_0_1_right[j,i2,i]; \n");
                stream << string("var F3_2_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_1_right[i1,j,i]*TF3_1_2_right[j,i2,i]; \n");
                stream << string("var F3_3_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_2_right[i1,j,i]*TF3_2_3_right[j,i2,i]; \n");
                stream << string("var F3_tip_right {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_3_right[i1,j,i]*TF3_3_4_right[j,i2,i]; \n\n");
            }
            stream << string("var Finger3_1_right {i1 in 1..4, i in Iterations} =  if i1<4 then F3_1_right[i1,4,i] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger3_2_right {i1 in 1..4, i in Iterations} =  if i1<4 then F3_2_right[i1,4,i] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger3_3_right {i1 in 1..4, i in Iterations} =  if i1<4 then F3_3_right[i1,4,i] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger3_tip_right {i1 in 1..4, i in Iterations} =  if i1<4 then F3_tip_right[i1,4,i] else ")+tolHand4+string("; \n\n");

        }
    }else{
        // left hand
        stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
        stream << string("#  Direct Kinematics model of the fingers in the LEFT HAND \n\n");

        stream << string("# Index finger left \n\n");

        stream << string("param TF1_H_0_left {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta0_fing1_left) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_fing1_left)*cos(alpha_fing1_left[1])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_fing1_left)*sin(alpha_fing1_left[1])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then u1x_left \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_fing1_left)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_fing1_left)*cos(alpha_fing1_left[1])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_fing1_left)*sin(alpha_fing1_left[1])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then u1y_left  \n");
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1_left[1])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1_left[1])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then u1z_left  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_left){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF1_0_1_left {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_left[2]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_left[2])*cos(alpha_fing1_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_left[2])*sin(alpha_fing1_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_left[2]*cos(joint_fingers_left[2]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_left[2])*cos(alpha_fing1_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_left[2])*sin(alpha_fing1_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_left[2]*sin(joint_fingers_left[2])  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF1_0_1_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,18]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,18])*cos(alpha_fing1_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,18])*sin(alpha_fing1_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_left[2]*cos(theta[i,18]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,18])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,18])*cos(alpha_fing1_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,18])*sin(alpha_fing1_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_left[2]*sin(theta[i,18])  \n");

        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1_left[2])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1_left[2])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing1_left[2]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_left){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF1_1_2_left {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(2*joint_fingers_left[2]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*joint_fingers_left[2]/3)*cos(alpha_fing1_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(2*joint_fingers_left[2]/3)*sin(alpha_fing1_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_left[3]*cos(2*joint_fingers_left[2]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(2*joint_fingers_left[2]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(2*joint_fingers_left[2]/3)*cos(alpha_fing1_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*joint_fingers_left[2]/3)*sin(alpha_fing1_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_left[3]*sin(2*joint_fingers_left[2]/3)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF1_1_2_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(2*theta[i,18]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*theta[i,18]/3)*cos(alpha_fing1_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(2*theta[i,18]/3)*sin(alpha_fing1_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_left[3]*cos(2*theta[i,18]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(2*theta[i,18]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(2*theta[i,18]/3)*cos(alpha_fing1_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*theta[i,18]/3)*sin(alpha_fing1_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_left[3]*sin(2*theta[i,18]/3)  \n");

        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1_left[3])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1_left[3])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing1_left[3]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_left){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF1_2_3_left {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_left[2]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_left[2]/3)*cos(alpha_fing1_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_left[2]/3)*sin(alpha_fing1_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_left[4]*cos(joint_fingers_left[2]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_left[2]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_left[2]/3)*cos(alpha_fing1_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_left[2]/3)*sin(alpha_fing1_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_left[4]*sin(joint_fingers_left[2]/3)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF1_2_3_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,18]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,18]/3)*cos(alpha_fing1_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,18]/3)*sin(alpha_fing1_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing1_left[4]*cos(theta[i,18]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,18]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,18]/3)*cos(alpha_fing1_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,18]/3)*sin(alpha_fing1_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_left[4]*sin(theta[i,18]/3)  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing1_left[4])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing1_left[4])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing1_left[4]  \n");
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
            stream << string("var F1_0_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H_left[i1,j]*TF1_H_0_left[j,i2]; \n");
            stream << string("var F1_1_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_0_left[i1,j]*TF1_0_1_left[j,i2]; \n");
            stream << string("var F1_2_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_1_left[i1,j]*TF1_1_2_left[j,i2]; \n");
            stream << string("var F1_tip_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F1_2_left[i1,j]*TF1_2_3_left[j,i2]; \n\n");

            stream << string("var Finger1_0_left {i1 in 1..4} =  if i1<4 then F1_0_left[i1,4] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger1_1_left {i1 in 1..4} =  if i1<4 then F1_1_left[i1,4] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger1_2_left {i1 in 1..4} =  if i1<4 then F1_2_left[i1,4] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger1_tip_left {i1 in 1..4} =  if i1<4 then F1_tip_left[i1,4] else ")+tolHand4+string("; \n\n");
        }else{
            //bounce posture selection
            stream << string("var F1_0_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H_left[i1,j,i]*TF1_H_0_left[j,i2]; \n");
            if(transport_left){
                stream << string("var F1_1_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_0_left[i1,j,i]*TF1_0_1_left[j,i2]; \n");
                stream << string("var F1_2_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_1_left[i1,j,i]*TF1_1_2_left[j,i2]; \n");
                stream << string("var F1_tip_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_2_left[i1,j,i]*TF1_2_3_left[j,i2]; \n\n");
            }else{
                stream << string("var F1_1_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_0_left[i1,j,i]*TF1_0_1_left[j,i2,i]; \n");
                stream << string("var F1_2_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_1_left[i1,j,i]*TF1_1_2_left[j,i2,i]; \n");
                stream << string("var F1_tip_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F1_2_left[i1,j,i]*TF1_2_3_left[j,i2,i]; \n\n");
            }
            stream << string("var Finger1_0_left {i1 in 1..4,i in Iterations} =  if i1<4 then F1_0_left[i1,4,i] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger1_1_left {i1 in 1..4,i in Iterations} =  if i1<4 then F1_1_left[i1,4,i] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger1_2_left {i1 in 1..4,i in Iterations} =  if i1<4 then F1_2_left[i1,4,i] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger1_tip_left {i1 in 1..4,i in Iterations} =  if i1<4 then F1_tip_left[i1,4,i] else ")+tolHand4+string("; \n\n");

        }

        stream << string("# Ring finger left \n\n");

        stream << string("param TF2_H_0_left {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta0_fing3_left) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_fing3_left)*cos(alpha_fing3_left[1])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_fing3_left)*sin(alpha_fing3_left[1])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then u3x_left \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_fing3_left)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_fing3_left)*cos(alpha_fing3_left[1])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_fing3_left)*sin(alpha_fing3_left[1])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then u3y_left  \n");
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3_left[1])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3_left[1])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then u3z_left  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_left){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF2_0_1_left {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_left[3]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_left[3])*cos(alpha_fing3_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_left[3])*sin(alpha_fing3_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_left[2]*cos(joint_fingers_left[3]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_left[3])*cos(alpha_fing3_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_left[3])*sin(alpha_fing3_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing3_left[2]*sin(joint_fingers_left[3])  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF2_0_1_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,18]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,18])*cos(alpha_fing3_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,18])*sin(alpha_fing3_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_left[2]*cos(theta[i,18]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,18])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,18])*cos(alpha_fing3_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,18])*sin(alpha_fing3_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing3_left[2]*sin(theta[i,18])  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3_left[2])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3_left[2])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing3_left[2]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_left){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF2_1_2_left {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(2*joint_fingers_left[3]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*joint_fingers_left[3]/3)*cos(alpha_fing3_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(2*joint_fingers_right[3]/3)*sin(alpha_fing3_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_left[3]*cos(2*joint_fingers_left[3]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(2*joint_fingers_left[3]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(2*joint_fingers_left[3]/3)*cos(alpha_fing3_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*joint_fingers_left[3]/3)*sin(alpha_fing3_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_left[3]*sin(2*joint_fingers_left[3]/3)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF2_1_2_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(2*theta[i,18]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(2*theta[i,18]/3)*cos(alpha_fing3_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(2*theta[i,18]/3)*sin(alpha_fing3_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_left[3]*cos(2*theta[i,18]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(2*theta[i,18]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(2*theta[i,18]/3)*cos(alpha_fing3_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(2*theta[i,18]/3)*sin(alpha_fing3_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing1_left[3]*sin(2*theta[i,18]/3)  \n");

        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3_left[3])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3_left[3])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing3_left[3]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_left){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF2_2_3_left {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_left[3]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_left[3]/3)*cos(alpha_fing3_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_left[3]/3)*sin(alpha_fing3_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_left[4]*cos(joint_fingers_left[3]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_left[3]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_left[3]/3)*cos(alpha_fing3_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_left[3]/3)*sin(alpha_fing3_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing3_left[4]*sin(joint_fingers_left[3]/3)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF2_2_3_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,18]/3) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,18]/3)*cos(alpha_fing3_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,18]/3)*sin(alpha_fing3_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_fing3_left[4]*cos(theta[i,18]/3) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,18]/3)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,18]/3)*cos(alpha_fing3_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,18]/3)*sin(alpha_fing3_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_fing3_left[4]*sin(theta[i,18]/3)  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_fing3_left[4])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_fing3_left[4])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_fing3_left[4]  \n");
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
            stream << string("var F2_0_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H_left[i1,j]*TF2_H_0_left[j,i2]; \n");
            stream << string("var F2_1_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} F2_0_left[i1,j]*TF2_0_1_left[j,i2]; \n");
            stream << string("var F2_2_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} F2_1_left[i1,j]*TF2_1_2_left[j,i2]; \n");
            stream << string("var F2_tip_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} F2_2_left[i1,j]*TF2_2_3_left[j,i2]; \n\n");

            stream << string("var Finger2_0_left {i1 in 1..4} =  if i1<4 then F2_0_left[i1,4] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger2_1_left {i1 in 1..4} =  if i1<4 then F2_1_left[i1,4] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger2_2_left {i1 in 1..4} =  if i1<4 then F2_2_left[i1,4] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger2_tip_left {i1 in 1..4} =  if i1<4 then F2_tip_left[i1,4] else ")+tolHand4+string("; \n\n");
        }else{
            //bounce posture selection
            stream << string("var F2_0_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H_left[i1,j,i]*TF2_H_0_left[j,i2]; \n");
            if (transport_left){
                stream << string("var F2_1_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_0_left[i1,j,i]*TF2_0_1_left[j,i2]; \n");
                stream << string("var F2_2_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_1_left[i1,j,i]*TF2_1_2_left[j,i2]; \n");
                stream << string("var F2_tip_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_2_left[i1,j,i]*TF2_2_3_left[j,i2]; \n\n");
            }else{
                stream << string("var F2_1_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_0_left[i1,j,i]*TF2_0_1_left[j,i2,i]; \n");
                stream << string("var F2_2_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_1_left[i1,j,i]*TF2_1_2_left[j,i2,i]; \n");
                stream << string("var F2_tip_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F2_2_left[i1,j,i]*TF2_2_3_left[j,i2,i]; \n\n");
            }
            stream << string("var Finger2_0_left {i1 in 1..4,i in Iterations} =  if i1<4 then F2_0_left[i1,4,i] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger2_1_left {i1 in 1..4,i in Iterations} =  if i1<4 then F2_1_left[i1,4,i] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger2_2_left {i1 in 1..4,i in Iterations} =  if i1<4 then F2_2_left[i1,4,i] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger2_tip_left {i1 in 1..4,i in Iterations} =  if i1<4 then F2_tip_left[i1,4,i] else ")+tolHand4+string("; \n\n");

        }

        stream << string("# Thumb finger left \n\n");

        stream << string("param TF3_H_0_left {i1 in 1..4, i2 in 1..4} =  \n");
        stream << string("# 1st row \n");
        stream << string("if ( i1=1 && i2=1 ) then cos(theta0_thumb_left) \n");
        stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta0_thumb_left)*cos(alpha_thumb_left[1])  \n");
        stream << string("else	if ( i1=1 && i2=3 ) then sin(theta0_thumb_left)*sin(alpha_thumb_left[1])  \n");
        stream << string("else	if ( i1=1 && i2=4 ) then uTx_left \n");
        stream << string("# 2nd row \n");
        stream << string("else	if ( i1=2 && i2=1 ) then sin(theta0_thumb_left)  \n");
        stream << string("else	if ( i1=2 && i2=2 ) then cos(theta0_thumb_left)*cos(alpha_thumb_left[1])  \n");
        stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta0_thumb_left)*sin(alpha_thumb_left[1])  \n");
        stream << string("else	if ( i1=2 && i2=4 ) then uTy_left  \n");
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb_left[1])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb_left[1])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then uTz_right  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_left){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF3_0_1_left {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_left[1]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_left[1])*cos(alpha_thumb_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_left[1])*sin(alpha_thumb_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_left[2]*cos(joint_fingers_left[1]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_left[1])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_left[1])*cos(alpha_thumb_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_right[1])*sin(alpha_thumb_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_left[2]*sin(joint_fingers_left[1])  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF3_0_1_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,17]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,17])*cos(alpha_thumb_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,17])*sin(alpha_thumb_left[2])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_left[2]*cos(theta[i,17]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,17])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,17])*cos(alpha_thumb_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,17])*sin(alpha_thumb_left[2])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_left[2]*sin(theta[i,17])  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb_left[2])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb_left[2])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_thumb_left[2]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_left){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF3_1_2_left {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(joint_fingers_left[4]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(joint_fingers_left[4])*cos(alpha_thumb_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(joint_fingers_left[4])*sin(alpha_thumb_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_left[3]*cos(joint_fingers_left[4]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(joint_fingers_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(joint_fingers_left[4])*cos(alpha_thumb_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(joint_fingers_left[4])*sin(alpha_thumb_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_left[3]*sin(joint_fingers_left[4])  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF3_1_2_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(theta[i,19]) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(theta[i,19])*cos(alpha_thumb_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(theta[i,19])*sin(alpha_thumb_left[3])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_left[3]*cos(theta[i,19]) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(theta[i,19])  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(theta[i,19])*cos(alpha_thumb_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(theta[i,19])*sin(alpha_thumb_left[3])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_left[3]*sin(theta[i,19])  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb_left[3])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb_left[3])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_thumb_left[3]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_left){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF3_2_3_left {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(11*joint_fingers_left[4]/10) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*joint_fingers_left[4]/10)*cos(alpha_thumb_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(11*joint_fingers_left[4]/10)*sin(alpha_thumb_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_left[4]*cos(11*joint_fingers_left[4]/10) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(11*joint_fingers_left[4]/10)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(11*joint_fingers_left[4]/10)*cos(alpha_thumb_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*joint_fingers_left[4]/10)*sin(alpha_thumb_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_left[4]*sin(11*joint_fingers_left[4]/10)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF3_2_3_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(11*theta[i,19]/10) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*theta[i,19]/10)*cos(alpha_thumb_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(11*theta[i,19]/10)*sin(alpha_thumb_left[4])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_left[4]*cos(11*theta[i,19]/10) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(11*theta[i,19]/10)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(11*theta[i,19]/10)*cos(alpha_thumb_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*theta[i,19]/10)*sin(alpha_thumb_left[4])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_left[4]*sin(11*theta[i,19]/10)  \n");

        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb_left[4])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb_left[4])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_thumb_left[4]  \n");
        stream << string("# 4th row \n");
        stream << string("else	if ( i1=4 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=2 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=3 ) then 0  \n");
        stream << string("else	if ( i1=4 && i2=4 ) then 1  \n");
        stream << string(";  \n");

        if (final || transport_left){
            // final posture selection or bounce posture selection for transporting movements
            stream << string("param TF3_3_4_left {i1 in 1..4, i2 in 1..4} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(11*joint_fingers_left[4]/12) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*joint_fingers_left[4]/12)*cos(alpha_thumb_left[5])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(11*joint_fingers_left[4]/12)*sin(alpha_thumb_left[5])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_left[5]*cos(11*joint_fingers_left[4]/12) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(11*joint_fingers_left[4]/12)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(11*joint_fingers_left[4]/12)*cos(alpha_thumb_left[5])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*joint_fingers_left[4]/12)*sin(alpha_thumb_left[5])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_left[5]*sin(11*joint_fingers_left[4]/12)  \n");
        }else{
            // bounce posture selection for reaching movements
            stream << string("var TF3_3_4_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  \n");
            stream << string("# 1st row \n");
            stream << string("if ( i1=1 && i2=1 ) then cos(11*theta[i,19]/12) \n");
            stream << string("else	if ( i1=1 && i2=2 ) then -sin(11*theta[i,19]/12)*cos(alpha_thumb_left[5])  \n");
            stream << string("else	if ( i1=1 && i2=3 ) then sin(11*theta[i,19]/12)*sin(alpha_thumb_left[5])  \n");
            stream << string("else	if ( i1=1 && i2=4 ) then a_thumb_left[5]*cos(11*theta[i,19]/12) \n");
            stream << string("# 2nd row \n");
            stream << string("else	if ( i1=2 && i2=1 ) then sin(11*theta[i,19]/12)  \n");
            stream << string("else	if ( i1=2 && i2=2 ) then cos(11*theta[i,19]/12)*cos(alpha_thumb_left[5])  \n");
            stream << string("else	if ( i1=2 && i2=3 ) then -cos(11*theta[i,19]/12)*sin(alpha_thumb_left[5])  \n");
            stream << string("else	if ( i1=2 && i2=4 ) then a_thumb_left[5]*sin(11*theta[i,19]/12)  \n");
        }
        stream << string("# 3rd row \n");
        stream << string("else	if ( i1=3 && i2=1 ) then 0  \n");
        stream << string("else	if ( i1=3 && i2=2 ) then sin(alpha_thumb_left[5])  \n");
        stream << string("else	if ( i1=3 && i2=3 ) then cos(alpha_thumb_left[5])  \n");
        stream << string("else	if ( i1=3 && i2=4 ) then d_thumb_left[5]  \n");
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
            stream << string("var F3_0_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H_left[i1,j]*TF3_H_0_left[j,i2]; \n");
            stream << string("var F3_1_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_0_left[i1,j]*TF3_0_1_left[j,i2]; \n");
            stream << string("var F3_2_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_1_left[i1,j]*TF3_1_2_left[j,i2]; \n");
            stream << string("var F3_3_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_2_left[i1,j]*TF3_2_3_left[j,i2]; \n");
            stream << string("var F3_tip_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F3_3_left[i1,j]*TF3_3_4_left[j,i2]; \n\n");

            //PostureMod << string("var Finger3_0   {i1 in 1..4} =  if i1<4 then F3_0[i1,4] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger3_1_left {i1 in 1..4} =  if i1<4 then F3_1_left[i1,4] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger3_2_left {i1 in 1..4} =  if i1<4 then F3_2_left[i1,4] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger3_3_left {i1 in 1..4} =  if i1<4 then F3_3_left[i1,4] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger3_tip_left {i1 in 1..4} =  if i1<4 then F3_tip_left[i1,4] else ")+tolHand4+string("; \n\n");

        }else{
            //bounce posture selection
            stream << string("var F3_0_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4} T_W_H_left[i1,j,i]*TF3_H_0_left[j,i2]; \n");
            if(transport_left){
                stream << string("var F3_1_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_0_left[i1,j,i]*TF3_0_1_left[j,i2]; \n");
                stream << string("var F3_2_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_1_left[i1,j,i]*TF3_1_2_left[j,i2]; \n");
                stream << string("var F3_3_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_2_left[i1,j,i]*TF3_2_3_left[j,i2]; \n");
                stream << string("var F3_tip_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_3_left[i1,j,i]*TF3_3_4_left[j,i2]; \n\n");
            }else{
                stream << string("var F3_1_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_0_left[i1,j,i]*TF3_0_1_left[j,i2,i]; \n");
                stream << string("var F3_2_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_1_left[i1,j,i]*TF3_1_2_left[j,i2,i]; \n");
                stream << string("var F3_3_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_2_left[i1,j,i]*TF3_2_3_left[j,i2,i]; \n");
                stream << string("var F3_tip_left {i1 in 1..4, i2 in 1..4, i in Iterations} =  sum {j in 1..4}  F3_3_left[i1,j,i]*TF3_3_4_left[j,i2,i]; \n\n");
            }
            stream << string("var Finger3_1_left {i1 in 1..4, i in Iterations} =  if i1<4 then F3_1_left[i1,4,i] 	else ")+tolHand1+string("; \n");
            stream << string("var Finger3_2_left {i1 in 1..4, i in Iterations} =  if i1<4 then F3_2_left[i1,4,i] 	else ")+tolHand2+string("; \n");
            stream << string("var Finger3_3_left {i1 in 1..4, i in Iterations} =  if i1<4 then F3_3_left[i1,4,i] 	else ")+tolHand3+string("; \n");
            stream << string("var Finger3_tip_left {i1 in 1..4, i in Iterations} =  if i1<4 then F3_tip_left[i1,4,i] else ")+tolHand4+string("; \n\n");

        }
    }
}

void HUMPlanner::writeBarrettHandDirKin(ofstream &stream, MatrixXd &tolsHand, bool final, bool place)
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
                 stream << string("var F")+to_string(i+1)+string("_0   {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H[i1,j]*TF")+to_string(i+1)+string("_1[j,i2]; \n");
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
        // bounce posture selection
        if (place){
            // place movement
            for (unsigned i = 0 ; i < hand_fingers; ++i){
                //for (int j = 0; j <N_PHALANGE; ++j){
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
                 stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 1 \n");
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
                 stream << string("var F")+to_string(i+1)+string("_0   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H[i1,j,i]*TF")+to_string(i+1)+string("_1[j,i2,i]; \n");
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
            // pick or move  movements
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
                 stream << string("var F")+to_string(i+1)+string("_0   {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H[i1,j,i]*TF")+to_string(i+1)+string("_1[j,i2,i]; \n");
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

void HUMPlanner::writeDualBarrettHandDirKin(std::ofstream& stream, MatrixXd& tolsHand, bool final, bool place_right, bool place_left, bool right)
{
    std::vector<int> rk; std::vector<int> jk;
    rk = this->bhand.rk; jk = this->bhand.jk;

    if(right)
    { // right hand

        stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
        stream << string("#  Direct Kinematics model of the fingers in the RIGHT HAND \n\n");

        if(final){
            // final posture selection
            for (unsigned i = 0 ; i < hand_fingers; ++i){
                //for (int j = 0; j <N_PHALANGE; ++j){

                string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
                string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                     stream << string("# Right Finger ")+to_string(i+1)+string(" \n\n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("_right")+string(" {i1 in 1..4, i2 in 1..4} :=   \n");
                     stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*joint_fingers_right[")+to_string(1)+string("]-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*joint_fingers_right[")+to_string(1)+string("]-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*joint_fingers_right[")+to_string(1)+string("]-(pi/2)*(")+jkk+string("))  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw_right 	 				 \n");
                     stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(2)+string("_right")+string(" {i1 in 1..4, i2 in 1..4} :=   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(joint_fingers_right[")+to_string(i+2)+string("]+phi_2_right)  \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(joint_fingers_right[")+to_string(i+2)+string("]+phi_2_right)  \n");
                     stream << string("else 	if (i1=3&&i2=1)					then    sin(joint_fingers_right[")+to_string(i+2)+string("]+phi_2_right)  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                     stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                     stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A1_right \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(3)+string("_right")+string(" {i1 in 1..4, i2 in 1..4} :=   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((joint_fingers_right[")+to_string(i+2)+string("])/3+phi_3_right) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin((joint_fingers_right[")+to_string(i+2)+string("])/3+phi_3_right) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then    sin((joint_fingers_right[")+to_string(i+2)+string("])/3+phi_3_right) \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                     stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 														1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A2_right \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("_right")+string(" {i1 in 1..4, i2 in 1..4} :=   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                     stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                     stream << string("else 	if (i1=1&&i2=4)					              then    A3_right \n");
                     stream << string("else 	if (i1=2&&i2=4)					              then    D3_right \n");
                     stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string(";\n\n");


                     // position of the fingers
                     stream << string("var F")+to_string(i+1)+string("_0_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H_right[i1,j]*TF")+to_string(i+1)+string("_1_right[j,i2]; \n");
                     stream << string("var F")+to_string(i+1)+string("_1_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0_right[i1,j]*TF")+to_string(i+1)+string("_2_right[j,i2]; \n");
                     stream << string("var F")+to_string(i+1)+string("_2_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1_right[i1,j]*TF")+to_string(i+1)+string("_3_right[j,i2]; \n");
                     stream << string("var F")+to_string(i+1)+string("_tip_right {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2_right[i1,j]*TF")+to_string(i+1)+string("_4_right[j,i2]; \n\n");

                     string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                     string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                     string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                     string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                     stream << string("var Finger")+to_string(i+1)+string("_0_right {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_0_right[i1,4] 	else ")+tolHand1+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_1_right {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_1_right[i1,4] 	else ")+tolHand2+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_2_right {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_2_right[i1,4] 	else ")+tolHand3+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_tip_right {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_tip_right[i1,4] else ")+tolHand4+string("; \n\n");

               // }
            }
        }else{
            // bounce posture selection
            if (place_right){
                // place right movement
                for (unsigned i = 0 ; i < hand_fingers; ++i){
                    //for (int j = 0; j <N_PHALANGE; ++j){
                    string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
                    string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                     stream << string("# Right Finger ")+to_string(i+1)+string(" \n\n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("_right")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string("))  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw_right 	 				 \n");
                     stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(2)+string("_right")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(joint_fingers_right[")+to_string(i+2)+string("]+phi_2_right)  \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(joint_fingers_right[")+to_string(i+2)+string("]+phi_2_right)  \n");
                     stream << string("else 	if (i1=3&&i2=1)					then    sin(joint_fingers_right[")+to_string(i+2)+string("]+phi_2_right)  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                     stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                     stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A1_right \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(3)+string("_right")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((joint_fingers_right[")+to_string(i+2)+string("])/3+phi_3_right) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin((joint_fingers_right[")+to_string(i+2)+string("])/3+phi_3_right) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then    sin((joint_fingers_right[")+to_string(i+2)+string("])/3+phi_3_right) \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                     stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A2_right \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("_right")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} :=   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                     stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                     stream << string("else 	if (i1=1&&i2=4)					              then    A3_right \n");
                     stream << string("else 	if (i1=2&&i2=4)					              then    D3_right \n");
                     stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string(";\n\n");


                     // position of the fingers
                     stream << string("var F")+to_string(i+1)+string("_0_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H_right[i1,j,i]*TF")+to_string(i+1)+string("_1_right[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_1_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0_right[i1,j,i]*TF")+to_string(i+1)+string("_2_right[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_2_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1_right[i1,j,i]*TF")+to_string(i+1)+string("_3_right[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_tip_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2_right[i1,j,i]*TF")+to_string(i+1)+string("_4_right[j,i2,i]; \n\n");


                     string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                     string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                     string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                     string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                     stream << string("var Finger")+to_string(i+1)+string("_0_right   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_0_right[i1,4,i] 	else ")+tolHand1+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_1_right   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_1_right[i1,4,i] 	else ")+tolHand2+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_2_right   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_2_right[i1,4,i] 	else ")+tolHand3+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_tip_right {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_tip_right[i1,4,i] else ")+tolHand4+string("; \n\n");


                   // }

                }

            }else{
                // pick or move  right movements
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

                     stream << string("# Right Finger ")+to_string(i+1)+string(" \n\n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("_right")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string("))  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw_right 	 				 \n");
                     stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                     stream << string("; \n");

                     stream << string("var TF")+to_string(i+1)+string("_")+to_string(2)+string("_right")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(theta[i,")+to_string(k)+string("]+phi_2_right)  \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(theta[i,")+to_string(k)+string("]+phi_2_right)  \n");
                     stream << string("else 	if (i1=3&&i2=1)					then    sin(theta[i,")+to_string(k)+string("]+phi_2_right)  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                     stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                     stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A1_right \n");
                     stream << string("; \n");

                     stream << string("var TF")+to_string(i+1)+string("_")+to_string(3)+string("_right")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((theta[i,")+to_string(k)+string("])/3+phi_3_right) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin((theta[i,")+to_string(k)+string("])/3+phi_3_right) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then    sin((theta[i,")+to_string(k)+string("])/3+phi_3_right) \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                     stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 														1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A2_right \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("_right")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} :=   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                     stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                     stream << string("else 	if (i1=1&&i2=4)					              then    A3_right \n");
                     stream << string("else 	if (i1=2&&i2=4)					              then    D3_right \n");
                     stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string(";\n\n");


                     // position of the fingers
                     stream << string("var F")+to_string(i+1)+string("_0_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H_right[i1,j,i]*TF")+to_string(i+1)+string("_1_right[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_1_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0_right[i1,j,i]*TF")+to_string(i+1)+string("_2_right[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_2_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1_right[i1,j,i]*TF")+to_string(i+1)+string("_3_right[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_tip_right {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2_right[i1,j,i]*TF")+to_string(i+1)+string("_4_right[j,i2,i]; \n\n");

                     string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                     string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                     string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                     string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                     stream << string("var Finger")+to_string(i+1)+string("_0_right {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_0_right[i1,4,i] 	else ")+tolHand1+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_1_right {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_1_right[i1,4,i] 	else ")+tolHand2+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_2_right {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_2_right[i1,4,i] 	else ")+tolHand3+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_tip_right {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_tip_right[i1,4,i] else ")+tolHand4+string("; \n\n");


                   // }

                }
            }
        }
    }else{
        // left hand
        stream << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
        stream << string("#  Direct Kinematics model of the fingers in the LEFT HAND \n\n");

        if(final){
            // final posture selection
            for (unsigned i = 0 ; i < hand_fingers; ++i){
                //for (int j = 0; j <N_PHALANGE; ++j){

                string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
                string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                     stream << string("# Left Finger ")+to_string(i+1)+string(" \n\n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("_left")+string(" {i1 in 1..4, i2 in 1..4} :=   \n");
                     stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*joint_fingers_left[")+to_string(1)+string("]-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*joint_fingers_left[")+to_string(1)+string("]-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*joint_fingers_left[")+to_string(1)+string("]-(pi/2)*(")+jkk+string("))  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw_left 	 				 \n");
                     stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(2)+string("_left")+string(" {i1 in 1..4, i2 in 1..4} :=   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(joint_fingers_left[")+to_string(i+2)+string("]+phi_2_left)  \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(joint_fingers_left[")+to_string(i+2)+string("]+phi_2_left)  \n");
                     stream << string("else 	if (i1=3&&i2=1)					then    sin(joint_fingers_left[")+to_string(i+2)+string("]+phi_2_left)  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                     stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                     stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A1_left \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(3)+string("_left")+string(" {i1 in 1..4, i2 in 1..4} :=   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((joint_fingers_left[")+to_string(i+2)+string("])/3+phi_3_left) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin((joint_fingers_left[")+to_string(i+2)+string("])/3+phi_3_left) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then    sin((joint_fingers_left[")+to_string(i+2)+string("])/3+phi_3_left) \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                     stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 														1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A2_left \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("_left")+string(" {i1 in 1..4, i2 in 1..4} :=   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                     stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                     stream << string("else 	if (i1=1&&i2=4)					              then    A3_left \n");
                     stream << string("else 	if (i1=2&&i2=4)					              then    D3_left \n");
                     stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string(";\n\n");


                     // position of the fingers
                     stream << string("var F")+to_string(i+1)+string("_0_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4} T_W_H_left[i1,j]*TF")+to_string(i+1)+string("_1_left[j,i2]; \n");
                     stream << string("var F")+to_string(i+1)+string("_1_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0_left[i1,j]*TF")+to_string(i+1)+string("_2_left[j,i2]; \n");
                     stream << string("var F")+to_string(i+1)+string("_2_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1_left[i1,j]*TF")+to_string(i+1)+string("_3_left[j,i2]; \n");
                     stream << string("var F")+to_string(i+1)+string("_tip_left {i1 in 1..4, i2 in 1..4} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2_left[i1,j]*TF")+to_string(i+1)+string("_4_left[j,i2]; \n\n");

                     string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                     string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                     string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                     string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                     stream << string("var Finger")+to_string(i+1)+string("_0_left {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_0_left[i1,4] 	else ")+tolHand1+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_1_left {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_1_left[i1,4] 	else ")+tolHand2+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_2_left {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_2_left[i1,4] 	else ")+tolHand3+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_tip_left {i1 in 1..4} =  if i1<4 then F")+to_string(i+1)+string("_tip_left[i1,4] else ")+tolHand4+string("; \n\n");

               // }
            }
        }else{
            // bounce posture selection
            if (place_left){
                // place left movement
                for (unsigned i = 0 ; i < hand_fingers; ++i){
                    //for (int j = 0; j <N_PHALANGE; ++j){
                    string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
                    string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                     stream << string("# Left Finger ")+to_string(i+1)+string(" \n\n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("_left")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string("))  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw_left 	 				 \n");
                     stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(2)+string("_left")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(joint_fingers_left[")+to_string(i+2)+string("]+phi_2_left)  \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(joint_fingers_left[")+to_string(i+2)+string("]+phi_2_left)  \n");
                     stream << string("else 	if (i1=3&&i2=1)					then    sin(joint_fingers_left[")+to_string(i+2)+string("]+phi_2_left)  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                     stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                     stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A1_left \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(3)+string("_left")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((joint_fingers_left[")+to_string(i+2)+string("])/3+phi_3_left) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin((joint_fingers_left[")+to_string(i+2)+string("])/3+phi_3_left) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then    sin((joint_fingers_left[")+to_string(i+2)+string("])/3+phi_3_left) \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                     stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A2_left \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("_left")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} :=   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                     stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                     stream << string("else 	if (i1=1&&i2=4)					              then    A3_left \n");
                     stream << string("else 	if (i1=2&&i2=4)					              then    D3_left \n");
                     stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string(";\n\n");


                     // position of the fingers
                     stream << string("var F")+to_string(i+1)+string("_0_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H_left[i1,j,i]*TF")+to_string(i+1)+string("_1_left[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_1_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0_left[i1,j,i]*TF")+to_string(i+1)+string("_2_left[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_2_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1_left[i1,j,i]*TF")+to_string(i+1)+string("_3_left[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_tip_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2_left[i1,j,i]*TF")+to_string(i+1)+string("_4_left[j,i2,i]; \n\n");


                     string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                     string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                     string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                     string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                     stream << string("var Finger")+to_string(i+1)+string("_0_left {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_0_left[i1,4,i] 	else ")+tolHand1+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_1_left {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_1_left[i1,4,i] 	else ")+tolHand2+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_2_left   {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_2_left[i1,4,i] 	else ")+tolHand3+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_tip_left {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_tip_left[i1,4,i] else ")+tolHand4+string("; \n\n");


                   // }

                }
            }else{
                // pick or move  left movements
                for (unsigned i = 0 ; i < hand_fingers; ++i){
                    //for (int j = 0; j <N_PHALANGE; ++j){

                    int k;
                    if (i == 2){
                        k = 17;
                    }else{
                        k = 18;
                    }

                    string rkk = boost::str(boost::format("%.2f") % rk.at(i)); boost::replace_all(rkk,",",".");
                    string jkk = boost::str(boost::format("%.2f") % jk.at(i)); boost::replace_all(jkk,",",".");

                     stream << string("# Left Finger ")+to_string(i+1)+string(" \n\n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(1)+string("_left")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string(" if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string(")) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then 	sin(")+rkk+string("*")+to_string(0)+string("-(pi/2)*(")+jkk+string("))  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2>2 )||( i1=3 && i2<3 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string("else	if (( i1=3 && i2=3 )||( i1=4 && i2=4 ) ) then 								1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 				")+rkk+string("*Aw_left 	 				 \n");
                     stream << string("else	if ( i1=3 && i2=4 ) then 										0 \n");
                     stream << string("; \n");

                     stream << string("var TF")+to_string(i+1)+string("_")+to_string(2)+string("_left")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=3&&i2=2)) then 	cos(theta[i,")+to_string(k)+string("]+phi_2_left)  \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin(theta[i,")+to_string(k)+string("]+phi_2_left)  \n");
                     stream << string("else 	if (i1=3&&i2=1)					then    sin(theta[i,")+to_string(k)+string("]+phi_2_left)  \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=4 && i2<4 )||( i1=2 && i2=4 )) then 	0 \n");
                     stream << string("else	if ( i1=4 && i2=4 )  then 								1 \n");
                     stream << string("else	if ( i1=2 && i2=3 )  then 								-1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A1_left \n");
                     stream << string("; \n");

                     stream << string("var TF")+to_string(i+1)+string("_")+to_string(3)+string("_left")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} =   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=2)) then 	cos((theta[i,")+to_string(k)+string("])/3+phi_3_left) \n");
                     stream << string("else 	if (i1=1&&i2=2)					then   -sin((theta[i,")+to_string(k)+string("])/3+phi_3_left) \n");
                     stream << string("else 	if (i1=2&&i2=1)					then    sin((theta[i,")+to_string(k)+string("])/3+phi_3_left) \n");
                     stream << string("else 	if (( i1=1 && i2=3 )||( i1=3 && i2<3 )||( i1=2 && i2>2 )||( i1=4 && i2<4 )||( i1=3 && i2=4 )) then 	0 \n");
                     stream << string("else	if (( i1=4 && i2=4 )||( i1=3 && i2=3 ))  then 														1 \n");
                     stream << string("else	if ( i1=1 && i2=4 ) then 								A2_left \n");
                     stream << string("; \n");

                     stream << string("param TF")+to_string(i+1)+string("_")+to_string(4)+string("_left")+string(" {i1 in 1..4, i2 in 1..4,i in Iterations} :=   \n");
                     stream << string("if ((i1=1&&i2=1)||(i1=2&&i2=3)||(i1=4&&i2=4)) then 	  1\n");
                     stream << string("else 	if (i1=3&&i2=2)								  then   -1 \n");
                     stream << string("else 	if (i1=1&&i2=4)					              then    A3_left \n");
                     stream << string("else 	if (i1=2&&i2=4)					              then    D3_left \n");
                     stream << string("else 	if (( i1=1 && i2=2 )||( i1=1 && i2=3 )||( i1=2 && i2<3 )||( i1=3 && i2>2 )||( i1=3 && i2=1 )||( i1=4 && i2<4 )) then 	0 \n");
                     stream << string(";\n\n");


                     // position of the fingers
                     stream << string("var F")+to_string(i+1)+string("_0_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4} T_W_H_left[i1,j,i]*TF")+to_string(i+1)+string("_1_left[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_1_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_0_left[i1,j,i]*TF")+to_string(i+1)+string("_2_left[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_2_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_1_left[i1,j,i]*TF")+to_string(i+1)+string("_3_left[j,i2,i]; \n");
                     stream << string("var F")+to_string(i+1)+string("_tip_left {i1 in 1..4, i2 in 1..4,i in Iterations} =  sum {j in 1..4}  F")+to_string(i+1)+string("_2_left[i1,j,i]*TF")+to_string(i+1)+string("_4_left[j,i2,i]; \n\n");

                     string tolHand1 =  boost::str(boost::format("%.2f") % tolsHand(0,i)); boost::replace_all(tolHand1,",",".");
                     string tolHand2 =  boost::str(boost::format("%.2f") % tolsHand(1,i)); boost::replace_all(tolHand2,",",".");
                     string tolHand3 =  boost::str(boost::format("%.2f") % tolsHand(2,i)); boost::replace_all(tolHand3,",",".");
                     string tolHand4 =  boost::str(boost::format("%.2f") % tolsHand(3,i)); boost::replace_all(tolHand4,",",".");

                     stream << string("var Finger")+to_string(i+1)+string("_0_left {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_0_left[i1,4,i] 	else ")+tolHand1+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_1_left {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_1_left[i1,4,i] 	else ")+tolHand2+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_2_left {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_2_left[i1,4,i] 	else ")+tolHand3+string("; \n");
                     stream << string("var Finger")+to_string(i+1)+string("_tip_left {i1 in 1..4,i in Iterations} =  if i1<4 then F")+to_string(i+1)+string("_tip_left[i1,4,i] else ")+tolHand4+string("; \n\n");


                   // }

                }
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
        //stream << string("subject to BodyArm_constr{j in 1..15}: (Points_Arm[j,1]/body[1])^2 + (Points_Arm[j,2]/body[2])^2 >= 1; \n");

        stream << string("subject to BodyArm_Elbow: (Elbow[1]/(body[1]+Elbow[4]))^2 + (Elbow[2]/(body[2]+Elbow[4]))^2 >= 1; \n");
        stream << string("subject to BodyArm_Wrist: (Wrist[1]/(body[1]+Wrist[4]))^2 + (Wrist[2]/(body[2]+Wrist[4]))^2 >= 1; \n");
        stream << string("subject to BodyArm_Hand:  (Hand[1]/(body[1]+Hand[4]))^2  + (Hand[2]/(body[2]+Hand[4]))^2  >= 1; \n\n");

    }else{
        //stream << string("subject to BodyArm_constr{j in 1..15,l in Iterations}: (Points_Arm[j,1,l]/body[1])^2 + (Points_Arm[j,2,l]/body[2])^2 >= 1; \n");

        stream << string("subject to BodyArm_Elbow{l in Iterations}: (Elbow[1,l]/(body[1]+Elbow[4,l]))^2 + (Elbow[2,l]/(body[2]+Elbow[4,l]))^2 >= 1; \n");
        stream << string("subject to BodyArm_Wrist{l in Iterations}: (Wrist[1,l]/(body[1]+Wrist[4,l]))^2 + (Wrist[2,l]/(body[2]+Wrist[4,l]))^2 >= 1; \n");
        stream << string("subject to BodyArm_Hand{l in Iterations}:  (Hand[1,l]/(body[1]+Hand[4,l]))^2  + (Hand[2,l]/(body[2]+Hand[4,l]))^2  >= 1; \n\n");
    }
}

void HUMPlanner::writeDualBodyConstraints(ofstream &stream, bool final,bool right)
{
    stream << string("\n\n");
    if(right)
    {
        // right arm
        stream << string("# Constraints of the right arm with the body: the body is modeled as a cylinder \n");
        if (final){
            stream << string("subject to BodyArm_Elbow_right: (Elbow_right[1]/(body[1]+Elbow_right[4]))^2 + (Elbow_right[2]/(body[2]+Elbow_right[4]))^2 >= 1; \n");
            stream << string("subject to BodyArm_Wrist_right: (Wrist_right[1]/(body[1]+Wrist_right[4]))^2 + (Wrist_right[2]/(body[2]+Wrist_right[4]))^2 >= 1; \n");
            stream << string("subject to BodyArm_Hand_right:  (Hand_right[1]/(body[1]+Hand_right[4]))^2  + (Hand_right[2]/(body[2]+Hand_right[4]))^2  >= 1; \n\n");
        }else{
            stream << string("subject to BodyArm_Elbow_right{l in Iterations}: (Elbow_right[1,l]/(body[1]+Elbow_right[4,l]))^2 + (Elbow_right[2,l]/(body[2]+Elbow_right[4,l]))^2 >= 1; \n");
            stream << string("subject to BodyArm_Wrist_right{l in Iterations}: (Wrist_right[1,l]/(body[1]+Wrist_right[4,l]))^2 + (Wrist_right[2,l]/(body[2]+Wrist_right[4,l]))^2 >= 1; \n");
            stream << string("subject to BodyArm_Hand_right{l in Iterations}:  (Hand_right[1,l]/(body[1]+Hand_right[4,l]))^2  + (Hand_right[2,l]/(body[2]+Hand_right[4,l]))^2  >= 1; \n\n");
        }
    }else{
        // left arm
        stream << string("# Constraints of the left arm with the body: the body is modeled as a cylinder \n");
        if (final){
            stream << string("subject to BodyArm_Elbow_left: (Elbow_left[1]/(body[1]+Elbow_left[4]))^2 + (Elbow_left[2]/(body[2]+Elbow_left[4]))^2 >= 1; \n");
            stream << string("subject to BodyArm_Wrist_left: (Wrist_left[1]/(body[1]+Wrist_left[4]))^2 + (Wrist_left[2]/(body[2]+Wrist_left[4]))^2 >= 1; \n");
            stream << string("subject to BodyArm_Hand_left:  (Hand_left[1]/(body[1]+Hand_left[4]))^2  + (Hand_left[2]/(body[2]+Hand_left[4]))^2  >= 1; \n\n");
        }else{
            stream << string("subject to BodyArm_Elbow_left{l in Iterations}: (Elbow_left[1,l]/(body[1]+Elbow_left[4,l]))^2 + (Elbow_left[2,l]/(body[2]+Elbow_left[4,l]))^2 >= 1; \n");
            stream << string("subject to BodyArm_Wrist_left{l in Iterations}: (Wrist_left[1,l]/(body[1]+Wrist_left[4,l]))^2 + (Wrist_left[2,l]/(body[2]+Wrist_left[4,l]))^2 >= 1; \n");
            stream << string("subject to BodyArm_Hand_left{l in Iterations}:  (Hand_left[1,l]/(body[1]+Hand_left[4,l]))^2  + (Hand_left[2,l]/(body[2]+Hand_left[4,l]))^2  >= 1; \n\n");
        }
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

double HUMPlanner::getRand(double min, double max)
{
    sleep(1);
    std::srand(std::time(NULL));
    double f = (double)std::rand() / RAND_MAX;
    return min + f * (max - min);
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

bool HUMPlanner::writeFilesFinalPosture(hump_params& params,int mov_type, int pre_post, std::vector<double> initArmPosture, std::vector<double> initialGuess,std::vector<objectPtr> obsts)
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
    //int griptype = params.mov_specs.griptype;
    bool coll = params.mov_specs.coll;
    bool coll_body = params.coll_body;
    std::vector<double> tar = params.mov_specs.target;
    int dHO = params.mov_specs.dHO;
    std::vector<double> finalHand = params.mov_specs.finalHand;
    std::string mov_infoLine = params.mov_specs.mov_infoline;
    Matrix4d T_tar_to_obj = params.mov_specs.T_tar_to_obj;
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
    case 1: // place
        obj_tar = params.mov_specs.obj;
        if(approach){pre_place_approach = params.mov_specs.pre_place_approach;}
        if(retreat){post_place_retreat = params.mov_specs.post_place_retreat;}
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
        k=1;
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
    if(coll && coll_body){
        this->writeBodyDim(this->torso_size.at(0),this->torso_size.at(1),PostureDat);
    }
    // D-H Parameters of the Arm
    this->writeArmDHParams(dh_arm,PostureDat,k);
    // distance between the hand and the object
    this->write_dHO(PostureDat,dHO);
    // joint limits
    this->writeArmLimits(PostureDat,minArmLimits,maxArmLimits,true);
    // initial pose of the arm
    this->writeArmInitPose(PostureDat,initArmPosture);
    // final posture of the fingers
    this->writeFingerFinalPose(PostureDat,finalHand);
    // joint expense factors of the arm
    this->writeLambda(PostureDat,lambda);
    // initial guess
    PostureDat << string("# INITIAL GUESS \n");
    PostureDat << string("var theta = \n");
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
    case 1: // place
        switch(pre_post){
        case 0: // no approach, no retreat
            break;
        case 1: // approach
            if(approach){this->writeInfoApproachRetreat(PostureDat,tar,pre_place_approach);}
            break;
        case 2: // retreat
            if(retreat){this->writeInfoApproachRetreat(PostureDat,tar,post_place_retreat);}
            break;
        }

        break;
    }
    if(coll){
        //info objects
        this->writeInfoObstacles(PostureDat,obsts);
        // object that has the target
        std::vector<double> dim;
        switch(mov_type){
        case 0: //pick
            obj_tar->getSize(dim);
            if(pre_post==2){// retreat stage
                this->writeInfoObjectTarget(PostureDat,tar,T_tar_to_obj,dim, obj_tar->getName());
            }else{
                this->writeInfoObjectTarget(PostureDat,obj_tar);
            }
            break;
        case 1: //place
            obj_tar->getSize(dim);
            if(pre_post==2){// retreat stage
                this->writeInfoObjectTargetPlaceRetreat(PostureDat,tar,T_tar_to_obj,dim, obj_tar->getName());
            }else{
                this->writeInfoObjectTarget(PostureDat,tar,T_tar_to_obj,dim, obj_tar->getName());
            }
            break;
        }
    }
    //close the file
    //PostureDat.close();

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
    if(coll && coll_body){
        this->writeBodyDimMod(PostureMod);
    }
    this->writeArmDHParamsMod(PostureMod);
    this->write_dHOMod(PostureMod);

    PostureMod << string("# Joint Limits \n");
    PostureMod << string("param llim {i in 1..")+to_string(joints_arm)+string("} ; \n");
    PostureMod << string("param ulim {i in 1..")+to_string(joints_arm)+string("} ; \n");

    PostureMod << string("# Initial posture \n");
    PostureMod << string("param thet_init {i in 1..")+to_string(joints_arm)+string("} ; \n");

    PostureMod << string("# Final finger posture \n");
    PostureMod << string("param joint_fingers {i in 1..")+to_string(finalHand.size())+string("} ; \n");

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
    //switch(mov_type){
    //case 0: case 1: // pick, place
    if((approach || retreat) && pre_post!=0){vec=true;}
    this->writeInfoObjectsMod(PostureMod,vec);
        //break;
    //}
    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("# DECISION VARIABLES \n");
    PostureMod << string("var theta {i in 1..")+to_string(joints_arm)+string("} >= llim[i], <= ulim[i]; \n");

    // Rotation matrix of the obstacles
    this->writeRotMatObsts(PostureMod);
    // Direct Kinematics of the arm
    this->writeArmDirKin(PostureMod,matWorldToArm,matHand,tolsArm,true);

    bool obj_place = false;
    // object to transport (place movements in the plan and approach target posture selection or
    //                        pick movement in the retreat target posture selection)
    int n_s = 0; // number of spheres
    if((mov_type==1 && (pre_post==0 || pre_post==1)) || (mov_type==0 && pre_post==2)){
        obj_place = true;
        std::vector<double> obj_tar_size; obj_tar->getSize(obj_tar_size);
        n_s = this->model_spheres(PostureDat,PostureMod,obj_tar_size,true);
    }

    switch(hand_code){
    case 0: // human hand
        this->writeHumanHandDirKin(PostureMod,tolsHand,true,false);
        break;
    case 1: // barrett hand
        this->writeBarrettHandDirKin(PostureMod,tolsHand,true,false);
        break;
    }

    // Points of the arm
    //PostureMod << string("var Points_Arm {j in 1..21, i in 1..4} = \n");
    std::string n_str;
    if(obj_place){
        n_str = to_string(15+n_s);
    }else{
        n_str = to_string(15);
    }
    PostureMod << string("var Points_Arm {j in 1..")+n_str+string(", i in 1..4} = \n");
    PostureMod << string("if ( j=1 ) then 	(Shoulder[i]+Elbow[i])/2  \n");
    PostureMod << string("else	if ( j=2 ) then 	Elbow[i] \n");
    PostureMod << string("else    if ( j=3 ) then 	(Wrist[i]+Elbow[i])/2  \n");
    PostureMod << string("else	if ( j=4 ) then 	Wrist[i] \n");
    PostureMod << string("else	if ( j=5 ) then 	Wrist[i]+0.45*(Hand[i]-Wrist[i]) \n");
    PostureMod << string("else	if ( j=6 ) then 	Wrist[i]+0.75*(Hand[i]-Wrist[i]) \n");
    PostureMod << string("else	if ( j=7 ) then 	Finger1_1[i] \n");
    PostureMod << string("else	if ( j=8 ) then 	Finger2_1[i] \n");
    PostureMod << string("else	if ( j=9 ) then 	Finger3_1[i]\n");
    /*
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
    */
    PostureMod << string("else	if ( j=10 ) then 	 Finger1_2[i] \n");
    PostureMod << string("else	if ( j=11 ) then 	 Finger2_2[i] \n");
    PostureMod << string("else	if ( j=12 ) then 	 Finger3_2[i] \n");
    PostureMod << string("else	if ( j=13 ) then 	Finger1_tip[i]\n");
    PostureMod << string("else	if ( j=14 ) then 	Finger2_tip[i] \n");
    PostureMod << string("else	if ( j=15 ) then 	Finger3_tip[i] \n");
    if (obj_place){
        int j_init = 15;
        for(int i=1;i<=n_s;++i){
            std::string i_str = to_string(i);
            int j = j_init+i; std::string j_str = to_string(j);
            PostureMod << string("else    if ( j=")+j_str+string(" ) then 	Obj2Transp_")+i_str+string("[i] \n");
        }
    }
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
    case 0: // pick
        if(pre_post == 0){
            // do not use approach/retreat options
            PostureMod << string("# subject to constr_hand_pos  {i in 1..3}: Hand[i] + dFH*z_H[i] - Tar_pos[i] = 0; \n");
            PostureMod << string("subject to constr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        }else if(pre_post==1){
            // use approach options
            PostureMod << string("subject to constr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - dist*v_t[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        }else if(pre_post==2){
            // use retreat options
            PostureMod << string("subject to constr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - dist*v_t[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        }
        break;
    case 1: // place
        if(pre_post==0){
            // do not use approach/retreat options
            PostureMod << string("# subject to constr_hand_pos  {i in 1..3}: Hand[i] + dFH*z_H[i] - Tar_pos[i] = 0; \n");
            PostureMod << string("subject to constr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        }else if(pre_post==1){
            // use approach options
            PostureMod << string("# subject to constr_hand_pos  {i in 1..3}: Hand[i] + dFH*z_H[i] - dist*v_t[i] - Tar_pos[i] = 0; \n");
            PostureMod << string("subject to constr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - dist*v_t[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        }else if(pre_post==2){
            // use retreat options
            PostureMod << string("# subject to constr_hand_pos  {i in 1..3}: Hand[i] + dFH*z_H[i] - dist*v_t[i] - Tar_pos[i] = 0; \n");
            PostureMod << string("subject to constr_hand_pos: (sum{i in 1..3} (Hand[i] + dFH*z_H[i] - dist*v_t[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        }
        break;
    case 2: // move
        PostureMod << string("# subject to constr_hand_pos  {i in 1..3}: Hand[i] - Tar_pos[i] = 0; \n");
        PostureMod << string("subject to constr_hand_pos: (sum{i in 1..3} (Hand[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        break;
    }


    PostureMod << string("# Hand orientation\n");
    PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - x_t[i])^2 + sum{i in 1..3} (z_H[i] - z_t[i])^2 )<= ")+taror+string("; #  x_H = x_t and z_H = x_t \n");
    /*
    switch (mov_type){
    case 0: //pick
        // hand constraints for approaching and retreating direction setting
        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - x_t[i])^2 + sum{i in 1..3} (z_H[i] - z_t[i])^2 )<= ")+taror+string("; #  x_H = x_t and z_H = x_t \n");
        break;
    case 1:// place
        //PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 + sum{i in 1..3} (z_H[i] + y_t[i])^2 )<= ")+taror+string("; #  x_H = z_t and z_H = -y_t \n");
        break;
    }
    */
    /*
    switch(griptype){
    case 111: case 211: // side thumb left
        switch (mov_type){
        case 0: //pick
            // hand constraints for approaching and retreating direction setting
            PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 + sum{i in 1..3} (z_H[i] + y_t[i])^2 )<= ")+taror+string("; #  x_H = z_t and z_H = -y_t \n");
            break;
        case 1:// place
            PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - z_t[i])^2 + sum{i in 1..3} (z_H[i] + y_t[i])^2 )<= ")+taror+string("; #  x_H = z_t and z_H = -y_t \n");
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
    default: // move movements (there is no griptype)
        PostureMod << string("subject to constr_hand_orient: (sum{i in 1..3} (x_H[i] - x_t[i])^2 + sum{i in 1..3} (y_H[i] - y_t[i])^2 )<= ")+taror+string("; #  x_H = x_t and y_H = y_t \n");
        break;

    } // switch griptype
    */

    if(obstacle_avoidance && coll){
        // obstacles
        //xx
        string txx1 = boost::str(boost::format("%.2f") % tolsObstacles(0,0)); boost::replace_all(txx1,",","."); if(tolsObstacles(0,0) >= 0){txx1=string("+")+txx1;}
        string txx2 = boost::str(boost::format("%.2f") % tolsObstacles(1,0)); boost::replace_all(txx2,",","."); if(tolsObstacles(1,0) >= 0){txx2=string("+")+txx2;}
        string txx3 = boost::str(boost::format("%.2f") % tolsObstacles(2,0)); boost::replace_all(txx3,",","."); if(tolsObstacles(2,0) >= 0){txx3=string("+")+txx3;}
        //yy
        string tyy1 = boost::str(boost::format("%.2f") % tolsObstacles(0,1)); boost::replace_all(tyy1,",","."); if(tolsObstacles(0,1) >= 0){tyy1=string("+")+tyy1;}
        string tyy2 = boost::str(boost::format("%.2f") % tolsObstacles(1,1)); boost::replace_all(tyy2,",","."); if(tolsObstacles(1,1) >= 0){tyy2=string("+")+tyy2;}
        string tyy3 = boost::str(boost::format("%.2f") % tolsObstacles(2,1)); boost::replace_all(tyy3,",","."); if(tolsObstacles(2,1) >= 0){tyy3=string("+")+tyy3;}
        //zz
        string tzz1 = boost::str(boost::format("%.2f") % tolsObstacles(0,2)); boost::replace_all(tzz1,",","."); if(tolsObstacles(0,2) >= 0){tzz1=string("+")+tzz1;}
        string tzz2 = boost::str(boost::format("%.2f") % tolsObstacles(1,2)); boost::replace_all(tzz2,",","."); if(tolsObstacles(1,2) >= 0){tzz2=string("+")+tzz2;}
        string tzz3 = boost::str(boost::format("%.2f") % tolsObstacles(2,2)); boost::replace_all(tzz3,",","."); if(tolsObstacles(2,2) >= 0){tzz3=string("+")+tzz3;}
        //xy
        string txy1 = boost::str(boost::format("%.2f") % tolsObstacles(0,3)); boost::replace_all(txy1,",","."); if(tolsObstacles(0,3) >= 0){txy1=string("+")+txy1;}
        string txy2 = boost::str(boost::format("%.2f") % tolsObstacles(1,3)); boost::replace_all(txy2,",","."); if(tolsObstacles(1,3) >= 0){txy2=string("+")+txy2;}
        string txy3 = boost::str(boost::format("%.2f") % tolsObstacles(2,3)); boost::replace_all(txy3,",","."); if(tolsObstacles(2,3) >= 0){txy3=string("+")+txy3;}
        //xz
        string txz1 = boost::str(boost::format("%.2f") % tolsObstacles(0,4)); boost::replace_all(txz1,",","."); if(tolsObstacles(0,4) >= 0){txz1=string("+")+txz1;}
        string txz2 = boost::str(boost::format("%.2f") % tolsObstacles(1,4)); boost::replace_all(txz2,",","."); if(tolsObstacles(1,4) >= 0){txz2=string("+")+txz2;}
        string txz3 = boost::str(boost::format("%.2f") % tolsObstacles(2,4)); boost::replace_all(txz3,",","."); if(tolsObstacles(2,4) >= 0){txz3=string("+")+txz3;}
        //yz
        string tyz1 = boost::str(boost::format("%.2f") % tolsObstacles(0,5)); boost::replace_all(tyz1,",","."); if(tolsObstacles(0,5) >= 0){tyz1=string("+")+tyz1;}
        string tyz2 = boost::str(boost::format("%.2f") % tolsObstacles(1,5)); boost::replace_all(tyz2,",","."); if(tolsObstacles(1,5) >= 0){tyz2=string("+")+tyz2;}
        string tyz3 = boost::str(boost::format("%.2f") % tolsObstacles(2,5)); boost::replace_all(tyz3,",","."); if(tolsObstacles(2,5) >= 0){tyz3=string("+")+tyz3;}


        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("# \n");
        //PostureMod << string("subject to obst_Arm{j in 1..21, i in 1..n_Obstacles}:  \n");
        PostureMod << string("subject to obst_Arm{j in 1..")+n_str+string(", i in 1..n_Obstacles}:  \n");
        PostureMod << string("(((Rot[1,1,i]*Points_Arm[j,1]+Rot[2,1,i]*Points_Arm[j,2]+Rot[3,1,i]*Points_Arm[j,3]\n");
        PostureMod << string("-Obstacles[i,1]*Rot[1,1,i]-Obstacles[i,2]*Rot[2,1,i]-Obstacles[i,3]*Rot[3,1,i])\n");
        PostureMod << string("/(Obstacles[i,4]+Points_Arm[j,4]")+txx1+string("))^2\n");
        PostureMod << string("+\n");
        PostureMod << string("((Rot[1,2,i]*Points_Arm[j,1]+Rot[2,2,i]*Points_Arm[j,2]+Rot[3,2,i]*Points_Arm[j,3]\n");
        PostureMod << string("-Obstacles[i,1]*Rot[1,2,i]-Obstacles[i,2]*Rot[2,2,i]-Obstacles[i,3]*Rot[3,2,i])\n");
        PostureMod << string("/(Obstacles[i,5]+Points_Arm[j,4]")+tyy1+string("))^2\n");
        PostureMod << string("+\n");
        PostureMod << string("((Rot[1,3,i]*Points_Arm[j,1]+Rot[2,3,i]*Points_Arm[j,2]+Rot[3,3,i]*Points_Arm[j,3]\n");
        PostureMod << string("-Obstacles[i,1]*Rot[1,3,i]-Obstacles[i,2]*Rot[2,3,i]-Obstacles[i,3]*Rot[3,3,i])\n");
        PostureMod << string("/(Obstacles[i,6]+Points_Arm[j,4]")+tzz1+string("))^2)");
        PostureMod << string(">= 1;\n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("#  \n");

        if(mov_type==1 && pre_post==2){
            // place movements (retreat stage)
            PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
            PostureMod << string("# \n");
            PostureMod << string("subject to obj_Arm_right{j in 1..")+n_str+string(", i in 1..n_ObjTar}:  \n");
            PostureMod << string("(((Rot_obj[1,1]*Points_Arm[j,1]+Rot_obj[2,1]*Points_Arm[j,2]+Rot_obj[3,1]*Points_Arm[j,3]\n");
            PostureMod << string("-ObjTar[i,1]*Rot_obj[1,1]-ObjTar[i,2]*Rot_obj[2,1]-ObjTar[i,3]*Rot_obj[3,1])\n");
            PostureMod << string("/(ObjTar[i,4]+Points_Arm[j,4]")+txx1+string("))^2\n");
            PostureMod << string("+\n");
            PostureMod << string("((Rot_obj[1,2]*Points_Arm[j,1]+Rot_obj[2,2]*Points_Arm[j,2]+Rot_obj[3,2]*Points_Arm[j,3]\n");
            PostureMod << string("-ObjTar[i,1]*Rot_obj[1,2]-ObjTar[i,2]*Rot_obj[2,2]-ObjTar[i,3]*Rot_obj[3,2])\n");
            PostureMod << string("/(ObjTar[i,5]+Points_Arm[j,4]")+tyy1+string("))^2\n");
            PostureMod << string("+\n");
            PostureMod << string("((Rot_obj[1,3]*Points_Arm[j,1]+Rot_obj[2,3]*Points_Arm[j,2]+Rot_obj[3,3]*Points_Arm[j,3]\n");
            PostureMod << string("-ObjTar[i,1]*Rot_obj[1,3]-ObjTar[i,2]*Rot_obj[2,3]-ObjTar[i,3]*Rot_obj[3,3])\n");
            PostureMod << string("/(ObjTar[i,6]+Points_Arm[j,4]")+tzz1+string("))^2)");
            PostureMod << string(">= 1;\n");
            PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
            PostureMod << string("#  \n");
        }


        /*
        PostureMod << string("((Points_Arm[j,1]-Obstacles[i,1])^2)*(  \n");
        PostureMod << string("(Rot[1,1,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4]")+txx1+string(")^2) + \n");
        PostureMod << string("(Rot[2,1,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4]")+txx2+string(")^2) + \n");
        PostureMod << string("(Rot[3,1,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4]")+txx3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm[j,2]-Obstacles[i,2])^2)*(  \n");
        PostureMod << string("(Rot[1,2,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4]")+tyy1+string(")^2) + \n");
        PostureMod << string("(Rot[2,2,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4]")+tyy2+string(")^2) + \n");
        PostureMod << string("(Rot[3,2,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4]")+tyy3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm[j,3]-Obstacles[i,3])^2)*( \n");
        PostureMod << string("(Rot[1,3,i])^2 / ((Obstacles[i,4]+Points_Arm[j,4]")+tzz1+string(")^2) + \n");
        PostureMod << string("(Rot[2,3,i])^2 / ((Obstacles[i,5]+Points_Arm[j,4]")+tzz2+string(")^2) +  \n");
        PostureMod << string("(Rot[3,3,i])^2 / ((Obstacles[i,6]+Points_Arm[j,4]")+tzz3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,1]-Obstacles[i,1])*(Points_Arm[j,2]-Obstacles[i,2])* ( \n");
        PostureMod << string("(Rot[1,1,i]*Rot[1,2,i])/((Obstacles[i,4]+Points_Arm[j,4]")+txy1+string(")^2) + \n");
        PostureMod << string("(Rot[2,1,i]*Rot[2,2,i])/((Obstacles[i,5]+Points_Arm[j,4]")+txy2+string(")^2) + \n");
        PostureMod << string("(Rot[3,1,i]*Rot[3,2,i])/((Obstacles[i,6]+Points_Arm[j,4]")+txy3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,1]-Obstacles[i,1])*(Points_Arm[j,3]-Obstacles[i,3])* ( \n");
        PostureMod << string("(Rot[1,1,i]*Rot[1,3,i])/((Obstacles[i,4]+Points_Arm[j,4]")+txz1+string(")^2) + \n");
        PostureMod << string("(Rot[2,1,i]*Rot[2,3,i])/((Obstacles[i,5]+Points_Arm[j,4]")+txz2+string(")^2) + \n");
        PostureMod << string("(Rot[3,1,i]*Rot[3,3,i])/((Obstacles[i,6]+Points_Arm[j,4]")+txz3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm[j,2]-Obstacles[i,2])*(Points_Arm[j,3]-Obstacles[i,3])* ( \n");
        PostureMod << string("(Rot[1,2,i]*Rot[1,3,i])/((Obstacles[i,4]+Points_Arm[j,4]")+tyz1+string(")^2) + \n");
        PostureMod << string("(Rot[2,2,i]*Rot[2,3,i])/((Obstacles[i,5]+Points_Arm[j,4]")+tyz2+string(")^2) + \n");
        PostureMod << string("(Rot[3,2,i]*Rot[3,3,i])/((Obstacles[i,6]+Points_Arm[j,4]")+tyz3+string(")^2)) \n");
        PostureMod << string("-1 >=0; \n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("#  \n");
        */

    }

    // constraints with the body
    if(coll && coll_body){
        this->writeBodyConstraints(PostureMod,true);
    }

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n\n\n");

    // close the files
    PostureMod.close();
    PostureDat.close();

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



bool HUMPlanner::writeFilesBouncePosture(int steps,hump_params& params,int mov_type, int pre_post,std::vector<double> minAuxLimits, std::vector<double> maxAuxLimits,std::vector<double> initAuxPosture, std::vector<double> finalAuxPosture,
                                         std::vector<double> initialGuess, std::vector<double> lambda,std::vector<objectPtr> objs,boundaryConditions b)
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
    //int griptype = params.mov_specs.griptype;
    double dHO = params.mov_specs.dHO;    
    std::vector<double> finalHand = params.mov_specs.finalHand;    
    std::vector<double> tar = params.mov_specs.target;
    Matrix4d T_tar_to_obj = params.mov_specs.T_tar_to_obj;
    objectPtr obj_tar = params.mov_specs.obj;    
    string mov_infoLine = params.mov_specs.mov_infoline;    
    bool approach = params.mov_specs.approach;
    bool retreat = params.mov_specs.retreat;
    bool use_plane = params.mov_specs.use_move_plane;
    std::vector<double> plane_params = params.mov_specs.plane_params;
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
    case 1: // place
        obj_tar = params.mov_specs.obj;
        if(approach){pre_place_approach = params.mov_specs.pre_place_approach;}
        if(retreat){post_place_retreat = params.mov_specs.post_place_retreat;}
        break;
    }

    // tolerances
    double timestep; MatrixXd traj_no_bound;
    this->directTrajectoryNoBound(steps,initAuxPosture,finalAuxPosture,traj_no_bound);
    int mod;
    if(pre_post==0){mod=0;}else{mod=1;}
    timestep = this->getTimeStep(params,traj_no_bound,mod);
    double totalTime = timestep*steps;
    //std::vector<double> lambda = params.lambda_bounce;
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
        k=1;
        matWorldToArm=this->matWorldToLeftArm;
        matHand = this->matLeftHand;
        dh=this->DH_leftArm;
        break;
    }
    //------------------------- Write the dat file --------------------------------------------------
     string filenamedat("BouncePosture.dat");
     ofstream PostureDat;
     // open the file
     PostureDat.open(path+filenamedat);

     PostureDat << string("# BOUNCE POSTURE DATA FILE \n");
     PostureDat << string("# Units of measure: [rad], [mm] \n\n");

     PostureDat << string("data; \n");

     // number of steps
     PostureDat << string("param Nsteps :=")+to_string(steps)+string(";\n");

     // total time
     string tottime_str =  boost::str(boost::format("%.2f") % (totalTime));
     boost::replace_all(tottime_str,",",".");
     PostureDat << string("param TotalTime :=")+tottime_str+string(";\n");
     // Body dimension
     this->writeBodyDim(this->torso_size.at(0),this->torso_size.at(1),PostureDat);

     // D-H Parameters of the Arm
     this->writeArmDHParams(dh,PostureDat,k);
     // distance between the hand and the object
     this->write_dHO(PostureDat,dHO);
     // joint limits
     this->writeArmLimits(PostureDat,minAuxLimits,maxAuxLimits,false);
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
     PostureDat << string("var theta_b = \n");
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
     // info approach/retreat
     switch(mov_type){
     case 0: //pick
         // info of the target to reach
         this->writeInfoTarget(PostureDat,tar);
         if (pre_post!=0){this->writeInfoApproachRetreat(PostureDat,tar,pre_grasp_approach);}
         break;
     case 1: // place
         // info of the target to reach
         this->writeInfoTarget(PostureDat,tar);
         if (pre_post!=0){this->writeInfoApproachRetreat(PostureDat,tar,pre_place_approach);}
         break;
     }

     //info objects
     this->writeInfoObstacles(PostureDat,objs);
     // object that has the target
     std::vector<double> dim;
     switch(mov_type){
     case 0: // pick
         obj_tar->getSize(dim);
         if(pre_post==2){ // retreat stage
             this->writeInfoObjectTarget(PostureDat,tar,T_tar_to_obj,dim, obj_tar->getName());
         }else{
            this->writeInfoObjectTarget(PostureDat,obj_tar);
         }
         break;
     case 1: //place
         obj_tar->getSize(dim);
         if(pre_post==2){ // retreat stage
             this->writeInfoObjectTargetPlaceRetreat(PostureDat,tar,T_tar_to_obj,dim, obj_tar->getName());
         }else{
            this->writeInfoObjectTarget(PostureDat,tar,T_tar_to_obj,dim, obj_tar->getName());
         }
         break;
     }
     //close the file
     //PostureDat.close();

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
     //PostureMod << string("param time {i in Iterations} = ((i-1)*TotalTime)/Nsteps;\n");
     PostureMod << string("param time {i in Iterations} = (i-1)/Nsteps;\n");

     PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
     PostureMod << string("# DECISION VARIABLES \n");
     PostureMod << string("# Bounce Posture \n");
     PostureMod << string("var theta_b {i in 1..")+to_string(initialGuess.size())+string("} >= llim[i], <= ulim[i]; \n");

     PostureMod << string("# Direct Movement \n");
     PostureMod << string("param the_direct {(i,j) in Iterations_nJoints} := thet_init[j]+ \n");
     PostureMod << string("( thet_final[j] - thet_init[j] ) * (10*(time[i])^3 -15*(time[i])^4 +6*(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("vel_0[j] * TotalTime * (time[i] - 6 *(time[i])^3 +8*(time[i])^4 -3*(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("vel_f[j] * TotalTime * (- 4 *(time[i])^3 +7*(time[i])^4 -3*(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("(acc_0[j]/2) * TotalTime^2 * (time[i]^2 - 3 *(time[i])^3 +3*(time[i])^4 -(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("(acc_f[j]/2) * TotalTime^2 * ((time[i])^3 -2*(time[i])^4 +(time[i])^5); \n");

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

     bool place = false; // true for place movements
     bool move = false; // true for move movements

     // object to transport (place movements)
     int n_s = 0; // number of spheres
     if(mov_type==1){ // place
         place = true;
         std::vector<double> obj_tar_size; obj_tar->getSize(obj_tar_size);
         n_s = this->model_spheres(PostureDat,PostureMod,obj_tar_size,false);
     }else if(mov_type==2){
         // move
         move=true;
     }

     switch(hand_code){
     case 0: // human hand
         this->writeHumanHandDirKin(PostureMod,tolsHand,false,place);
         break;
     case 1: // barrett hand
         this->writeBarrettHandDirKin(PostureMod,tolsHand,false,place);
         break;
     }
     // Points of the arm
     string n_str;
     if (place){
         n_str = to_string(15+n_s);
     }else{
         n_str = to_string(15);
     }
     PostureMod << string("var Points_Arm {j in 1..")+n_str+string(", i in 1..4,k in Iterations} = \n");
     PostureMod << string("if ( j=1 ) then 	(Shoulder[i,k]+Elbow[i,k])/2  \n");
     PostureMod << string("else	if ( j=2 ) then 	Elbow[i,k] \n");
     PostureMod << string("else    if ( j=3 ) then 	(Wrist[i,k]+Elbow[i,k])/2  \n");
     PostureMod << string("else	if ( j=4 ) then 	Wrist[i,k] \n");
     PostureMod << string("else	if ( j=5 ) then 	Wrist[i,k]+0.45*(Hand[i,k]-Wrist[i,k]) \n");
     PostureMod << string("else	if ( j=6 ) then 	Wrist[i,k]+0.75*(Hand[i,k]-Wrist[i,k]) \n");
     /*
     PostureMod << string("else	if ( j=7 ) then 	Finger1_1[i,k] \n");
     PostureMod << string("else	if ( j=8 ) then 	Finger2_1[i,k] \n");
     PostureMod << string("else	if ( j=9 ) then 	Finger3_1[i,k]\n");
     PostureMod << string("else	if ( j=10 ) then 	(Finger1_1[i,k]+Finger1_2[i,k])/2 \n");
     PostureMod << string("else	if ( j=11 ) then 	(Finger2_1[i,k]+Finger2_2[i,k])/2 \n");
     PostureMod << string("else	if ( j=12 ) then 	(Finger3_1[i,k]+Finger3_2[i,k])/2 \n");
     PostureMod << string("else	if ( j=13 ) then 	 Finger1_2[i,k] \n");
     PostureMod << string("else	if ( j=14 ) then 	 Finger2_2[i,k] \n");
     PostureMod << string("else	if ( j=15 ) then 	 Finger3_2[i,k] \n");
     PostureMod << string("else	if ( j=16 ) then 	(Finger1_2[i,k]+Finger1_tip[i,k])/2	 \n");
     PostureMod << string("else	if ( j=17 ) then 	(Finger2_2[i,k]+Finger2_tip[i,k])/2 \n");
     PostureMod << string("else	if ( j=18 ) then 	(Finger3_2[i,k]+Finger3_tip[i,k])/2 \n");
     PostureMod << string("else	if ( j=19 ) then 	Finger1_tip[i,k]\n");
     PostureMod << string("else	if ( j=20 ) then 	Finger2_tip[i,k] \n");
     PostureMod << string("else	if ( j=21 ) then 	Finger3_tip[i,k] \n");
     */
     PostureMod << string("else	if ( j=7 ) then 	Finger1_1[i,k] \n");
     PostureMod << string("else	if ( j=8 ) then 	Finger2_1[i,k] \n");
     PostureMod << string("else	if ( j=9 ) then 	Finger3_1[i,k]\n");
     PostureMod << string("else	if ( j=10 ) then 	 Finger1_2[i,k] \n");
     PostureMod << string("else	if ( j=11 ) then 	 Finger2_2[i,k] \n");
     PostureMod << string("else	if ( j=12 ) then 	 Finger3_2[i,k] \n");
     PostureMod << string("else	if ( j=13 ) then 	Finger1_tip[i,k]\n");
     PostureMod << string("else	if ( j=14 ) then 	Finger2_tip[i,k] \n");
     PostureMod << string("else	if ( j=15 ) then 	Finger3_tip[i,k] \n");
     if (place){
         int j_init = 15;
         for(int i=1;i<=n_s;++i){
             std::string i_str = to_string(i);
             int j = j_init+i; std::string j_str = to_string(j);
             PostureMod << string("else    if ( j=")+j_str+string(" ) then 	Obj2Transp_")+i_str+string("[i,k] \n");
         }
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

     // plane paramters
     string a_str; string b_str; string c_str; string d_str;
     // hand constraints for approaching direction setting
     string n_steps_init_str;
     if(N_STEP_MIN>2){
         n_steps_init_str = boost::str(boost::format("%d") % (N_STEP_MIN-2));
     }else{
         n_steps_init_str = boost::str(boost::format("%d") % 1);
     }
     switch (mov_type) {
     case 0: // pick
         // hand constraints for approaching direction settings
         if(approach && pre_post==1){
             PostureMod << string("# Hand approach orientation\n");
             PostureMod << string("subject to constr_hand_or {k in (Nsteps-")+n_steps_init_str+string(")..(Nsteps+1)}: ( sum{i in 1..3} (x_H[i,k] - x_t[i])^2)<= 0.01; #  x_H = x_t \n\n");
         }
         break;
     case 1: // place
         // hand constraints for approaching and retreating direction settings
         /*
         if(approach && pre_post==1){
             PostureMod << string("# Hand approach orientation\n");
             PostureMod << string("subject to constr_hand_or {k in (Nsteps-")+n_steps_init_str+string(")..(Nsteps+1)}: ( sum{i in 1..3} (x_H[i,k] - z_t[i])^2 + sum{i in 1..3} (z_H[i,k] + y_t[i])^2 )<= 0.010; #  x_H = z_t  and z_H = -y_t \n\n");
         }
         */
         break;
     }
     /*
     switch (griptype) {
     case 111: case 211: // side thumb left

         switch (mov_type) {
         case 0: // pick
             // hand constraints for approaching direction settings
             if(approach && pre_post==1){
                 PostureMod << string("# Hand approach orientation\n");
                 PostureMod << string("subject to constr_hand_or {k in (Nsteps-")+n_steps_init_str+string(")..(Nsteps+1)}: ( sum{i in 1..3} (x_H[i,k] - z_t[i])^2)<= 0.01; #  x_H = z_t \n\n");
             }
             break;
         case 1: // place
             // hand constraints for approaching and retreating direction settings

             //if(approach && pre_post==1){
             //    PostureMod << string("# Hand approach orientation\n");
             //    PostureMod << string("subject to constr_hand_or {k in (Nsteps-")+n_steps_init_str+string(")..(Nsteps+1)}: ( sum{i in 1..3} (x_H[i,k] - z_t[i])^2 + sum{i in 1..3} (z_H[i,k] + y_t[i])^2 )<= 0.010; #  x_H = z_t  and z_H = -y_t \n\n");
             //}

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
     default:// move movements (there is no griptype)
         break;
     }
    */

     // move plane constraints
     if(move && use_plane && !plane_params.empty()){
        a_str =  boost::str(boost::format("%.2f") % (plane_params.at(0))); boost::replace_all(a_str,",",".");
        b_str =  boost::str(boost::format("%.2f") % (plane_params.at(1))); boost::replace_all(b_str,",",".");
        if(plane_params.at(1)>=0){b_str = string("+")+b_str;}
        c_str =  boost::str(boost::format("%.2f") % (plane_params.at(2))); boost::replace_all(c_str,",",".");
        if(plane_params.at(2)>=0){c_str = string("+")+c_str;}
        d_str =  boost::str(boost::format("%.2f") % (plane_params.at(3))); boost::replace_all(d_str,",",".");
        if(plane_params.at(3)>=0){d_str = string("+")+d_str;}
        PostureMod << string("# Hand plane constraints\n");
        PostureMod << string("subject to constr_hand_plane {k in ")+n_steps_init_str+string("..(Nsteps-")+n_steps_init_str+string(")}: ((")+a_str+string("*Hand[1,k]")+b_str+string("*Hand[2,k]")+
                      c_str+string("*Hand[3,k]")+d_str+string(")^2) <= 10; # the hand must be a point of the plane \n\n");
     }

   if(target_avoidance && !place && !move){

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


        // in pick shorts movements (movements with N_STEP_MIN steps) collisions with the target are not considered
        int diff_steps = (int) (steps*BLANK_PERCENTAGE_TAR);
        string n_steps_end_str = boost::str(boost::format("%d") % (diff_steps));
        PostureMod << string("subject to target_Arm{j in 4..15, l in 1..Nsteps-")+n_steps_end_str+("}:   \n");
        /*
        if(pre_post!=0){
            PostureMod << string("subject to target_Arm{j in 4..15, l in 1..Nsteps+1}:   \n");
        }else{
            int diff_steps = (int) (steps*BLANK_PERCENTAGE);
            string n_steps_end_str = boost::str(boost::format("%d") % (diff_steps));
            PostureMod << string("subject to target_Arm{j in 4..15, l in 1..Nsteps-")+n_steps_end_str+("}:   \n");
        }
        */

        PostureMod << string("(((x_t[1]*Points_Arm[j,1,l]+x_t[2]*Points_Arm[j,2,l]+x_t[3]*Points_Arm[j,3,l]\n");
        PostureMod << string("-ObjTar[1,1]*x_t[1]-ObjTar[1,2]*x_t[2]-ObjTar[1,3]*x_t[3])\n");
        PostureMod << string("/(ObjTar[1,4]+Points_Arm[j,4,l]+tol_target_xx1[l]))^2\n");
        PostureMod << string("+\n");
        PostureMod << string("((y_t[1]*Points_Arm[j,1,l]+y_t[2]*Points_Arm[j,2,l]+y_t[3]*Points_Arm[j,3,l]\n");
        PostureMod << string("-ObjTar[1,1]*y_t[1]-ObjTar[1,2]*y_t[2]-ObjTar[1,3]*y_t[3])\n");
        PostureMod << string("/(ObjTar[1,5]+Points_Arm[j,4,l]+tol_target_yy1[l]))^2\n");
        PostureMod << string("+\n");
        PostureMod << string("((z_t[1]*Points_Arm[j,1,l]+z_t[2]*Points_Arm[j,2,l]+z_t[3]*Points_Arm[j,3,l]\n");
        PostureMod << string("-ObjTar[1,1]*z_t[1]-ObjTar[1,2]*z_t[2]-ObjTar[1,3]*z_t[3])\n");
        PostureMod << string("/(ObjTar[1,6]+Points_Arm[j,4,l]+tol_target_zz1[l]))^2)\n");
        PostureMod << string(">= 1;\n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("# \n");

        /*
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
        */
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
        if(place){
            // the object to place has to be considered
             PostureMod << string("subject to obst_Arm{j in 1..")+n_str+string(", i in 1..n_Obstacles, l in 1..Nsteps+1}:\n"); // approach stage is necessary
        }else if(move){
            // for the first number of diff_steps, no obstacle is considered because the movement is very short and the planner may get stuck
            int diff_steps = std::max(1,(int)(steps*BLANK_PERCENTAGE_OBS));
            string n_steps_init_str = boost::str(boost::format("%d") % (diff_steps));
            PostureMod << string("subject to obst_Arm{j in 1..15, i in 1..(n_Obstacles), l in ")+n_steps_init_str+("..Nsteps+1}:\n");
        }else{
             PostureMod << string("subject to obst_Arm{j in 1..15, i in 1..(n_Obstacles), l in 1..Nsteps+1}:\n"); // pick movements
        }

        PostureMod << string("(((Rot[1,1,i]*Points_Arm[j,1,l]+Rot[2,1,i]*Points_Arm[j,2,l]+Rot[3,1,i]*Points_Arm[j,3,l]\n");
        PostureMod << string("-Obstacles[i,1]*Rot[1,1,i]-Obstacles[i,2]*Rot[2,1,i]-Obstacles[i,3]*Rot[3,1,i])\n");
        PostureMod << string("/(Obstacles[i,4]+Points_Arm[j,4,l]+tol_obs_xx1[l]))^2\n");
        PostureMod << string("+\n");
        PostureMod << string("((Rot[1,2,i]*Points_Arm[j,1,l]+Rot[2,2,i]*Points_Arm[j,2,l]+Rot[3,2,i]*Points_Arm[j,3,l]\n");
        PostureMod << string("-Obstacles[i,1]*Rot[1,2,i]-Obstacles[i,2]*Rot[2,2,i]-Obstacles[i,3]*Rot[3,2,i])\n");
        PostureMod << string("/(Obstacles[i,5]+Points_Arm[j,4,l]+tol_obs_yy1[l]))^2\n");
        PostureMod << string("+\n");
        PostureMod << string("((Rot[1,3,i]*Points_Arm[j,1,l]+Rot[2,3,i]*Points_Arm[j,2,l]+Rot[3,3,i]*Points_Arm[j,3,l]\n");
        PostureMod << string("-Obstacles[i,1]*Rot[1,3,i]-Obstacles[i,2]*Rot[2,3,i]-Obstacles[i,3]*Rot[3,3,i])\n");
        PostureMod << string("/(Obstacles[i,6]+Points_Arm[j,4,l]+tol_obs_zz1[l]))^2)\n");
        PostureMod << string(">= 1;\n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
        PostureMod << string("# \n");



        /*
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
        */

     }
     // constraints with the body
     this->writeBodyConstraints(PostureMod,false);

     PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n\n\n");

     // close the files
     PostureMod.close(); PostureDat.close();

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

int HUMPlanner::model_spheres(ofstream &stream_dat, ofstream &stream_model, std::vector<double>& obj_tar_size,bool final)
{
    int n_s=0;
    string sphere_radius_str; string sphere_diam_str;
    // Rotation matrix of the object to place
    this->writeRotMatObjTar(stream_model);
    std::map< std::string, double > axis_map;
    axis_map["x"] = obj_tar_size.at(0);
    axis_map["y"] = obj_tar_size.at(1);
    axis_map["z"] = obj_tar_size.at(2);
    std::vector<std::pair<std::string,double> > obj_sizes(axis_map.begin(), axis_map.end());
    std::sort(obj_sizes.begin(),obj_sizes.end(),&compare_sizes);
    std::string axis_min = (obj_sizes.at(0)).first; std::string axis_middle = (obj_sizes.at(1)).first; std::string axis_max = (obj_sizes.at(2)).first;
    double obj_min = (obj_sizes.at(0)).second; double obj_middle = (obj_sizes.at(1)).second; double obj_max = (obj_sizes.at(2)).second;
    double sphere_diam = obj_min; sphere_diam_str = boost::str(boost::format("%.2f") % (sphere_diam)); boost::replace_all(sphere_diam_str,",",".");
    sphere_radius_str = boost::str(boost::format("%.2f") % (sphere_diam/2)); boost::replace_all(sphere_radius_str,",",".");
    int n_sphere_1 = round(obj_middle/sphere_diam+0.5); int ns1 = round(n_sphere_1/2-0.5);
    int n_sphere_2 = round(obj_max/sphere_diam+0.5); int ns2 = round(n_sphere_2/2-0.5);

    int x=0; int y=0; int z=0; int k=0;
    if(axis_min.compare("x")){ // the object minimum size is x
        y=1; z=1;
    }else if(axis_min.compare("y")){ // the object minimum size is y
        x=1;z=1;
    }else{ // the object minimum size is z
        x=1; y=1;
    }

    if(final){
        stream_model << string("var Rot_s {i1 in 1..3, i2 in 1..3} =  sum {j in 1..3} Rot_H[i1,j]*Rot_obj[j,i2];\n");
        // Modellization of the object in spheres
        stream_model << string("# Modelization of the object to place \n");
        stream_dat << string("# Data of the modelization of the object to place \n");

        stream_model << string("var Obj2Transp_center {j in 1..3} = Hand[j] + dFH * z_H[j] + tar_to_obj[j]; \n"); // center of the object

        for(int j=0;j<ns2;++j){// max size axis: positive direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s++; std::string n_s_str = to_string(n_s);
            stream_model << string("# Sphere_")+n_s_str+string("\n");
            stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

            stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
            stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
        }
        for(int j=1;j<ns2;++j){// max size axis: negative direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s++; std::string n_s_str = to_string(n_s);
            stream_model << string("# Sphere_")+n_s_str+string("\n");
            stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

            stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
            stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
        }
    }else{
        stream_model << string("var Rot_s {i1 in 1..3, i2 in 1..3,i in Iterations} =  sum {j in 1..3} Rot_H[i1,j,i]*Rot_obj[j,i2];\n");
        // Modellization of the object in spheres
        stream_model << string("# Modelization of the object to place \n");
        stream_dat << string("# Data of the modelization of the object to place \n");

        stream_model << string("var Obj2Transp_center {j in 1..3, i in Iterations} = Hand[j,i] + dFH * z_H[j,i] + tar_to_obj[j]; \n"); // center of the object

        for(int j=0;j<ns2;++j){// max size axis: positive direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s++; std::string n_s_str = to_string(n_s);
            stream_model << string("# Sphere_")+n_s_str+string("\n");
            stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

            stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
            stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
        }
        for(int j=1;j<ns2;++j){// max size axis: negative direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s++; std::string n_s_str = to_string(n_s);
            stream_model << string("# Sphere_")+n_s_str+string("\n");
            stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

            stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
            stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
        }
    }
    return n_s;
}

int HUMPlanner::dual_obj_model_spheres(ofstream &stream_dat,ofstream &stream_model,std::vector<double>& obj_tar_size,bool final)
{
    int n_s=0;
    string sphere_radius_str; string sphere_diam_str; double sphere_diam;
    // Rotation matrix of the object to place
    this->writeRotMatObjTarDual(stream_model);
    std::map< std::string, double > axis_map;
    std::string axis_min; std::string axis_middle; std::string axis_max;
    double obj_min; double obj_middle; double obj_max;
    int n_sphere_1; int n_sphere_2; int ns1; int ns2;
    int x=0; int y=0; int z=0; int k=0;

    axis_map["x"] = obj_tar_size.at(0);
    axis_map["y"] = obj_tar_size.at(1);
    axis_map["z"] = obj_tar_size.at(2);
    std::vector<std::pair<std::string,double> > obj_sizes(axis_map.begin(), axis_map.end());
    std::sort(obj_sizes.begin(),obj_sizes.end(),&compare_sizes);
    axis_min = (obj_sizes.at(0)).first; axis_middle = (obj_sizes.at(1)).first; axis_max = (obj_sizes.at(2)).first;
    obj_min = (obj_sizes.at(0)).second; obj_middle = (obj_sizes.at(1)).second; obj_max = (obj_sizes.at(2)).second;
    sphere_diam = obj_min; sphere_diam_str = boost::str(boost::format("%.2f") % (sphere_diam)); boost::replace_all(sphere_diam_str,",",".");
    sphere_radius_str = boost::str(boost::format("%.2f") % (sphere_diam/2)); boost::replace_all(sphere_radius_str,",",".");
    n_sphere_1 = round(obj_middle/sphere_diam+0.5); ns1 = round(n_sphere_1/2-0.5);
    n_sphere_2 = round(obj_max/sphere_diam+0.5); ns2 = round(n_sphere_2/2-0.5);
    if(axis_min.compare("x")){ // the object minimum size is x
        y=1; z=1;
    }else if(axis_min.compare("y")){ // the object minimum size is y
        x=1;z=1;
    }else{ // the object minimum size is z
        x=1; y=1;
    }
    if(final){
        stream_model << string("var Rot_s {i1 in 1..3, i2 in 1..3} =  sum {j in 1..3} Rot_H_right[i1,j]*Rot_tar_obj_right[j,i2];\n");
        // Modellization of the object in spheres
        stream_model << string("# Modelization of the object to place by both arms\n");
        stream_dat << string("# Data of the modelization of the object to place \n");

        stream_model << string("var Obj2Transp_center {j in 1..3} = Hand_right[j] + dFH_right * z_H_right[j] + tar_to_obj_right[j]; \n"); // center of the object

        for(int j=0;j<ns2;++j){// max size axis: positive direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s++; std::string n_s_str = to_string(n_s);
            stream_model << string("# Sphere_")+n_s_str+string("\n");
            stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

            stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} =")+string(" #xyz+radius \n");
            stream_model << string("if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
            stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
        }
        for(int j=1;j<ns2;++j){// max size axis: negative direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s++; std::string n_s_str = to_string(n_s);
            stream_model << string("# Sphere_")+n_s_str+string("\n");
            stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

            stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
            stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s[j,k]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4} = if j<4 then Obj2Transp_center[j] + ext_s_")+n_s_str+string("[j]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
        }
    }else{ //bounce posture selection
        stream_model << string("var Rot_s {i1 in 1..3, i2 in 1..3,i in Iterations} =  sum {j in 1..3} Rot_H_right[i1,j,i]*Rot_tar_obj_right[j,i2];\n");
        // Modellization of the object in spheres
        stream_model << string("# Modelization of the object to place by both arms\n");
        stream_dat << string("# Data of the modelization of the object to place \n");

        stream_model << string("var Obj2Transp_center {j in 1..3, i in Iterations} = Hand_right[j,i] + dFH_right * z_H_right[j,i] + tar_to_obj_right[j]; \n"); // center of the object

        for(int j=0;j<ns2;++j){// max size axis: positive direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s++; std::string n_s_str = to_string(n_s);
            stream_model << string("# Sphere_")+n_s_str+string("\n");
            stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

            stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
            stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
        }
        for(int j=1;j<ns2;++j){// max size axis: negative direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s++; std::string n_s_str = to_string(n_s);
            stream_model << string("# Sphere_")+n_s_str+string("\n");
            stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

            stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
            stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s++; std::string n_s_str = to_string(n_s);
                stream_model << string("# Sphere_")+n_s_str+string("\n");
                stream_model << string("param ext_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s[j,k,i]*ext_")+n_s_str+string("[k];\n");

                stream_model << string("var Obj2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = if j<4 then Obj2Transp_center[j,i] + ext_s_")+n_s_str+string("[j,i]*(")+sphere_diam_str+string(") \n");
                stream_model << string("else 	if (j=4) then ")+sphere_radius_str+string("; \n");
            }
        }
    }
    return n_s;
}

void HUMPlanner::dual_obj_model_spheres(ofstream &stream_dat,ofstream &stream_model,std::vector<double>& obj_tar_right_size,std::vector<double>& obj_tar_left_size,bool final,int& n_s_right, int& n_s_left)
{
    string sphere_radius_str; string sphere_diam_str; double sphere_diam;
    // Rotation matrix of the objects to place
    this->writeRotMatObjTarDual(stream_model);
    std::map< std::string, double > axis_map;
    std::string axis_min; std::string axis_middle; std::string axis_max;
    double obj_min; double obj_middle; double obj_max;
    int n_sphere_1; int n_sphere_2; int ns1; int ns2;
    int x=0; int y=0; int z=0; int k=0;

    // ------------------------------------ Object right target --------------------------------------------------------------- //
    n_s_right=0;
    axis_map["x"] = obj_tar_right_size.at(0);
    axis_map["y"] = obj_tar_right_size.at(1);
    axis_map["z"] = obj_tar_right_size.at(2);
    std::vector<std::pair<std::string,double> > obj_right_sizes(axis_map.begin(), axis_map.end());
    std::sort(obj_right_sizes.begin(),obj_right_sizes.end(),&compare_sizes);
    axis_min = (obj_right_sizes.at(0)).first; axis_middle = (obj_right_sizes.at(1)).first; axis_max = (obj_right_sizes.at(2)).first;
    obj_min = (obj_right_sizes.at(0)).second; obj_middle = (obj_right_sizes.at(1)).second; obj_max = (obj_right_sizes.at(2)).second;
    sphere_diam = obj_min; sphere_diam_str = boost::str(boost::format("%.2f") % (sphere_diam)); boost::replace_all(sphere_diam_str,",",".");
    sphere_radius_str = boost::str(boost::format("%.2f") % (sphere_diam/2)); boost::replace_all(sphere_radius_str,",",".");
    n_sphere_1 = round(obj_middle/sphere_diam+0.5); ns1 = round(n_sphere_1/2-0.5);
    n_sphere_2 = round(obj_max/sphere_diam+0.5); ns2 = round(n_sphere_2/2-0.5);
    if(axis_min.compare("x")){ // the object minimum size is x
        y=1; z=1;
    }else if(axis_min.compare("y")){ // the object minimum size is y
        x=1;z=1;
    }else{ // the object minimum size is z
        x=1; y=1;
    }
    if(final){
        stream_model << string("var Rot_s_right {i1 in 1..3, i2 in 1..3} = sum {j in 1..3} Rot_H_right[i1,j]*Rot_tar_obj_right[j,i2];\n");
        // Modellization of the object in spheres
        stream_model << string("# Modelization of the object to place by the right arm\n");
        stream_dat << string("# Data of the modelization of the object to place by the right hand \n");

        stream_model << string("var ObjRight2Transp_center {j in 1..3} = Hand_right[j] + dFH_right * z_H_right[j] + tar_to_obj_right[j]; \n"); // center of the object

        for(int j=0;j<ns2;++j){// max size axis: positive direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s_right++; std::string n_s_str = to_string(n_s_right);
            stream_model << string("# Sphere_right_")+n_s_str+string("\n");
            stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_right_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3} = sum {k in 1..3} Rot_s_right[j,k]*ext_right_")+n_s_str+string("[k];\n");

            stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4} =")+string(" # xyz+radius \n");
            stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j] + ext_s_right_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
            stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
            stream_model << string("; \n");

            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s_right++; std::string n_s_str = to_string(n_s_right);
                stream_model << string("# Sphere_right_")+n_s_str+string("\n");
                stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_right_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_right[j,k]*ext_right_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4} =")+string(" #xyz+radius \n");
                stream_model << string(" if ( j<4 ) then ObjRight2Transp_center[j] + ext_s_right_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s_right++; std::string n_s_str = to_string(n_s_right);
                stream_model << string("# Sphere_right_")+n_s_str+string("\n");
                stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_right_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_right[j,k]*ext_right_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j] + ext_s_right_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
        }
        for(int j=1;j<ns2;++j){// max size axis: negative direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s_right++; std::string n_s_str = to_string(n_s_right);
            stream_model << string("# Sphere_right_")+n_s_str+string("\n");
            stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_right_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_right[j,k]*ext_right_")+n_s_str+string("[k];\n");
            stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4} = ")+string(" #xyz+radius \n");
            stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j] + ext_s_right_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
            stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
            stream_model << string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s_right++; std::string n_s_str = to_string(n_s_right);
                stream_model << string("# Sphere_right_")+n_s_str+string("\n");
                stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_right_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_right[j,k]*ext_right_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j] + ext_s_right_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=i;    int x=0; int y=0; int z=0; int k=0;
                }else{k=j;}
                n_s_right++; std::string n_s_str = to_string(n_s_right);
                stream_model << string("# Sphere_right_")+n_s_str+string("\n");
                stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_right_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_right[j,k]*ext_right_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j] + ext_s_right_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
        }
    }else{ // bounce posture selection        
        stream_model << string("var Rot_s_right {i1 in 1..3, i2 in 1..3,i in Iterations} =  sum {j in 1..3} Rot_H_right[i1,j,i]*Rot_tar_obj_right[j,i2];\n");
        // Modellization of the object in spheres
        stream_model << string("# Modelization of the object to place by the right arm\n");
        stream_dat << string("# Data of the modelization of the object to place by the right hand\n");

        stream_model << string("var ObjRight2Transp_center {j in 1..3, i in Iterations} = Hand_right[j,i] + dFH_right * z_H_right[j,i] + tar_to_obj_right[j]; \n"); // center of the object

        for(int j=0;j<ns2;++j){// max size axis: positive direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s_right++; std::string n_s_str = to_string(n_s_right);
            stream_model << string("# Sphere_right_")+n_s_str+string("\n");
            stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_right_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_right[j,k,i]*ext_right_")+n_s_str+string("[k];\n");           
            stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
            stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j,i] + ext_s_right_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
            stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
            stream_model << string("; \n");

            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s_right++; std::string n_s_str = to_string(n_s_right);
                stream_model << string("# Sphere_right_")+n_s_str+string("\n");
                stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_right_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_right[j,k,i]*ext_right_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j,i] + ext_s_right_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s_right++; std::string n_s_str = to_string(n_s_right);
                stream_model << string("# Sphere_right_")+n_s_str+string("\n");
                stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_right_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_right[j,k,i]*ext_right_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j,i] + ext_s_right_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
        }
        for(int j=1;j<ns2;++j){// max size axis: negative direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s_right++; std::string n_s_str = to_string(n_s_right);
            stream_model << string("# Sphere_right_")+n_s_str+string("\n");
            stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_right_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_right[j,k,i]*ext_right_")+n_s_str+string("[k];\n");
            stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
            stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j,i] + ext_s_right_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
            stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
            stream_model << string("; \n");

            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s_right++; std::string n_s_str = to_string(n_s_right);
                stream_model << string("# Sphere_right_")+n_s_str+string("\n");
                stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_right_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_right[j,k,i]*ext_right_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j,i] + ext_s_right_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s_right++; std::string n_s_str = to_string(n_s_right);
                stream_model << string("# Sphere_right_")+n_s_str+string("\n");
                stream_model << string("param ext_right_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_right_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_right_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_right[j,k,i]*ext_right_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjRight2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjRight2Transp_center[j,i] + ext_s_right_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
        }
    }
    // ------------------------------------ Object left target --------------------------------------------------------------- //
    n_s_left=0;
    axis_map["x"] = obj_tar_left_size.at(0);
    axis_map["y"] = obj_tar_left_size.at(1);
    axis_map["z"] = obj_tar_left_size.at(2);
    std::vector<std::pair<std::string,double> > obj_left_sizes(axis_map.begin(), axis_map.end());
    std::sort(obj_left_sizes.begin(),obj_left_sizes.end(),&compare_sizes);
    axis_min = (obj_left_sizes.at(0)).first; axis_middle = (obj_left_sizes.at(1)).first; axis_max = (obj_left_sizes.at(2)).first;
    obj_min = (obj_left_sizes.at(0)).second; obj_middle = (obj_left_sizes.at(1)).second; obj_max = (obj_left_sizes.at(2)).second;
    sphere_diam = obj_min; sphere_diam_str = boost::str(boost::format("%.2f") % (sphere_diam)); boost::replace_all(sphere_diam_str,",",".");
    sphere_radius_str = boost::str(boost::format("%.2f") % (sphere_diam/2)); boost::replace_all(sphere_radius_str,",",".");
    n_sphere_1 = round(obj_middle/sphere_diam+0.5); ns1 = round(n_sphere_1/2-0.5);
    n_sphere_2 = round(obj_max/sphere_diam+0.5); ns2 = round(n_sphere_2/2-0.5);
    if(axis_min.compare("x")){ // the object minimum size is x
        y=1; z=1;
    }else if(axis_min.compare("y")){ // the object minimum size is y
        x=1;z=1;
    }else{ // the object minimum size is z
        x=1; y=1;
    }
    if(final){
        stream_model << string("var Rot_s_left {i1 in 1..3, i2 in 1..3} =  sum {j in 1..3} Rot_H_left[i1,j]*Rot_tar_obj_left[j,i2];\n");
        // Modellization of the object in spheres
        stream_model << string("# Modelization of the object to place by the left arm\n");
        stream_dat << string("# Data of the modelization of the object to place by the left hand \n");

        stream_model << string("var ObjLeft2Transp_center {j in 1..3} = Hand_left[j] + dFH_left * z_H_left[j] + tar_to_obj_left[j]; \n"); // center of the object

        for(int j=0;j<ns2;++j){// max size axis: positive direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s_left++; std::string n_s_str = to_string(n_s_left);
            stream_model << string("# Sphere_left_")+n_s_str+string("\n");
            stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_left_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_left[j,k]*ext_left_")+n_s_str+string("[k];\n");
            stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4} = ")+string(" #xyz+radius \n");
            stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j] + ext_s_left_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
            stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
            stream_model << string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s_left++; std::string n_s_str = to_string(n_s_left);
                stream_model << string("# Sphere_left_")+n_s_str+string("\n");
                stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_left_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_left[j,k]*ext_right_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j] + ext_s_left_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s_left++; std::string n_s_str = to_string(n_s_left);
                stream_model << string("# Sphere_left_")+n_s_str+string("\n");
                stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_left_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_left[j,k]*ext_left_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j] + ext_s_left_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
        }
        for(int j=1;j<ns2;++j){// max size axis: negative direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s_left++; std::string n_s_str = to_string(n_s_left);
            stream_model << string("# Sphere_left_")+n_s_str+string("\n");
            stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_left_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_left[j,k]*ext_left_")+n_s_str+string("[k];\n");
            stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4} = ")+string(" #xyz+radius \n");
            stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j] + ext_s_left_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
            stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
            stream_model << string("; \n");
            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s_left++; std::string n_s_str = to_string(n_s_left);
                stream_model << string("# Sphere_left_")+n_s_str+string("\n");
                stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_left_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_left[j,k]*ext_left_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j] + ext_s_left_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s_left++; std::string n_s_str = to_string(n_s_left);
                stream_model << string("# Sphere_left_")+n_s_str+string("\n");
                stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_left_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3} =  sum {k in 1..3} Rot_s_left[j,k]*ext_left_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j] + ext_s_left_")+n_s_str+string("[j]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");
            }
        }
    }else{ // bounce posture selection
        stream_model << string("var Rot_s_left {i1 in 1..3, i2 in 1..3,i in Iterations} =  sum {j in 1..3} Rot_H_left[i1,j,i]*Rot_tar_obj_left[j,i2];\n");
        // Modellization of the object in spheres
        stream_model << string("# Modelization of the object to place by the left arm\n");
        stream_dat << string("# Data of the modelization of the object to place by the left hand\n");

        stream_model << string("var ObjLeft2Transp_center {j in 1..3, i in Iterations} = Hand_left[j,i] + dFH_left * z_H_left[j,i] + tar_to_obj_left[j]; \n"); // center of the object

        for(int j=0;j<ns2;++j){// max size axis: positive direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s_left++; std::string n_s_str = to_string(n_s_left);
            stream_model << string("# Sphere_left_")+n_s_str+string("\n");
            stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_left_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_left[j,k,i]*ext_left_")+n_s_str+string("[k];\n");
            stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
            stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j,i] + ext_s_left_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
            stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
            stream_model << string("; \n");

            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s_left++; std::string n_s_str = to_string(n_s_left);
                stream_model << string("# Sphere_left_")+n_s_str+string("\n");
                stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_left_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_left[j,k,i]*ext_left_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j,i] + ext_s_left_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");

            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s_left++; std::string n_s_str = to_string(n_s_left);
                stream_model << string("# Sphere_left_")+n_s_str+string("\n");
                stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_left_")+n_s_str+string(" := \n");
                string ext_1 =  to_string(x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_left[j,k,i]*ext_left_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j,i] + ext_s_left_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");

            }
        }
        for(int j=1;j<ns2;++j){// max size axis: negative direction
            if(x==1 && y==1){
                k=j;
            }else{k=0;}
            n_s_left++; std::string n_s_str = to_string(n_s_left);
            stream_model << string("# Sphere_left_")+n_s_str+string("\n");
            stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

            stream_dat << string("param ext_left_")+n_s_str+string(" := \n");
            string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
            string ext_2 =  to_string(0); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
            string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
            stream_dat << string(";\n");

            stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_left[j,k,i]*ext_left_")+n_s_str+string("[k];\n");
            stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
            stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j,i] + ext_s_left_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
            stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
            stream_model << string("; \n");

            for(int i=1;i<ns1;++i){ // middle size axis: positive direction
                if(x==1 && z==1){
                    k=-i;
                }else{k=j;}
                n_s_left++; std::string n_s_str = to_string(n_s_left);
                stream_model << string("# Sphere_left_")+n_s_str+string("\n");
                stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_left_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_left[j,k,i]*ext_left_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j,i] + ext_s_left_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");

            }
            for(int i=1;i<ns1;++i){ // middle size axis: negative direction
                if(x==1 && z==1){
                    k=i;
                }else{k=j;}
                n_s_left++; std::string n_s_str = to_string(n_s_left);
                stream_model << string("# Sphere_left_")+n_s_str+string("\n");
                stream_model << string("param ext_left_")+n_s_str+string("{i in 1..3};\n");

                stream_dat << string("param ext_left_")+n_s_str+string(" {i in 1..3}= \n");
                string ext_1 =  to_string(-x*k); stream_dat << to_string(1)+string(" ")+ext_1+string("\n");
                string ext_2 =  to_string(-y*i); stream_dat << to_string(2)+string(" ")+ext_2+string("\n");
                string ext_3 =  to_string(-z*j); stream_dat << to_string(3)+string(" ")+ext_3+string("\n");
                stream_dat << string(";\n");

                stream_model << string("var ext_s_left_")+n_s_str+string(" {j in 1..3,i in Iterations} =  sum {k in 1..3} Rot_s_left[j,k,i]*ext_left_")+n_s_str+string("[k];\n");
                stream_model << string("var ObjLeft2Transp_")+n_s_str+string(" {j in 1..4, i in Iterations} = ")+string(" #xyz+radius \n");
                stream_model << string("if ( j<4 ) then ObjLeft2Transp_center[j,i] + ext_s_left_")+n_s_str+string("[j,i]*")+sphere_diam_str+string("\n");
                stream_model << string("else if (j=4) then ")+sphere_radius_str+string("\n");
                stream_model << string("; \n");

            }
        }
    }

}

bool HUMPlanner::compare_sizes (std::pair<std::string,double> pair_1, std::pair<std::string,double> pair_2)
{
    return (pair_1.second < pair_2.second);
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

void HUMPlanner::getObstaclesDualArm(std::vector<double> center_right, std::vector<double> center_left,double radius_right, double radius_left, std::vector<objectPtr>& obsts_right, std::vector<objectPtr>& obsts_left, int hand_code_right,int hand_code_left)
{
    double tol_right;
    switch (hand_code_right){
    case 0: // human hand
        tol_right = 50.0;
        break;
    case 1://barrett hand
        tol_right = 5;
        break;
    }

    double tol_left;
    switch (hand_code_left){
    case 0: // human hand
        tol_left = 50.0;
        break;
    case 1://barrett hand
        tol_left = 5;
        break;
    }

    for (std::size_t i =0; i < this->obstacles_right.size(); ++i){
        objectPtr obj = this->obstacles_right.at(i);
        // get the RPY matrix
        Matrix3d Rot; std::vector<double> rpy; obj->getOr(rpy); this->RPY_matrix(rpy,Rot);
        // get the position of the object
        std::vector<double> pos; obj->getPos(pos);
        // get the size of the object
        std::vector<double> dim; obj->getSize(dim);

        // distance vector (right)
        Vector3d diff_right;
        diff_right(0) = center_right.at(0) - pos.at(0);
        diff_right(1) = center_right.at(1) - pos.at(1);
        diff_right(2) = center_right.at(2) - pos.at(2);
        // A matrix (right)
        Matrix3d A_right;
        A_right(0,0) = 1/pow(radius_right+dim.at(0)+tol_right,2); A_right(0,1) = 0; A_right(0,2) = 0;
        A_right(1,0) = 0; A_right(1,1) = 1/pow(radius_right+dim.at(1)+tol_right,2); A_right(1,2) = 0;
        A_right(2,0) = 0; A_right(2,1) = 0; A_right(2,2) = 1/pow(radius_right+dim.at(2)+tol_right,2);

        MatrixXd diff1_right = MatrixXd::Zero(3,1);
        diff1_right(0,0) = diff_right(0); diff1_right(1,0) = diff_right(1); diff1_right(2,0) = diff_right(2);
        MatrixXd diffT_right = diff1_right.transpose();

        // distance vector (left)
        Vector3d diff_left;
        diff_left(0) = center_left.at(0) - pos.at(0);
        diff_left(1) = center_left.at(1) - pos.at(1);
        diff_left(2) = center_left.at(2) - pos.at(2);
        // A matrix (left)
        Matrix3d A_left;
        A_left(0,0) = 1/pow(radius_left+dim.at(0)+tol_left,2); A_left(0,1) = 0; A_left(0,2) = 0;
        A_left(1,0) = 0; A_left(1,1) = 1/pow(radius_left+dim.at(1)+tol_left,2); A_left(1,2) = 0;
        A_left(2,0) = 0; A_left(2,1) = 0; A_left(2,2) = 1/pow(radius_left+dim.at(2)+tol_left,2);

        MatrixXd diff1_left = MatrixXd::Zero(3,1);
        diff1_left(0,0) = diff_left(0); diff1_left(1,0) = diff_left(1); diff1_left(2,0) = diff_left(2);
        MatrixXd diffT_left = diff1_left.transpose();


        MatrixXd RotT(Rot.transpose());

        MatrixXd to_check_right(diffT_right*RotT);
        to_check_right = to_check_right * A_right;
        to_check_right = to_check_right * Rot;
        to_check_right = to_check_right * diff_right;

        MatrixXd to_check_left(diffT_left*RotT);
        to_check_left = to_check_left * A_left;
        to_check_left = to_check_left * Rot;
        to_check_left = to_check_left * diff_left;

        if (to_check_right(0,0) < 1){
            // the object is a real obstacle
            obsts_right.push_back(obj);
        }
    }

    for (std::size_t i =0; i < this->obstacles_left.size(); ++i){
        objectPtr obj = this->obstacles_left.at(i);
        // get the RPY matrix
        Matrix3d Rot; std::vector<double> rpy; obj->getOr(rpy); this->RPY_matrix(rpy,Rot);
        // get the position of the object
        std::vector<double> pos; obj->getPos(pos);
        // get the size of the object
        std::vector<double> dim; obj->getSize(dim);

        // distance vector (right)
        Vector3d diff_right;
        diff_right(0) = center_right.at(0) - pos.at(0);
        diff_right(1) = center_right.at(1) - pos.at(1);
        diff_right(2) = center_right.at(2) - pos.at(2);
        // A matrix (right)
        Matrix3d A_right;
        A_right(0,0) = 1/pow(radius_right+dim.at(0)+tol_right,2); A_right(0,1) = 0; A_right(0,2) = 0;
        A_right(1,0) = 0; A_right(1,1) = 1/pow(radius_right+dim.at(1)+tol_right,2); A_right(1,2) = 0;
        A_right(2,0) = 0; A_right(2,1) = 0; A_right(2,2) = 1/pow(radius_right+dim.at(2)+tol_right,2);

        MatrixXd diff1_right = MatrixXd::Zero(3,1);
        diff1_right(0,0) = diff_right(0); diff1_right(1,0) = diff_right(1); diff1_right(2,0) = diff_right(2);
        MatrixXd diffT_right = diff1_right.transpose();

        // distance vector (left)
        Vector3d diff_left;
        diff_left(0) = center_left.at(0) - pos.at(0);
        diff_left(1) = center_left.at(1) - pos.at(1);
        diff_left(2) = center_left.at(2) - pos.at(2);
        // A matrix (left)
        Matrix3d A_left;
        A_left(0,0) = 1/pow(radius_left+dim.at(0)+tol_left,2); A_left(0,1) = 0; A_left(0,2) = 0;
        A_left(1,0) = 0; A_left(1,1) = 1/pow(radius_left+dim.at(1)+tol_left,2); A_left(1,2) = 0;
        A_left(2,0) = 0; A_left(2,1) = 0; A_left(2,2) = 1/pow(radius_left+dim.at(2)+tol_left,2);

        MatrixXd diff1_left = MatrixXd::Zero(3,1);
        diff1_left(0,0) = diff_left(0); diff1_left(1,0) = diff_left(1); diff1_left(2,0) = diff_left(2);
        MatrixXd diffT_left = diff1_left.transpose();


        MatrixXd RotT(Rot.transpose());

        MatrixXd to_check_right(diffT_right*RotT);
        to_check_right = to_check_right * A_right;
        to_check_right = to_check_right * Rot;
        to_check_right = to_check_right * diff_right;

        MatrixXd to_check_left(diffT_left*RotT);
        to_check_left = to_check_left * A_left;
        to_check_left = to_check_left * Rot;
        to_check_left = to_check_left * diff_left;

        if (to_check_left(0,0) < 1){
            // the object is a real obstacle
            obsts_left.push_back(obj);
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

bool HUMPlanner::optimize(string &nlfile, std::vector<Number> &x, double tol, double acc_tol, double constr_viol_tol)
{
    // Create a new instance of IpoptApplication
    //  (use a SmartPtr, not raw)
    // We are using the factory, since this allows us to compile this
    // example with an Ipopt Windows DLL
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    app->RethrowNonIpoptException(true);

    app->Options()->SetNumericValue("tol", tol);
    app->Options()->SetNumericValue("acceptable_tol", acc_tol);
    //app->Options()->SetNumericValue("constr_viol_tol", constr_viol_tol);
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    app->Options()->SetIntegerValue("print_level",3);
    //app->Options()->SetIntegerValue("max_iter",10000);
    //double bound_frac = 0.01;//k2
    //double bound_push = 0.01;//k1
    //double bound_relax_factor = 0.0;
    //app->Options()->SetNumericValue("bound_frac",bound_frac);
    //app->Options()->SetNumericValue("bound_push",bound_push);
    //app->Options()->SetNumericValue("bound_relax_factor",bound_relax_factor);

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

    if ((ampl_tnlp->get_status() == SolverReturn::SUCCESS) || (ampl_tnlp->get_status() == SolverReturn::STOP_AT_ACCEPTABLE_POINT)){
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


bool HUMPlanner::singleArmFinalPosture(int mov_type,int pre_post,hump_params& params, std::vector<double> initPosture, std::vector<double>& finalPosture)

{
    // movement settings
    int arm_code = params.mov_specs.arm_code;
    int hand_code = params.mov_specs.hand_code;
    bool rand_init = params.mov_specs.rand_init;
    bool straight = params.mov_specs.straight_line;
    std::vector<double> target = params.mov_specs.target;
    std::vector<double> minLimits;
    std::vector<double> maxLimits;
    bool approach = params.mov_specs.approach;
    bool retreat = params.mov_specs.retreat;
    std::vector<double> approach_vec;
    std::vector<double> retreat_vec;
    switch(mov_type){
    case 0: // pick
        if(approach){approach_vec = params.mov_specs.pre_grasp_approach;}
        if(retreat){retreat_vec = params.mov_specs.post_grasp_retreat;}
        break;
    case 1: // place
        if(approach){approach_vec = params.mov_specs.pre_place_approach;}
        if(retreat){retreat_vec = params.mov_specs.post_place_retreat;}
        break;
    }

    std::vector<double> initArmPosture(initPosture.begin(),initPosture.begin()+joints_arm);


    double Lu; double Ll; double Lh;
    switch(arm_code){
    case 0: // dual arm
        //TO DO
        break;
    case 1: // right arm
        Lu = this->DH_rightArm.d.at(2);
        Ll = this->DH_rightArm.d.at(4);
        Lh = this->DH_rightArm.d.at(6);
        minLimits = this->minRightLimits;
        maxLimits = this->maxRightLimits;
        break;
    case 2: // left arm
        Lu = this->DH_leftArm.d.at(2);
        Ll = this->DH_leftArm.d.at(4);
        Lh = this->DH_leftArm.d.at(6);
        minLimits = this->minLeftLimits;
        maxLimits = this->maxLeftLimits;
        break;
    }

    std::vector<double> shPos; this->getShoulderPos(arm_code,initPosture,shPos);
    double max_ext = Lh+Ll+Lu;
    if(!straight){
        // check if the target is in the workspace of the robotic arm
        Vector3d tar_pos(target.at(0),target.at(1),target.at(2));
        Vector3d tar_pos_app(target.at(0),target.at(1),target.at(2));
        Vector3d tar_pos_ret(target.at(0),target.at(1),target.at(2));
        if(approach){
            std::vector<double> rpy = {target.at(3),target.at(4),target.at(5)};
            Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
            double dist = approach_vec.at(3);
            Vector3d v(approach_vec.at(0),approach_vec.at(1),approach_vec.at(2));
            Vector3d vv = Rot_tar*v;
            tar_pos_app = tar_pos_app + dist*vv;
        }else if(retreat){
            std::vector<double> rpy = {target.at(3),target.at(4),target.at(5)};
            Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
            double dist = retreat_vec.at(3);
            Vector3d v(retreat_vec.at(0),retreat_vec.at(1),retreat_vec.at(2));
            Vector3d vv = Rot_tar*v;
            tar_pos_ret = tar_pos_ret + dist*vv;
        }
        if((sqrt(pow(tar_pos(0) - shPos.at(0),2)+
                pow(tar_pos(1) - shPos.at(1),2)+
                pow(tar_pos(2) - shPos.at(2),2))>= max_ext)||
                (sqrt(pow(tar_pos_app(0) - shPos.at(0),2)+
                      pow(tar_pos_app(1) - shPos.at(1),2)+
                      pow(tar_pos_app(2) - shPos.at(2),2))>= max_ext)||
                (sqrt(pow(tar_pos_ret(0) - shPos.at(0),2)+
                      pow(tar_pos_ret(1) - shPos.at(1),2)+
                      pow(tar_pos_ret(2) - shPos.at(2),2))>= max_ext)){
            throw string("The movement to be planned goes out of the reachable workspace");
        }
    }

    // initial guess
    std::vector<double> minArmLimits(minLimits.begin(),minLimits.begin()+joints_arm);
    std::vector<double> maxArmLimits(maxLimits.begin(),maxLimits.begin()+joints_arm);
    std::vector<double> initialGuess(minArmLimits.size(),0.0);
    if ((pre_post==1) && rand_init){ // pre_posture for approaching
        for(size_t i=0; i < minArmLimits.size();++i){
            initialGuess.at(i)=getRand(minArmLimits.at(i)+SPACER,maxArmLimits.at(i)-SPACER);
        }
    }else{initialGuess = initArmPosture;}
    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    this->getObstaclesSingleArm(shPos,max_ext,obsts,hand_code);
    if(mov_type==1 && pre_post==0){
        // place movement, plan stage
        // remove the support object from the obstacles
        if(params.mov_specs.support_obj.compare("")){
            std::string support_obj_name = params.mov_specs.support_obj;
            for(size_t i=0; i < obsts.size();++i){
                objectPtr curr_obj = obsts.at(i);
                std::string curr_obj_name = curr_obj->getName();
                if(support_obj_name.compare(curr_obj_name)==0){
                    obsts.erase (obsts.begin()+i);
                }
            }
        }
    }

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
                if (this->optimize(nlfile,x_sol,FINAL_TOL,FINAL_ACC_TOL,FINAL_CONSTR_VIOL_TOL)){
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


bool HUMPlanner::singleArmBouncePosture(int steps,int mov_type,int pre_post,hump_params& params,std::vector<double> initPosture,std::vector<double> finalPosture,std::vector<double>& bouncePosture)
{


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
    bool place = false;
    if(mov_type==1){place=true;}

    if(!place){// not a place movement
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
    }
    bAux.vel_0=vel0Aux;
    bAux.vel_f=velfAux;
    bAux.acc_0=acc0Aux;
    bAux.acc_f=accfAux;
    // initial guess
    std::vector<double> initialGuess = initAuxPosture;
    /*
    std::vector<double> initialGuess(initAuxPosture.size(),0.0);
    for(size_t i=0; i < initAuxPosture.size();++i){
        initialGuess.at(i) = (initAuxPosture.at(i)+finalAuxPosture.at(i))/2;
    }
    */
    double Lu = dh.d.at(2);
    double Ll = dh.d.at(4);
    double Lh = dh.d.at(6);
    double max_ext = Lh+Ll+Lu;
    // get the obstacles of the workspace
    std::vector<objectPtr> obsts;
    std::vector<double> shPos; this->getShoulderPos(arm_code,initPosture,shPos);
    this->getObstaclesSingleArm(shPos,max_ext,obsts,hand_code);

    bool written = this->writeFilesBouncePosture(steps,params,mov_type,pre_post,minAuxLimits,maxAuxLimits,initAuxPosture,finalAuxPosture,initialGuess,lambdaAux,obsts,bAux);

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
                if (this->optimize(nlfile,x_sol,BOUNCE_TOL,BOUNCE_ACC_TOL,BOUNCE_CONSTR_VIOL_TOL)){
                    size_t size;
                    bouncePosture = std::vector<double>(joints_arm+joints_hand);
                    if(place){
                      size =x_sol.size();
                    }else{
                      size =x_sol.size()-2;
                    }
                    for (std::size_t i=0; i < size; ++i){
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
                    if(!place){
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
                    }else{
                        bouncePosture.at(7) = finalHand.at(0);
                        bouncePosture.at(8) = finalHand.at(1);
                        bouncePosture.at(9) = finalHand.at(2);
                        bouncePosture.at(10) = finalHand.at(3);
                    }
                    return true;
                }else{return false;}
            }catch(const std::exception &exc){throw string(exc.what());}
        }else{throw string("Error in writing the files for optimization");}
    }else{throw string("Error in writing the files for optimization");}
}

bool HUMPlanner::singleDualArmBouncePosture(int steps,int dual_mov_type,int pre_post,hump_dual_params& params,std::vector<double> initPosture,std::vector<double> finalPosture,std::vector<double>& bouncePosture)
{
    std::vector<double> minLimits_right; std::vector<double> maxLimits_right;
    std::vector<double> minLimits_left; std::vector<double> maxLimits_left;
    DHparameters dh_right; DHparameters dh_left;
    // movement settings
    int hand_code_right = params.mov_specs_right.hand_code;
    int hand_code_left = params.mov_specs_left.hand_code;
    std::vector<double> finalHand_right(params.mov_specs_right.finalHand);
    std::vector<double> finalHand_left(params.mov_specs_left.finalHand);
    //tolerances
    boundaryConditions b_right = params.bounds_right;
    boundaryConditions b_left = params.bounds_left;
    boundaryConditions bAux;
    std::vector<double> lambda_right(params.lambda_bounce_right);
    std::vector<double> lambda_left(params.lambda_bounce_left);

    minLimits_right = this->minRightLimits; maxLimits_right = this->maxRightLimits;
    dh_right = this->DH_rightArm;
    minLimits_left = this->minLeftLimits; maxLimits_left = this->maxLeftLimits;
    dh_left = this->DH_leftArm;

    // initial posture
    std::vector<double> initAuxPosture(joints_arm+joints_arm,0);
    std::copy(initPosture.begin(),initPosture.begin()+joints_arm,initAuxPosture.begin());
    std::copy(initPosture.begin()+joints_arm+joints_hand,initPosture.begin()+joints_arm+joints_hand+joints_arm,initAuxPosture.begin()+joints_arm);
    //final posture
    std::vector<double> finalAuxPosture(finalPosture);
    // minimum limits
    std::vector<double> minAuxLimits(joints_arm+joints_arm,0);
    std::copy(minLimits_right.begin(),minLimits_right.begin()+joints_arm,minAuxLimits.begin());
    std::copy(minLimits_left.begin(),minLimits_left.begin()+joints_arm,minAuxLimits.begin()+joints_arm);
    // maximum limits
    std::vector<double> maxAuxLimits(joints_arm+joints_arm,0);
    std::copy(maxLimits_right.begin(),maxLimits_right.begin()+joints_arm,maxAuxLimits.begin());
    std::copy(maxLimits_left.begin(),maxLimits_left.begin()+joints_arm,maxAuxLimits.begin()+joints_arm);
    // lambda
    std::vector<double> lambdaAux(joints_arm+joints_arm,0);
    std::copy(lambda_right.begin(),lambda_right.begin()+joints_arm,lambdaAux.begin());
    std::copy(lambda_left.begin(),lambda_left.begin()+joints_arm,lambdaAux.begin()+joints_arm);
    // initial velocity
    std::vector<double> vel0Aux(joints_arm+joints_arm,0);
    std::copy(b_right.vel_0.begin(),b_right.vel_0.begin()+joints_arm,vel0Aux.begin());
    std::copy(b_left.vel_0.begin(),b_left.vel_0.begin()+joints_arm,vel0Aux.begin()+joints_arm);
    // final velocity
    std::vector<double> velfAux(joints_arm+joints_arm,0);
    std::copy(b_right.vel_f.begin(),b_right.vel_f.begin()+joints_arm,velfAux.begin());
    std::copy(b_left.vel_f.begin(),b_left.vel_f.begin()+joints_arm,velfAux.begin()+joints_arm);
    // initial acceleration
    std::vector<double> acc0Aux(joints_arm+joints_arm,0);
    std::copy(b_right.acc_0.begin(),b_right.acc_0.begin()+joints_arm,acc0Aux.begin());
    std::copy(b_left.acc_0.begin(),b_left.acc_0.begin()+joints_arm,acc0Aux.begin()+joints_arm);
    // final acceleration
    std::vector<double> accfAux(joints_arm+joints_arm,0);
    std::copy(b_right.acc_f.begin(),b_right.acc_f.begin()+joints_arm,accfAux.begin());
    std::copy(b_left.acc_f.begin(),b_left.acc_f.begin()+joints_arm,accfAux.begin()+joints_arm);

    bool place = false;  if(dual_mov_type==1){place=true;}
    if(!place){// not a place movement
        switch(hand_code_right){
        case 0:// human hand
            // TO DO
            break;
        case 1:// barrett hand
            initAuxPosture.insert(initAuxPosture.begin()+joints_arm,initPosture.at(8));
            initAuxPosture.insert(initAuxPosture.begin()+joints_arm+1,initPosture.at(10));

            finalAuxPosture.insert(finalAuxPosture.begin()+joints_arm,finalHand_right.at(1));
            finalAuxPosture.insert(finalAuxPosture.begin()+joints_arm+1,finalHand_right.at(3));

            minAuxLimits.insert(minAuxLimits.begin()+joints_arm,minLimits_right.at(8));
            minAuxLimits.insert(minAuxLimits.begin()+joints_arm+1,minLimits_right.at(10));

            maxAuxLimits.insert(maxAuxLimits.begin()+joints_arm,maxLimits_right.at(8));
            maxAuxLimits.insert(maxAuxLimits.begin()+joints_arm+1,maxLimits_right.at(10));

            lambdaAux.insert(lambdaAux.begin()+joints_arm,lambda_right.at(8));
            lambdaAux.insert(lambdaAux.begin()+joints_arm+1,lambda_right.at(10));

            vel0Aux.insert(vel0Aux.begin()+joints_arm,b_right.vel_0.at(8));
            vel0Aux.insert(vel0Aux.begin()+joints_arm+1,b_right.vel_0.at(10));

            velfAux.insert(velfAux.begin()+joints_arm,b_right.vel_f.at(8));
            velfAux.insert(velfAux.begin()+joints_arm+1,b_right.vel_f.at(10));

            acc0Aux.insert(acc0Aux.begin()+joints_arm,b_right.acc_0.at(8));
            acc0Aux.insert(acc0Aux.begin()+joints_arm+1,b_right.acc_0.at(10));

            accfAux.insert(accfAux.begin()+joints_arm,b_right.acc_f.at(8));
            accfAux.insert(accfAux.begin()+joints_arm+1,b_right.acc_f.at(10));
            break;
        }
        switch(hand_code_left){
        case 0:// human hand
            // TO DO
            break;
        case 1:// barrett hand

            initAuxPosture.push_back(initPosture.at(19));
            initAuxPosture.push_back(initPosture.at(21));

            finalAuxPosture.push_back(finalHand_left.at(1));
            finalAuxPosture.push_back(finalHand_left.at(3));

            minAuxLimits.push_back(minLimits_left.at(8));
            minAuxLimits.push_back(minLimits_left.at(10));

            maxAuxLimits.push_back(maxLimits_left.at(8));
            maxAuxLimits.push_back(maxLimits_left.at(10));

            lambdaAux.push_back(lambda_left.at(8));
            lambdaAux.push_back(lambda_left.at(10));

            vel0Aux.push_back(b_left.vel_0.at(8));
            vel0Aux.push_back(b_left.vel_0.at(10));

            velfAux.push_back(b_left.vel_f.at(8));
            velfAux.push_back(b_left.vel_f.at(10));

            acc0Aux.push_back(b_left.acc_0.at(8));
            acc0Aux.push_back(b_left.acc_0.at(10));

            accfAux.push_back(b_left.acc_f.at(8));
            accfAux.push_back(b_left.acc_f.at(10));
            break;
        }
    }
    bAux.vel_0=vel0Aux;
    bAux.vel_f=velfAux;
    bAux.acc_0=acc0Aux;
    bAux.acc_f=accfAux;
    // initial guess
    std::vector<double> initialGuess = initAuxPosture;
    /*
    std::vector<double> initialGuess(initAuxPosture.size(),0.0);
    for(size_t i=0; i < initAuxPosture.size();++i){
        initialGuess.at(i) = (initAuxPosture.at(i)+finalAuxPosture.at(i))/2;
    }
    */

    std::vector<double> initRightPosture(initPosture.begin(),initPosture.begin()+joints_arm+joints_hand);
    std::vector<double> initLeftPosture(initPosture.begin(),initPosture.begin()+joints_arm+joints_hand);
    double Lu_right; double Ll_right; double Lh_right;
    Lu_right = abs(dh_right.d.at(2));
    Ll_right = abs(dh_right.d.at(4));
    Lh_right = abs(dh_right.d.at(6));
    double Lu_left; double Ll_left; double Lh_left;
    Lu_left = abs(dh_left.d.at(2));
    Ll_left = abs(dh_left.d.at(4));
    Lh_left = abs(dh_left.d.at(6));

    std::vector<double> shPos_right; this->getShoulderPos(1,initRightPosture,shPos_right);
    double max_ext_right = Lh_right+Ll_right+Lu_right;
    std::vector<double> shPos_left; this->getShoulderPos(2,initLeftPosture,shPos_left);
    double max_ext_left = Lh_left+Ll_left+Lu_left;
    // get the obstacles of the workspace
    std::vector<objectPtr> obsts_right; std::vector<objectPtr> obsts_left;
    this->getObstaclesDualArm(shPos_right,shPos_left,max_ext_right,max_ext_left,obsts_right,obsts_left,hand_code_right,hand_code_left);

    bool written = this->writeFilesDualBouncePosture(steps,params,dual_mov_type,pre_post,minAuxLimits,maxAuxLimits,initAuxPosture,finalAuxPosture,initialGuess,lambdaAux,obsts_right,obsts_left,bAux);

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
                if (this->optimize(nlfile,x_sol,BOUNCE_DUAL_TOL,BOUNCE_DUAL_ACC_TOL,BOUNCE_DUAL_CONSTR_VIOL_TOL)){

                    bouncePosture = std::vector<double>(2*(joints_arm+joints_hand));
                    // right arm
                    bouncePosture.at(0) = x_sol[0];
                    bouncePosture.at(1) = x_sol[1];
                    bouncePosture.at(2) = x_sol[2];
                    bouncePosture.at(3) = x_sol[3];
                    bouncePosture.at(4) = x_sol[4];
                    bouncePosture.at(5) = x_sol[5];
                    bouncePosture.at(6) = x_sol[6];
                    //left arm
                    if(place){
                        bouncePosture.at(11) = x_sol[7];
                        bouncePosture.at(12) = x_sol[8];
                        bouncePosture.at(13) = x_sol[9];
                        bouncePosture.at(14) = x_sol[10];
                        bouncePosture.at(15) = x_sol[11];
                        bouncePosture.at(16) = x_sol[12];
                        bouncePosture.at(17) = x_sol[13];
                    }else{
                        bouncePosture.at(11) = x_sol[9];
                        bouncePosture.at(12) = x_sol[10];
                        bouncePosture.at(13) = x_sol[11];
                        bouncePosture.at(14) = x_sol[12];
                        bouncePosture.at(15) = x_sol[13];
                        bouncePosture.at(16) = x_sol[14];
                        bouncePosture.at(17) = x_sol[15];
                    }

                    if(!place){
                        switch(hand_code_right){
                        case 0://human hand
                            //TO DO
                            //bouncePosture.at(7) = x_sol[7];
                            //bouncePosture.at(8) = x_sol[8];
                            //bouncePosture.at(9) = x_sol[8];
                            //bouncePosture.at(10) = x_sol[9];
                            break;
                        case 1:// barrett hand
                            bouncePosture.at(7) = 0.0;
                            bouncePosture.at(8) = x_sol[8];
                            bouncePosture.at(9) = x_sol[8];
                            bouncePosture.at(10) = x_sol[7];
                            break;
                        }
                        switch(hand_code_left){
                        case 0://human hand
                            //TO DO
                            //bouncePosture.at(7) = x_sol[7];
                            //bouncePosture.at(8) = x_sol[8];
                            //bouncePosture.at(9) = x_sol[8];
                            //bouncePosture.at(10) = x_sol[9];
                            break;
                        case 1:// barrett hand
                            bouncePosture.at(18) = 0.0;
                            bouncePosture.at(19) = x_sol[16];
                            bouncePosture.at(20) = x_sol[16];
                            bouncePosture.at(21) = x_sol[17];
                            break;
                        }
                    }else{
                        bouncePosture.at(7) = finalHand_right.at(0);
                        bouncePosture.at(8) = finalHand_right.at(1);
                        bouncePosture.at(9) = finalHand_right.at(2);
                        bouncePosture.at(10) = finalHand_right.at(3);
                        bouncePosture.at(18) = finalHand_left.at(0);
                        bouncePosture.at(19) = finalHand_left.at(1);
                        bouncePosture.at(20) = finalHand_left.at(2);
                        bouncePosture.at(21) = finalHand_left.at(3);
                    }
                    return true;
                }else{return false;}
            }catch(const std::exception &exc){throw string(exc.what());}
        }else{throw string("Error in writing the files for optimization");}
    }else{throw string("Error in writing the files for optimization");}
}




void HUMPlanner::getDerivative(std::vector<double> &function, std::vector<double> &step_values, std::vector<double> &derFunction)
{
    // Formula of the numarical differentiation with 5 points
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)


       const double MIN_STEP_VALUE = 0.1;
       const double MIN_DER_VALUE = 0.001;

       int h = 1;
       int tnsample;
       double f0;
       double f1;
       double f2;
       double f3;
       double f4;
       double step_value;

       // 1st point
       // f'0 = (-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h) + h^4/5*f^(5)(c_0)
       tnsample = 0;
       f0 = function.at(tnsample);
       f1 = function.at(tnsample+1);
       f2 = function.at(tnsample+2);
       f3 = function.at(tnsample+3);
       f4 = function.at(tnsample+4);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
            derFunction.push_back((double)(-25*f0 + 48*f1 - 36*f2 + 16*f3 -  3*f4)/(12*h*step_value));
       }

       // 2nd point
       // f'1 = ( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h) - h^4/20*f^(5)(c_1)
       tnsample = 1;
       f0 = function.at(tnsample-1);
       f1 = function.at(tnsample);
       f2 = function.at(tnsample+1);
       f3 = function.at(tnsample+2);
       f4 = function.at(tnsample+3);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
           derFunction.push_back((double)( -3*f0 - 10*f1 + 18*f2 -  6*f3 +  1*f4)/(12*h*step_value));
       }

       // 3rd point
       // f'2 = (  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h) + h^4/30*f^(5)(c_2)
       for (int i=2; i< function.size() -2;++i){     // centered
           f0 = function.at(i-2);
           f1 = function.at(i-1);
           f2 = function.at(i);
           f3 = function.at(i+1);
           f4 = function.at(i+2);
           step_value = step_values.at(i);
           if(step_value==0){
               //step_value=MIN_STEP_VALUE;
               derFunction.push_back(MIN_DER_VALUE);
           }else{
               derFunction.push_back((double)(  1*f0 -  8*f1         +  8*f3 -  1*f4)/(12*h*step_value));
           }
       }

       // 4th point
       // f'3 = ( -1*f0 +  6*f1 - 18*f2 + 10*f3 +  3*f4)/(12*h) - h^4/20*f^(5)(c_3)
       tnsample = function.size()-2;
       f0 = function.at(tnsample-3);
       f1 = function.at(tnsample-2);
       f2 = function.at(tnsample-1);
       f3 = function.at(tnsample);
       f4 = function.at(tnsample+1);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
           derFunction.push_back((double)( -f0+6*f1-18*f2+10*f3+3*f4)/(12*h*step_value));
       }

       // 5th point
       // f'4 = (  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h) + h^4/5*f^(5)(c_4)
       tnsample = function.size()-1;
       f0 = function.at(tnsample-4);
       f1 = function.at(tnsample-3);
       f2 = function.at(tnsample-2);
       f3 = function.at(tnsample-1);
       f4 = function.at(tnsample);
       step_value = step_values.at(tnsample);
       if(step_value==0){
           //step_value=MIN_STEP_VALUE;
           derFunction.push_back(MIN_DER_VALUE);
       }else{
           derFunction.push_back((double)(  3*f0 - 16*f1 + 36*f2 - 48*f3 + 25*f4)/(12*h*step_value));
       }

}



double HUMPlanner::getTimeStep(hump_params &tols, MatrixXd &jointTraj,int mod)
{
    int steps = jointTraj.rows();
    int n_joints = jointTraj.cols();
    double timestep;

    std::vector<double> w_max = tols.w_max;
    std::vector<double> alpha_max = tols.alpha_max;
    std::vector<double> lambda = tols.lambda_bounce;


    double num = 0.0;
    double den = 0.0;

    for (int k =0; k < n_joints; ++k){
        double deltaTheta_k = 0.0;
        std::vector<double> diffs;
        for (int i = 1; i < steps; ++i){
            double diff = abs(jointTraj(i,k) - jointTraj(i-1,k))*180/M_PI;
            diffs.push_back(diff);
            deltaTheta_k += diff;
        }
        std::vector<double>::iterator res = std::max_element(diffs.begin(),diffs.end());
        int poss = std::distance(diffs.begin(),res);
        double deltaThetaMax = diffs.at(poss);
        double w_max_degree = w_max.at(k)*180/M_PI;

        double timestep_k_h = (lambda.at(k)/(steps-1))*log(1+deltaTheta_k); // human-like timestep
        double timestep_k_w = deltaThetaMax/w_max_degree; // max velocity timestep
        double timestep_k = timestep_k_h + timestep_k_w;
        double time_k = (steps-1)*timestep_k;
        num += lambda.at(k)*deltaTheta_k*time_k;
        den += lambda.at(k)*deltaTheta_k;

    }

    double totalTime = num/den;
    timestep = (totalTime/(steps-1));

    if(HAS_JOINT_ACCELEARATION_MAX_LIMIT){
        // check the joint maximum velocity and acceleration
        if(mod!=2 && mod!=3){ //  move or pre_approach
            std::vector<double> vel_0 = tols.bounds.vel_0;
            std::vector<double> acc_0 = tols.bounds.acc_0;
            std::vector<double> vel_f;
            std::vector<double> acc_f;
            if(mod==0){
              vel_f = tols.bounds.vel_f;
              acc_f = tols.bounds.acc_f;
            }else if(mod==1){
              vel_f = tols.vel_approach;
              acc_f = std::vector<double>(tols.bounds.acc_0.size(),0.0);
            }
            for (int k =0; k < n_joints; ++k){
                bool check = false;
                double alpha_max_degree = alpha_max.at(k)*180/M_PI;
                double vel_0_k = vel_0.at(k); double vel_f_k = vel_f.at(k);
                double acc_0_k = acc_0.at(k); double acc_f_k = acc_f.at(k);
                double deltat = 0.02; // value to increase the timestep when it does not respect the joint velocity and acceleration limits [sec]
                do
                {
                    VectorXd jointTraj_k = jointTraj.col(k);
                    //VectorXd jointTraj_k_deg = jointTraj_k*180/M_PI;
                    std::vector<double> std_jointTraj_k;
                    std::vector<double> jointVel_k; std::vector<double> jointAcc_k;
                    std_jointTraj_k.resize(jointTraj_k.size()); VectorXd::Map(&std_jointTraj_k[0], jointTraj_k.size()) = jointTraj_k;
                    std::vector<double> timestep_vec(std_jointTraj_k.size(),timestep);
                    this->getDerivative(std_jointTraj_k,timestep_vec,jointVel_k);
                    jointVel_k.at(0) = vel_0_k; jointVel_k.at(jointVel_k.size()-1) = vel_f_k;
                    this->getDerivative(jointVel_k,timestep_vec,jointAcc_k);
                    jointAcc_k.at(0) = acc_0_k; jointAcc_k.at(jointAcc_k.size()-1) = acc_f_k;
                    //double* ptr_vel = &jointVel_k[0];
                    //Eigen::Map<Eigen::VectorXd> jointVel_k_vec(ptr_vel, jointVel_k.size());
                    //VectorXd jointVel_k_deg = jointVel_k_vec*180/M_PI;
                    //double* ptr_acc = &jointAcc_k[0];
                    //Eigen::Map<Eigen::VectorXd> jointAcc_k_vec(ptr_acc, jointAcc_k.size());
                    //VectorXd jointAcc_k_deg = jointAcc_k_vec*180/M_PI;
                    //std::vector<double>::iterator max_vel_it = std::max_element(jointVel_k.begin(), jointVel_k.end(), abs_compare);
                    //double max_vel_traj_k = std::abs((*max_vel_it))*180/M_PI;
                    std::vector<double>::iterator max_acc_it = std::max_element(jointAcc_k.begin(), jointAcc_k.end(), abs_compare);
                    double max_acc_traj_k = std::abs((*max_acc_it))*180/M_PI;
                    double acc_th = alpha_max_degree - std::max(std::abs(acc_0_k),std::abs(acc_f_k))*180/M_PI;
                    if(max_acc_traj_k > acc_th){
                        timestep += deltat;
                        check=true;
                    }else{check=false;}
                }while(check && (timestep < 1.5));
            }
        }
    }

    return timestep;
}

double HUMPlanner::getDualTimeStep(hump_dual_params& tols, MatrixXd& jointTraj, int mod)
{
    int steps = jointTraj.rows();
    int n_joints = jointTraj.cols();
    double timestep;

    std::vector<double> w_max = tols.w_max;
    std::vector<double> alpha_max = tols.alpha_max;
    std::vector<double> lambda_r = tols.lambda_bounce_right;
    std::vector<double> lambda_l = tols.lambda_bounce_left;

    std::vector<double> lambda = lambda_r;
    lambda.insert(lambda.end(),lambda_l.begin(),lambda_l.end());

    double num = 0.0;
    double den = 0.0;

    for (int k =0; k < n_joints; ++k){
        double deltaTheta_k = 0.0;
        std::vector<double> diffs;
        for (int i = 1; i < steps; ++i){
            double diff = abs(jointTraj(i,k) - jointTraj(i-1,k))*180/M_PI;
            diffs.push_back(diff);
            deltaTheta_k += diff;
        }
        std::vector<double>::iterator res = std::max_element(diffs.begin(),diffs.end());
        int poss = std::distance(diffs.begin(),res);
        double deltaThetaMax = diffs.at(poss);
        double w_max_degree = w_max.at(k)*180/M_PI;

        double timestep_k_h = (lambda.at(k)/(steps-1))*log(1+deltaTheta_k); // human-like timestep
        double timestep_k_w = deltaThetaMax/w_max_degree; // max velocity timestep
        double timestep_k = timestep_k_h + timestep_k_w;
        double time_k = (steps-1)*timestep_k;
        num += lambda.at(k)*deltaTheta_k*time_k;
        den += lambda.at(k)*deltaTheta_k;

    }
    double totalTime = num/den;
    timestep = (totalTime/(steps-1));

    if(HAS_JOINT_ACCELEARATION_MAX_LIMIT){
        // check the joint maximum velocity and acceleration
        if(mod!=2 && mod!=3){ //  move or pre_approach
            std::vector<double> vel_r_0 = tols.bounds_right.vel_0;
            std::vector<double> vel_l_0 = tols.bounds_left.vel_0;
            std::vector<double> acc_r_0 = tols.bounds_right.acc_0;
            std::vector<double> acc_l_0 = tols.bounds_left.acc_0;
            std::vector<double> vel_r_f; std::vector<double> vel_l_f;
            std::vector<double> acc_r_f; std::vector<double> acc_l_f;
            if(mod==0){
              vel_r_f = tols.bounds_right.vel_f;
              vel_l_f = tols.bounds_left.vel_f;
              acc_r_f = tols.bounds_right.acc_f;
              acc_l_f = tols.bounds_left.acc_f;
            }else if(mod==1){
              vel_r_f = tols.vel_approach_right;
              vel_l_f = tols.vel_approach_left;
              acc_r_f = std::vector<double>(tols.bounds_right.acc_0.size(),0.0);
              acc_l_f = std::vector<double>(tols.bounds_left.acc_0.size(),0.0);
            }
            std::vector<double> vel_0; std::vector<double> vel_f;
            std::vector<double> acc_0; std::vector<double> acc_f;
            vel_0 = vel_r_0; vel_0.insert(vel_0.end(),vel_l_0.begin(),vel_l_0.end());
            vel_f = vel_r_f; vel_f.insert(vel_f.end(),vel_l_f.begin(),vel_l_f.end());
            acc_0 = acc_r_0; acc_0.insert(acc_0.end(),acc_l_0.begin(),acc_l_0.end());
            acc_f = acc_r_f; acc_f.insert(acc_f.end(),acc_l_f.begin(),acc_l_f.end());

            for (int k =0; k < n_joints; ++k){
                bool check = false;
                double alpha_max_degree = alpha_max.at(k)*180/M_PI;
                double vel_0_k = vel_0.at(k); double vel_f_k = vel_f.at(k);
                double acc_0_k = acc_0.at(k); double acc_f_k = acc_f.at(k);
                double deltat = 0.02; // value to increase the timestep when it does not respect the joint velocity and acceleration limits [sec]

                do
                {
                    VectorXd jointTraj_k = jointTraj.col(k);
                    //VectorXd jointTraj_k_deg = jointTraj_k*180/M_PI;
                    std::vector<double> std_jointTraj_k;
                    std::vector<double> jointVel_k; std::vector<double> jointAcc_k;
                    std_jointTraj_k.resize(jointTraj_k.size()); VectorXd::Map(&std_jointTraj_k[0], jointTraj_k.size()) = jointTraj_k;
                    std::vector<double> timestep_vec(std_jointTraj_k.size(),timestep);
                    this->getDerivative(std_jointTraj_k,timestep_vec,jointVel_k);
                    jointVel_k.at(0) = vel_0_k; jointVel_k.at(jointVel_k.size()-1) = vel_f_k;
                    this->getDerivative(jointVel_k,timestep_vec,jointAcc_k);
                    jointAcc_k.at(0) = acc_0_k; jointAcc_k.at(jointAcc_k.size()-1) = acc_f_k;
                    //double* ptr_vel = &jointVel_k[0];
                    //Eigen::Map<Eigen::VectorXd> jointVel_k_vec(ptr_vel, jointVel_k.size());
                    //VectorXd jointVel_k_deg = jointVel_k_vec*180/M_PI;
                    //double* ptr_acc = &jointAcc_k[0];
                    //Eigen::Map<Eigen::VectorXd> jointAcc_k_vec(ptr_acc, jointAcc_k.size());
                    //VectorXd jointAcc_k_deg = jointAcc_k_vec*180/M_PI;
                    //std::vector<double>::iterator max_vel_it = std::max_element(jointVel_k.begin(), jointVel_k.end(), abs_compare);
                    //double max_vel_traj_k = std::abs((*max_vel_it))*180/M_PI;
                    std::vector<double>::iterator max_acc_it = std::max_element(jointAcc_k.begin(), jointAcc_k.end(), abs_compare);
                    double max_acc_traj_k = std::abs((*max_acc_it))*180/M_PI;
                    double acc_th = alpha_max_degree - std::max(std::abs(acc_0_k),std::abs(acc_f_k))*180/M_PI;
                    if(max_acc_traj_k > acc_th){
                        timestep += deltat;
                        check=true;
                    }else{check=false;}
                }while(check && (timestep < 1.5));
            }
        }
    }


    return timestep;
}

bool HUMPlanner::setBoundaryConditions(int mov_type,hump_params &params, int steps, std::vector<double> &initPosture, std::vector<double> &finalPosture, int mod)
{

    MatrixXd fakeTraj; bool success = true;
    std::vector<double> acc_0; std::vector<double> acc_f;
    std::vector<double> vel_0; std::vector<double> vel_f;
    bool straight_line = params.mov_specs.straight_line;
    this->directTrajectoryNoBound(steps,initPosture,finalPosture,fakeTraj);

    double timestep = this->getTimeStep(params,fakeTraj,2);


    double T = timestep*steps;
    VectorXd w_max_vec = VectorXd::Map(params.w_max.data(),7);
    double w_max = w_max_vec.maxCoeff();
    VectorXd init = VectorXd::Map(initPosture.data(),7);
    VectorXd final = VectorXd::Map(finalPosture.data(),7);
    double num = (final-init).norm();
    double w_red_app_max = params.mov_specs.w_red_app_max;
    double w_red_ret_max = params.mov_specs.w_red_ret_max;
    double w_red_app = W_RED_MIN + (w_red_app_max-W_RED_MIN)*((num/T)/w_max);
    double w_red_ret = W_RED_MIN + (w_red_ret_max-W_RED_MIN)*((num/T)/w_max);

    int pre_post = 0;
    switch(mod)
    {
    case 0:// approach
        timestep = timestep*w_red_app;
        pre_post=1;
        break;
    case 1://retreat
        timestep = timestep*w_red_ret;
        pre_post=0;
        break;
    default: // approach
        timestep = timestep*w_red_app;
        pre_post=1;
        break;
    }
    T = timestep*steps;



    int arm = params.mov_specs.arm_code;
    std::vector<double> init_target = params.mov_specs.target;
    bool init_coll = params.mov_specs.coll;
    std::vector<double> init_hand_pos; this->getHandPos(arm,initPosture,init_hand_pos);
    //std::vector<double> init_hand_or; this->getHandOr(arm,initPosture,init_hand_or);
    std::vector<double> hand_tar = init_target;
    std::vector<double> final_hand_pos; this->getHandPos(arm,finalPosture,final_hand_pos);
    double delta_x = (final_hand_pos.at(0)-init_hand_pos.at(0))/(steps+1);
    double delta_y = (final_hand_pos.at(1)-init_hand_pos.at(1))/(steps+1);
    double delta_z = (final_hand_pos.at(2)-init_hand_pos.at(2))/(steps+1);


    std::vector<double> new_posture;

    if(straight_line){
        hand_tar.at(0) = hand_tar.at(0) + delta_x;
        hand_tar.at(1) = hand_tar.at(1) + delta_y;
        hand_tar.at(2) = hand_tar.at(2) + delta_z;
        params.mov_specs.target = hand_tar;
        params.mov_specs.coll = false;
        success = this->singleArmFinalPosture(mov_type,pre_post,params,initPosture, new_posture);
        params.mov_specs.target = init_target;
        params.mov_specs.coll = init_coll;

        if(success){
            std::vector<double> new_posture_ext = new_posture;
            for(size_t i=new_posture.size();i<finalPosture.size();++i){
                new_posture_ext.push_back(initPosture.at(i)+((finalPosture.at(i)-initPosture.at(i))/(steps+1)));
            }
            for (std::size_t i = 0; i<new_posture_ext.size(); ++i){
                //vel_0
                double vel_0_value =((double)(new_posture_ext.at(i)-initPosture.at(i)))/timestep;
                vel_0.push_back(vel_0_value);
                //vel_f
                double vel_f_value =((double)(new_posture_ext.at(i)-initPosture.at(i)))/timestep;
                vel_f.push_back(vel_f_value);
                //acc_0
                //double acc_0_value =(double)2*vel_0_value/timestep;
                double acc_0_value = 0.0;
                acc_0.push_back(acc_0_value);
                //acc_f
                //double acc_f_value =(double)2*vel_f_value/timestep;
                double acc_f_value = 0.0;
                acc_f.push_back(acc_f_value);
            }
        }
    }else{

        for (std::size_t i = 0; i<finalPosture.size(); ++i){
            //vel_0
            double vel_0_value =((double)5*(finalPosture.at(i)-initPosture.at(i)))/(4*T);
            vel_0.push_back(vel_0_value);
            //vel_f
            double vel_f_value =((double)10*(finalPosture.at(i)-initPosture.at(i)))/(3*T);
            vel_f.push_back(vel_f_value);
            //acc_0
            double acc_0_value =(double)4*vel_0_value/T;
            acc_0.push_back(acc_0_value);
            //acc_f
            double acc_f_value =(double)2*vel_f_value/T;
            acc_f.push_back(acc_f_value);
        }
    }

    switch(mod)
    {
    case 0:// approach
        params.vel_approach.clear(); params.acc_approach.clear();
        params.acc_approach = acc_0;
        params.vel_approach = vel_0;
        break;
    case 1://retreat
        params.bounds.vel_f.clear(); params.bounds.acc_f.clear();
        params.bounds.acc_f = acc_f;
        params.bounds.vel_f = vel_f;
        break;
    default: // approach
        params.vel_approach.clear(); params.acc_approach.clear();
        params.acc_approach = acc_0;
        params.vel_approach = vel_0;
        break;
    }


    return success;
}

bool HUMPlanner::setDualBoundaryConditions(int dual_mov_type, hump_dual_params& params, int steps, std::vector<double>& initPosture, std::vector<double>& finalPosture, int mod)
{
    MatrixXd fakeTraj; bool success = true;
    std::vector<double> acc_0; std::vector<double> acc_f;
    std::vector<double> vel_0; std::vector<double> vel_f;
    bool straight_line_right = params.mov_specs_right.straight_line;
    bool straight_line_left = params.mov_specs_left.straight_line;
    this->directTrajectoryNoBound(steps,initPosture,finalPosture,fakeTraj);

    double timestep = this->getDualTimeStep(params,fakeTraj,2);
    VectorXd w_max_vec; VectorXd init; VectorXd final;


    double T = timestep*steps;
    w_max_vec = VectorXd::Map(params.w_max.data(),params.w_max.size());
    double w_max = w_max_vec.maxCoeff();
    init = VectorXd::Map(initPosture.data(),initPosture.size());
    final = VectorXd::Map(finalPosture.data(),finalPosture.size());
    double num = (final-init).norm();
    double w_red_app_max_right = params.mov_specs_right.w_red_app_max;
    double w_red_app_max_left = params.mov_specs_left.w_red_app_max;
    double w_red_ret_max_right = params.mov_specs_right.w_red_ret_max;
    double w_red_ret_max_left = params.mov_specs_left.w_red_ret_max;
    double w_red_app_right = W_RED_MIN + (w_red_app_max_right-W_RED_MIN)*((num/T)/w_max);
    double w_red_app_left = W_RED_MIN + (w_red_app_max_left-W_RED_MIN)*((num/T)/w_max);
    double w_red_ret_right = W_RED_MIN + (w_red_ret_max_right-W_RED_MIN)*((num/T)/w_max);
    double w_red_ret_left = W_RED_MIN + (w_red_ret_max_left-W_RED_MIN)*((num/T)/w_max);

    int pre_post = 0;
    switch(mod)
    {
    case 0:// approach
        timestep = timestep*(w_red_app_right+w_red_app_left)/2;
        pre_post=1;
        break;
    case 1://retreat
        timestep = timestep*(w_red_ret_right+w_red_ret_left)/2;
        pre_post=0;
        break;
    default: // approach
        timestep = timestep*(w_red_app_right+w_red_app_left)/2;
        pre_post=1;
        break;
    }
    T = timestep*steps;


    if(straight_line_right && straight_line_left ){
        //TO DO
    }else{
        for (std::size_t i = 0; i<finalPosture.size(); ++i){
            //vel_0
            double vel_0_value =((double)5*(finalPosture.at(i)-initPosture.at(i)))/(4*T);
            vel_0.push_back(vel_0_value);
            //vel_f
            double vel_f_value =((double)10*(finalPosture.at(i)-initPosture.at(i)))/(3*T);
            vel_f.push_back(vel_f_value);
            //acc_0
            double acc_0_value =(double)4*vel_0_value/T;
            acc_0.push_back(acc_0_value);
            //acc_f
            double acc_f_value =(double)2*vel_f_value/T;
            acc_f.push_back(acc_f_value);
        }
    }

    switch(mod)
    {
    case 0:// approach
        params.vel_approach_right.clear(); params.acc_approach_right.clear();
        params.vel_approach_left.clear(); params.acc_approach_left.clear();
        params.vel_approach_right.assign(vel_0.begin(),vel_0.begin()+joints_arm+joints_hand+1);
        params.vel_approach_left.assign(vel_0.begin()+joints_arm+joints_hand+1,vel_0.end());
        params.acc_approach_right.assign(acc_0.begin(),acc_0.begin()+joints_arm+joints_hand+1);
        params.acc_approach_left.assign(acc_0.begin()+joints_arm+joints_hand+1,acc_0.end());
        break;
    case 1://retreat
        params.bounds_right.vel_f.clear(); params.bounds_right.acc_f.clear();
        params.bounds_left.vel_f.clear(); params.bounds_left.acc_f.clear();
        params.bounds_right.vel_f.assign(vel_f.begin(),vel_f.begin()+joints_arm+joints_hand+1);
        params.bounds_right.acc_f.assign(acc_f.begin(),acc_f.begin()+joints_arm+joints_hand+1);
        params.bounds_left.vel_f.assign(vel_f.begin()+joints_arm+joints_hand+1,vel_f.end());
        params.bounds_left.acc_f.assign(acc_f.begin()+joints_arm+joints_hand+1,acc_f.end());
        //std::copy(vel_f.begin(),vel_f.begin()+joints_arm+joints_hand+1,params.bounds_right.vel_f.begin());
        //std::copy(acc_f.begin(),acc_f.begin()+joints_arm+joints_hand+1,params.bounds_right.acc_f.begin());
        //std::copy(vel_f.begin()+joints_arm+joints_hand+1,vel_f.end(),params.bounds_left.vel_f.begin());
        //std::copy(acc_f.begin()+joints_arm+joints_hand+1,acc_f.end(),params.bounds_left.acc_f.begin());
        break;
    default: // approach
        params.vel_approach_right.clear(); params.acc_approach_right.clear();
        params.vel_approach_left.clear(); params.acc_approach_left.clear();
        params.vel_approach_right.assign(vel_0.begin(),vel_0.begin()+joints_arm+joints_hand+1);
        params.vel_approach_left.assign(vel_0.begin()+joints_arm+joints_hand+1,vel_0.end());
        params.acc_approach_right.assign(acc_0.begin(),acc_0.begin()+joints_arm+joints_hand+1);
        params.acc_approach_left.assign(acc_0.begin()+joints_arm+joints_hand+1,acc_0.end());
        break;
    }


    return success;
}


bool HUMPlanner::directTrajectory(int mov_type,int steps,hump_params &tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd &Traj, MatrixXd &vel_app_ret, int mod)
{
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time
    std::vector<double> vel_0;
    std::vector<double> vel_f;
    std::vector<double> acc_0;
    std::vector<double> acc_f;
    double app = 0; double ret = 0; int pre_post = 0;
    bool straight_line = tols.mov_specs.straight_line;
    bool success = true;

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
        acc_f = std::vector<double>(tols.bounds.acc_0.size(),0.0);
        break;
    case 2: // approach
        vel_0 = tols.vel_approach;
        vel_f = std::vector<double>(tols.vel_approach.size(),0.0);
        acc_0 = tols.acc_approach;
        acc_f = std::vector<double>(tols.acc_approach.size(),0.0);
        app=1;
        pre_post=1;
        break;
    case 3:// retreat
        vel_0 = std::vector<double>(tols.bounds.vel_0.size(),0.0);
        vel_f = tols.bounds.vel_f;
        acc_0 = std::vector<double>(tols.bounds.acc_0.size(),0.0);
        acc_f = tols.bounds.acc_f;
        ret=1;
        pre_post=0;
        break;
    }


    double T = timestep*steps;
    double delta = ((double)1)/steps;

    tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        tau.at(i) = tau.at(i-1)+delta;
    }
    Traj = MatrixXd::Constant(steps+1,initPosture.size(),0);
    vel_app_ret = MatrixXd::Constant(steps+1,initPosture.size(),0);


    if((app==1 || ret==1) && straight_line){
        hump_params params = tols;
        int arm = params.mov_specs.arm_code;
        std::vector<double> init_target = params.mov_specs.target;
        std::vector<double> init_hand_pos; this->getHandPos(arm,initPosture,init_hand_pos);
        //std::vector<double> init_hand_or; this->getHandOr(arm,initPosture,init_hand_or);
        std::vector<double> hand_tar = init_target;
        std::vector<double> final_hand_pos; this->getHandPos(arm,finalPosture,final_hand_pos);
        double delta_x = (final_hand_pos.at(0)-init_hand_pos.at(0))/(steps+1);
        double delta_y = (final_hand_pos.at(1)-init_hand_pos.at(1))/(steps+1);
        double delta_z = (final_hand_pos.at(2)-init_hand_pos.at(2))/(steps+1);

        hand_tar.at(0) = hand_tar.at(0) + delta_x;
        hand_tar.at(1) = hand_tar.at(1) + delta_y;
        hand_tar.at(2) = hand_tar.at(2) + delta_z;
        params.mov_specs.target = hand_tar;
        bool init_coll = params.mov_specs.coll;
        params.mov_specs.coll = false;

        std::vector<double> new_posture;
        std::vector<double> new_posture_ext;
        std::vector<double> init_posture_0;
        for (int i = 0; i <= steps;++i){
            if(i==0){
                init_posture_0 = initPosture;
            }else{
               init_posture_0 = new_posture_ext;
            }
            if(i==steps){
                success=true;
            }else{
                success = this->singleArmFinalPosture(mov_type,pre_post,params,init_posture_0, new_posture);
            }
            if(success){
                if(i!=steps){
                    new_posture_ext = new_posture;
                    for(size_t k=new_posture.size();k<finalPosture.size();++k){
                        double delta_theta_fing = (finalPosture.at(k)-initPosture.at(k))/(steps+1);
                        new_posture_ext.push_back(init_posture_0.at(k)+delta_theta_fing);
                    }
                }
                for (std::size_t j = 0; j<finalPosture.size(); ++j){
                    if(i==0){
                        Traj(i,j) = init_posture_0.at(j);
                        vel_app_ret(i,j) = vel_0.at(j);
                    }else if(i==steps){
                        Traj(i,j) = finalPosture.at(j);
                        vel_app_ret(i,j) = (finalPosture.at(j) - init_posture_0.at(j))/timestep;
                    }else{
                        Traj(i,j) = new_posture_ext.at(j);
                        vel_app_ret(i,j) = (new_posture_ext.at(j) - init_posture_0.at(j))/timestep;
                    }
                }
                hand_tar.at(0) = hand_tar.at(0) + delta_x;
                hand_tar.at(1) = hand_tar.at(1) + delta_y;
                hand_tar.at(2) = hand_tar.at(2) + delta_z;
                params.mov_specs.target = hand_tar;
            }else{
                break;
            }
        }// for loop
        params.mov_specs.target = init_target;
        params.mov_specs.coll = init_coll;
    }else{
        for (int i = 0; i <= steps;++i){
            for (std::size_t j = 0; j<initPosture.size(); ++j){
                Traj(i,j) = initPosture.at(j) +
                        (1-app)*(1-ret)*(finalPosture.at(j) - initPosture.at(j))*(10*pow(tau.at(i),3)-15*pow(tau.at(i),4)+6*pow(tau.at(i),5))+
                        app*0.25*(finalPosture.at(j) - initPosture.at(j))*(5*tau.at(i)-pow(tau.at(i),5))+
                        ret*0.33*(finalPosture.at(j) - initPosture.at(j))*(5*pow(tau.at(i),4)-2*pow(tau.at(i),5))+
                        (1-app)*(1-ret)*vel_0.at(j)*T*(tau.at(i)-6*pow(tau.at(i),3)+8*pow(tau.at(i),4)-3*pow(tau.at(i),5))+
                        (1-app)*(1-ret)*vel_f.at(j)*T*(-4*pow(tau.at(i),3)+7*pow(tau.at(i),4)-3*pow(tau.at(i),5))+
                        (1-app)*(1-ret)*0.5*acc_0.at(j)*pow(T,2)*(pow(tau.at(i),2)-3*pow(tau.at(i),3)+3*pow(tau.at(i),4)-pow(tau.at(i),5))+
                        (1-app)*(1-ret)*0.5*acc_f.at(j)*pow(T,2)*(pow(tau.at(i),3)-2*pow(tau.at(i),4)+pow(tau.at(i),5));

            }
        }
    }
    return success;
}
bool HUMPlanner::directDualTrajectory(int dual_mov_type, int steps, hump_dual_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Traj, MatrixXd &vel_app_ret, int mod)
{
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time
    std::vector<double> vel_right_0; std::vector<double> vel_left_0;
    std::vector<double> vel_right_f; std::vector<double> vel_left_f;
    std::vector<double> acc_right_0; std::vector<double> acc_left_0;
    std::vector<double> acc_right_f; std::vector<double> acc_left_f;

    std::vector<double> vel_0; std::vector<double> vel_f;
    std::vector<double> acc_0; std::vector<double> acc_f;

    double app = 0; double ret = 0; int pre_post = 0;
    //bool straight_line = tols.mov_specs.straight_line;
    bool success = true;

    switch(mod){
    case 0: // move
        vel_right_0 = tols.bounds_right.vel_0; vel_left_0 = tols.bounds_left.vel_0;
        vel_right_f = tols.bounds_right.vel_f; vel_left_f = tols.bounds_left.vel_f;
        acc_right_0 = tols.bounds_right.acc_0; acc_left_0 = tols.bounds_left.acc_0;
        acc_right_f = tols.bounds_right.acc_f; acc_left_f = tols.bounds_left.acc_f;
        break;
    case 1:// pre_approach
        vel_right_0 = tols.bounds_right.vel_0; vel_left_0 = tols.bounds_left.vel_0;
        vel_right_f = tols.vel_approach_right; vel_left_f = tols.vel_approach_left;
        acc_right_0 = tols.bounds_right.acc_0; acc_left_0 = tols.bounds_left.acc_0;
        acc_right_f = std::vector<double>(tols.bounds_right.acc_0.size(),0.0); acc_left_f = std::vector<double>(tols.bounds_left.acc_0.size(),0.0);
        break;
    case 2: // approach
        vel_right_0 = tols.vel_approach_right; vel_left_0 = tols.vel_approach_left;
        vel_right_f = std::vector<double>(tols.vel_approach_right.size(),0.0); vel_left_f = std::vector<double>(tols.vel_approach_left.size(),0.0);
        acc_right_0 = tols.acc_approach_right; acc_left_0 = tols.acc_approach_left;
        acc_right_f = std::vector<double>(tols.acc_approach_right.size(),0.0); acc_left_f = std::vector<double>(tols.acc_approach_left.size(),0.0);
        app=1;
        pre_post=1;
        break;
    case 3:// retreat
        vel_right_0 = std::vector<double>(tols.bounds_right.vel_0.size(),0.0); vel_left_0 = std::vector<double>(tols.bounds_left.vel_0.size(),0.0);
        vel_right_f = tols.bounds_right.vel_f; vel_left_f = tols.bounds_left.vel_f;
        acc_right_0 = std::vector<double>(tols.bounds_right.acc_0.size(),0.0); acc_left_0 = std::vector<double>(tols.bounds_left.acc_0.size(),0.0);
        acc_right_f = tols.bounds_right.acc_f; acc_left_f = tols.bounds_left.acc_f;
        ret=1;
        pre_post=0;
        break;
    }
    vel_0 = vel_right_0; vel_0.insert(vel_0.end(),vel_left_0.begin(),vel_left_0.end());
    vel_f = vel_right_f; vel_f.insert(vel_f.end(),vel_left_f.begin(),vel_left_f.end());
    acc_0 = acc_right_0; acc_0.insert(acc_0.end(),acc_left_0.begin(),acc_left_0.end());
    acc_f = acc_right_f; acc_f.insert(acc_f.end(),acc_left_f.begin(),acc_left_f.end());

    double T = timestep*steps;
    double delta = ((double)1)/steps;

    tau.at(0)=0.0;
    for (int i = 1; i <= steps; ++i){
        tau.at(i) = tau.at(i-1)+delta;
    }
    Traj = MatrixXd::Constant(steps+1,initPosture.size(),0);
    vel_app_ret = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i <= steps;++i){
        for (std::size_t j = 0; j < initPosture.size(); ++j){
            Traj(i,j) = initPosture.at(j) +
                    (1-app)*(1-ret)*(finalPosture.at(j) - initPosture.at(j))*(10*pow(tau.at(i),3)-15*pow(tau.at(i),4)+6*pow(tau.at(i),5))+
                    app*0.25*(finalPosture.at(j) - initPosture.at(j))*(5*tau.at(i)-pow(tau.at(i),5))+
                    ret*0.33*(finalPosture.at(j) - initPosture.at(j))*(5*pow(tau.at(i),4)-2*pow(tau.at(i),5))+
                    (1-app)*(1-ret)*vel_0.at(j)*T*(tau.at(i)-6*pow(tau.at(i),3)+8*pow(tau.at(i),4)-3*pow(tau.at(i),5))+
                    (1-app)*(1-ret)*vel_f.at(j)*T*(-4*pow(tau.at(i),3)+7*pow(tau.at(i),4)-3*pow(tau.at(i),5))+
                    (1-app)*(1-ret)*0.5*acc_0.at(j)*pow(T,2)*(pow(tau.at(i),2)-3*pow(tau.at(i),3)+3*pow(tau.at(i),4)-pow(tau.at(i),5))+
                    (1-app)*(1-ret)*0.5*acc_f.at(j)*pow(T,2)*(pow(tau.at(i),3)-2*pow(tau.at(i),4)+pow(tau.at(i),5));
        }
    }
    return success;
}

void HUMPlanner::directTrajectoryNoBound(int steps,std::vector<double>& initPosture, std::vector<double>& finalPosture, MatrixXd &Traj)
{

    std::vector<double> tau = std::vector<double>(steps+1); // normalized time

    double delta = ((double)1)/steps;

    tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        tau.at(i) = tau.at(i-1)+delta;
    }
    Traj = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i <= steps;++i){
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            Traj(i,j) = initPosture.at(j) +
                    (finalPosture.at(j) - initPosture.at(j))*
                    (10*pow(tau.at(i),3)-15*pow(tau.at(i),4)+6*pow(tau.at(i),5));

        }
    }

}

bool HUMPlanner::directVelocity(int steps,hump_params &tols, std::vector<double> &initPosture, std::vector<double> &finalPosture,double timestep, MatrixXd &Vel, MatrixXd &vel_app_ret,int mod)
{

    std::vector<double> tau = std::vector<double>(steps+1); // normalized time
    std::vector<double> vel_0;
    std::vector<double> vel_f;
    std::vector<double> acc_0;
    std::vector<double> acc_f;
    double app = 0; double ret = 0;
    bool success = true;
    bool straight_line = tols.mov_specs.straight_line;

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
        acc_f = std::vector<double>(tols.bounds.acc_0.size(),0.0);
        break;
    case 2: // approach
        vel_0 = tols.vel_approach;
        vel_f = std::vector<double>(tols.vel_approach.size(),0.0);
        acc_0 = tols.acc_approach;
        acc_f = std::vector<double>(tols.acc_approach.size(),0.0);
        app=1;
        break;
    case 3:// retreat
        vel_0 = std::vector<double>(tols.bounds.vel_0.size(),0.0);
        vel_f = tols.bounds.vel_f;
        acc_0 = std::vector<double>(tols.bounds.acc_0.size(),0.0);
        acc_f = tols.bounds.acc_f;
        ret=1;
        break;
    }



    double T = timestep*steps;
    double delta = ((double)1)/steps;

    tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        tau.at(i) = tau.at(i-1)+delta;
    }
    Vel = MatrixXd::Constant(steps+1,initPosture.size(),0);

    if((app==1 || ret==1) && straight_line){
        for (int i = 0; i <= steps;++i){
            for (std::size_t j = 0; j<initPosture.size(); ++j){
                if((i==steps) && (app==1)){
                    Vel(i,j) =  0.0;
                }else if((i==0) && (ret==1)){
                    Vel(i,j) =  0.0;
                }else{
                    Vel(i,j) =  vel_app_ret(i,j);
                }
            }
        }
    }else{
        for (int i = 0; i <= steps;++i){
            for (std::size_t j = 0; j<initPosture.size(); ++j){
                Vel(i,j) = (1-app)*(1-ret)*(30/T)*(finalPosture.at(j) - initPosture.at(j))*
                        (pow(tau.at(i),2)-2*pow(tau.at(i),3)+pow(tau.at(i),4))+
                        (1-app)*vel_0.at(j)*(1-18*pow(tau.at(i),2)+32*pow(tau.at(i),3)-15*pow(tau.at(i),4)) + app*vel_0.at(j)*(1-pow(tau.at(i),4)) +
                        (1-ret)*vel_f.at(j)*(-12*pow(tau.at(i),2)+28*pow(tau.at(i),3)-15*pow(tau.at(i),4)) + ret*vel_f.at(j)*(2*pow(tau.at(i),3)-pow(tau.at(i),4)) +
                        (1-app)*(1-ret)*0.5*acc_0.at(j)*T*(2*tau.at(i)-9*pow(tau.at(i),2)+12*pow(tau.at(i),3)-5*pow(tau.at(i),4))+
                        (1-app)*(1-ret)*0.5*acc_f.at(j)*T*(3*pow(tau.at(i),2)-8*pow(tau.at(i),3)+5*pow(tau.at(i),4));

            }
        }
    }
    return success;
}

bool HUMPlanner::directDualVelocity(int steps, hump_dual_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Vel, MatrixXd &vel_app_ret, int mod)
{
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time
    std::vector<double> vel_right_0; std::vector<double> vel_left_0;
    std::vector<double> vel_right_f; std::vector<double> vel_left_f;
    std::vector<double> acc_right_0; std::vector<double> acc_left_0;
    std::vector<double> acc_right_f; std::vector<double> acc_left_f;

    std::vector<double> vel_0; std::vector<double> vel_f;
    std::vector<double> acc_0; std::vector<double> acc_f;

    double app = 0; double ret = 0; int pre_post = 0;
    //bool straight_line = tols.mov_specs.straight_line;
    bool success = true;

    switch(mod){
    case 0: // move
        vel_right_0 = tols.bounds_right.vel_0; vel_left_0 = tols.bounds_left.vel_0;
        vel_right_f = tols.bounds_right.vel_f; vel_left_f = tols.bounds_left.vel_f;
        acc_right_0 = tols.bounds_right.acc_0; acc_left_0 = tols.bounds_left.acc_0;
        acc_right_f = tols.bounds_right.acc_f; acc_left_f = tols.bounds_left.acc_f;
        break;
    case 1:// pre_approach
        vel_right_0 = tols.bounds_right.vel_0; vel_left_0 = tols.bounds_left.vel_0;
        vel_right_f = tols.vel_approach_right; vel_left_f = tols.vel_approach_left;
        acc_right_0 = tols.bounds_right.acc_0; acc_left_0 = tols.bounds_left.acc_0;
        acc_right_f = std::vector<double>(tols.bounds_right.acc_0.size(),0.0); acc_left_f = std::vector<double>(tols.bounds_left.acc_0.size(),0.0);
        break;
    case 2: // approach
        vel_right_0 = tols.vel_approach_right; vel_left_0 = tols.vel_approach_left;
        vel_right_f = std::vector<double>(tols.vel_approach_right.size(),0.0); vel_left_f = std::vector<double>(tols.vel_approach_left.size(),0.0);
        acc_right_0 = tols.acc_approach_right; acc_left_0 = tols.acc_approach_left;
        acc_right_f = std::vector<double>(tols.acc_approach_right.size(),0.0); acc_left_f = std::vector<double>(tols.acc_approach_left.size(),0.0);
        app=1;
        pre_post=1;
        break;
    case 3:// retreat
        vel_right_0 = std::vector<double>(tols.bounds_right.vel_0.size(),0.0); vel_left_0 = std::vector<double>(tols.bounds_left.vel_0.size(),0.0);
        vel_right_f = tols.bounds_right.vel_f; vel_left_f = tols.bounds_left.vel_f;
        acc_right_0 = std::vector<double>(tols.bounds_right.acc_0.size(),0.0); acc_left_0 = std::vector<double>(tols.bounds_left.acc_0.size(),0.0);
        acc_right_f = tols.bounds_right.acc_f; acc_left_f = tols.bounds_left.acc_f;
        ret=1;
        pre_post=0;
        break;
    }
    vel_0 = vel_right_0; vel_0.insert(vel_0.end(),vel_left_0.begin(),vel_left_0.end());
    vel_f = vel_right_f; vel_f.insert(vel_f.end(),vel_left_f.begin(),vel_left_f.end());
    acc_0 = acc_right_0; acc_0.insert(acc_0.end(),acc_left_0.begin(),acc_left_0.end());
    acc_f = acc_right_f; acc_f.insert(acc_f.end(),acc_left_f.begin(),acc_left_f.end());

    double T = timestep*steps;
    double delta = ((double)1)/steps;

    tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        tau.at(i) = tau.at(i-1)+delta;
    }
    Vel = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i <= steps;++i){
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            Vel(i,j) = (1-app)*(1-ret)*(30/T)*(finalPosture.at(j) - initPosture.at(j))*
                    (pow(tau.at(i),2)-2*pow(tau.at(i),3)+pow(tau.at(i),4))+
                    (1-app)*vel_0.at(j)*(1-18*pow(tau.at(i),2)+32*pow(tau.at(i),3)-15*pow(tau.at(i),4)) + app*vel_0.at(j)*(1-pow(tau.at(i),4)) +
                    (1-ret)*vel_f.at(j)*(-12*pow(tau.at(i),2)+28*pow(tau.at(i),3)-15*pow(tau.at(i),4)) + ret*vel_f.at(j)*(2*pow(tau.at(i),3)-pow(tau.at(i),4)) +
                    (1-app)*(1-ret)*0.5*acc_0.at(j)*T*(2*tau.at(i)-9*pow(tau.at(i),2)+12*pow(tau.at(i),3)-5*pow(tau.at(i),4))+
                    (1-app)*(1-ret)*0.5*acc_f.at(j)*T*(3*pow(tau.at(i),2)-8*pow(tau.at(i),3)+5*pow(tau.at(i),4));

        }
    }
    return success;
}

bool HUMPlanner::directAcceleration(int steps,hump_params &tols, std::vector<double> &initPosture, std::vector<double> &finalPosture, double timestep, MatrixXd &Acc, MatrixXd &vel_app_ret, int mod)
{

    std::vector<double> tau = std::vector<double>(steps+1); // normalized time
    std::vector<double> vel_0;
    std::vector<double> vel_f;
    std::vector<double> acc_0;
    std::vector<double> acc_f;
    double app = 0; double ret = 0;
    bool straight_line = tols.mov_specs.straight_line;
    bool success = true;

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
        acc_f = std::vector<double>(tols.bounds.acc_0.size(),0.0);
        break;
    case 2: // approach
        vel_0 = tols.vel_approach;
        vel_f = std::vector<double>(tols.vel_approach.size(),0.0);
        acc_0 = tols.acc_approach;
        acc_f = std::vector<double>(tols.acc_approach.size(),0.0);
        app=1;
        break;
    case 3:// retreat
        vel_0 = std::vector<double>(tols.bounds.vel_0.size(),0.0);
        vel_f = tols.bounds.vel_f;
        acc_0 = std::vector<double>(tols.bounds.acc_0.size(),0.0);
        acc_f = tols.bounds.acc_f;
        ret=1;
        break;
    }



    double T = timestep*steps;
    double delta = ((double)1)/steps;

    tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        tau.at(i) = tau.at(i-1)+delta;
    }
    Acc = MatrixXd::Constant(steps+1,initPosture.size(),0);

    if((app==1 || ret==1) && straight_line){
        for (int i = 0; i <= steps;++i){
            for (std::size_t j = 0; j<initPosture.size(); ++j){
                if(app==1){
                    if(i==0){
                        Acc(i,j) = 0.0;
                    }else if(i==steps){
                        Acc(i,j) = (-vel_app_ret(i,j))/timestep;
                    }else{
                        Acc(i,j) = (vel_app_ret(i,j) - vel_app_ret(i+1,j))/timestep;
                    }
                }else if(ret==1){
                    if(i==0){
                        Acc(i,j) = vel_app_ret(i,j)/timestep;
                    }else if(i==steps){
                        Acc(i,j) = 0.0;
                    }else{
                       Acc(i,j) = (vel_app_ret(i,j) - vel_app_ret(i+1,j))/timestep;
                    }
                }
            }
        }
    }else{
        for (int i = 0; i <= steps;++i){
            for (std::size_t j = 0; j<initPosture.size(); ++j){
                Acc(i,j) = (1-app)*(1-ret)*(60/pow(T,2))*(finalPosture.at(j) - initPosture.at(j))*
                        (tau.at(i)-3*pow(tau.at(i),2)+2*pow(tau.at(i),3))+
                        (1-app)*(1-ret)*12*(vel_0.at(j)/T)*(-3*tau.at(i)+8*pow(tau.at(i),2)-5*pow(tau.at(i),3))+
                        (1-app)*(1-ret)*12*(vel_f.at(j)/T)*(-2*tau.at(i)+7*pow(tau.at(i),2)-5*pow(tau.at(i),3))+
                        (1-app)*acc_0.at(j)*(1-9*tau.at(i)+18*pow(tau.at(i),2)-10*pow(tau.at(i),3))+ app*acc_0.at(j)*(-pow(tau.at(i),3))+
                        (1-ret)*acc_f.at(j)*(3*tau.at(i)-12*pow(tau.at(i),2)+10*pow(tau.at(i),3))+ret*acc_f.at(j)*(3*pow(tau.at(i),2)-2*pow(tau.at(i),3));

            }
        }
    }
    return success;
}

bool HUMPlanner::directDualAcceleration(int steps,hump_dual_params& tols, std::vector<double>& initPosture, std::vector<double>& finalPosture, double timestep, MatrixXd& Acc,MatrixXd &vel_app_ret,int mod)
{
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time
    std::vector<double> vel_right_0; std::vector<double> vel_left_0;
    std::vector<double> vel_right_f; std::vector<double> vel_left_f;
    std::vector<double> acc_right_0; std::vector<double> acc_left_0;
    std::vector<double> acc_right_f; std::vector<double> acc_left_f;

    std::vector<double> vel_0; std::vector<double> vel_f;
    std::vector<double> acc_0; std::vector<double> acc_f;

    double app = 0; double ret = 0; int pre_post = 0;
    //bool straight_line = tols.mov_specs.straight_line;
    bool success = true;

    switch(mod){
    case 0: // move
        vel_right_0 = tols.bounds_right.vel_0; vel_left_0 = tols.bounds_left.vel_0;
        vel_right_f = tols.bounds_right.vel_f; vel_left_f = tols.bounds_left.vel_f;
        acc_right_0 = tols.bounds_right.acc_0; acc_left_0 = tols.bounds_left.acc_0;
        acc_right_f = tols.bounds_right.acc_f; acc_left_f = tols.bounds_left.acc_f;
        break;
    case 1:// pre_approach
        vel_right_0 = tols.bounds_right.vel_0; vel_left_0 = tols.bounds_left.vel_0;
        vel_right_f = tols.vel_approach_right; vel_left_f = tols.vel_approach_left;
        acc_right_0 = tols.bounds_right.acc_0; acc_left_0 = tols.bounds_left.acc_0;
        acc_right_f = std::vector<double>(tols.bounds_right.acc_0.size(),0.0); acc_left_f = std::vector<double>(tols.bounds_left.acc_0.size(),0.0);
        break;
    case 2: // approach
        vel_right_0 = tols.vel_approach_right; vel_left_0 = tols.vel_approach_left;
        vel_right_f = std::vector<double>(tols.vel_approach_right.size(),0.0); vel_left_f = std::vector<double>(tols.vel_approach_left.size(),0.0);
        acc_right_0 = tols.acc_approach_right; acc_left_0 = tols.acc_approach_left;
        acc_right_f = std::vector<double>(tols.acc_approach_right.size(),0.0); acc_left_f = std::vector<double>(tols.acc_approach_left.size(),0.0);
        app=1;
        pre_post=1;
        break;
    case 3:// retreat
        vel_right_0 = std::vector<double>(tols.bounds_right.vel_0.size(),0.0); vel_left_0 = std::vector<double>(tols.bounds_left.vel_0.size(),0.0);
        vel_right_f = tols.bounds_right.vel_f; vel_left_f = tols.bounds_left.vel_f;
        acc_right_0 = std::vector<double>(tols.bounds_right.acc_0.size(),0.0); acc_left_0 = std::vector<double>(tols.bounds_left.acc_0.size(),0.0);
        acc_right_f = tols.bounds_right.acc_f; acc_left_f = tols.bounds_left.acc_f;
        ret=1;
        pre_post=0;
        break;
    }
    vel_0 = vel_right_0; vel_0.insert(vel_0.end(),vel_left_0.begin(),vel_left_0.end());
    vel_f = vel_right_f; vel_f.insert(vel_f.end(),vel_left_f.begin(),vel_left_f.end());
    acc_0 = acc_right_0; acc_0.insert(acc_0.end(),acc_left_0.begin(),acc_left_0.end());
    acc_f = acc_right_f; acc_f.insert(acc_f.end(),acc_left_f.begin(),acc_left_f.end());

    double T = timestep*steps;
    double delta = ((double)1)/steps;

    tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        tau.at(i) = tau.at(i-1)+delta;
    }
    Acc = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i <= steps;++i){
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            Acc(i,j) = (1-app)*(1-ret)*(60/pow(T,2))*(finalPosture.at(j) - initPosture.at(j))*
                    (tau.at(i)-3*pow(tau.at(i),2)+2*pow(tau.at(i),3))+
                    (1-app)*(1-ret)*12*(vel_0.at(j)/T)*(-3*tau.at(i)+8*pow(tau.at(i),2)-5*pow(tau.at(i),3))+
                    (1-app)*(1-ret)*12*(vel_f.at(j)/T)*(-2*tau.at(i)+7*pow(tau.at(i),2)-5*pow(tau.at(i),3))+
                    (1-app)*acc_0.at(j)*(1-9*tau.at(i)+18*pow(tau.at(i),2)-10*pow(tau.at(i),3))+ app*acc_0.at(j)*(-pow(tau.at(i),3))+
                    (1-ret)*acc_f.at(j)*(3*tau.at(i)-12*pow(tau.at(i),2)+10*pow(tau.at(i),3))+ret*acc_f.at(j)*(3*pow(tau.at(i),2)-2*pow(tau.at(i),3));

        }
    }
    return success;
}

void HUMPlanner::backForthTrajectory(int steps, std::vector<double> &initPosture, std::vector<double> &bouncePosture, MatrixXd &Traj)
{

    //int steps = tols.steps;
    //std::vector<double> time = std::vector<double>(steps+1); // time
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time



    double delta = ((double)1)/steps;

    tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        tau.at(i) = tau.at(i-1)+delta;
    }
    Traj = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i<=steps;++i){
        double ttau = tau.at(i);
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            Traj(i,j) = ((ttau*(1-ttau))/(TB*(1-TB)))*(bouncePosture.at(j) - initPosture.at(j))* pow(sin(M_PI*pow(ttau,PHI)),2);
        }
    }
}

void HUMPlanner::backForthVelocity(int steps, std::vector<double> &initPosture, std::vector<double> &bouncePosture, double timestep, MatrixXd &Vel)
{
    //int steps = tols.steps;
    //std::vector<double> time = std::vector<double>(steps+1); // time
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time


    double T = timestep*steps;
    double delta = ((double)1)/steps;

    tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        tau.at(i) = tau.at(i-1)+delta;
    }
    Vel = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i<=steps;++i){
        double ttau = tau.at(i);
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            Vel(i,j) = ((ttau*(1-ttau))/(T*(TB*(1-TB))))*(bouncePosture.at(j) - initPosture.at(j))*((1-2*ttau)*pow(sin(M_PI*pow(ttau,PHI)),2)+
                                                                                                (1-ttau)*M_PI*PHI*pow(ttau,PHI)*sin(2*M_PI*pow(ttau,PHI)));
        }
    }
}

void HUMPlanner::backForthAcceleration(int steps, std::vector<double> &initPosture, std::vector<double> &bouncePosture, double timestep, MatrixXd &Acc)
{
    //int steps = tols.steps;
    //std::vector<double> time = std::vector<double>(steps+1); // time
    std::vector<double> tau = std::vector<double>(steps+1); // normalized time


    double T = timestep*steps;
    double delta = ((double)1)/steps;

    tau.at(0)=0.0;
    for (int i = 1; i<=steps; ++i){
        tau.at(i) = tau.at(i-1)+delta;
    }
    Acc = MatrixXd::Constant(steps+1,initPosture.size(),0);

    for (int i = 0; i<=steps;++i){
        double ttau = tau.at(i);
        for (std::size_t j = 0; j<initPosture.size(); ++j){
            Acc(i,j) = ((2*(ttau*(1-ttau)))/(pow(T,2)*(TB*(1-TB))))*(bouncePosture.at(j) - initPosture.at(j))*(pow(M_PI,2)*pow(PHI,2)*pow(ttau,((2*PHI)-1))*(1-ttau)*cos(2*M_PI*pow(ttau,PHI))
                                                                                                -pow(sin(M_PI*pow(ttau,2)),2)
                                                                                                +M_PI*PHI*pow(ttau,(PHI-1))*(1-2*ttau)*sin(2*M_PI*pow(ttau,PHI)));
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

double HUMPlanner::getTrajectory(int mov_type,int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel_app_ret, bool &success,int mod)
{
    double timestep; MatrixXd traj_no_bound;
    this->directTrajectoryNoBound(steps,initPosture,finalPosture,traj_no_bound);
    timestep = this->getTimeStep(tols,traj_no_bound,mod);
    if((mod==2)||(mod==3)){ // approach or retreat
        VectorXd w_max_vec = VectorXd::Map(tols.w_max.data(),joints_arm);
        double w_max = w_max_vec.maxCoeff();
        VectorXd init = VectorXd::Map(initPosture.data(),joints_arm);
        VectorXd final = VectorXd::Map(finalPosture.data(),joints_arm);
        double num = (final-init).norm();
        double T = timestep*steps;
        double w_red_app_max = tols.mov_specs.w_red_app_max;
        double w_red_ret_max = tols.mov_specs.w_red_ret_max;
        double w_red_app = W_RED_MIN + (w_red_app_max-W_RED_MIN)*((num/T)/w_max);
        double w_red_ret = W_RED_MIN + (w_red_ret_max-W_RED_MIN)*((num/T)/w_max);
        if(mod==2){timestep = timestep*w_red_app;}
        if(mod==3){timestep = timestep*w_red_ret;}
    }
    success = this->directTrajectory(mov_type,steps,tols,initPosture,finalPosture,timestep,traj,vel_app_ret,mod);

    return timestep;
}

double HUMPlanner::getDualTrajectory(int dual_mov_type,int steps,hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel_app_ret,bool &success, int mod)
{
    double timestep; MatrixXd traj_no_bound;
    this->directTrajectoryNoBound(steps,initPosture,finalPosture,traj_no_bound);
    timestep = this->getDualTimeStep(tols,traj_no_bound,mod);
    if((mod==2)||(mod==3)){ // approach or retreat
        VectorXd w_max_vec = VectorXd::Map(tols.w_max.data(),joints_arm+joints_arm);
        double w_max = w_max_vec.maxCoeff();
        VectorXd init = VectorXd::Map(initPosture.data(),joints_arm+joints_arm);
        VectorXd final = VectorXd::Map(finalPosture.data(),joints_arm+joints_arm);
        double num = (final-init).norm();
        double T = timestep*steps;

        double w_red_app_max_right = tols.mov_specs_right.w_red_app_max; double w_red_app_max_left = tols.mov_specs_left.w_red_app_max;
        double w_red_ret_max_right = tols.mov_specs_right.w_red_ret_max; double w_red_ret_max_left = tols.mov_specs_left.w_red_ret_max;
        double w_red_app_right = W_RED_MIN + (w_red_app_max_right-W_RED_MIN)*((num/T)/w_max); double w_red_app_left = W_RED_MIN + (w_red_app_max_left-W_RED_MIN)*((num/T)/w_max);
        double w_red_ret_right = W_RED_MIN + (w_red_ret_max_right-W_RED_MIN)*((num/T)/w_max); double w_red_ret_left = W_RED_MIN + (w_red_ret_max_left-W_RED_MIN)*((num/T)/w_max);

        if(mod==2){timestep = timestep*(w_red_app_right+w_red_app_left)/2;}
        if(mod==3){timestep = timestep*(w_red_ret_right+w_red_ret_left)/2;}
    }
    success = this->directDualTrajectory(dual_mov_type,steps,tols,initPosture,finalPosture,timestep,traj,vel_app_ret,mod);

    return timestep;
}

double HUMPlanner::getTrajectory(int mov_type,int steps,hump_params &tols,std::vector<double> initPosture,
                                 std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj, MatrixXd &vel_app_ret, bool &success, int mod)
{

    double timestep;
    MatrixXd d_traj_no_bound;
    this->directTrajectoryNoBound(steps,initPosture,finalPosture,d_traj_no_bound);
    timestep = this->getTimeStep(tols,d_traj_no_bound,mod);

    MatrixXd dTraj;
    MatrixXd bTraj;
    success = this->directTrajectory(mov_type,steps,tols,initPosture,finalPosture,timestep,dTraj,vel_app_ret,mod);
    this->backForthTrajectory(steps,initPosture,bouncePosture,bTraj);
    this->computeMovement(dTraj,bTraj,traj);

    return timestep;
}

double HUMPlanner::getDualTrajectory(int dual_mov_type,int steps,hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture,
                     MatrixXd &traj,MatrixXd &vel_app_ret,bool &success,int mod)
{
    double timestep;
    MatrixXd d_traj_no_bound;
    this->directTrajectoryNoBound(steps,initPosture,finalPosture,d_traj_no_bound);
    timestep = this->getDualTimeStep(tols,d_traj_no_bound,mod);

    MatrixXd dTraj;
    MatrixXd bTraj;
    success = this->directDualTrajectory(dual_mov_type,steps,tols,initPosture,finalPosture,timestep,dTraj,vel_app_ret,mod);
    this->backForthTrajectory(steps,initPosture,bouncePosture,bTraj);
    this->computeMovement(dTraj,bTraj,traj);

    return timestep;
}


double HUMPlanner::getVelocity(int mov_type,int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &vel_app_ret,bool &success,int mod)
{

    double timestep = this->getTrajectory(mov_type,steps,tols,initPosture,finalPosture,traj,vel_app_ret,success,mod);

    this->directVelocity(steps,tols,initPosture,finalPosture,timestep,vel,vel_app_ret,mod);

    return timestep;

}

double HUMPlanner::getDualVelocity(int dual_mov_type, int steps, hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &vel_app_ret,bool &success, int mod)
{
    double timestep = this->getDualTrajectory(dual_mov_type,steps,tols,initPosture,finalPosture,traj,vel_app_ret,success,mod);
    this->directDualVelocity(steps,tols,initPosture,finalPosture,timestep,vel,vel_app_ret,mod);
    return timestep;
}


double HUMPlanner::getVelocity(int mov_type,int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture,
                               MatrixXd &traj, MatrixXd &vel,MatrixXd &vel_app_ret,bool &success,int mod)
{


    double timestep = this->getTrajectory(mov_type,steps,tols,initPosture,finalPosture,bouncePosture,traj,vel_app_ret,success,mod);

    MatrixXd dVel;
    MatrixXd bVel;

    this->directVelocity(steps,tols,initPosture,finalPosture,timestep,dVel,vel_app_ret,mod);
    this->backForthVelocity(steps,initPosture,bouncePosture,timestep,bVel);
    this->computeMovement(dVel,bVel,vel);

    return timestep;

}

double HUMPlanner::getDualVelocity(int dual_mov_type, int steps, hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture,
                   MatrixXd &traj, MatrixXd &vel, MatrixXd &vel_app_ret, bool &success, int mod)
{
    double timestep = this->getDualTrajectory(dual_mov_type,steps,tols,initPosture,finalPosture,bouncePosture,traj,vel_app_ret,success,mod);

    MatrixXd dVel;
    MatrixXd bVel;

    this->directDualVelocity(steps,tols,initPosture,finalPosture,timestep,dVel,vel_app_ret,mod);
    this->backForthVelocity(steps,initPosture,bouncePosture,timestep,bVel);
    this->computeMovement(dVel,bVel,vel);

    return timestep;
}

double HUMPlanner::getAcceleration(int mov_type,int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, bool &success, int mod)
{
    MatrixXd vel_app_ret;
    double timestep = this->getVelocity(mov_type,steps,tols,initPosture,finalPosture,traj,vel,vel_app_ret,success,mod);

    this->directAcceleration(steps,tols,initPosture,finalPosture,timestep,acc,vel_app_ret,mod);

    return timestep;

}

double HUMPlanner::getDualAcceleration(int dual_mov_type, int steps, hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, bool &success, int mod)
{
    MatrixXd vel_app_ret;
    double timestep = this->getDualVelocity(dual_mov_type,steps,tols,initPosture,finalPosture,traj,vel,vel_app_ret,success,mod);

    this->directDualAcceleration(steps,tols,initPosture,finalPosture,timestep,acc,vel_app_ret,mod);

    return timestep;
}

double HUMPlanner::getAcceleration(int mov_type,int steps,hump_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, bool &success, int mod)
{
    MatrixXd vel_app_ret;
    double timestep = this->getVelocity(mov_type,steps,tols,initPosture,finalPosture,bouncePosture,traj,vel,vel_app_ret,success,mod);

    MatrixXd dAcc;
    MatrixXd bAcc;

    this->directAcceleration(steps,tols,initPosture,finalPosture,timestep,dAcc,vel_app_ret,mod);
    this->backForthAcceleration(steps,initPosture,bouncePosture,timestep,bAcc);
    this->computeMovement(dAcc,bAcc,acc);

    return timestep;
}

double HUMPlanner::getDualAcceleration(int dual_mov_type, int steps, hump_dual_params &tols, std::vector<double> initPosture, std::vector<double> finalPosture, std::vector<double> bouncePosture, MatrixXd &traj, MatrixXd &vel, MatrixXd &acc, bool &success, int mod)
{
    MatrixXd vel_app_ret;
    double timestep = this->getDualVelocity(dual_mov_type,steps,tols,initPosture,finalPosture,bouncePosture,traj,vel,vel_app_ret,success,mod);

    MatrixXd dAcc;
    MatrixXd bAcc;

    this->directDualAcceleration(steps,tols,initPosture,finalPosture,timestep,dAcc,vel_app_ret,mod);
    this->backForthAcceleration(steps,initPosture,bouncePosture,timestep,bAcc);
    this->computeMovement(dAcc,bAcc,acc);

    return timestep;
}


planning_result_ptr HUMPlanner::plan_pick(hump_params &params, std::vector<double> initPosture)
{
    planning_result_ptr res;
    res.reset(new planning_result);

    int mov_type = 0; // pick
    bool coll = params.mov_specs.coll;
    res->mov_type = mov_type;
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

    res->object_id = params.mov_specs.obj->getName();
    bool approach = params.mov_specs.approach;
    bool retreat = params.mov_specs.retreat;
    bool straight_line = params.mov_specs.straight_line;
    int pre_post = 0; // 0 = use no options, 1 = use approach options, 2 = use retreat options
    int mod; // 0 = move, 1 = pre_approach, 2 = approach, 3 = retreat

    try
    {
        std::vector<double> finalPosture_pre_grasp; bool FPosture_pre_grasp = false;
        std::vector<double> bouncePosture_pre_grasp; //bool BPosture_pre_grasp = false;
        std::vector<double> bouncePosture; bool BPosture = false;
        std::vector<double> finalPosture; bool FPosture = false; std::vector<double> finalPosture_ext;
        std::vector<double> finalPosture_post_grasp; bool FPosture_post_grasp = false;
        if(approach){
            pre_post = 1;
            FPosture_pre_grasp = this->singleArmFinalPosture(mov_type,pre_post,params,initPosture,finalPosture_pre_grasp);
            if (FPosture_pre_grasp){
                // extend the final postures
                std::vector<double> finalPosture_pre_grasp_ext = finalPosture_pre_grasp;
                finalPosture_pre_grasp_ext.push_back(finalHand.at(0));
                for(size_t i=1;i<finalHand.size();++i){
                    if(((finalHand.at(i) -AP) > minLimits.at(i+7))){
                        finalPosture_pre_grasp_ext.push_back(finalHand.at(i)-AP);
                    }else{
                       finalPosture_pre_grasp_ext.push_back(minLimits.at(i+7));
                    }
                }
                int steps = this->getSteps(maxLimits, minLimits,initPosture,finalPosture_pre_grasp_ext);

                if(straight_line){
                    bool init_coll = params.mov_specs.coll;
                    params.mov_specs.coll = false;
                    pre_post = 0;
                    FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture_pre_grasp,finalPosture);
                    params.mov_specs.coll = init_coll;
                }else{
                    pre_post = 0;
                    FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture_pre_grasp,finalPosture);
                }

                if(FPosture){
                    // extend the final postures
                    finalPosture_ext = finalPosture;
                    finalPosture_ext.push_back(finalHand.at(0));
                    for(size_t i=1;i<finalHand.size();++i){
                        finalPosture_ext.push_back(finalHand.at(i));
                    }
                    int steps_app = this->getSteps(maxLimits, minLimits,finalPosture_pre_grasp_ext,finalPosture_ext);

                    // calculate the approach boundary conditions
                    // the velocity approach is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    //
                    if(this->setBoundaryConditions(mov_type,params,steps_app,finalPosture_pre_grasp_ext,finalPosture_ext,0)){
                        if(coll){// collisions
                            pre_post = 1;
                            BPosture = this->singleArmBouncePosture(steps,mov_type,pre_post,params,initPosture,finalPosture_pre_grasp,bouncePosture_pre_grasp);
                            if(BPosture){
                                res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully");
                                res->time_steps.clear();
                                res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                                res->velocity_stages.clear();
                                res->acceleration_stages.clear();
                                // approach stage
                                MatrixXd traj_app; MatrixXd vel_app; MatrixXd acc_app; double timestep_app;
                                mod = 2; bool success = true;
                                timestep_app = this->getAcceleration(mov_type,steps_app,params,finalPosture_pre_grasp_ext,finalPosture_ext,traj_app,vel_app,acc_app,success,mod);
                                // pre-approach stage
                                MatrixXd traj_pre_app; MatrixXd vel_pre_app; MatrixXd acc_pre_app; double timestep_pre_app;
                                mod = 1;
                                timestep_pre_app = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_pre_grasp_ext,bouncePosture_pre_grasp,traj_pre_app,vel_pre_app,acc_pre_app,success,mod);
                                if(success){
                                    // pre-approach
                                    res->time_steps.push_back(timestep_pre_app);
                                    res->trajectory_stages.push_back(traj_pre_app); res->trajectory_descriptions.push_back("plan");
                                    res->velocity_stages.push_back(vel_pre_app);
                                    res->acceleration_stages.push_back(acc_pre_app);
                                    // approach
                                    res->time_steps.push_back(timestep_app);
                                    res->trajectory_stages.push_back(traj_app); res->trajectory_descriptions.push_back("approach");
                                    res->velocity_stages.push_back(vel_app);
                                    res->acceleration_stages.push_back(acc_app);
                                }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to grasp selection failed ");}
                            }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture pre grasp selection failed ");}
                        }else{ // no collisions
                            //pre_post = 0;
                            //FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture_pre_grasp,finalPosture);
                            if(FPosture){
                                res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                                res->time_steps.clear();
                                res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                                res->velocity_stages.clear();
                                res->acceleration_stages.clear();
                                // extend the final postures
                                finalPosture_ext = finalPosture;
                                finalPosture_ext.push_back(finalHand.at(0));
                                for(size_t i=1;i<finalHand.size();++i){
                                    finalPosture_ext.push_back(finalHand.at(i));
                                }
                                bool success = true;
                                // pre-approach stage
                                MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 1;
                                timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_pre_grasp_ext,traj,vel,acc,success,mod);
                                res->time_steps.push_back(timestep);
                                res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                                res->velocity_stages.push_back(vel);
                                res->acceleration_stages.push_back(acc);
                                // approach stage
                                mod = 2;
                                //int steps_app = this->getSteps(maxLimits, minLimits,finalPosture_pre_grasp_ext,finalPosture_ext);
                                timestep = this->getAcceleration(mov_type,steps_app,params,finalPosture_pre_grasp_ext,finalPosture_ext,traj,vel,acc,success,mod);
                                if(success){
                                    res->time_steps.push_back(timestep);
                                    res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("approach");
                                    res->velocity_stages.push_back(vel);
                                    res->acceleration_stages.push_back(acc);
                                }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to grasp selection failed ");}
                            }else{res->status = 30; res->status_msg = string("HUMP: final posture grasp selection failed ");}
                        }
                    }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to grasp selection failed ");}
                }else{res->status = 30; res->status_msg = string("HUMP: final posture grasp selection failed ");}
            }else{res->status = 10; res->status_msg = string("HUMP: final posture pre grasp selection failed ");}
        }else{ // no approach
            pre_post = 0;
            FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,initPosture,finalPosture);
            if (FPosture){
                // extend the final postures
                finalPosture_ext = finalPosture;
                for(size_t i=0;i<finalHand.size();++i){
                    finalPosture_ext.push_back(finalHand.at(i));
                }
                int steps = this->getSteps(maxLimits, minLimits,initPosture,finalPosture_ext);
                if(coll){ // collisions
                    BPosture = this->singleArmBouncePosture(steps,mov_type,pre_post,params,initPosture,finalPosture,bouncePosture);
                    if(BPosture){
                        res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                        res->time_steps.clear();
                        res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                        res->velocity_stages.clear();
                        res->acceleration_stages.clear();
                        int mod;
                        // plan stage
                        MatrixXd traj; MatrixXd vel; MatrixXd acc; mod = 0; bool success = true;
                        double timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_ext,bouncePosture,traj,vel,acc,success,mod);
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture selection failed ");}
                }else{ // no collisions
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    res->time_steps.clear();
                    res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                    res->velocity_stages.clear();
                    res->acceleration_stages.clear();
                    int mod;
                    // plan stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; mod = 0; bool success = true;
                    double timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_ext,traj,vel,acc,success,mod);
                    res->time_steps.push_back(timestep);
                    res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                    res->velocity_stages.push_back(vel);
                    res->acceleration_stages.push_back(acc);
                }
            }else{res->status = 10; res->status_msg = string("HUMP: final posture selection failed ");}
        }
        if(coll){ // collisions
            if(retreat && FPosture && BPosture){// retreat stage
                if(straight_line){
                    bool init_coll = params.mov_specs.coll;
                    params.mov_specs.coll = false;
                    pre_post=2;
                    FPosture_post_grasp = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture,finalPosture_post_grasp);
                    params.mov_specs.coll = init_coll;
                }else{
                    pre_post=2;
                    FPosture_post_grasp = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture,finalPosture_post_grasp);
                }
                if (FPosture_post_grasp){
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    std::vector<double> finalPosture_post_grasp_ext = finalPosture_post_grasp;
                    for(size_t i=0;i<finalHand.size();++i){
                        finalPosture_post_grasp_ext.push_back(finalHand.at(i));
                    }
                    int steps_ret = this->getSteps(maxLimits, minLimits,finalPosture_ext,finalPosture_post_grasp_ext);
                    // calculate the retreat boundary conditions
                    // the final velocity is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    this->setBoundaryConditions(mov_type,params,steps_ret,finalPosture_ext,finalPosture_post_grasp_ext,1);
                    // retreat stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 3; bool success = true;
                    timestep = this->getAcceleration(mov_type,steps_ret,params,finalPosture_ext,finalPosture_post_grasp_ext,traj,vel,acc,success,mod);
                    if(success){
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("retreat");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{res->status = 50; res->status_msg = string("HUMP: final posture retreat from grasp selection failed ");}
                }else{res->status = 40; res->status_msg = string("HUMP: final posture post grasp selection failed ");}
            }
        }else{ // no collisions
            if(retreat && FPosture){// retreat stage
                pre_post=2;
                FPosture_post_grasp = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture,finalPosture_post_grasp);
                if (FPosture_post_grasp){
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    std::vector<double> finalPosture_post_grasp_ext = finalPosture_post_grasp;
                    for(size_t i=0;i<finalHand.size();++i){
                        finalPosture_post_grasp_ext.push_back(finalHand.at(i));
                    }
                    int steps_ret = this->getSteps(maxLimits, minLimits,finalPosture_ext,finalPosture_post_grasp_ext);
                    // calculate the retreat boundary conditions
                    // the final velocity is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    this->setBoundaryConditions(mov_type,params,steps_ret,finalPosture_ext,finalPosture_post_grasp_ext,1);
                    // retreat stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 3; bool success = true;
                    timestep = this->getAcceleration(mov_type,steps_ret,params,finalPosture_ext,finalPosture_post_grasp_ext,traj,vel,acc,success,mod);
                    if(success){
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("retreat");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{res->status = 50; res->status_msg = string("HUMP: final posture retreat from grasp selection failed ");}
                }else{res->status = 40; res->status_msg = string("HUMP: final posture post grasp selection failed ");}
            }
        }
    }catch (const string message){throw message;
    }catch( ... ){throw string ("HUMP: error in optimizing the trajecory");}

    return res;


}

planning_result_ptr HUMPlanner::plan_place(hump_params &params, std::vector<double> initPosture)
{
    planning_result_ptr res;
    res.reset(new planning_result);

    int mov_type = 1; // place
    bool coll = params.mov_specs.coll;
    res->mov_type = mov_type;
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

    res->object_id = params.mov_specs.obj->getName();
    bool approach = params.mov_specs.approach;
    bool retreat = params.mov_specs.retreat;
    bool straight_line = params.mov_specs.straight_line;
    int pre_post = 0; // 0 = use no options, 1 = use approach options, 2 = use retreat options
    int mod; // 0 = move, 1 = pre_approach, 2 = approach, 3 = retreat

    try
    {
        std::vector<double> finalPosture_pre_place; bool FPosture_pre_place = false;
        std::vector<double> bouncePosture_pre_place; //bool BPosture_pre_place = false;
        std::vector<double> bouncePosture; bool BPosture = false;
        std::vector<double> finalPosture; bool FPosture = false; std::vector<double> finalPosture_ext;
        std::vector<double> finalPosture_post_place; bool FPosture_post_place = false;

        if(approach){
            pre_post = 1;
            FPosture_pre_place = this->singleArmFinalPosture(mov_type,pre_post,params,initPosture,finalPosture_pre_place);
            if (FPosture_pre_place){
                // extend the final postures
                std::vector<double> finalPosture_pre_place_ext = finalPosture_pre_place;
                for(size_t i=0;i<finalHand.size();++i){
                    finalPosture_pre_place_ext.push_back(finalHand.at(i));
                }
                int steps = this->getSteps(maxLimits, minLimits,initPosture,finalPosture_pre_place_ext);

                if(straight_line){
                    bool init_coll = params.mov_specs.coll;
                    params.mov_specs.coll = false;
                    pre_post = 0;
                    FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture_pre_place,finalPosture);
                    params.mov_specs.coll = init_coll;
                }else{
                    pre_post = 0;
                    FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture_pre_place,finalPosture);
                }

                if(FPosture){
                    // extend the final postures
                    finalPosture_ext = finalPosture;
                    for(size_t i=0;i<finalHand.size();++i){
                        finalPosture_ext.push_back(finalHand.at(i));
                    }
                    int steps_app = this->getSteps(maxLimits, minLimits,finalPosture_pre_place_ext,finalPosture_ext);
                    // calculate the approach boundary conditions
                    // the velocity approach is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    if(this->setBoundaryConditions(mov_type,params,steps_app,finalPosture_pre_place_ext,finalPosture_ext,0)){
                        if(coll){ // collisions
                            pre_post = 1;
                            BPosture = this->singleArmBouncePosture(steps,mov_type,pre_post,params,initPosture,finalPosture_pre_place,bouncePosture_pre_place);
                            if(BPosture){
                                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                                    res->time_steps.clear();
                                    res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                                    res->velocity_stages.clear();
                                    res->acceleration_stages.clear();
                                    // pre-approach stage
                                    MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 1; bool success = true;
                                    timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_pre_place_ext,bouncePosture_pre_place,traj,vel,acc,success,mod);
                                    res->time_steps.push_back(timestep);
                                    res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                                    res->velocity_stages.push_back(vel);
                                    res->acceleration_stages.push_back(acc);
                                    // approach stage
                                    mod = 2;
                                    timestep = this->getAcceleration(mov_type,steps_app,params,finalPosture_pre_place_ext,finalPosture_ext,traj,vel,acc,success,mod);
                                    if(success){
                                        res->time_steps.push_back(timestep);
                                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("approach");
                                        res->velocity_stages.push_back(vel);
                                        res->acceleration_stages.push_back(acc);
                                    }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to place selection failed ");}
                            }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture pre place selection failed ");}
                        }else{ // no collisions
                            //pre_post = 0;
                            //FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture_pre_place,finalPosture);
                            if(FPosture){
                                // extend the final postures
                                finalPosture_ext = finalPosture;
                                for(size_t i=0;i<finalHand.size();++i){
                                    finalPosture_ext.push_back(finalHand.at(i));
                                }
                                res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                                res->time_steps.clear();
                                res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                                res->velocity_stages.clear();
                                res->acceleration_stages.clear();
                                // pre-approach stage
                                MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 1; bool success = true;
                                timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_pre_place_ext,traj,vel,acc,success,mod);
                                res->time_steps.push_back(timestep);
                                res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                                res->velocity_stages.push_back(vel);
                                res->acceleration_stages.push_back(acc);
                                // approach stage
                                mod = 2;
                                //int steps_app = this->getSteps(maxLimits, minLimits,finalPosture_pre_place_ext,finalPosture_ext);
                                timestep = this->getAcceleration(mov_type,steps_app,params,finalPosture_pre_place_ext,finalPosture_ext,traj,vel,acc,success,mod);
                                if(success){
                                    res->time_steps.push_back(timestep);
                                    res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("approach");
                                    res->velocity_stages.push_back(vel);
                                    res->acceleration_stages.push_back(acc);
                                }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to place selection failed ");}
                            }else{res->status = 30; res->status_msg = string("HUMP: final posture place selection failed ");}
                        }
                    }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to place selection failed ");}
                }else{res->status = 30; res->status_msg = string("HUMP: final posture place selection failed ");}
            }else{res->status = 10; res->status_msg = string("HUMP: final posture pre place selection failed ");}
        }else{
            pre_post = 0;
            FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,initPosture,finalPosture);
            if (FPosture){
                // extend the final posture
                finalPosture_ext = finalPosture;
                for(size_t i=0;i<finalHand.size();++i){
                    finalPosture_ext.push_back(finalHand.at(i));
                }
                int steps = this->getSteps(maxLimits, minLimits,initPosture,finalPosture_ext);
                if(coll){ // collisions
                    BPosture = this->singleArmBouncePosture(steps,mov_type,pre_post,params,initPosture,finalPosture,bouncePosture);
                    if(BPosture){
                        res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                        res->time_steps.clear();
                        res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                        res->velocity_stages.clear();
                        res->acceleration_stages.clear();
                        int mod;
                        // plan stage
                        MatrixXd traj; MatrixXd vel; MatrixXd acc; mod = 0; bool success = true;
                        double timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_ext,bouncePosture,traj,vel,acc,success,mod);
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture selection failed ");}
                }else{ // no collisions
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    res->time_steps.clear();
                    res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                    res->velocity_stages.clear();
                    res->acceleration_stages.clear();
                    int mod;
                    // plan stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; mod = 0; bool success = true;
                    double timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_ext,traj,vel,acc,success,mod);
                    res->time_steps.push_back(timestep);
                    res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                    res->velocity_stages.push_back(vel);
                    res->acceleration_stages.push_back(acc);
                }
            }else{res->status = 10; res->status_msg = string("HUMP: final posture selection failed ");}
        }
        if(coll){// collisions
            if(retreat && FPosture && BPosture){
                if(straight_line){
                    bool init_coll = params.mov_specs.coll;
                    params.mov_specs.coll = false;
                    pre_post=2;
                    FPosture_post_place = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture,finalPosture_post_place);
                    params.mov_specs.coll = init_coll;
                }else{
                    pre_post=2;
                    FPosture_post_place = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture,finalPosture_post_place);
                }
                if (FPosture_post_place){
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    // extend the final postures
                    std::vector<double> finalPosture_post_place_ext = finalPosture_post_place;
                    finalPosture_ext = finalPosture;
                    finalPosture_ext.push_back(finalHand.at(0));
                    finalPosture_post_place_ext.push_back(finalHand.at(0));
                    for(size_t i=1;i<finalHand.size();++i){
                        finalPosture_ext.push_back(finalHand.at(i));
                        if(((finalHand.at(i) -AP) > minLimits.at(i+7))){
                            finalPosture_post_place_ext.push_back(finalHand.at(i)-AP);
                        }else{
                           finalPosture_post_place_ext.push_back(minLimits.at(i+7));
                        }
                    }
                    int steps_ret = this->getSteps(maxLimits, minLimits,finalPosture_ext,finalPosture_post_place_ext);
                    // calculate the retreat boundary conditions
                    // the final velocity is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    this->setBoundaryConditions(mov_type,params,steps_ret,finalPosture_ext,finalPosture_post_place_ext,1);
                    // retreat stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 3; bool success = true;
                    timestep = this->getAcceleration(mov_type,steps_ret,params,finalPosture_ext,finalPosture_post_place_ext,traj,vel,acc,success,mod);
                    if(success){
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("retreat");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{res->status = 50; res->status_msg = string("HUMP: final posture retreat from place selection failed ");}
                }else{res->status = 40; res->status_msg = string("HUMP: final posture post place selection failed ");}
            }
        }else{// no collisions
            if(retreat && FPosture){
                pre_post=2;
                FPosture_post_place = this->singleArmFinalPosture(mov_type,pre_post,params,finalPosture,finalPosture_post_place);
                if (FPosture_post_place){
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    // extend the final postures
                    std::vector<double> finalPosture_post_place_ext = finalPosture_post_place;
                    finalPosture_ext = finalPosture;
                    finalPosture_ext.push_back(finalHand.at(0));
                    finalPosture_post_place_ext.push_back(finalHand.at(0));
                    for(size_t i=1;i<finalHand.size();++i){
                        finalPosture_ext.push_back(finalHand.at(i));
                        if(((finalHand.at(i) -AP) > minLimits.at(i+7))){
                            finalPosture_post_place_ext.push_back(finalHand.at(i)-AP);
                        }else{
                           finalPosture_post_place_ext.push_back(minLimits.at(i+7));
                        }
                    }
                    int steps_ret = this->getSteps(maxLimits, minLimits,finalPosture_ext,finalPosture_post_place_ext);
                    // calculate the retreat boundary conditions
                    // the final velocity is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    this->setBoundaryConditions(mov_type,params,steps_ret,finalPosture_ext,finalPosture_post_place_ext,1);
                    // retreat stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 3; bool success = true;
                    timestep = this->getAcceleration(mov_type,steps_ret,params,finalPosture_ext,finalPosture_post_place_ext,traj,vel,acc,success,mod);
                    if(success){
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("retreat");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{res->status = 50; res->status_msg = string("HUMP: final posture retreat from place selection failed ");}
                }else{res->status = 40; res->status_msg = string("HUMP: final posture post place selection failed ");}
            }

        }
    }catch (const string message){throw message;
    }catch( ... ){throw string ("HUMP: error in optimizing the trajecory");}

    return res;
}

planning_result_ptr HUMPlanner::plan_move(hump_params &params, std::vector<double> initPosture)
{
    planning_result_ptr res;
    res.reset(new planning_result);

    int mov_type = 2; // move
    bool coll = params.mov_specs.coll;
    res->mov_type = mov_type;
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

    int pre_post = 0; // 0 = use no options, 1 = use approach options, 2 = use retreat options
    int mod = 0; // 0 = move, 1 = pre_approach, 2 = approach, 3 = retreat

    try
    {
        std::vector<double> bouncePosture; bool BPosture = false;
        std::vector<double> finalPosture; bool FPosture = false; std::vector<double> finalPosture_ext;


        FPosture = this->singleArmFinalPosture(mov_type,pre_post,params,initPosture,finalPosture);
        if (FPosture){
            // extend the final posture
            finalPosture_ext = finalPosture;
            for(size_t i=0;i<finalHand.size();++i){
                finalPosture_ext.push_back(finalHand.at(i));
            }
            int steps = this->getSteps(maxLimits, minLimits,initPosture,finalPosture_ext);
            if(coll){ // collisions enabled
                BPosture = this->singleArmBouncePosture(steps,mov_type,pre_post,params,initPosture,finalPosture,bouncePosture);
                if(BPosture){
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    res->time_steps.clear();
                    res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                    res->velocity_stages.clear();
                    res->acceleration_stages.clear();
                    // plan stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; bool success = true;
                    double timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_ext,bouncePosture,traj,vel,acc,success,mod);
                    res->time_steps.push_back(timestep);
                    res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                    res->velocity_stages.push_back(vel);
                    res->acceleration_stages.push_back(acc);
                }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture selection failed ");}
            }else{// collisions disabled
                res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                res->time_steps.clear();
                res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                res->velocity_stages.clear();
                res->acceleration_stages.clear();
                // plan stage
                MatrixXd traj; MatrixXd vel; MatrixXd acc; bool success = true;
                double timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_ext,traj,vel,acc,success,mod);
                res->time_steps.push_back(timestep);
                res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                res->velocity_stages.push_back(vel);
                res->acceleration_stages.push_back(acc);
            }
        }else{res->status = 10; res->status_msg = string("HUMP: final posture selection failed ");}

    }catch (const string message){throw message;
    }catch( ... ){throw string ("HUMP: error in optimizing the trajecory");}

    return res;

}

planning_result_ptr HUMPlanner::plan_move(hump_params &params, std::vector<double> initPosture, std::vector<double> finalPosture)
{
    planning_result_ptr res;
    res.reset(new planning_result);

    int mov_type = 2; // move
    bool coll = params.mov_specs.coll;
    res->mov_type = mov_type;
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

    int pre_post = 0; // 0 = use no options, 1 = use approach options, 2 = use retreat options
    int mod = 0; // 0 = move, 1 = pre_approach, 2 = approach, 3 = retreat

    try
    {
        std::vector<double> bouncePosture; bool BPosture = false;
        std::vector<double> finalPosture_ext;
        // extend the final posture
        finalPosture_ext = finalPosture;
        for(size_t i=0;i<finalHand.size();++i){
            finalPosture_ext.push_back(finalHand.at(i));
        }
        int steps = this->getSteps(maxLimits, minLimits,initPosture,finalPosture_ext);
        if(coll){
            BPosture = this->singleArmBouncePosture(steps,mov_type,pre_post,params,initPosture,finalPosture,bouncePosture);
            if(BPosture){
                res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                res->time_steps.clear();
                res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                res->velocity_stages.clear();
                res->acceleration_stages.clear();
                // plan stage
                MatrixXd traj; MatrixXd vel; MatrixXd acc; bool success = true;
                double timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_ext,bouncePosture,traj,vel,acc,success,mod);
                res->time_steps.push_back(timestep);
                res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                res->velocity_stages.push_back(vel);
                res->acceleration_stages.push_back(acc);
            }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture selection failed ");}
        }else{
            res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
            res->time_steps.clear();
            res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
            res->velocity_stages.clear();
            res->acceleration_stages.clear();
            // plan stage
            MatrixXd traj; MatrixXd vel; MatrixXd acc; bool success = true;
            double timestep = this->getAcceleration(mov_type,steps,params,initPosture,finalPosture_ext,traj,vel,acc,success,mod);
            res->time_steps.push_back(timestep);
            res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
            res->velocity_stages.push_back(vel);
            res->acceleration_stages.push_back(acc);
        }

    }catch (const string message){throw message;
    }catch( ... ){throw string ("HUMP: error in optimizing the trajecory");}

    return res;
}


planning_dual_result_ptr HUMPlanner::plan_dual_pick_pick(hump_dual_params &params, std::vector<double> initPosture_right, std::vector<double> initPosture_left)
{
    planning_dual_result_ptr res;
    res.reset(new planning_dual_result);

    int dual_mov_type = 0; // pick right and pick left
    int mov_type_right = 0; // pick
    int mov_type_left = 0; // pick
    bool coll_right = params.mov_specs_right.coll;
    bool coll_left = params.mov_specs_left.coll;
    res->mov_type_right = mov_type_right;
    res->mov_type_left = mov_type_left;
    std::vector<double> finalHand_right = params.mov_specs_right.finalHand;
    std::vector<double> finalHand_left = params.mov_specs_left.finalHand;
    std::vector<double> initPosture;
    std::vector<double> minLimits; std::vector<double> maxLimits;
    std::vector<double> minLimits_right; std::vector<double> maxLimits_right;
    std::vector<double> minLimits_left; std::vector<double> maxLimits_left;
    minLimits_right = this->minRightLimits; maxLimits_right = this->maxRightLimits;
    minLimits_left = this->minLeftLimits; maxLimits_left = this->maxLeftLimits;

    res->object_right_id = params.mov_specs_right.obj->getName();
    res->object_left_id = params.mov_specs_left.obj->getName();
    bool approach_right = params.mov_specs_right.approach;
    bool approach_left = params.mov_specs_left.approach;
    bool retreat_right = params.mov_specs_right.retreat;
    bool retreat_left = params.mov_specs_left.retreat;
    //bool straight_line_right = params.mov_specs_right.straight_line;
    //bool straight_line_left = params.mov_specs_left.straight_line;

    int pre_post = 0; // 0 = use no options, 1 = use approach options, 2 = use retreat options
    int mod; // 0 = move, 1 = pre_approach, 2 = approach, 3 = retreat

    initPosture = initPosture_right;
    initPosture.insert(initPosture.end(),initPosture_left.begin(),initPosture_left.end());
    minLimits = minLimits_right;
    minLimits.insert(minLimits.end(),minLimits_left.begin(),minLimits_left.end());
    maxLimits = maxLimits_right;
    maxLimits.insert(maxLimits.end(),maxLimits_left.begin(),maxLimits_left.end());

    try
    {
        // the posture is given by
        // right arm(7) + right hand(4) + left arm(7) + left hand(4) = 22 joints
        std::vector<double> finalPosture_pre_grasp; bool FPosture_pre_grasp = false; std::vector<double> finalPosture_pre_grasp_ext;
        std::vector<double> hand_r; std::vector<double> hand_l;
        std::vector<double> bouncePosture_pre_grasp; //bool BPosture_pre_grasp = false;
        std::vector<double> bouncePosture; bool BPosture = false;
        std::vector<double> finalPosture; bool FPosture = false; std::vector<double> finalPosture_ext;
        std::vector<double> finalPosture_post_grasp; bool FPosture_post_grasp = false;

        if(approach_right && approach_left)
        {
            pre_post = 1;
            FPosture_pre_grasp = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,initPosture,finalPosture_pre_grasp);
            if(FPosture_pre_grasp){
                // extend the final posture
                finalPosture_pre_grasp_ext = finalPosture_pre_grasp;
                hand_r.clear();
                finalPosture_pre_grasp_ext.insert(finalPosture_pre_grasp_ext.begin()+joints_arm,finalHand_right.at(0));                
                for(size_t i=1;i<finalHand_right.size();++i){
                    if(((finalHand_right.at(i)-AP) > minLimits_right.at(i+joints_arm))){
                        hand_r.push_back(finalHand_right.at(i)-AP);
                    }else{
                       hand_r.push_back(minLimits_right.at(i+joints_arm));
                    }
                }
                finalPosture_pre_grasp_ext.insert(finalPosture_pre_grasp_ext.begin()+joints_arm+1,hand_r.begin(),hand_r.end());
                hand_l.clear();
                finalPosture_pre_grasp_ext.insert(finalPosture_pre_grasp_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.at(0));
                for(size_t i=1;i<finalHand_left.size();++i){
                    if(((finalHand_left.at(i)-AP) > minLimits_left.at(i+joints_arm))){
                        hand_l.push_back(finalHand_left.at(i)-AP);
                    }else{
                       hand_l.push_back(minLimits_left.at(i+joints_arm));
                    }
                }
                finalPosture_pre_grasp_ext.insert(finalPosture_pre_grasp_ext.begin()+joints_arm+joints_hand+joints_arm+1,hand_l.begin(),hand_l.end());

                // get the steps of the trajectory
                int steps = this->getSteps(maxLimits,minLimits,initPosture,finalPosture_pre_grasp_ext);

                pre_post = 0;
                FPosture = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,finalPosture_pre_grasp_ext,finalPosture);
                if(FPosture){
                    // extend the final posture
                    finalPosture_ext = finalPosture;
                    finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm,finalHand_right.begin(),finalHand_right.end());
                    finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.begin(),finalHand_left.end());

                    // get the steps of the trajectory
                    int steps_app = this->getSteps(maxLimits, minLimits,finalPosture_pre_grasp_ext,finalPosture_ext);
                    // calculate the approach boundary conditions
                    // the velocity approach is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    //
                    if(this->setDualBoundaryConditions(dual_mov_type,params,steps_app,finalPosture_pre_grasp_ext,finalPosture_ext,0)){
                        if(coll_right && coll_left){// collisions
                            pre_post = 1;
                            BPosture = this->singleDualArmBouncePosture(steps,dual_mov_type,pre_post,params,initPosture,finalPosture_pre_grasp,bouncePosture_pre_grasp);
                            if(BPosture){
                                res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully");
                                res->time_steps.clear();
                                res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                                res->velocity_stages.clear();
                                res->acceleration_stages.clear();
                                // approach stage
                                MatrixXd traj_app; MatrixXd vel_app; MatrixXd acc_app; double timestep_app;
                                mod = 2; bool success = true;
                                timestep_app = this->getDualAcceleration(dual_mov_type,steps_app,params,finalPosture_pre_grasp_ext,finalPosture_ext,traj_app,vel_app,acc_app,success,mod);
                                // pre-approach stage
                                MatrixXd traj_pre_app; MatrixXd vel_pre_app; MatrixXd acc_pre_app; double timestep_pre_app;
                                mod = 1;
                                timestep_pre_app = this->getDualAcceleration(dual_mov_type,steps,params,initPosture,finalPosture_pre_grasp_ext,bouncePosture_pre_grasp,traj_pre_app,vel_pre_app,acc_pre_app,success,mod);
                                if(success){
                                    // pre-approach
                                    res->time_steps.push_back(timestep_pre_app);
                                    res->trajectory_stages.push_back(traj_pre_app); res->trajectory_descriptions.push_back("plan");
                                    res->velocity_stages.push_back(vel_pre_app);
                                    res->acceleration_stages.push_back(acc_pre_app);
                                    // approach
                                    res->time_steps.push_back(timestep_app);
                                    res->trajectory_stages.push_back(traj_app); res->trajectory_descriptions.push_back("approach");
                                    res->velocity_stages.push_back(vel_app);
                                    res->acceleration_stages.push_back(acc_app);
                                }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to grasp selection failed ");}
                            }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture pre grasp selection failed ");}
                        }else{ // no collisions
                            //pre_post = 0;
                            //FPosture = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,finalPosture_pre_grasp,finalPosture);
                            if(FPosture){
                                res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                                res->time_steps.clear();
                                res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                                res->velocity_stages.clear();
                                res->acceleration_stages.clear();
                                // extend the final posture
                                finalPosture_ext = finalPosture;
                                finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm,finalHand_right.begin(),finalHand_right.end());
                                finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.begin(),finalHand_left.end());

                                bool success = true;
                                // pre-approach stage
                                MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 1;
                                timestep = this->getDualAcceleration(dual_mov_type,steps,params,initPosture,finalPosture_pre_grasp_ext,traj,vel,acc,success,mod);
                                res->time_steps.push_back(timestep);
                                res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                                res->velocity_stages.push_back(vel);
                                res->acceleration_stages.push_back(acc);
                                // approach stage
                                mod = 2;
                                //int steps_app = this->getSteps(maxLimits, minLimits,finalPosture_pre_grasp_ext,finalPosture_ext);
                                timestep = this->getDualAcceleration(dual_mov_type,steps_app,params,finalPosture_pre_grasp_ext,finalPosture_ext,traj,vel,acc,success,mod);
                                if(success){
                                    res->time_steps.push_back(timestep);
                                    res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("approach");
                                    res->velocity_stages.push_back(vel);
                                    res->acceleration_stages.push_back(acc);
                                }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to grasp selection failed ");}
                            }else{res->status = 30; res->status_msg = string("HUMP: final posture grasp selection failed ");}
                        }
                   }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to grasp selection failed ");}
                }else{res->status = 30; res->status_msg = string("HUMP: final posture grasp selection failed ");}
            }else{res->status = 10; res->status_msg = string("HUMP: final posture pre grasp selection failed ");}
        }else{ // no approach
            pre_post = 0;
            FPosture = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,initPosture,finalPosture);
            if (FPosture){
                // extend the final posture
                finalPosture_ext = finalPosture;
                finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm,finalHand_right.begin(),finalHand_right.end());
                finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.begin(),finalHand_left.end());

                int steps = this->getSteps(maxLimits, minLimits,initPosture,finalPosture_ext);
                if(coll_right && coll_left){ // collisions
                    BPosture = this->singleDualArmBouncePosture(steps,dual_mov_type,pre_post,params,initPosture,finalPosture,bouncePosture);
                    if(BPosture){
                        res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                        res->time_steps.clear();
                        res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                        res->velocity_stages.clear();
                        res->acceleration_stages.clear();
                        int mod;
                        // plan stage
                        MatrixXd traj; MatrixXd vel; MatrixXd acc; mod = 0; bool success = true;
                        double timestep = this->getDualAcceleration(dual_mov_type,steps,params,initPosture,finalPosture_ext,bouncePosture,traj,vel,acc,success,mod);
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture selection failed ");}
                }else{ // no collisions
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    res->time_steps.clear();
                    res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                    res->velocity_stages.clear();
                    res->acceleration_stages.clear();
                    int mod;
                    // plan stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; mod = 0; bool success = true;
                    double timestep = this->getDualAcceleration(dual_mov_type,steps,params,initPosture,finalPosture_ext,traj,vel,acc,success,mod);
                    res->time_steps.push_back(timestep);
                    res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                    res->velocity_stages.push_back(vel);
                    res->acceleration_stages.push_back(acc);
                }
            }else{res->status = 10; res->status_msg = string("HUMP: final posture selection failed ");}
        }
        if(coll_right && coll_left){ // collisions
            if(retreat_right && retreat_left && FPosture && BPosture){// retreat stage
                pre_post=2;
                FPosture_post_grasp = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,finalPosture_ext,finalPosture_post_grasp);
                if (FPosture_post_grasp){
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    std::vector<double> finalPosture_post_grasp_ext = finalPosture_post_grasp;
                    finalPosture_post_grasp_ext.insert(finalPosture_post_grasp_ext.begin()+joints_arm,finalHand_right.begin(),finalHand_right.end());
                    finalPosture_post_grasp_ext.insert(finalPosture_post_grasp_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.begin(),finalHand_left.end());
                    int steps_ret = this->getSteps(maxLimits, minLimits,finalPosture_ext,finalPosture_post_grasp_ext);
                    // calculate the retreat boundary conditions
                    // the final velocity is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    this->setDualBoundaryConditions(dual_mov_type,params,steps_ret,finalPosture_ext,finalPosture_post_grasp_ext,1);
                    // retreat stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 3; bool success = true;
                    timestep = this->getDualAcceleration(dual_mov_type,steps_ret,params,finalPosture_ext,finalPosture_post_grasp_ext,traj,vel,acc,success,mod);
                    if(success){
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("retreat");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{res->status = 50; res->status_msg = string("HUMP: final posture retreat from grasp selection failed ");}
                }else{res->status = 40; res->status_msg = string("HUMP: final posture post grasp selection failed ");}
            }
        }else{ // no collisions
            if(retreat_right && retreat_right && FPosture){// retreat stage
                pre_post=2;
                FPosture_post_grasp = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,finalPosture_ext,finalPosture_post_grasp);
                if (FPosture_post_grasp){
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    std::vector<double> finalPosture_post_grasp_ext = finalPosture_post_grasp;
                    finalPosture_post_grasp_ext.insert(finalPosture_post_grasp_ext.begin()+joints_arm,finalHand_right.begin(),finalHand_right.end());
                    finalPosture_post_grasp_ext.insert(finalPosture_post_grasp_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.begin(),finalHand_left.end());
                    int steps_ret = this->getSteps(maxLimits, minLimits,finalPosture_ext,finalPosture_post_grasp_ext);
                    // calculate the retreat boundary conditions
                    // the final velocity is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    this->setDualBoundaryConditions(dual_mov_type,params,steps_ret,finalPosture_ext,finalPosture_post_grasp_ext,1);
                    // retreat stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 3; bool success = true;
                    timestep = this->getDualAcceleration(dual_mov_type,steps_ret,params,finalPosture_ext,finalPosture_post_grasp_ext,traj,vel,acc,success,mod);
                    if(success){
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("retreat");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{res->status = 50; res->status_msg = string("HUMP: final posture retreat from grasp selection failed ");}
                }else{res->status = 40; res->status_msg = string("HUMP: final posture post grasp selection failed ");}
            }
        }
    }catch (const string message){throw message;
    }catch( ... ){throw string ("HUMP: error in optimizing the trajecory");}

    return res;
}

planning_dual_result_ptr HUMPlanner::plan_dual_place_place(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> initPosture_left)
{
    planning_dual_result_ptr res;
    res.reset(new planning_dual_result);

    int dual_mov_type = 1; // place right and place left
    int mov_type_right = 1; // place
    int mov_type_left = 1; // place
    bool coll_right = params.mov_specs_right.coll;
    bool coll_left = params.mov_specs_left.coll;
    res->mov_type_right = mov_type_right;
    res->mov_type_left = mov_type_left;
    std::vector<double> finalHand_right = params.mov_specs_right.finalHand;
    std::vector<double> finalHand_left = params.mov_specs_left.finalHand;
    std::vector<double> initPosture;
    std::vector<double> minLimits; std::vector<double> maxLimits;
    std::vector<double> minLimits_right; std::vector<double> maxLimits_right;
    std::vector<double> minLimits_left; std::vector<double> maxLimits_left;
    minLimits_right = this->minRightLimits; maxLimits_right = this->maxRightLimits;
    minLimits_left = this->minLeftLimits; maxLimits_left = this->maxLeftLimits;

    res->object_right_id = params.mov_specs_right.obj->getName();
    res->object_left_id = params.mov_specs_left.obj->getName();
    bool approach_right = params.mov_specs_right.approach;
    bool approach_left = params.mov_specs_left.approach;
    bool retreat_right = params.mov_specs_right.retreat;
    bool retreat_left = params.mov_specs_left.retreat;
    //bool straight_line_right = params.mov_specs_right.straight_line;
    //bool straight_line_left = params.mov_specs_left.straight_line;

    int pre_post = 0; // 0 = use no options, 1 = use approach options, 2 = use retreat options
    int mod; // 0 = move, 1 = pre_approach, 2 = approach, 3 = retreat

    initPosture = initPosture_right;
    initPosture.insert(initPosture.end(),initPosture_left.begin(),initPosture_left.end());
    minLimits = minLimits_right;
    minLimits.insert(minLimits.end(),minLimits_left.begin(),minLimits_left.end());
    maxLimits = maxLimits_right;
    maxLimits.insert(maxLimits.end(),maxLimits_left.begin(),maxLimits_left.end());

    try
    {
        // the posture is given by
        // right arm(7) + right hand(4) + left arm(7) + left hand(4) = 22 joints
        std::vector<double> finalPosture_pre_place; bool FPosture_pre_place = false; std::vector<double> finalPosture_pre_place_ext;
        std::vector<double> hand_r; std::vector<double> hand_l;
        std::vector<double> bouncePosture_pre_place; //bool BPosture_pre_grasp = false;
        std::vector<double> bouncePosture; bool BPosture = false;
        std::vector<double> finalPosture; bool FPosture = false; std::vector<double> finalPosture_ext;
        std::vector<double> finalPosture_post_place; bool FPosture_post_place = false;

        if(approach_right && approach_left)
        {
            pre_post = 1;
            FPosture_pre_place = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,initPosture,finalPosture_pre_place);
            if(FPosture_pre_place){
                // extend the final posture
                finalPosture_pre_place_ext = finalPosture_pre_place;
                finalPosture_pre_place_ext.insert(finalPosture_pre_place_ext.begin()+joints_arm,finalHand_right.at(0));
                for(size_t i=1;i<finalHand_right.size();++i){
                    if(((finalHand_right.at(i)-AP) > minLimits_right.at(i+joints_arm))){
                        hand_r.push_back(finalHand_right.at(i)-AP);
                    }else{
                       hand_r.push_back(minLimits_right.at(i+joints_arm));
                    }
                }
                finalPosture_pre_place_ext.insert(finalPosture_pre_place_ext.begin()+joints_arm+1,hand_r.begin(),hand_r.end());
                finalPosture_pre_place_ext.insert(finalPosture_pre_place_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.at(0));
                for(size_t i=1;i<finalHand_left.size();++i){
                    if(((finalHand_left.at(i)-AP) > minLimits_left.at(i+joints_arm))){
                        hand_l.push_back(finalHand_left.at(i)-AP);
                    }else{
                       hand_l.push_back(minLimits_left.at(i+joints_arm));
                    }
                }
                finalPosture_pre_place_ext.insert(finalPosture_pre_place_ext.begin()+joints_arm+joints_hand+joints_arm+1,hand_l.begin(),hand_l.end());

                // get the steps of the trajectory
                int steps = this->getSteps(maxLimits,minLimits,initPosture,finalPosture_pre_place_ext);

                pre_post = 0;
                FPosture = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,finalPosture_pre_place_ext,finalPosture);
                if(FPosture){
                    // extend the final posture
                    finalPosture_ext = finalPosture;
                    finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm,finalHand_right.begin(),finalHand_right.end());
                    finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.begin(),finalHand_left.end());

                    // get the steps of the trajectory
                    int steps_app = this->getSteps(maxLimits, minLimits,finalPosture_pre_place_ext,finalPosture_ext);
                    // calculate the approach boundary conditions
                    // the velocity approach is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    //
                    if(this->setDualBoundaryConditions(dual_mov_type,params,steps_app,finalPosture_pre_place_ext,finalPosture_ext,0)){
                        if(coll_right && coll_left){// collisions
                            pre_post = 1;
                            BPosture = this->singleDualArmBouncePosture(steps,dual_mov_type,pre_post,params,initPosture,finalPosture_pre_place,bouncePosture_pre_place);
                            if(BPosture){
                                res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully");
                                res->time_steps.clear();
                                res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                                res->velocity_stages.clear();
                                res->acceleration_stages.clear();
                                // approach stage
                                MatrixXd traj_app; MatrixXd vel_app; MatrixXd acc_app; double timestep_app;
                                mod = 2; bool success = true;
                                timestep_app = this->getDualAcceleration(dual_mov_type,steps_app,params,finalPosture_pre_place_ext,finalPosture_ext,traj_app,vel_app,acc_app,success,mod);
                                // pre-approach stage
                                MatrixXd traj_pre_app; MatrixXd vel_pre_app; MatrixXd acc_pre_app; double timestep_pre_app;
                                mod = 1;
                                timestep_pre_app = this->getDualAcceleration(dual_mov_type,steps,params,initPosture,finalPosture_pre_place_ext,bouncePosture_pre_place,traj_pre_app,vel_pre_app,acc_pre_app,success,mod);
                                if(success){
                                    // pre-approach
                                    res->time_steps.push_back(timestep_pre_app);
                                    res->trajectory_stages.push_back(traj_pre_app); res->trajectory_descriptions.push_back("plan");
                                    res->velocity_stages.push_back(vel_pre_app);
                                    res->acceleration_stages.push_back(acc_pre_app);
                                    // approach
                                    res->time_steps.push_back(timestep_app);
                                    res->trajectory_stages.push_back(traj_app); res->trajectory_descriptions.push_back("approach");
                                    res->velocity_stages.push_back(vel_app);
                                    res->acceleration_stages.push_back(acc_app);
                                }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to place selection failed ");}
                            }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture pre place selection failed ");}
                        }else{ // no collisions
                            //pre_post = 0;
                            //FPosture = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,finalPosture_pre_place,finalPosture);
                            if(FPosture){
                                res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                                res->time_steps.clear();
                                res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                                res->velocity_stages.clear();
                                res->acceleration_stages.clear();
                                // extend the final posture
                                finalPosture_ext = finalPosture;
                                finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm,finalHand_right.begin(),finalHand_right.end());
                                finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.begin(),finalHand_left.end());

                                bool success = true;
                                // pre-approach stage
                                MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 1;
                                timestep = this->getDualAcceleration(dual_mov_type,steps,params,initPosture,finalPosture_pre_place_ext,traj,vel,acc,success,mod);
                                res->time_steps.push_back(timestep);
                                res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                                res->velocity_stages.push_back(vel);
                                res->acceleration_stages.push_back(acc);
                                // approach stage
                                mod = 2;
                                //int steps_app = this->getSteps(maxLimits, minLimits,finalPosture_pre_place_ext,finalPosture_ext);
                                timestep = this->getDualAcceleration(dual_mov_type,steps_app,params,finalPosture_pre_place_ext,finalPosture_ext,traj,vel,acc,success,mod);
                                if(success){
                                    res->time_steps.push_back(timestep);
                                    res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("approach");
                                    res->velocity_stages.push_back(vel);
                                    res->acceleration_stages.push_back(acc);
                                }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to place selection failed ");}
                            }else{res->status = 30; res->status_msg = string("HUMP: final posture place selection failed ");}
                        }
                   }else{res->status = 50; res->status_msg = string("HUMP: final posture approach to place selection failed ");}
                }else{res->status = 30; res->status_msg = string("HUMP: final posture place selection failed ");}
            }else{res->status = 10; res->status_msg = string("HUMP: final posture pre place selection failed ");}
        }else{ // no approach
            pre_post = 0;
            FPosture = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,initPosture,finalPosture);
            if (FPosture){
                // extend the final posture
                finalPosture_ext = finalPosture;
                finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm,finalHand_right.begin(),finalHand_right.end());
                finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.begin(),finalHand_left.end());

                int steps = this->getSteps(maxLimits, minLimits,initPosture,finalPosture_ext);
                if(coll_right && coll_left){ // collisions
                    BPosture = this->singleDualArmBouncePosture(steps,dual_mov_type,pre_post,params,initPosture,finalPosture,bouncePosture);
                    if(BPosture){
                        res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                        res->time_steps.clear();
                        res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                        res->velocity_stages.clear();
                        res->acceleration_stages.clear();
                        int mod;
                        // plan stage
                        MatrixXd traj; MatrixXd vel; MatrixXd acc; mod = 0; bool success = true;
                        double timestep = this->getDualAcceleration(dual_mov_type,steps,params,initPosture,finalPosture_ext,bouncePosture,traj,vel,acc,success,mod);
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture selection failed ");}
                }else{ // no collisions
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    res->time_steps.clear();
                    res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                    res->velocity_stages.clear();
                    res->acceleration_stages.clear();
                    int mod;
                    // plan stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; mod = 0; bool success = true;
                    double timestep = this->getDualAcceleration(dual_mov_type,steps,params,initPosture,finalPosture_ext,traj,vel,acc,success,mod);
                    res->time_steps.push_back(timestep);
                    res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                    res->velocity_stages.push_back(vel);
                    res->acceleration_stages.push_back(acc);
                }
            }else{res->status = 10; res->status_msg = string("HUMP: final posture selection failed ");}
        }
        if(coll_right && coll_left){ // collisions
            if(retreat_right && retreat_left && FPosture && BPosture){// retreat stage
                pre_post=2;
                FPosture_post_place = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,finalPosture_ext,finalPosture_post_place);
                if (FPosture_post_place){
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    // extend the final posture
                    hand_r.clear();
                    std::vector<double> finalPosture_post_place_ext = finalPosture_post_place;
                    finalPosture_post_place_ext.insert(finalPosture_post_place_ext.begin()+joints_arm,finalHand_right.at(0));
                    for(size_t i=1;i<finalHand_right.size();++i){
                        if(((finalHand_right.at(i)-AP) > minLimits_right.at(i+joints_arm))){
                            hand_r.push_back(finalHand_right.at(i)-AP);
                        }else{
                           hand_r.push_back(minLimits_right.at(i+joints_arm));
                        }
                    }
                    finalPosture_post_place_ext.insert(finalPosture_post_place_ext.begin()+joints_arm+1,hand_r.begin(),hand_r.end());
                    hand_l.clear();
                    finalPosture_post_place_ext.insert(finalPosture_post_place_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.at(0));
                    for(size_t i=1;i<finalHand_left.size();++i){
                        if(((finalHand_left.at(i)-AP) > minLimits_left.at(i+joints_arm))){
                            hand_l.push_back(finalHand_left.at(i)-AP);
                        }else{
                           hand_l.push_back(minLimits_left.at(i+joints_arm));
                        }
                    }
                    finalPosture_post_place_ext.insert(finalPosture_post_place_ext.begin()+joints_arm+joints_hand+joints_arm+1,hand_l.begin(),hand_l.end());
                    int steps_ret = this->getSteps(maxLimits, minLimits,finalPosture_ext,finalPosture_post_place_ext);
                    // calculate the retreat boundary conditions
                    // the final velocity is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    this->setDualBoundaryConditions(dual_mov_type,params,steps_ret,finalPosture_ext,finalPosture_post_place_ext,1);
                    // retreat stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 3; bool success = true;
                    timestep = this->getDualAcceleration(dual_mov_type,steps_ret,params,finalPosture_ext,finalPosture_post_place_ext,traj,vel,acc,success,mod);
                    if(success){
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("retreat");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{res->status = 50; res->status_msg = string("HUMP: final posture retreat from place selection failed ");}
                }else{res->status = 40; res->status_msg = string("HUMP: final posture post place selection failed ");}
            }
        }else{ // no collisions
            if(retreat_right && retreat_right && FPosture){// retreat stage
                pre_post=2;
                FPosture_post_place = this->singleDualArmFinalPosture(dual_mov_type,pre_post,params,finalPosture_ext,finalPosture_post_place);
                if (FPosture_post_place){
                    res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
                    // extend the final posture
                    std::vector<double> finalPosture_post_place_ext = finalPosture_post_place;
                    finalPosture_post_place_ext.insert(finalPosture_post_place_ext.begin()+joints_arm,finalHand_right.at(0));
                    for(size_t i=1;i<finalHand_right.size();++i){
                        if(((finalHand_right.at(i)-AP) > minLimits_right.at(i+joints_arm))){
                            hand_r.push_back(finalHand_right.at(i)-AP);
                        }else{
                           hand_r.push_back(minLimits_right.at(i+joints_arm));
                        }
                    }
                    finalPosture_post_place_ext.insert(finalPosture_post_place_ext.begin()+joints_arm+1,hand_r.begin(),hand_r.end());
                    for(size_t i=1;i<finalHand_left.size();++i){
                        if(((finalHand_left.at(i)-AP) > minLimits_left.at(i+joints_arm))){
                            hand_l.push_back(finalHand_left.at(i)-AP);
                        }else{
                           hand_l.push_back(minLimits_left.at(i+joints_arm));
                        }
                    }
                    finalPosture_post_place_ext.insert(finalPosture_post_place_ext.begin()+joints_arm+joints_hand+joints_arm+1,hand_l.begin(),hand_l.end());
                    int steps_ret = this->getSteps(maxLimits, minLimits,finalPosture_ext,finalPosture_post_place_ext);
                    // calculate the retreat boundary conditions
                    // the final velocity is the maximum velocity reached at tau=0.5 of the trajectory with null boundary conditions
                    this->setDualBoundaryConditions(dual_mov_type,params,steps_ret,finalPosture_ext,finalPosture_post_place_ext,1);
                    // retreat stage
                    MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; mod = 3; bool success = true;
                    timestep = this->getDualAcceleration(dual_mov_type,steps_ret,params,finalPosture_ext,finalPosture_post_place_ext,traj,vel,acc,success,mod);
                    if(success){
                        res->time_steps.push_back(timestep);
                        res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("retreat");
                        res->velocity_stages.push_back(vel);
                        res->acceleration_stages.push_back(acc);
                    }else{res->status = 50; res->status_msg = string("HUMP: final posture retreat from place selection failed ");}
                }else{res->status = 40; res->status_msg = string("HUMP: final posture post place selection failed ");}
            }
        }
    }catch (const string message){throw message;
    }catch( ... ){throw string ("HUMP: error in optimizing the trajecory");}

    return res;
}

planning_dual_result_ptr HUMPlanner::plan_dual_move_move(hump_dual_params& params, std::vector<double> initPosture_right, std::vector<double> finalPosture_right, std::vector<double> initPosture_left, std::vector<double> finalPosture_left)
{
    planning_dual_result_ptr res;
    res.reset(new planning_dual_result);

    int dual_mov_type = 2; // move right and move left
    int mov_type_right = 2; // move
    int mov_type_left = 2; // move
    bool coll_right = params.mov_specs_right.coll;
    bool coll_left = params.mov_specs_left.coll;
    res->mov_type_right = mov_type_right;
    res->mov_type_left = mov_type_left;
    std::vector<double> finalHand_right = params.mov_specs_right.finalHand;
    std::vector<double> finalHand_left = params.mov_specs_left.finalHand;
    std::vector<double> initPosture;
    std::vector<double> minLimits; std::vector<double> maxLimits;
    std::vector<double> minLimits_right; std::vector<double> maxLimits_right;
    std::vector<double> minLimits_left; std::vector<double> maxLimits_left;
    minLimits_right = this->minRightLimits; maxLimits_right = this->maxRightLimits;
    minLimits_left = this->minLeftLimits; maxLimits_left = this->maxLeftLimits;

    //res->object_right_id = params.mov_specs_right.obj->getName();
    //res->object_left_id = params.mov_specs_left.obj->getName();
    //bool approach_right = params.mov_specs_right.approach;
    //bool approach_left = params.mov_specs_left.approach;
    //bool retreat_right = params.mov_specs_right.retreat;
    //bool retreat_left = params.mov_specs_left.retreat;
    //bool straight_line_right = params.mov_specs_right.straight_line;
    //bool straight_line_left = params.mov_specs_left.straight_line;

    int pre_post = 0; // 0 = use no options, 1 = use approach options, 2 = use retreat options
    int mod = 0; // 0 = move, 1 = pre_approach, 2 = approach, 3 = retreat

    initPosture = initPosture_right;
    initPosture.insert(initPosture.end(),initPosture_left.begin(),initPosture_left.end());
    std::vector<double> finalPosture;
    finalPosture = finalPosture_right;
    finalPosture.insert(finalPosture.end(),finalPosture_left.begin(),finalPosture_left.end());
    minLimits = minLimits_right;
    minLimits.insert(minLimits.end(),minLimits_left.begin(),minLimits_left.end());
    maxLimits = maxLimits_right;
    maxLimits.insert(maxLimits.end(),maxLimits_left.begin(),maxLimits_left.end());

    try
    {
        // the posture is given by
        // right arm(7) + right hand(4) + left arm(7) + left hand(4) = 22 joints
        std::vector<double> hand_r; std::vector<double> hand_l;
        std::vector<double> bouncePosture; bool BPosture = false;
        std::vector<double> finalPosture_ext;
        // extend the final posture
        finalPosture_ext = finalPosture_right;
        finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm,finalHand_right.at(0));
        for(size_t i=1;i<finalHand_right.size();++i){
            if(((finalHand_right.at(i)) > minLimits_right.at(i+joints_arm))){
                hand_r.push_back(finalHand_right.at(i));
            }else{
               hand_r.push_back(minLimits_right.at(i+joints_arm));
            }
        }
        finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm+1,hand_r.begin(),hand_r.end());

        finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm+joints_hand,finalPosture_left.begin(),finalPosture_left.end());

        finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm+joints_hand+joints_arm,finalHand_left.at(0));
        for(size_t i=1;i<finalHand_left.size();++i){
            if(((finalHand_left.at(i)) > minLimits_left.at(i+joints_arm))){
                hand_l.push_back(finalHand_left.at(i));
            }else{
               hand_l.push_back(minLimits_left.at(i+joints_arm));
            }
        }
        finalPosture_ext.insert(finalPosture_ext.begin()+joints_arm+joints_hand+joints_arm+1,hand_l.begin(),hand_l.end());
        // get the steps of the trajectory
        int steps = this->getSteps(maxLimits,minLimits,initPosture,finalPosture_ext);

        if(coll_right && coll_left){// collisions
            BPosture = this->singleDualArmBouncePosture(steps,dual_mov_type,pre_post,params,initPosture,finalPosture,bouncePosture);
            if(BPosture){
                res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully");
                res->time_steps.clear();
                res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
                res->velocity_stages.clear();
                res->acceleration_stages.clear();
                // plan stage
                MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; bool success = true;
                timestep = this->getDualAcceleration(dual_mov_type,steps,params,initPosture,finalPosture_ext,bouncePosture,traj,vel,acc,success,mod);
                // plan
                res->time_steps.push_back(timestep);
                res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
                res->velocity_stages.push_back(vel);
                res->acceleration_stages.push_back(acc);
            }else{ res->status = 20; res->status_msg = string("HUMP: bounce posture pre place selection failed ");}
        }else{ // no collisions
            res->status = 0; res->status_msg = string("HUMP: trajectory planned successfully ");
            res->time_steps.clear();
            res->trajectory_stages.clear(); res->trajectory_descriptions.clear();
            res->velocity_stages.clear();
            res->acceleration_stages.clear();
            // plan stage
            MatrixXd traj; MatrixXd vel; MatrixXd acc; double timestep; bool success = true;
            timestep = this->getDualAcceleration(dual_mov_type,steps,params,initPosture,finalPosture_ext,traj,vel,acc,success,mod);
            res->time_steps.push_back(timestep);
            res->trajectory_stages.push_back(traj); res->trajectory_descriptions.push_back("plan");
            res->velocity_stages.push_back(vel);
            res->acceleration_stages.push_back(acc);
        }
    }catch (const string message){throw message;
    }catch( ... ){throw string ("HUMP: error in optimizing the trajecory");}

    return res;
}


bool HUMPlanner::singleDualArmFinalPosture(int dual_mov_type,int pre_post,hump_dual_params& params,std::vector<double> initPosture,std::vector<double>& finalPosture)
{
    // movement settings
    //int arm_code = params.mov_specs.arm_code;
    int hand_code_right = params.mov_specs_right.hand_code;
    int hand_code_left = params.mov_specs_left.hand_code;
    bool rand_init_right = params.mov_specs_right.rand_init;
    bool rand_init_left = params.mov_specs_left.rand_init;
    bool straight_right = params.mov_specs_right.straight_line;
    bool straight_left = params.mov_specs_left.straight_line;
    std::vector<double> target_right = params.mov_specs_right.target;
    std::vector<double> target_left = params.mov_specs_left.target;
    std::vector<double> minLimits_right; std::vector<double> minLimits_left;
    std::vector<double> maxLimits_right; std::vector<double> maxLimits_left;
    bool approach_right = params.mov_specs_right.approach;
    bool approach_left = params.mov_specs_left.approach;
    bool retreat_right = params.mov_specs_right.retreat;
    bool retreat_left = params.mov_specs_left.retreat;
    std::vector<double> approach_right_vec; std::vector<double> approach_left_vec;
    std::vector<double> retreat_right_vec; std::vector<double> retreat_left_vec;
    switch(dual_mov_type){
    case 0: // pick right pick left
        if(approach_right){approach_right_vec = params.mov_specs_right.pre_grasp_approach;}
        if(approach_left){approach_left_vec = params.mov_specs_left.pre_grasp_approach;}
        if(retreat_right){retreat_right_vec = params.mov_specs_right.post_grasp_retreat;}
        if(retreat_left){retreat_left_vec = params.mov_specs_left.post_grasp_retreat;}
        break;
    case 1: // place right place left
        if(approach_right){approach_right_vec = params.mov_specs_right.pre_place_approach;}
        if(approach_left){approach_left_vec = params.mov_specs_left.pre_place_approach;}
        if(retreat_right){retreat_right_vec = params.mov_specs_right.post_place_retreat;}
        if(retreat_left){retreat_left_vec = params.mov_specs_left.post_place_retreat;}
        break;
    }

    std::vector<double> initRightPosture(initPosture.begin(),initPosture.begin()+joints_arm+joints_hand);
    std::vector<double> initLeftPosture(initPosture.begin()+joints_arm+joints_hand,initPosture.begin()+2*(joints_arm+joints_hand));
    std::vector<double> initRightArmPosture(initPosture.begin(),initPosture.begin()+joints_arm);
    std::vector<double> initLeftArmPosture(initPosture.begin()+joints_arm+joints_hand,initPosture.begin()+joints_arm+joints_hand+joints_arm);

    double Lu_right; double Ll_right; double Lh_right;
    Lu_right = abs(this->DH_rightArm.d.at(2));
    Ll_right = abs(this->DH_rightArm.d.at(4));
    Lh_right = abs(this->DH_rightArm.d.at(6));
    minLimits_right = this->minRightLimits;
    maxLimits_right = this->maxRightLimits;
    double Lu_left; double Ll_left; double Lh_left;
    Lu_left = abs(this->DH_leftArm.d.at(2));
    Ll_left = abs(this->DH_leftArm.d.at(4));
    Lh_left = abs(this->DH_leftArm.d.at(6));
    minLimits_left = this->minLeftLimits;
    maxLimits_left = this->maxLeftLimits;

    std::vector<double> shPos_right; this->getShoulderPos(1,initRightPosture,shPos_right);
    double max_ext_right = Lh_right+Ll_right+Lu_right;
    if(!straight_right){
        // check if the target is in the workspace of the right arm
        Vector3d tar_pos(target_right.at(0),target_right.at(1),target_right.at(2));
        Vector3d tar_pos_app(target_right.at(0),target_right.at(1),target_right.at(2));
        Vector3d tar_pos_ret(target_right.at(0),target_right.at(1),target_right.at(2));
        if(approach_right){
            std::vector<double> rpy = {target_right.at(3),target_right.at(4),target_right.at(5)};
            Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
            double dist = approach_right_vec.at(3);
            Vector3d v(approach_right_vec.at(0),approach_right_vec.at(1),approach_right_vec.at(2));
            Vector3d vv = Rot_tar*v;
            tar_pos_app = tar_pos_app + dist*vv;
        }else if(retreat_right){
            std::vector<double> rpy = {target_right.at(3),target_right.at(4),target_right.at(5)};
            Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
            double dist = retreat_right_vec.at(3);
            Vector3d v(retreat_right_vec.at(0),retreat_right_vec.at(1),retreat_right_vec.at(2));
            Vector3d vv = Rot_tar*v;
            tar_pos_ret = tar_pos_ret + dist*vv;
        }
        if((sqrt(pow(tar_pos(0) - shPos_right.at(0),2)+
                pow(tar_pos(1) - shPos_right.at(1),2)+
                pow(tar_pos(2) - shPos_right.at(2),2))>= max_ext_right)||
                (sqrt(pow(tar_pos_app(0) - shPos_right.at(0),2)+
                      pow(tar_pos_app(1) - shPos_right.at(1),2)+
                      pow(tar_pos_app(2) - shPos_right.at(2),2))>= max_ext_right)||
                (sqrt(pow(tar_pos_ret(0) - shPos_right.at(0),2)+
                      pow(tar_pos_ret(1) - shPos_right.at(1),2)+
                      pow(tar_pos_ret(2) - shPos_right.at(2),2))>= max_ext_right)){
            throw string("The movement to be planned goes out of the right arm reachable workspace");
        }
    }
    std::vector<double> shPos_left; this->getShoulderPos(2,initLeftPosture,shPos_left);
    double max_ext_left = Lh_left+Ll_left+Lu_left;
    if(!straight_left){
        // check if the target is in the workspace of the left arm
        Vector3d tar_pos(target_left.at(0),target_left.at(1),target_left.at(2));
        Vector3d tar_pos_app(target_left.at(0),target_left.at(1),target_left.at(2));
        Vector3d tar_pos_ret(target_left.at(0),target_left.at(1),target_left.at(2));
        if(approach_left){
            std::vector<double> rpy = {target_left.at(3),target_left.at(4),target_left.at(5)};
            Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
            double dist = approach_left_vec.at(3);
            Vector3d v(approach_left_vec.at(0),approach_left_vec.at(1),approach_left_vec.at(2));
            Vector3d vv = Rot_tar*v;
            tar_pos_app = tar_pos_app + dist*vv;
        }else if(retreat_left){
            std::vector<double> rpy = {target_left.at(3),target_left.at(4),target_left.at(5)};
            Matrix3d Rot_tar; this->RPY_matrix(rpy,Rot_tar);
            double dist = retreat_left_vec.at(3);
            Vector3d v(retreat_left_vec.at(0),retreat_left_vec.at(1),retreat_left_vec.at(2));
            Vector3d vv = Rot_tar*v;
            tar_pos_ret = tar_pos_ret + dist*vv;
        }
        if((sqrt(pow(tar_pos(0) - shPos_left.at(0),2)+
                pow(tar_pos(1) - shPos_left.at(1),2)+
                pow(tar_pos(2) - shPos_left.at(2),2))>= max_ext_left)||
                (sqrt(pow(tar_pos_app(0) - shPos_left.at(0),2)+
                      pow(tar_pos_app(1) - shPos_left.at(1),2)+
                      pow(tar_pos_app(2) - shPos_left.at(2),2))>= max_ext_left)||
                (sqrt(pow(tar_pos_ret(0) - shPos_left.at(0),2)+
                      pow(tar_pos_ret(1) - shPos_left.at(1),2)+
                      pow(tar_pos_ret(2) - shPos_left.at(2),2))>= max_ext_left)){
            throw string("The movement to be planned goes out of the left arm reachable workspace");
        }
    }

    // initial guess
    std::vector<double> minRightArmLimits(minLimits_right.begin(),minLimits_right.begin()+joints_arm);
    std::vector<double> maxRightArmLimits(maxLimits_right.begin(),maxLimits_right.begin()+joints_arm);
    std::vector<double> minLeftArmLimits(minLimits_left.begin(),minLimits_left.begin()+joints_arm);
    std::vector<double> maxLeftArmLimits(maxLimits_left.begin(),maxLimits_left.begin()+joints_arm);
    std::vector<double> initialGuess(minRightArmLimits.size()+minLeftArmLimits.size(),0.0);
    if ((pre_post==1) && rand_init_right && rand_init_left){ // pre_posture for approaching
        for(size_t i=0; i < minRightArmLimits.size() + minLeftArmLimits.size();++i){
            if(i < minRightArmLimits.size()){
                initialGuess.at(i)=getRand(minRightArmLimits.at(i)+SPACER,maxRightArmLimits.at(i)-SPACER);
            }else{
                initialGuess.at(i)=getRand(minLeftArmLimits.at(i-minLeftArmLimits.size())+SPACER,maxLeftArmLimits.at(i-minLeftArmLimits.size())-SPACER);
            }
        }
        // TO DO the combos rand_init_right && !rand_init_left and !rand_init_right && rand_init_left
    }else{
        initialGuess = initRightArmPosture;
        initialGuess.insert(initialGuess.end(),initLeftArmPosture.begin(),initLeftArmPosture.end());
    }

    // get the obstacles of the workspace
    std::vector<objectPtr> obsts_right; std::vector<objectPtr> obsts_left;
    this->getObstaclesDualArm(shPos_right,shPos_left,max_ext_right,max_ext_left,obsts_right,obsts_left,hand_code_right,hand_code_left);
    if(dual_mov_type==1 && pre_post==0){ // dual place movement, plan stage
        // remove the support object from the obstacles of the right arm
        if(params.mov_specs_right.support_obj.compare("")){
            std::string support_obj_name = params.mov_specs_right.support_obj;
            for(size_t i=0; i < obsts_right.size();++i){
                objectPtr curr_obj = obsts_right.at(i);
                std::string curr_obj_name = curr_obj->getName();
                if(support_obj_name.compare(curr_obj_name)==0){
                    obsts_right.erase (obsts_right.begin()+i);
                }
            }
        }
        // remove the support object from the obstacles of the left arm
        if(params.mov_specs_left.support_obj.compare("")){
            std::string support_obj_name = params.mov_specs_left.support_obj;
            for(size_t i=0; i < obsts_left.size();++i){
                objectPtr curr_obj = obsts_left.at(i);
                std::string curr_obj_name = curr_obj->getName();
                if(support_obj_name.compare(curr_obj_name)==0){
                    obsts_left.erase (obsts_left.begin()+i);
                }
            }
        }
    }

    // write the files for the final posture selection
    bool written = this->writeFilesDualFinalPosture(params,dual_mov_type,pre_post,initRightArmPosture,initLeftArmPosture,initialGuess,obsts_right,obsts_left);


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
                if (this->optimize(nlfile,x_sol,FINAL_DUAL_TOL,FINAL_DUAL_ACC_TOL,FINAL_DUAL_CONSTR_VIOL_TOL)){
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

bool HUMPlanner::writeFilesDualFinalPosture(hump_dual_params& params,int dual_mov_type, int pre_post,std::vector<double> initRightArmPosture, std::vector<double> initLeftArmPosture, std::vector<double> initialGuess,std::vector<objectPtr> obsts_right,std::vector<objectPtr> obsts_left)
{
    //  --- create the "Models" directory if it does not exist ---
    struct stat st = {0};
    if (stat("Models", &st) == -1) {
        mkdir("Models", 0700);
    }
    string path("Models/");

    // movement settings
    //int arm_code = params.mov_specs.arm_code;
    int hand_code_right = params.mov_specs_right.hand_code;
    int hand_code_left = params.mov_specs_left.hand_code;
    //int griptype = params.mov_specs.griptype;
    bool coll_right = params.mov_specs_right.coll;
    bool coll_left = params.mov_specs_left.coll;
    bool coll_body = params.coll_body;
    bool coll_arms = params.coll_arms;
    std::vector<double> tar_right = params.mov_specs_right.target;
    std::vector<double> tar_left = params.mov_specs_left.target;
    Matrix4d T_tar_to_obj_right = params.mov_specs_right.T_tar_to_obj;
    Matrix4d T_tar_to_obj_left = params.mov_specs_left.T_tar_to_obj;
    int dHO_right = params.mov_specs_right.dHO;
    int dHO_left = params.mov_specs_left.dHO;
    std::vector<double> finalHand_right = params.mov_specs_right.finalHand;
    std::vector<double> finalHand_left = params.mov_specs_left.finalHand;
    std::string mov_infoLine_right = params.mov_specs_right.mov_infoline;
    std::string mov_infoLine_left = params.mov_specs_left.mov_infoline;
    objectPtr obj_tar_right = params.mov_specs_right.obj;
    objectPtr obj_tar_left = params.mov_specs_left.obj;
    bool approach_right = params.mov_specs_right.approach;
    bool approach_left = params.mov_specs_left.approach;
    bool retreat_right = params.mov_specs_right.retreat;
    bool retreat_left = params.mov_specs_left.retreat;
    std::vector<double> pre_grasp_approach_right; std::vector<double> pre_grasp_approach_left;
    std::vector<double> post_grasp_retreat_right; std::vector<double> post_grasp_retreat_left;
    std::vector<double> pre_place_approach_right; std::vector<double> pre_place_approach_left;
    std::vector<double> post_place_retreat_right; std::vector<double> post_place_retreat_left;

    switch(dual_mov_type){
    case 0: // pick right pick left
        if(approach_right){pre_grasp_approach_right = params.mov_specs_right.pre_grasp_approach;}
        if(approach_left){pre_grasp_approach_left = params.mov_specs_left.pre_grasp_approach;}
        if(retreat_right){post_grasp_retreat_right = params.mov_specs_right.post_grasp_retreat;}
        if(retreat_left){post_grasp_retreat_left = params.mov_specs_left.post_grasp_retreat;}
        break;
    case 1: // place right place left
        if(approach_right){pre_place_approach_right = params.mov_specs_right.pre_place_approach;}
        if(approach_left){pre_place_approach_left = params.mov_specs_left.pre_place_approach;}
        if(retreat_right){post_place_retreat_right = params.mov_specs_right.post_place_retreat;}
        if(retreat_left){post_place_retreat_left = params.mov_specs_left.post_place_retreat;}
        break;
    }

    // tolerances
    std::vector<double> lambda_right(params.lambda_final_right.begin(),params.lambda_final_right.begin()+joints_arm);
    std::vector<double> lambda_left(params.lambda_final_left.begin(),params.lambda_final_left.begin()+joints_arm);
    std::vector<double> tolsArm_right = params.tolsArm_right; std::vector<double> tolsArm_left = params.tolsArm_left;
    MatrixXd tolsHand_right = params.tolsHand_right; MatrixXd tolsHand_left = params.tolsHand_left;
    MatrixXd tolsObstacles_right = params.final_tolsObstacles_right; MatrixXd tolsObstacles_left = params.final_tolsObstacles_left;
    double tolTarPos_right = params.tolTarPos_right; double tolTarPos_left = params.tolTarPos_left;
    double tolTarOr_right = params.tolTarOr_right; double tolTarOr_left = params.tolTarOr_left;
    bool obstacle_avoidance = params.obstacle_avoidance;

    Matrix4d matWorldToArm_right; Matrix4d matWorldToArm_left;
    Matrix4d matHand_right; Matrix4d matHand_left;
    std::vector<double> minLimits_right; std::vector<double> minLimits_left;
    std::vector<double> maxLimits_right; std::vector<double> maxLimits_left;
    DHparameters dh_arm_right; DHparameters dh_arm_left;

    matWorldToArm_right = this->matWorldToRightArm; matWorldToArm_left = this->matWorldToLeftArm;
    matHand_right = this->matRightHand; matHand_left = this->matLeftHand;
    minLimits_right = this->minRightLimits; minLimits_left = this->minLeftLimits;
    maxLimits_right = this->maxRightLimits; maxLimits_left = this->maxLeftLimits;
    dh_arm_right = this->DH_rightArm; dh_arm_left = this->DH_leftArm;

    std::vector<double> minRightArmLimits(minLimits_right.begin(),minLimits_right.begin()+joints_arm);
    std::vector<double> maxRightArmLimits(maxLimits_right.begin(),maxLimits_right.begin()+joints_arm);
    std::vector<double> minLeftArmLimits(minLimits_left.begin(),minLimits_left.begin()+joints_arm);
    std::vector<double> maxLeftArmLimits(maxLimits_left.begin(),maxLimits_left.begin()+joints_arm);

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
    if((coll_right || coll_left) && coll_body){
        this->writeBodyDim(this->torso_size.at(0),this->torso_size.at(1),PostureDat);
    }
    // D-H Parameters of the Arms
    this->writeDualArmDHParams(dh_arm_right,dh_arm_left,PostureDat);
    // distance between the hands and the object
    this->write_dual_dHO(PostureDat,dHO_right,dHO_left);
    // joint limits of the arms
    this->writeDualArmLimits(PostureDat,minRightArmLimits,maxRightArmLimits,minLeftArmLimits,maxLeftArmLimits,true);
    // initial pose of the arms
    this->writeDualArmInitPose(PostureDat,initRightArmPosture,initLeftArmPosture);
    // final posture of the fingers
    this->writeFingerDualFinalPose(PostureDat,finalHand_right,finalHand_left);
    // joint expense factors of the arms
    this->writeDualLambda(PostureDat,lambda_right,lambda_left);
    // initial guess
    PostureDat << string("# INITIAL GUESS \n");
    PostureDat << string("var theta = \n");
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
    switch(hand_code_right){
    case 0: // human hand
        this->writeDualHumanHandParams(this->hhand,PostureDat,true);
        break;
    case 1: // barrett hand
        this->writeDualBarrettHandParams(this->bhand,PostureDat,true);
        break;
    }
    switch(hand_code_left){
    case 0: // human hand
        this->writeDualHumanHandParams(this->hhand,PostureDat,false);
        break;
    case 1: // barrett hand
        this->writeDualBarrettHandParams(this->bhand,PostureDat,false);
        break;
    }
    // info of the targets to reach
    this->writeDualInfoTarget(PostureDat,tar_right,tar_left);
    // info approach/retreat
    switch(dual_mov_type){
    case 0: // pick right pick left
        switch(pre_post){
        case 0: // no approach, no retreat
            break;
        case 1: // approach
            if(approach_right){this->writeDualInfoApproachRetreat(PostureDat,tar_right,pre_grasp_approach_right,true);}
            if(approach_left){this->writeDualInfoApproachRetreat(PostureDat,tar_left,pre_grasp_approach_left,false);}
            break;
        case 2: // retreat
            if(retreat_right){this->writeDualInfoApproachRetreat(PostureDat,tar_right,post_grasp_retreat_right,true);}
            if(retreat_left){this->writeDualInfoApproachRetreat(PostureDat,tar_left,post_grasp_retreat_left,false);}
            break;
        }
        break;
    case 1: // place right place left
        switch(pre_post){
        case 0: // no approach, no retreat
            break;
        case 1: // approach
            if(approach_right){this->writeDualInfoApproachRetreat(PostureDat,tar_right,pre_place_approach_right,true);}
            if(approach_left){this->writeDualInfoApproachRetreat(PostureDat,tar_left,pre_place_approach_left,false);}
            break;
        case 2: // retreat
            if(retreat_right){this->writeDualInfoApproachRetreat(PostureDat,tar_right,post_place_retreat_right,true);}
            if(retreat_left){this->writeDualInfoApproachRetreat(PostureDat,tar_left,post_place_retreat_left,false);}
            break;
        }
        break;
    }
    if(coll_right){
        //info obstacles of the right arm-hand
        this->writeDualInfoObstacles(PostureDat,obsts_right,true);
    }
    if(coll_left){
        //info obstacles of the left arm-hand
        this->writeDualInfoObstacles(PostureDat,obsts_left,false);
    }
    // object that has the target
    std::vector<double> dim_right; std::vector<double> dim_left;    
    switch(dual_mov_type){
    case 0: //dual pick
        obj_tar_right->getSize(dim_right); obj_tar_left->getSize(dim_left);
        if(pre_post==2){ // retreat stage
            this->writeDualInfoObjectTarget(PostureDat,tar_right,T_tar_to_obj_right,dim_right, obj_tar_right->getName(),tar_left,T_tar_to_obj_left,dim_left,obj_tar_left->getName());
        }else{
            this->writeDualInfoObjectTarget(PostureDat,obj_tar_right,obj_tar_left);
        }
        break;
    case 1: // dual place
        obj_tar_right->getSize(dim_right); obj_tar_left->getSize(dim_left);
        if(pre_post==2){ // retreat stage
            this->writeDualInfoObjectTargetPlaceRetreat(PostureDat,tar_right,T_tar_to_obj_right,dim_right, obj_tar_right->getName(),tar_left,T_tar_to_obj_left,dim_left,obj_tar_left->getName());
            //this->writeDualInfoObjectTarget(PostureDat,tar_right,T_tar_to_obj_right,dim_right, obj_tar_right->getName(),tar_left,T_tar_to_obj_left,dim_left,obj_tar_left->getName());
        }else{
            this->writeDualInfoObjectTarget(PostureDat,tar_right,T_tar_to_obj_right,dim_right, obj_tar_right->getName(),tar_left,T_tar_to_obj_left,dim_left,obj_tar_left->getName());
        }
        break;
    }
    //close the file
    //PostureDat.close();

    // ------------- Write the mod file ------------------------- //
    string filenamemod("FinalPosture.mod");
    ofstream PostureMod;
    // open the file
    PostureMod.open(path+filenamemod);
    PostureMod << string("# DUAL FINAL POSTURE MODEL FILE \n");
    PostureMod << string("# Right arm Movement to plan: \n");
    PostureMod << string("# ")+mov_infoLine_right+string("\n");
    PostureMod << string("# Left arm Movement to plan: \n");
    PostureMod << string("# ")+mov_infoLine_left+string("\n\n");

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("# PARAMETERS \n\n");
    PostureMod << string("set nJoints := 1..")+to_string(initialGuess.size())+string(";\n");

    this->writePI(PostureMod);
    if((coll_right || coll_left) && coll_body){
        this->writeBodyDimMod(PostureMod);
    }
    this->writeDualArmDHParamsMod(PostureMod);
    this->write_dual_dHOMod(PostureMod);

    PostureMod << string("# Joint Limits \n");
    PostureMod << string("param llim {i in 1..")+to_string(initialGuess.size())+string("} ; \n");
    PostureMod << string("param ulim {i in 1..")+to_string(initialGuess.size())+string("} ; \n");

    PostureMod << string("# Initial posture \n");
    PostureMod << string("param thet_init {i in 1..")+to_string(initialGuess.size())+string("} ; \n");

    PostureMod << string("# Final finger right posture \n");
    PostureMod << string("param joint_fingers_right {i in 1..")+to_string(finalHand_right.size())+string("} ; \n");
    PostureMod << string("# Final finger left posture \n");
    PostureMod << string("param joint_fingers_left {i in 1..")+to_string(finalHand_left.size())+string("} ; \n");

    PostureMod << string("# Joint Expense Factors \n");
    PostureMod << string("param lambda {i in 1..")+to_string(initialGuess.size())+string("} ; \n");

    switch(hand_code_right){
    case 0: // human hand
        this->writeDualHumanHandParamsMod(PostureMod,true);
        break;
    case 1: // barrett hand
        this->writeDualBarrettHandParamsMod(PostureMod,true);
        break;
    }
    switch(hand_code_left){
    case 0: // human hand
        this->writeDualHumanHandParamsMod(PostureMod,false);
        break;
    case 1: // barrett hand
        this->writeDualBarrettHandParamsMod(PostureMod,false);
        break;
    }
    // info objects
    bool vec_right = false;// true if there is some pre or post operation (right)
    bool vec_left = false;// true if there is some pre or post operation (left)
    if((approach_right || retreat_right) && pre_post!=0){vec_right=true;}
    if((approach_left || retreat_left) && pre_post!=0){vec_left=true;}
    this->writeDualInfoObjectsMod(PostureMod,vec_right,vec_left);

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("# DECISION VARIABLES \n");
    PostureMod << string("var theta {i in 1..")+to_string(initialGuess.size())+string("} >= llim[i], <= ulim[i]; \n");
    // Rotation matrix of the obstacles
    this->writeDualRotMatObsts(PostureMod);
    // Direct Kinematics of the arms
    this->writeDualArmDirKin(PostureMod,dual_mov_type,matWorldToArm_right,matHand_right,tolsArm_right,matWorldToArm_left,matHand_left,tolsArm_left,true);

    //PostureMod << string("# TESTS \n");
    //this->writeInitDualArmDirKin(PostureMod,tolsArm_right,tolsArm_left);


    bool obj_right_place = false; bool obj_left_place = false;
    // object to transport (dual place movements in the plan and approach target posture selection or
    //                        dual pick movement in the retreat target posture selection)
    int n_s_dual = 0; // number of spheres of the object transported with both hands
    int n_s_right = 0; // number of spheres of the object (right)
    int n_s_left = 0; // number of spheres of the object (left)
    if((dual_mov_type==1 && (pre_post==0 || pre_post==1)) || (dual_mov_type==0 && pre_post==2)){
    //if((dual_mov_type==1 && (pre_post==0 || pre_post==1))){
        obj_right_place = true; obj_left_place = true;
        std::vector<double> obj_tar_right_size; std::string obj_right_name = obj_tar_right->getName();
        obj_tar_right->getSize(obj_tar_right_size);
        std::vector<double> obj_tar_left_size; std::string obj_left_name =obj_tar_left->getName();
        obj_tar_left->getSize(obj_tar_left_size);
        if(obj_right_name.compare(obj_left_name)==0){
            n_s_dual = this->dual_obj_model_spheres(PostureDat,PostureMod,obj_tar_right_size,true);
        }else{
            this->dual_obj_model_spheres(PostureDat,PostureMod,obj_tar_right_size,obj_tar_left_size,true,n_s_right,n_s_left);
        }
    }
    switch(hand_code_right){
    case 0: // human hand
        this->writeDualHumanHandDirKin(PostureMod,tolsHand_right,true,false,false,true);
        break;
    case 1: // barrett hand
        this->writeDualBarrettHandDirKin(PostureMod,tolsHand_right,true,false,false,true);
        break;
    }    
    switch(hand_code_left){
    case 0: // human hand
        this->writeDualHumanHandDirKin(PostureMod,tolsHand_left,true,false,false,false);
        break;
    case 1: // barrett hand
        this->writeDualBarrettHandDirKin(PostureMod,tolsHand_left,true,false,false,false);
        break;
    }

    // Points of the right arm
    //PostureMod << string("var Points_Arm_right {j in 1..21, i in 1..4} = \n");
    std::string n_str_right;
    if(obj_right_place && n_s_dual==0){
        n_str_right = to_string(15+n_s_right);
    }else if(obj_right_place && n_s_dual!=0){
        n_str_right = to_string(15+n_s_dual);
    }else{
       n_str_right = to_string(15);
    }
    PostureMod << string("var Points_Arm_right {j in 1..")+n_str_right+string(", i in 1..4} = \n");
    PostureMod << string("if ( j=1 ) then 	(Shoulder_right[i]+Elbow_right[i])/2  \n");
    PostureMod << string("else	if ( j=2 ) then 	Elbow_right[i] \n");
    PostureMod << string("else    if ( j=3 ) then 	(Wrist_right[i]+Elbow_right[i])/2  \n");
    PostureMod << string("else	if ( j=4 ) then 	Wrist_right[i] \n");
    PostureMod << string("else	if ( j=5 ) then 	Wrist_right[i]+0.45*(Hand_right[i]-Wrist_right[i]) \n");
    PostureMod << string("else	if ( j=6 ) then 	Wrist_right[i]+0.75*(Hand_right[i]-Wrist_right[i]) \n");
    PostureMod << string("else	if ( j=7 ) then 	Finger1_1_right[i] \n");
    PostureMod << string("else	if ( j=8 ) then 	Finger2_1_right[i] \n");
    PostureMod << string("else	if ( j=9 ) then 	Finger3_1_right[i]\n");
    /*
    PostureMod << string("else	if ( j=10 ) then 	(Finger1_1_right[i]+Finger1_2_right[i])/2 \n");
    PostureMod << string("else	if ( j=11 ) then 	(Finger2_1_right[i]+Finger2_2_right[i])/2 \n");
    PostureMod << string("else	if ( j=12 ) then 	(Finger3_1_right[i]+Finger3_2_right[i])/2 \n");
    PostureMod << string("else	if ( j=13 ) then 	 Finger1_2_right[i] \n");
    PostureMod << string("else	if ( j=14 ) then 	 Finger2_2_right[i] \n");
    PostureMod << string("else	if ( j=15 ) then 	 Finger3_2_right[i] \n");
    PostureMod << string("else	if ( j=16 ) then 	(Finger1_2_right[i]+Finger1_tip_right[i])/2	 \n");
    PostureMod << string("else	if ( j=17 ) then 	(Finger2_2_right[i]+Finger2_tip_right[i])/2 \n");
    PostureMod << string("else	if ( j=18 ) then 	(Finger3_2_right[i]+Finger3_tip_right[i])/2 \n");
    PostureMod << string("else	if ( j=19 ) then 	Finger1_tip_right[i]\n");
    PostureMod << string("else	if ( j=20 ) then 	Finger2_tip_right[i] \n");
    PostureMod << string("else	if ( j=21 ) then 	Finger3_tip_right[i] \n");
    */
    PostureMod << string("else	if ( j=10 ) then 	 Finger1_2_right[i] \n");
    PostureMod << string("else	if ( j=11 ) then 	 Finger2_2_right[i] \n");
    PostureMod << string("else	if ( j=12 ) then 	 Finger3_2_right[i] \n");
    PostureMod << string("else	if ( j=13 ) then 	Finger1_tip_right[i]\n");
    PostureMod << string("else	if ( j=14 ) then 	Finger2_tip_right[i] \n");
    PostureMod << string("else	if ( j=15 ) then 	Finger3_tip_right[i] \n");

    if(n_s_dual==0)
    {
        if (obj_right_place){
            int j_init = 15;
            for(int i=1;i<=n_s_right;++i){
                std::string i_str = to_string(i);
                int j = j_init+i; std::string j_str = to_string(j);
                PostureMod << string("else    if ( j=")+j_str+string(" ) then 	ObjRight2Transp_")+i_str+string("[i] \n");
            }
        }
    }else{
        if (obj_right_place){
            int j_init = 15;
            for(int i=1;i<=n_s_dual;++i){
                std::string i_str = to_string(i);
                int j = j_init+i; std::string j_str = to_string(j);
                PostureMod << string("else    if ( j=")+j_str+string(" ) then 	Obj2Transp_")+i_str+string("[i] \n");
            }
        }
    }

    PostureMod << string("; \n\n");

    // Points of the left arm
    //PostureMod << string("var Points_Arm_left {j in 1..21, i in 1..4} = \n");
    std::string n_str_left;
    if(obj_left_place && n_s_dual==0){
        n_str_left = to_string(15+n_s_left);
    }else if(obj_left_place && n_s_dual!=0){
        //n_str_left = to_string(15+n_s_dual);
        n_str_left = to_string(15);
    }else{
       n_str_left = to_string(15);
    }
    PostureMod << string("var Points_Arm_left {j in 1..")+n_str_left+string(", i in 1..4} = \n");
    PostureMod << string("if ( j=1 ) then 	(Shoulder_left[i]+Elbow_left[i])/2  \n");
    PostureMod << string("else	if ( j=2 ) then 	Elbow_left[i] \n");
    PostureMod << string("else    if ( j=3 ) then 	(Wrist_left[i]+Elbow_left[i])/2  \n");
    PostureMod << string("else	if ( j=4 ) then 	Wrist_left[i] \n");
    PostureMod << string("else	if ( j=5 ) then 	Wrist_left[i]+0.45*(Hand_left[i]-Wrist_left[i]) \n");
    PostureMod << string("else	if ( j=6 ) then 	Wrist_left[i]+0.75*(Hand_left[i]-Wrist_left[i]) \n");
    PostureMod << string("else	if ( j=7 ) then 	Finger1_1_left[i] \n");
    PostureMod << string("else	if ( j=8 ) then 	Finger2_1_left[i] \n");
    PostureMod << string("else	if ( j=9 ) then 	Finger3_1_left[i]\n");
    /*
    PostureMod << string("else	if ( j=10 ) then 	(Finger1_1_left[i]+Finger1_2_left[i])/2 \n");
    PostureMod << string("else	if ( j=11 ) then 	(Finger2_1_left[i]+Finger2_2_left[i])/2 \n");
    PostureMod << string("else	if ( j=12 ) then 	(Finger3_1_left[i]+Finger3_2_left[i])/2 \n");
    PostureMod << string("else	if ( j=13 ) then 	 Finger1_2_left[i] \n");
    PostureMod << string("else	if ( j=14 ) then 	 Finger2_2_left[i] \n");
    PostureMod << string("else	if ( j=15 ) then 	 Finger3_2_left[i] \n");
    PostureMod << string("else	if ( j=16 ) then 	(Finger1_2_left[i]+Finger1_tip_left[i])/2	 \n");
    PostureMod << string("else	if ( j=17 ) then 	(Finger2_2_left[i]+Finger2_tip_left[i])/2 \n");
    PostureMod << string("else	if ( j=18 ) then 	(Finger3_2_left[i]+Finger3_tip_left[i])/2 \n");
    PostureMod << string("else	if ( j=19 ) then 	Finger1_tip_left[i]\n");
    PostureMod << string("else	if ( j=20 ) then 	Finger2_tip_left[i] \n");
    PostureMod << string("else	if ( j=21 ) then 	Finger3_tip_left[i] \n");
    */
    PostureMod << string("else	if ( j=10 ) then 	 Finger1_2_left[i] \n");
    PostureMod << string("else	if ( j=11 ) then 	 Finger2_2_left[i] \n");
    PostureMod << string("else	if ( j=12 ) then 	 Finger3_2_left[i] \n");
    PostureMod << string("else	if ( j=13 ) then 	Finger1_tip_left[i]\n");
    PostureMod << string("else	if ( j=14 ) then 	Finger2_tip_left[i] \n");
    PostureMod << string("else	if ( j=15 ) then 	Finger3_tip_left[i] \n");

    if(n_s_dual==0)
    {
        if (obj_left_place){
            int j_init = 15;
            for(int i=1;i<=n_s_left;++i){
                std::string i_str = to_string(i);
                int j = j_init+i; std::string j_str = to_string(j);
                PostureMod << string("else    if ( j=")+j_str+string(" ) then 	ObjLeft2Transp_")+i_str+string("[i] \n");
            }
        }
    }else{
        /*
        if (obj_right_place){
            int j_init = 15;
            for(int i=1;i<=n_s_dual;++i){
                std::string i_str = to_string(i);
                int j = j_init+i; std::string j_str = to_string(j);
                PostureMod << string("else    if ( j=")+j_str+string(" ) then 	Obj2Transp_")+i_str+string("[i] \n");
            }
        }
        */
    }
    PostureMod << string("; \n\n");

    // objective function
    this->writeObjective(PostureMod,true);


    // constraints
    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
    PostureMod << string("#  \n");
    PostureMod << string("#		      Constraints                  # \n");
    PostureMod << string("#  \n");
    string tarpos_right = boost::str(boost::format("%.2f") % tolTarPos_right); boost::replace_all(tarpos_right,",",".");
    string taror_right = boost::str(boost::format("%.4f") % tolTarOr_right); boost::replace_all(taror_right,",",".");
    string tarpos_left = boost::str(boost::format("%.2f") % tolTarPos_left); boost::replace_all(tarpos_left,",",".");
    string taror_left = boost::str(boost::format("%.4f") % tolTarOr_left); boost::replace_all(taror_left,",",".");

    switch (dual_mov_type){
    case 0: // pick right and pick left
        if(pre_post == 0){
            // do not use approach/retreat options
            PostureMod << string("# Hand position right \n");
            PostureMod << string("subject to constr_hand_pos_right: (sum{i in 1..3} (Hand_right[i] + dFH_right*z_H_right[i] - Tar_pos_right[i])^2) <= ")+tarpos_right+string("; \n\n");
            PostureMod << string("# Hand position left \n");
            PostureMod << string("subject to constr_hand_pos_left: (sum{i in 1..3} (Hand_left[i] + dFH_left*z_H_left[i] - Tar_pos_left[i])^2) <= ")+tarpos_left+string("; \n\n");
        }else if(pre_post==1){
            // use approach options
            PostureMod << string("# Hand position right \n");
            PostureMod << string("subject to constr_hand_pos_right: (sum{i in 1..3} (Hand_right[i] + dFH_right*z_H_right[i] - dist_right*v_t_right[i] - Tar_pos_right[i])^2) <= ")+tarpos_right+string("; \n\n");
            PostureMod << string("# Hand position left \n");
            PostureMod << string("subject to constr_hand_pos_left: (sum{i in 1..3} (Hand_left[i] + dFH_left*z_H_left[i] - dist_left*v_t_left[i] - Tar_pos_left[i])^2) <= ")+tarpos_left+string("; \n\n");
        }else if(pre_post==2){
            // use retreat options
            PostureMod << string("# Hand position right \n");
            PostureMod << string("subject to constr_hand_pos_right: (sum{i in 1..3} (Hand_right[i] + dFH_right*z_H_right[i] - dist_right*v_t_right[i] - Tar_pos_right[i])^2) <= ")+tarpos_right+string("; \n\n");
            PostureMod << string("# Hand position left \n");
            PostureMod << string("subject to constr_hand_pos_left: (sum{i in 1..3} (Hand_left[i] + dFH_left*z_H_left[i] - dist_left*v_t_left[i] - Tar_pos_left[i])^2) <= ")+tarpos_left+string("; \n\n");
        }
        break;
    case 1: // place right place left
        if(pre_post==0){
            // do not use approach/retreat options
            PostureMod << string("# Hand position right \n");
            PostureMod << string("subject to constr_hand_pos_right: (sum{i in 1..3} (Hand_right[i] + dFH_right*z_H_right[i] - Tar_pos_right[i])^2) <= ")+tarpos_right+string("; \n\n");
            PostureMod << string("# Hand position left \n");
            PostureMod << string("subject to constr_hand_pos_left: (sum{i in 1..3} (Hand_left[i] + dFH_left*z_H_left[i] - Tar_pos_left[i])^2) <= ")+tarpos_left+string("; \n\n");
        }else if(pre_post==1){
            // use approach options
            PostureMod << string("# Hand position right \n");
            PostureMod << string("subject to constr_hand_pos_right: (sum{i in 1..3} (Hand_right[i] + dFH_right*z_H_right[i] - dist_right*v_t_right[i] - Tar_pos_right[i])^2) <= ")+tarpos_right+string("; \n\n");
            PostureMod << string("# Hand position left \n");
            PostureMod << string("subject to constr_hand_pos_left: (sum{i in 1..3} (Hand_left[i] + dFH_left*z_H_left[i] - dist_left*v_t_left[i] - Tar_pos_left[i])^2) <= ")+tarpos_left+string("; \n\n");
        }else if(pre_post==2){
            // use retreat options
            PostureMod << string("# Hand position right \n");
            PostureMod << string("subject to constr_hand_pos_right: (sum{i in 1..3} (Hand_right[i] + dFH_right*z_H_right[i] - dist_right*v_t_right[i] - Tar_pos_right[i])^2) <= ")+tarpos_right+string("; \n\n");
            PostureMod << string("# Hand position left \n");
            PostureMod << string("subject to constr_hand_pos_left: (sum{i in 1..3} (Hand_left[i] + dFH_left*z_H_left[i] - dist_left*v_t_left[i] - Tar_pos_left[i])^2) <= ")+tarpos_left+string("; \n\n");
        }
        break;
    case 2: // move right move left
        /*
        PostureMod << string("# subject to contr_hand_pos  {i in 1..3}: Hand[i] - Tar_pos[i] = 0; \n");
        PostureMod << string("subject to contr_hand_pos: (sum{i in 1..3} (Hand[i] - Tar_pos[i])^2) <= ")+tarpos+string("; \n\n");
        */
        break;
    }

    PostureMod << string("# Right Hand orientation\n");
    PostureMod << string("subject to constr_hand_orient_right: (sum{i in 1..3} (x_H_right[i] - x_t_right[i])^2 + sum{i in 1..3} (z_H_right[i] - z_t_right[i])^2 )<= ")+taror_right+string("; #  x_H_right = x_t_right and z_H_right = z_t_right \n");
    PostureMod << string("# Left Hand orientation\n");
    PostureMod << string("subject to constr_hand_orient_left: (sum{i in 1..3} (x_H_left[i] - x_t_left[i])^2 + sum{i in 1..3} (z_H_left[i] - z_t_left[i])^2 )<= ")+taror_left+string("; #  x_H_left = x_t_left and z_H_left = z_t_left \n");

    if(obstacle_avoidance && coll_right){
        // obstacles and right arm-hand
        //xx
        string txx1 = boost::str(boost::format("%.2f") % tolsObstacles_right(0,0)); boost::replace_all(txx1,",","."); if(tolsObstacles_right(0,0) >= 0){txx1=string("+")+txx1;}
        string txx2 = boost::str(boost::format("%.2f") % tolsObstacles_right(1,0)); boost::replace_all(txx2,",","."); if(tolsObstacles_right(1,0) >= 0){txx2=string("+")+txx2;}
        string txx3 = boost::str(boost::format("%.2f") % tolsObstacles_right(2,0)); boost::replace_all(txx3,",","."); if(tolsObstacles_right(2,0) >= 0){txx3=string("+")+txx3;}
        //yy
        string tyy1 = boost::str(boost::format("%.2f") % tolsObstacles_right(0,1)); boost::replace_all(tyy1,",","."); if(tolsObstacles_right(0,1) >= 0){tyy1=string("+")+tyy1;}
        string tyy2 = boost::str(boost::format("%.2f") % tolsObstacles_right(1,1)); boost::replace_all(tyy2,",","."); if(tolsObstacles_right(1,1) >= 0){tyy2=string("+")+tyy2;}
        string tyy3 = boost::str(boost::format("%.2f") % tolsObstacles_right(2,1)); boost::replace_all(tyy3,",","."); if(tolsObstacles_right(2,1) >= 0){tyy3=string("+")+tyy3;}
        //zz
        string tzz1 = boost::str(boost::format("%.2f") % tolsObstacles_right(0,2)); boost::replace_all(tzz1,",","."); if(tolsObstacles_right(0,2) >= 0){tzz1=string("+")+tzz1;}
        string tzz2 = boost::str(boost::format("%.2f") % tolsObstacles_right(1,2)); boost::replace_all(tzz2,",","."); if(tolsObstacles_right(1,2) >= 0){tzz2=string("+")+tzz2;}
        string tzz3 = boost::str(boost::format("%.2f") % tolsObstacles_right(2,2)); boost::replace_all(tzz3,",","."); if(tolsObstacles_right(2,2) >= 0){tzz3=string("+")+tzz3;}
        //xy
        string txy1 = boost::str(boost::format("%.2f") % tolsObstacles_right(0,3)); boost::replace_all(txy1,",","."); if(tolsObstacles_right(0,3) >= 0){txy1=string("+")+txy1;}
        string txy2 = boost::str(boost::format("%.2f") % tolsObstacles_right(1,3)); boost::replace_all(txy2,",","."); if(tolsObstacles_right(1,3) >= 0){txy2=string("+")+txy2;}
        string txy3 = boost::str(boost::format("%.2f") % tolsObstacles_right(2,3)); boost::replace_all(txy3,",","."); if(tolsObstacles_right(2,3) >= 0){txy3=string("+")+txy3;}
        //xz
        string txz1 = boost::str(boost::format("%.2f") % tolsObstacles_right(0,4)); boost::replace_all(txz1,",","."); if(tolsObstacles_right(0,4) >= 0){txz1=string("+")+txz1;}
        string txz2 = boost::str(boost::format("%.2f") % tolsObstacles_right(1,4)); boost::replace_all(txz2,",","."); if(tolsObstacles_right(1,4) >= 0){txz2=string("+")+txz2;}
        string txz3 = boost::str(boost::format("%.2f") % tolsObstacles_right(2,4)); boost::replace_all(txz3,",","."); if(tolsObstacles_right(2,4) >= 0){txz3=string("+")+txz3;}
        //yz
        string tyz1 = boost::str(boost::format("%.2f") % tolsObstacles_right(0,5)); boost::replace_all(tyz1,",","."); if(tolsObstacles_right(0,5) >= 0){tyz1=string("+")+tyz1;}
        string tyz2 = boost::str(boost::format("%.2f") % tolsObstacles_right(1,5)); boost::replace_all(tyz2,",","."); if(tolsObstacles_right(1,5) >= 0){tyz2=string("+")+tyz2;}
        string tyz3 = boost::str(boost::format("%.2f") % tolsObstacles_right(2,5)); boost::replace_all(tyz3,",","."); if(tolsObstacles_right(2,5) >= 0){tyz3=string("+")+tyz3;}


        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("# \n");
        //PostureMod << string("subject to obst_Arm_right{j in 1..21, i in 1..n_Obstacles_right}:  \n");
        //PostureMod << string("subject to obst_Arm_right{j in 1..15, i in 1..n_Obstacles_right}:  \n");
        PostureMod << string("subject to obst_Arm_right{j in 1..")+n_str_right+string(", i in 1..n_Obstacles_right}:  \n");
        PostureMod << string("(((Rot_right[1,1,i]*Points_Arm_right[j,1]+Rot_right[2,1,i]*Points_Arm_right[j,2]+Rot_right[3,1,i]*Points_Arm_right[j,3]\n");
        PostureMod << string("-Obstacles_right[i,1]*Rot_right[1,1,i]-Obstacles_right[i,2]*Rot_right[2,1,i]-Obstacles_right[i,3]*Rot_right[3,1,i])\n");
        PostureMod << string("/(Obstacles_right[i,4]+Points_Arm_right[j,4]")+txx1+string("))^2\n");
        PostureMod << string("+\n");
        PostureMod << string("((Rot_right[1,2,i]*Points_Arm_right[j,1]+Rot_right[2,2,i]*Points_Arm_right[j,2]+Rot_right[3,2,i]*Points_Arm_right[j,3]\n");
        PostureMod << string("-Obstacles_right[i,1]*Rot_right[1,2,i]-Obstacles_right[i,2]*Rot_right[2,2,i]-Obstacles_right[i,3]*Rot_right[3,2,i])\n");
        PostureMod << string("/(Obstacles_right[i,5]+Points_Arm_right[j,4]")+tyy1+string("))^2\n");
        PostureMod << string("+\n");
        PostureMod << string("((Rot_right[1,3,i]*Points_Arm_right[j,1]+Rot_right[2,3,i]*Points_Arm_right[j,2]+Rot_right[3,3,i]*Points_Arm_right[j,3]\n");
        PostureMod << string("-Obstacles_right[i,1]*Rot_right[1,3,i]-Obstacles_right[i,2]*Rot_right[2,3,i]-Obstacles_right[i,3]*Rot_right[3,3,i])\n");
        PostureMod << string("/(Obstacles_right[i,6]+Points_Arm_right[j,4]")+tzz1+string("))^2)\n");
        PostureMod << string(">= 1;\n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("#  \n");

        if((dual_mov_type==0 && pre_post!=2) || (dual_mov_type==1 && pre_post==2)){
            // pick movements (plan and approach stages)
            // OR
            // place movements (retreat stage)
            if(obj_tar_right->getName().compare(obj_tar_left->getName())!=0)
            {
                PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
                PostureMod << string("# \n");
                PostureMod << string("subject to objLeft_Arm_right{j in 1..")+n_str_right+string(", i in 1..n_ObjTar_left}:  \n");
                PostureMod << string("(((Rot_obj_left[1,1]*Points_Arm_right[j,1]+Rot_obj_left[2,1]*Points_Arm_right[j,2]+Rot_obj_left[3,1]*Points_Arm_right[j,3]\n");
                PostureMod << string("-ObjTar_left[i,1]*Rot_obj_left[1,1]-ObjTar_left[i,2]*Rot_obj_left[2,1]-ObjTar_left[i,3]*Rot_obj_left[3,1])\n");
                PostureMod << string("/(ObjTar_left[i,4]+Points_Arm_right[j,4]")+txx1+string("))^2\n");
                PostureMod << string("+\n");
                PostureMod << string("((Rot_obj_left[1,2]*Points_Arm_right[j,1]+Rot_obj_left[2,2]*Points_Arm_right[j,2]+Rot_obj_left[3,2]*Points_Arm_right[j,3]\n");
                PostureMod << string("-ObjTar_left[i,1]*Rot_obj_left[1,2]-ObjTar_left[i,2]*Rot_obj_left[2,2]-ObjTar_left[i,3]*Rot_obj_left[3,2])\n");
                PostureMod << string("/(ObjTar_left[i,5]+Points_Arm_right[j,4]")+tyy1+string("))^2\n");
                PostureMod << string("+\n");
                PostureMod << string("((Rot_obj_left[1,3]*Points_Arm_right[j,1]+Rot_obj_left[2,3]*Points_Arm_right[j,2]+Rot_obj_left[3,3]*Points_Arm_right[j,3]\n");
                PostureMod << string("-ObjTar_left[i,1]*Rot_obj_left[1,3]-ObjTar_left[i,2]*Rot_obj_left[2,3]-ObjTar_left[i,3]*Rot_obj_left[3,3])\n");
                PostureMod << string("/(ObjTar_left[i,6]+Points_Arm_right[j,4]")+tzz1+string("))^2)\n");
                PostureMod << string(">= 1;\n");
                PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
                PostureMod << string("#  \n");
            }
        }

        /*
        PostureMod << string("((Points_Arm_right[j,1]-Obstacles_right[i,1])^2)*(  \n");
        PostureMod << string("(Rot_right[1,1,i])^2 / ((Obstacles_right[i,4]+Points_Arm_right[j,4]")+txx1+string(")^2) + \n");
        PostureMod << string("(Rot_right[2,1,i])^2 / ((Obstacles_right[i,5]+Points_Arm_right[j,4]")+txx2+string(")^2) + \n");
        PostureMod << string("(Rot_right[3,1,i])^2 / ((Obstacles_right[i,6]+Points_Arm_right[j,4]")+txx3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm_right[j,2]-Obstacles_right[i,2])^2)*(  \n");
        PostureMod << string("(Rot_right[1,2,i])^2 / ((Obstacles_right[i,4]+Points_Arm_right[j,4]")+tyy1+string(")^2) + \n");
        PostureMod << string("(Rot_right[2,2,i])^2 / ((Obstacles_right[i,5]+Points_Arm_right[j,4]")+tyy2+string(")^2) + \n");
        PostureMod << string("(Rot_right[3,2,i])^2 / ((Obstacles_right[i,6]+Points_Arm_right[j,4]")+tyy3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm_right[j,3]-Obstacles_right[i,3])^2)*( \n");
        PostureMod << string("(Rot_right[1,3,i])^2 / ((Obstacles_right[i,4]+Points_Arm_right[j,4]")+tzz1+string(")^2) + \n");
        PostureMod << string("(Rot_right[2,3,i])^2 / ((Obstacles_right[i,5]+Points_Arm_right[j,4]")+tzz2+string(")^2) +  \n");
        PostureMod << string("(Rot_right[3,3,i])^2 / ((Obstacles_right[i,6]+Points_Arm_right[j,4]")+tzz3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm_right[j,1]-Obstacles_right[i,1])*(Points_Arm_right[j,2]-Obstacles_right[i,2])* ( \n");
        PostureMod << string("(Rot_right[1,1,i]*Rot_right[1,2,i])/((Obstacles_right[i,4]+Points_Arm_right[j,4]")+txy1+string(")^2) + \n");
        PostureMod << string("(Rot_right[2,1,i]*Rot_right[2,2,i])/((Obstacles_right[i,5]+Points_Arm_right[j,4]")+txy2+string(")^2) + \n");
        PostureMod << string("(Rot_right[3,1,i]*Rot_right[3,2,i])/((Obstacles_right[i,6]+Points_Arm_right[j,4]")+txy3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm_right[j,1]-Obstacles_right[i,1])*(Points_Arm_right[j,3]-Obstacles_right[i,3])* ( \n");
        PostureMod << string("(Rot_right[1,1,i]*Rot_right[1,3,i])/((Obstacles_right[i,4]+Points_Arm_right[j,4]")+txz1+string(")^2) + \n");
        PostureMod << string("(Rot_right[2,1,i]*Rot_right[2,3,i])/((Obstacles_right[i,5]+Points_Arm_right[j,4]")+txz2+string(")^2) + \n");
        PostureMod << string("(Rot_right[3,1,i]*Rot_right[3,3,i])/((Obstacles_right[i,6]+Points_Arm_right[j,4]")+txz3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm_right[j,2]-Obstacles_right[i,2])*(Points_Arm_right[j,3]-Obstacles_right[i,3])* ( \n");
        PostureMod << string("(Rot_right[1,2,i]*Rot_right[1,3,i])/((Obstacles_right[i,4]+Points_Arm_right[j,4]")+tyz1+string(")^2) + \n");
        PostureMod << string("(Rot_right[2,2,i]*Rot_right[2,3,i])/((Obstacles_right[i,5]+Points_Arm_right[j,4]")+tyz2+string(")^2) + \n");
        PostureMod << string("(Rot_right[3,2,i]*Rot_right[3,3,i])/((Obstacles_right[i,6]+Points_Arm_right[j,4]")+tyz3+string(")^2)) \n");
        PostureMod << string("-1 >=0; \n");
        */
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("#  \n");

    }

    if(obstacle_avoidance && coll_left){
        // obstacles and left arm-hand
        //xx
        string txx1 = boost::str(boost::format("%.2f") % tolsObstacles_left(0,0)); boost::replace_all(txx1,",","."); if(tolsObstacles_left(0,0) >= 0){txx1=string("+")+txx1;}
        string txx2 = boost::str(boost::format("%.2f") % tolsObstacles_left(1,0)); boost::replace_all(txx2,",","."); if(tolsObstacles_left(1,0) >= 0){txx2=string("+")+txx2;}
        string txx3 = boost::str(boost::format("%.2f") % tolsObstacles_left(2,0)); boost::replace_all(txx3,",","."); if(tolsObstacles_left(2,0) >= 0){txx3=string("+")+txx3;}
        //yy
        string tyy1 = boost::str(boost::format("%.2f") % tolsObstacles_left(0,1)); boost::replace_all(tyy1,",","."); if(tolsObstacles_left(0,1) >= 0){tyy1=string("+")+tyy1;}
        string tyy2 = boost::str(boost::format("%.2f") % tolsObstacles_left(1,1)); boost::replace_all(tyy2,",","."); if(tolsObstacles_left(1,1) >= 0){tyy2=string("+")+tyy2;}
        string tyy3 = boost::str(boost::format("%.2f") % tolsObstacles_left(2,1)); boost::replace_all(tyy3,",","."); if(tolsObstacles_left(2,1) >= 0){tyy3=string("+")+tyy3;}
        //zz
        string tzz1 = boost::str(boost::format("%.2f") % tolsObstacles_left(0,2)); boost::replace_all(tzz1,",","."); if(tolsObstacles_left(0,2) >= 0){tzz1=string("+")+tzz1;}
        string tzz2 = boost::str(boost::format("%.2f") % tolsObstacles_left(1,2)); boost::replace_all(tzz2,",","."); if(tolsObstacles_left(1,2) >= 0){tzz2=string("+")+tzz2;}
        string tzz3 = boost::str(boost::format("%.2f") % tolsObstacles_left(2,2)); boost::replace_all(tzz3,",","."); if(tolsObstacles_left(2,2) >= 0){tzz3=string("+")+tzz3;}
        //xy
        string txy1 = boost::str(boost::format("%.2f") % tolsObstacles_left(0,3)); boost::replace_all(txy1,",","."); if(tolsObstacles_left(0,3) >= 0){txy1=string("+")+txy1;}
        string txy2 = boost::str(boost::format("%.2f") % tolsObstacles_left(1,3)); boost::replace_all(txy2,",","."); if(tolsObstacles_left(1,3) >= 0){txy2=string("+")+txy2;}
        string txy3 = boost::str(boost::format("%.2f") % tolsObstacles_left(2,3)); boost::replace_all(txy3,",","."); if(tolsObstacles_left(2,3) >= 0){txy3=string("+")+txy3;}
        //xz
        string txz1 = boost::str(boost::format("%.2f") % tolsObstacles_left(0,4)); boost::replace_all(txz1,",","."); if(tolsObstacles_left(0,4) >= 0){txz1=string("+")+txz1;}
        string txz2 = boost::str(boost::format("%.2f") % tolsObstacles_left(1,4)); boost::replace_all(txz2,",","."); if(tolsObstacles_left(1,4) >= 0){txz2=string("+")+txz2;}
        string txz3 = boost::str(boost::format("%.2f") % tolsObstacles_left(2,4)); boost::replace_all(txz3,",","."); if(tolsObstacles_left(2,4) >= 0){txz3=string("+")+txz3;}
        //yz
        string tyz1 = boost::str(boost::format("%.2f") % tolsObstacles_left(0,5)); boost::replace_all(tyz1,",","."); if(tolsObstacles_left(0,5) >= 0){tyz1=string("+")+tyz1;}
        string tyz2 = boost::str(boost::format("%.2f") % tolsObstacles_left(1,5)); boost::replace_all(tyz2,",","."); if(tolsObstacles_left(1,5) >= 0){tyz2=string("+")+tyz2;}
        string tyz3 = boost::str(boost::format("%.2f") % tolsObstacles_left(2,5)); boost::replace_all(tyz3,",","."); if(tolsObstacles_left(2,5) >= 0){tyz3=string("+")+tyz3;}


        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("# \n");
        //PostureMod << string("subject to obst_Arm_left{j in 1..21, i in 1..n_Obstacles_left}:  \n");
        PostureMod << string("subject to obst_Arm_left{j in 1..")+n_str_left+string(", i in 1..n_Obstacles_left}:  \n");
        PostureMod << string("(((Rot_left[1,1,i]*Points_Arm_left[j,1]+Rot_left[2,1,i]*Points_Arm_left[j,2]+Rot_left[3,1,i]*Points_Arm_left[j,3]\n");
        PostureMod << string("-Obstacles_left[i,1]*Rot_left[1,1,i]-Obstacles_left[i,2]*Rot_left[2,1,i]-Obstacles_left[i,3]*Rot_left[3,1,i])\n");
        PostureMod << string("/(Obstacles_left[i,4]+Points_Arm_left[j,4]")+txx1+string("))^2\n");
        PostureMod << string("+\n");
        PostureMod << string("((Rot_left[1,2,i]*Points_Arm_left[j,1]+Rot_left[2,2,i]*Points_Arm_left[j,2]+Rot_left[3,2,i]*Points_Arm_left[j,3]\n");
        PostureMod << string("-Obstacles_left[i,1]*Rot_left[1,2,i]-Obstacles_left[i,2]*Rot_left[2,2,i]-Obstacles_left[i,3]*Rot_left[3,2,i])\n");
        PostureMod << string("/(Obstacles_left[i,5]+Points_Arm_left[j,4]")+tyy1+string("))^2\n");
        PostureMod << string("+\n");
        PostureMod << string("((Rot_left[1,3,i]*Points_Arm_left[j,1]+Rot_left[2,3,i]*Points_Arm_left[j,2]+Rot_left[3,3,i]*Points_Arm_left[j,3]\n");
        PostureMod << string("-Obstacles_left[i,1]*Rot_left[1,3,i]-Obstacles_left[i,2]*Rot_left[2,3,i]-Obstacles_left[i,3]*Rot_left[3,3,i])\n");
        PostureMod << string("/(Obstacles_left[i,6]+Points_Arm_left[j,4]")+tzz1+string("))^2)\n");
        PostureMod << string(">= 1;\n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("#  \n");

        if((dual_mov_type==0 && pre_post!=2) || (dual_mov_type==1 && pre_post==2)){
            // pick movements (plan and approach stages)
            // OR
            // place movements (retreat stage)
            if(obj_tar_right->getName().compare(obj_tar_left->getName())!=0)
            {
                PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
                PostureMod << string("# \n");
                PostureMod << string("subject to objRight_Arm_left{j in 1..")+n_str_right+string(", i in 1..n_ObjTar_right}:  \n");
                PostureMod << string("(((Rot_obj_right[1,1]*Points_Arm_left[j,1]+Rot_obj_right[2,1]*Points_Arm_left[j,2]+Rot_obj_right[3,1]*Points_Arm_left[j,3]\n");
                PostureMod << string("-ObjTar_right[i,1]*Rot_obj_right[1,1]-ObjTar_right[i,2]*Rot_obj_right[2,1]-ObjTar_right[i,3]*Rot_obj_right[3,1])\n");
                PostureMod << string("/(ObjTar_right[i,4]+Points_Arm_right[j,4]")+txx1+string("))^2\n");
                PostureMod << string("+\n");
                PostureMod << string("((Rot_obj_right[1,2]*Points_Arm_left[j,1]+Rot_obj_right[2,2]*Points_Arm_left[j,2]+Rot_obj_right[3,2]*Points_Arm_left[j,3]\n");
                PostureMod << string("-ObjTar_right[i,1]*Rot_obj_right[1,2]-ObjTar_right[i,2]*Rot_obj_right[2,2]-ObjTar_right[i,3]*Rot_obj_right[3,2])\n");
                PostureMod << string("/(ObjTar_right[i,5]+Points_Arm_left[j,4]")+tyy1+string("))^2\n");
                PostureMod << string("+\n");
                PostureMod << string("((Rot_obj_right[1,3]*Points_Arm_left[j,1]+Rot_obj_right[2,3]*Points_Arm_left[j,2]+Rot_obj_right[3,3]*Points_Arm_left[j,3]\n");
                PostureMod << string("-ObjTar_right[i,1]*Rot_obj_right[1,3]-ObjTar_right[i,2]*Rot_obj_right[2,3]-ObjTar_right[i,3]*Rot_obj_right[3,3])\n");
                PostureMod << string("/(ObjTar_right[i,6]+Points_Arm_left[j,4]")+tzz1+string("))^2)\n");
                PostureMod << string(">= 1;\n");
                PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
                PostureMod << string("#  \n");
            }
        }

        /*
        PostureMod << string("((Points_Arm_left[j,1]-Obstacles_left[i,1])^2)*(  \n");
        PostureMod << string("(Rot_left[1,1,i])^2 / ((Obstacles_left[i,4]+Points_Arm_left[j,4]")+txx1+string(")^2) + \n");
        PostureMod << string("(Rot_left[2,1,i])^2 / ((Obstacles_left[i,5]+Points_Arm_left[j,4]")+txx2+string(")^2) + \n");
        PostureMod << string("(Rot_left[3,1,i])^2 / ((Obstacles_left[i,6]+Points_Arm_left[j,4]")+txx3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm_left[j,2]-Obstacles_left[i,2])^2)*(  \n");
        PostureMod << string("(Rot_left[1,2,i])^2 / ((Obstacles_left[i,4]+Points_Arm_left[j,4]")+tyy1+string(")^2) + \n");
        PostureMod << string("(Rot_left[2,2,i])^2 / ((Obstacles_left[i,5]+Points_Arm_left[j,4]")+tyy2+string(")^2) + \n");
        PostureMod << string("(Rot_left[3,2,i])^2 / ((Obstacles_left[i,6]+Points_Arm_left[j,4]")+tyy3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("((Points_Arm_left[j,3]-Obstacles_left[i,3])^2)*( \n");
        PostureMod << string("(Rot_left[1,3,i])^2 / ((Obstacles_left[i,4]+Points_Arm_left[j,4]")+tzz1+string(")^2) + \n");
        PostureMod << string("(Rot_left[2,3,i])^2 / ((Obstacles_left[i,5]+Points_Arm_left[j,4]")+tzz2+string(")^2) +  \n");
        PostureMod << string("(Rot_left[3,3,i])^2 / ((Obstacles_left[i,6]+Points_Arm_left[j,4]")+tzz3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm_left[j,1]-Obstacles_left[i,1])*(Points_Arm_left[j,2]-Obstacles_left[i,2])* ( \n");
        PostureMod << string("(Rot_left[1,1,i]*Rot_left[1,2,i])/((Obstacles_left[i,4]+Points_Arm_left[j,4]")+txy1+string(")^2) + \n");
        PostureMod << string("(Rot_left[2,1,i]*Rot_left[2,2,i])/((Obstacles_left[i,5]+Points_Arm_left[j,4]")+txy2+string(")^2) + \n");
        PostureMod << string("(Rot_left[3,1,i]*Rot_left[3,2,i])/((Obstacles_left[i,6]+Points_Arm_left[j,4]")+txy3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm_left[j,1]-Obstacles_left[i,1])*(Points_Arm_left[j,3]-Obstacles_left[i,3])* ( \n");
        PostureMod << string("(Rot_left[1,1,i]*Rot_left[1,3,i])/((Obstacles_left[i,4]+Points_Arm_left[j,4]")+txz1+string(")^2) + \n");
        PostureMod << string("(Rot_left[2,1,i]*Rot_left[2,3,i])/((Obstacles_left[i,5]+Points_Arm_left[j,4]")+txz2+string(")^2) + \n");
        PostureMod << string("(Rot_left[3,1,i]*Rot_left[3,3,i])/((Obstacles_left[i,6]+Points_Arm_left[j,4]")+txz3+string(")^2)) \n");
        PostureMod << string("+ \n");
        PostureMod << string("2*(Points_Arm_left[j,2]-Obstacles_left[i,2])*(Points_Arm_left[j,3]-Obstacles_left[i,3])* ( \n");
        PostureMod << string("(Rot_left[1,2,i]*Rot_left[1,3,i])/((Obstacles_left[i,4]+Points_Arm_left[j,4]")+tyz1+string(")^2) + \n");
        PostureMod << string("(Rot_left[2,2,i]*Rot_left[2,3,i])/((Obstacles_left[i,5]+Points_Arm_left[j,4]")+tyz2+string(")^2) + \n");
        PostureMod << string("(Rot_left[3,2,i]*Rot_left[3,3,i])/((Obstacles_left[i,6]+Points_Arm_left[j,4]")+tyz3+string(")^2)) \n");
        PostureMod << string("-1 >=0; \n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("#  \n");
        */

    }

    // constraints with the body
    if(coll_right && coll_body){
        this->writeDualBodyConstraints(PostureMod,true,true);
    }
    if(coll_left && coll_body){
        this->writeDualBodyConstraints(PostureMod,true,false);
    }

    // constraints between the arms
    if(coll_arms)
    {
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("# \n");
        PostureMod << string("# Constraints between the arms \n");
        //PostureMod << string("subject to Arm_Arm{i1 in 4..")+n_str_left+string(", i2 in 4..")+n_str_right+string("}:  \n");

        if(dual_mov_type==0){ // dual pick
            PostureMod << string("subject to Arm_Arm{i1 in 4..9, i2 in 4..9}:  \n");
        }else if(dual_mov_type==1){ // dual place
            if(obj_tar_right->getName().compare(obj_tar_left->getName())!=0)
            {
                PostureMod << string("subject to Arm_Arm{i1 in 4..")+n_str_left+string(", i2 in 4..")+n_str_right+string("}:  \n");
            }else{
                PostureMod << string("subject to Arm_Arm{i1 in 4..9, i2 in 4..9}:  \n");
            }
        }

        PostureMod << string("(Points_Arm_left[i1,1] - Points_Arm_right[i2,1])^2 + \n");
        PostureMod << string("(Points_Arm_left[i1,2] - Points_Arm_right[i2,2])^2 + \n");
        PostureMod << string("(Points_Arm_left[i1,3] - Points_Arm_right[i2,3])^2 - \n");
        PostureMod << string("(Points_Arm_left[i1,4] + Points_Arm_right[i2,4])^2 >= 0; \n");
        PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
        PostureMod << string("# \n");
    }

    PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n\n\n");

    // close the files
    PostureMod.close();
    PostureDat.close();

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

bool HUMPlanner::writeFilesDualBouncePosture(int steps,hump_dual_params& params,int dual_mov_type, int pre_post,std::vector<double> minAuxLimits, std::vector<double> maxAuxLimits,std::vector<double> initAuxPosture, std::vector<double> finalAuxPosture,
                                         std::vector<double> initialGuess, std::vector<double> lambda, std::vector<objectPtr> objs_right,std::vector<objectPtr> objs_left,boundaryConditions bAux)
{
    //  --- create the "Models" directory if it does not exist ---
    struct stat st = {0};
    if (stat("Models", &st) == -1) {
        mkdir("Models", 0700);
    }
    string path("Models/");

    Matrix4d matWorldToArm_right = this->matWorldToRightArm;
    Matrix4d matWorldToArm_left = this->matWorldToLeftArm;
    Matrix4d matHand_right = this->matRightHand;
    Matrix4d matHand_left = this->matLeftHand;
    DHparameters dh_right = this->DH_rightArm;
    DHparameters dh_left = this->DH_leftArm;

    // movement setting
    //int arm_code = params.mov_specs.arm_code;
    bool coll_arms = params.coll_arms;
    bool coll_body = params.coll_body;
    int hand_code_right = params.mov_specs_right.hand_code;
    int hand_code_left = params.mov_specs_left.hand_code;
    double dHO_right = params.mov_specs_right.dHO;
    double dHO_left = params.mov_specs_left.dHO;
    std::vector<double> finalHand_right = params.mov_specs_right.finalHand;
    std::vector<double> finalHand_left = params.mov_specs_left.finalHand;
    std::vector<double> tar_right = params.mov_specs_right.target;
    std::vector<double> tar_left = params.mov_specs_left.target;
    Matrix4d T_tar_to_obj_right = params.mov_specs_right.T_tar_to_obj;
    Matrix4d T_tar_to_obj_left = params.mov_specs_left.T_tar_to_obj;
    objectPtr obj_tar_right = params.mov_specs_right.obj;
    objectPtr obj_tar_left = params.mov_specs_left.obj;
    string mov_infoLine_right = params.mov_specs_right.mov_infoline;
    string mov_infoLine_left = params.mov_specs_left.mov_infoline;
    bool approach_right = params.mov_specs_right.approach;
    bool approach_left = params.mov_specs_left.approach;
    bool retreat_right = params.mov_specs_right.retreat;
    bool retreat_left = params.mov_specs_left.retreat;
    //bool use_plane = params.mov_specs.use_move_plane;
    //std::vector<double> plane_params = params.mov_specs.plane_params;
    std::vector<double> pre_grasp_approach_right; std::vector<double> pre_grasp_approach_left;
    std::vector<double> post_grasp_retreat_right; std::vector<double> post_grasp_retreat_left;
    std::vector<double> pre_place_approach_right; std::vector<double> pre_place_approach_left;
    std::vector<double> post_place_retreat_right; std::vector<double> post_place_retreat_left;
    std::vector<double> vel_approach_right(params.vel_approach_right);
    std::vector<double> vel_approach_left(params.vel_approach_left);
    std::vector<double> vel_approach(vel_approach_right);
    vel_approach.insert(vel_approach.end(),vel_approach_left.begin(),vel_approach_left.end());
    std::vector<double> acc_approach_right(params.acc_approach_right);
    std::vector<double> acc_approach_left(params.acc_approach_left);
    std::vector<double> acc_approach(acc_approach_right);
    acc_approach.insert(acc_approach.end(),acc_approach_left.begin(),acc_approach_left.end());

    switch(dual_mov_type){
    case 0: // pick right pick left
        if(approach_right){pre_grasp_approach_right = params.mov_specs_right.pre_grasp_approach;}
        if(approach_left){pre_grasp_approach_left = params.mov_specs_left.pre_grasp_approach;}
        if(retreat_right){post_grasp_retreat_right = params.mov_specs_right.post_grasp_retreat;}
        if(retreat_left){post_grasp_retreat_left = params.mov_specs_left.post_grasp_retreat;}
        break;
    case 1: // place right place left
        if(approach_right){pre_place_approach_right = params.mov_specs_right.pre_place_approach;}
        if(approach_left){pre_place_approach_left = params.mov_specs_left.pre_place_approach;}
        if(retreat_right){post_place_retreat_right = params.mov_specs_right.post_place_retreat;}
        if(retreat_left){post_place_retreat_left = params.mov_specs_left.post_place_retreat;}
        break;
    }
    // tolerances
    double timestep; MatrixXd traj_no_bound;
    this->directTrajectoryNoBound(steps,initAuxPosture,finalAuxPosture,traj_no_bound);
    int mod;
    if(pre_post==0){mod=0;}else{mod=1;}
    timestep = this->getDualTimeStep(params,traj_no_bound,mod);
    double totalTime = timestep*steps;
    //std::vector<double> lambda_right = params.lambda_bounce_right;
    //std::vector<double> lambda_left = params.lambda_bounce_left;
    std::vector<double> tolsArm_right(params.tolsArm_right);
    std::vector<double> tolsArm_left(params.tolsArm_left);
    MatrixXd tolsHand_right = params.tolsHand_right;
    MatrixXd tolsHand_left = params.tolsHand_left;
    vector< MatrixXd > tolsTarget_right(params.singleArm_tolsTarget_right);
    vector< MatrixXd > tolsTarget_left(params.singleArm_tolsTarget_left);
    vector< MatrixXd > tolsObstacles_right(params.singleArm_tolsObstacles_right);
    vector< MatrixXd > tolsObstacles_left(params.singleArm_tolsObstacles_left);
    bool obstacle_avoidance = params.obstacle_avoidance;
    bool target_avoidance = params.target_avoidance;

    //------------------------- Write the dat file --------------------------------------------------
     string filenamedat("BouncePosture.dat");
     ofstream PostureDat;
     // open the file
     PostureDat.open(path+filenamedat);

     PostureDat << string("# BOUNCE POSTURE DATA FILE \n");
     PostureDat << string("# Units of measure: [rad], [mm] \n\n");

     PostureDat << string("data; \n");

     // number of steps
     PostureDat << string("param Nsteps :=")+to_string(steps)+string(";\n");

     // total time
     string tottime_str =  boost::str(boost::format("%.2f") % (totalTime));
     boost::replace_all(tottime_str,",",".");
     PostureDat << string("param TotalTime :=")+tottime_str+string(";\n");
     // Body dimension
     if(coll_body){
        this->writeBodyDim(this->torso_size.at(0),this->torso_size.at(1),PostureDat);
     }
     // D-H Parameters of the Arm
     this->writeDualArmDHParams(dh_right,dh_left,PostureDat);
     // distance between the hand and the object
     this->write_dual_dHO(PostureDat,dHO_right,dHO_left);
     // joint limits
     this->writeArmLimits(PostureDat,minAuxLimits,maxAuxLimits,false);
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
     this->writeFingerDualFinalPose(PostureDat,finalHand_right,finalHand_left);
     // joint expense factors of the arm
     this->writeLambda(PostureDat,lambda);
     // initial guess
     PostureDat << string("# INITIAL GUESS \n");
     PostureDat << string("var theta_b = \n");
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
     if(pre_post==0 || (!approach_right && !approach_left)){
         // boundary conditions initial velocity
         PostureDat << string("# INITIAL VELOCITY \n");
         PostureDat << string("param vel_0 := \n");
         for (std::size_t i=0; i < bAux.vel_0.size(); ++i){
             string vel_0 =  boost::str(boost::format("%.2f") % (bAux.vel_0.at(i)));
             boost::replace_all(vel_0,",",".");
             if (i == bAux.vel_0.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+vel_0+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+vel_0+string("\n");
             }
         }
         // boundary conditions initial acceleration
         PostureDat << string("# INITIAL ACCELERATION \n");
         PostureDat << string("param acc_0 := \n");
         for (std::size_t i=0; i < bAux.acc_0.size(); ++i){
             string acc_0 =  boost::str(boost::format("%.2f") % (bAux.acc_0.at(i)));
             boost::replace_all(acc_0,",",".");
             if (i == bAux.acc_0.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+acc_0+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+acc_0+string("\n");
             }
         }
         // boundary conditions final velocity
         PostureDat << string("# FINAL VELOCITY \n");
         PostureDat << string("param vel_f := \n");
         for (std::size_t i=0; i < bAux.vel_f.size(); ++i){
             string vel_f =  boost::str(boost::format("%.2f") % (bAux.vel_f.at(i)));
             boost::replace_all(vel_f,",",".");
             if (i == bAux.vel_f.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+vel_f+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+vel_f+string("\n");
             }
         }
         // boundary conditions final acceleration
         PostureDat << string("# FINAL ACCELERATION \n");
         PostureDat << string("param acc_f := \n");
         for (std::size_t i=0; i < bAux.acc_f.size(); ++i){
             string acc_f =  boost::str(boost::format("%.2f") % (bAux.acc_f.at(i)));
             boost::replace_all(acc_f,",",".");
             if (i == bAux.acc_f.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+acc_f+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+acc_f+string("\n");
             }
         }
     }else if ((approach_right && approach_left) && pre_post==1){ // use approach options
         // boundary conditions initial velocity
         PostureDat << string("# INITIAL VELOCITY \n");
         PostureDat << string("param vel_0 := \n");
         for (std::size_t i=0; i < bAux.vel_0.size(); ++i){
             string vel_0 =  boost::str(boost::format("%.2f") % (bAux.vel_0.at(i)));
             boost::replace_all(vel_0,",",".");
             if (i == bAux.vel_0.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+vel_0+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+vel_0+string("\n");
             }
         }
         // boundary conditions initial acceleration
         PostureDat << string("# INITIAL ACCELERATION \n");
         PostureDat << string("param acc_0 := \n");
         for (std::size_t i=0; i < bAux.acc_0.size(); ++i){
             string acc_0 =  boost::str(boost::format("%.2f") % (bAux.acc_0.at(i)));
             boost::replace_all(acc_0,",",".");
             if (i == bAux.acc_0.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+acc_0+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+acc_0+string("\n");
             }
         }
         // boundary conditions final velocity
         PostureDat << string("# FINAL VELOCITY \n");
         PostureDat << string("param vel_f := \n");
         for (std::size_t i=0; i < bAux.vel_f.size(); ++i){
             string vel_f =  boost::str(boost::format("%.2f") % (vel_approach.at(i)));
             boost::replace_all(vel_f,",",".");
             if (i == bAux.vel_f.size()-1){
                 PostureDat << to_string(i+1)+string(" ")+vel_f+string(";\n");
             }else{
                 PostureDat << to_string(i+1)+string(" ")+vel_f+string("\n");
             }
         }
         // boundary conditions final acceleration
         PostureDat << string("# FINAL ACCELERATION \n");
         PostureDat << string("param acc_f := \n");
         for (std::size_t i=0; i < bAux.acc_f.size(); ++i){
             string acc_f =  boost::str(boost::format("%.2f") % (acc_approach.at(i)));
             boost::replace_all(acc_f,",",".");
             if (i == bAux.acc_f.size()-1){
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
     switch(hand_code_right){
     case 0: // human hand
         this->writeDualHumanHandParams(this->hhand,PostureDat,true);
         break;
     case 1: // barrett hand
         this->writeDualBarrettHandParams(this->bhand,PostureDat,true);
         break;
     }
     switch(hand_code_left){
     case 0: // human hand
         this->writeDualHumanHandParams(this->hhand,PostureDat,false);
         break;
     case 1: // barrett hand
         this->writeDualBarrettHandParams(this->bhand,PostureDat,false);
         break;
     }
     // info approach/retreat
     switch(dual_mov_type){
     case 0: // pick right pick left
         // info of the targets to reach
         this->writeDualInfoTarget(PostureDat,tar_right,tar_left);
         if (pre_post!=0){
             this->writeDualInfoApproachRetreat(PostureDat,tar_right,pre_grasp_approach_right,true);
             this->writeDualInfoApproachRetreat(PostureDat,tar_left,pre_grasp_approach_left,false);
         }
         break;
     case 1: // place right place left
         // info of the targets to reach
         this->writeDualInfoTarget(PostureDat,tar_right,tar_left);
         if (pre_post!=0){
             this->writeDualInfoApproachRetreat(PostureDat,tar_right,pre_place_approach_right,true);
             this->writeDualInfoApproachRetreat(PostureDat,tar_left,pre_place_approach_left,false);
         }
         break;
     }
     //info objects
     this->writeDualInfoObstacles(PostureDat,objs_right,true);
     this->writeDualInfoObstacles(PostureDat,objs_left,false);
     // object that has the target
     std::vector<double> dim_right; std::vector<double> dim_left;
     switch(dual_mov_type){
     case 0: //dual pick
         obj_tar_right->getSize(dim_right); obj_tar_left->getSize(dim_left);
         if(pre_post==2){ // retreat stage
             this->writeDualInfoObjectTarget(PostureDat,tar_right,T_tar_to_obj_right,dim_right, obj_tar_right->getName(),tar_left,T_tar_to_obj_left,dim_left,obj_tar_left->getName());
         }else{
             this->writeDualInfoObjectTarget(PostureDat,obj_tar_right,obj_tar_left);
         }
         break;
     case 1: // dual place
         obj_tar_right->getSize(dim_right); obj_tar_left->getSize(dim_left);
         if(pre_post==2){ // retreat stage
             this->writeDualInfoObjectTargetPlaceRetreat(PostureDat,tar_right,T_tar_to_obj_right,dim_right, obj_tar_right->getName(),tar_left,T_tar_to_obj_left,dim_left,obj_tar_left->getName());
         }else{
             this->writeDualInfoObjectTarget(PostureDat,tar_right,T_tar_to_obj_right,dim_right, obj_tar_right->getName(),tar_left,T_tar_to_obj_left,dim_left,obj_tar_left->getName());
         }
         break;
     }
     //close the file
     //PostureDat.close();

     // ------------- Write the mod file ------------------------- //
     string filenamemod("BouncePosture.mod");
     ofstream PostureMod;
     // open the file
     PostureMod.open(path+filenamemod);

     PostureMod << string("# BOUNCE POSTURE MODEL FILE \n");
     PostureMod << string("# Right Movement to plan: \n");
     PostureMod << string("# ")+mov_infoLine_right+string("\n");
     PostureMod << string("# Left Movement to plan: \n");
     PostureMod << string("# ")+mov_infoLine_left+string("\n\n");

     PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
     PostureMod << string("# PARAMETERS \n\n");

     this->writePI(PostureMod);
     if(coll_body){
        this->writeBodyDimMod(PostureMod);
     }
     this->writeDualArmDHParamsMod(PostureMod);
     this->write_dual_dHOMod(PostureMod);

     PostureMod << string("# Joint Limits \n");
     PostureMod << string("param llim {i in 1..")+to_string(minAuxLimits.size())+string("} ; \n");
     PostureMod << string("param ulim {i in 1..")+to_string(maxAuxLimits.size())+string("} ; \n");
     PostureMod << string("# Initial posture \n");
     PostureMod << string("param thet_init {i in 1..")+to_string(initAuxPosture.size())+string("} ; \n");
     PostureMod << string("# Final posture \n");
     PostureMod << string("param thet_final {i in 1..")+to_string(finalAuxPosture.size())+string("} ; \n");
     PostureMod << string("# Right Final finger posture \n");
     PostureMod << string("param joint_fingers_right {i in 1..")+to_string(joints_hand)+string("} ; \n");
     PostureMod << string("# Left Final finger posture \n");
     PostureMod << string("param joint_fingers_left {i in 1..")+to_string(joints_hand)+string("} ; \n");
     PostureMod << string("# Joint Expense Factors \n");
     PostureMod << string("param lambda {i in 1..")+to_string(lambda.size())+string("} ; \n");

     switch(hand_code_right){
     case 0: // human hand
         this->writeDualHumanHandParamsMod(PostureMod,true);
         break;
     case 1: // barrett hand
         this->writeDualBarrettHandParamsMod(PostureMod,true);
         break;
     }
     switch(hand_code_left){
     case 0: // human hand
         this->writeDualHumanHandParamsMod(PostureMod,false);
         break;
     case 1: // barrett hand
         this->writeDualBarrettHandParamsMod(PostureMod,false);
         break;
     }
     // info objects
     bool vec_right = false;// true if there is some pre or post operation (right)
     bool vec_left = false;// true if there is some pre or post operation (left)
     if((approach_right || retreat_right) && pre_post!=0){vec_right=true;}
     if((approach_left || retreat_left) && pre_post!=0){vec_left=true;}
     this->writeDualInfoObjectsMod(PostureMod,vec_right,vec_left);

     PostureMod << string("# Boundary Conditions \n");
     PostureMod << string("param vel_0 {i in 1..")+to_string(bAux.vel_0.size())+string("} ; \n");
     PostureMod << string("param vel_f {i in 1..")+to_string(bAux.vel_f.size())+string("} ; \n");
     PostureMod << string("param acc_0 {i in 1..")+to_string(bAux.acc_0.size())+string("} ; \n");
     PostureMod << string("param acc_f {i in 1..")+to_string(bAux.acc_f.size())+string("} ; \n");

     PostureMod << string("# Time and iterations\n");
     PostureMod << string("param Nsteps;\n");
     PostureMod << string("param TB;\n");
     PostureMod << string("param PHI;\n");
     PostureMod << string("param TotalTime;\n");
     PostureMod << string("set Iterations := 1..(Nsteps+1);\n");
     PostureMod << string("set nJoints := 1..")+to_string(initialGuess.size())+string(";\n");
     PostureMod << string("set Iterations_nJoints := Iterations cross nJoints;\n");
     //PostureMod << string("param time {i in Iterations} = ((i-1)*TotalTime)/Nsteps;\n");
     PostureMod << string("param time {i in Iterations} = (i-1)/Nsteps;\n");

     PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n");
     PostureMod << string("# DECISION VARIABLES \n");
     PostureMod << string("# Bounce Posture \n");
     PostureMod << string("var theta_b {i in 1..")+to_string(initialGuess.size())+string("} >= llim[i], <= ulim[i]; \n");

     PostureMod << string("# Direct Movement \n");
     PostureMod << string("param the_direct {(i,j) in Iterations_nJoints} := thet_init[j]+ \n");
     PostureMod << string("( thet_final[j] - thet_init[j] ) * (10*(time[i])^3 -15*(time[i])^4 +6*(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("vel_0[j] * TotalTime * (time[i] - 6 *(time[i])^3 +8*(time[i])^4 -3*(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("vel_f[j] * TotalTime * (- 4 *(time[i])^3 +7*(time[i])^4 -3*(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("(acc_0[j]/2) * TotalTime^2 * (time[i]^2 - 3 *(time[i])^3 +3*(time[i])^4 -(time[i])^5) \n");
     PostureMod << string("+ \n");
     PostureMod << string("(acc_f[j]/2) * TotalTime^2 * ((time[i])^3 -2*(time[i])^4 +(time[i])^5); \n");

     PostureMod << string("# Back and forth Movement \n");
     PostureMod << string("var the_bf 	{(i,j) in Iterations_nJoints} =  \n");
     PostureMod << string("((time[i]*(1-time[i]))/(TB*(1-TB)))*(theta_b[j] - thet_init[j])*(sin(pi*(time[i])^PHI))^2; \n");

     PostureMod << string("# Composite Movement \n");
     PostureMod << string("var theta 	{(i,j) in Iterations_nJoints} = \n");
     PostureMod << string("the_direct[i,j] + the_bf[i,j];\n");

     // Rotation matrix of the obstacles
     this->writeDualRotMatObsts(PostureMod);
     // Direct Kinematics of the arms
     this->writeDualArmDirKin(PostureMod,dual_mov_type,matWorldToArm_right,matHand_right,tolsArm_right,matWorldToArm_left,matHand_left,tolsArm_left,false);

     bool obj_right_place = false; bool obj_left_place = false;
     bool move = false; // true for move movements
     // object to transport (dual place movements in the plan and approach target posture selection or
     //                        dual pick movement in the retreat target posture selection)
     int n_s_dual = 0; // number of spheres of the object transported with both hands
     int n_s_right = 0; // number of spheres of the object (right)
     int n_s_left = 0; // number of spheres of the object (left)
     if(dual_mov_type==1){ // dual place
         obj_right_place = true; obj_left_place = true;
         std::vector<double> obj_tar_right_size; std::string obj_right_name = obj_tar_right->getName();
         obj_tar_right->getSize(obj_tar_right_size);
         std::vector<double> obj_tar_left_size; std::string obj_left_name =obj_tar_left->getName();
         obj_tar_left->getSize(obj_tar_left_size);
         if(obj_right_name.compare(obj_left_name)==0){
             n_s_dual = this->dual_obj_model_spheres(PostureDat,PostureMod,obj_tar_right_size,false);
         }else{
             this->dual_obj_model_spheres(PostureDat,PostureMod,obj_tar_right_size,obj_tar_left_size,false,n_s_right,n_s_left);
         }
     }else if(dual_mov_type==2){
         // dual move
         move=true;
     }
     switch(hand_code_right){
     case 0: // human hand
         this->writeDualHumanHandDirKin(PostureMod,tolsHand_right,false,obj_right_place,obj_left_place,true);
         break;
     case 1: // barrett hand
         this->writeDualBarrettHandDirKin(PostureMod,tolsHand_right,false,obj_right_place,obj_left_place,true);
         break;
     }
     switch(hand_code_left){
     case 0: // human hand
         this->writeDualHumanHandDirKin(PostureMod,tolsHand_left,false,obj_right_place,obj_left_place,false);
         break;
     case 1: // barrett hand
         this->writeDualBarrettHandDirKin(PostureMod,tolsHand_left,false,obj_right_place,obj_left_place,false);
         break;
     }
    // --------- Points of the right arm --------------------------------------- //
     std::string n_str_right;
     if(obj_right_place && n_s_dual==0){
         n_str_right = to_string(15+n_s_right);
     }else if(obj_right_place && n_s_dual!=0){
         n_str_right = to_string(15+n_s_dual);
     }else{
        n_str_right = to_string(15);
     }
     PostureMod << string("var Points_Arm_right {j in 1..")+n_str_right+string(", i in 1..4,k in Iterations} = \n");
     PostureMod << string("if ( j=1 ) then 	(Shoulder_right[i,k]+Elbow_right[i,k])/2  \n");
     PostureMod << string("else	if ( j=2 ) then 	Elbow_right[i,k] \n");
     PostureMod << string("else    if ( j=3 ) then 	(Wrist_right[i,k]+Elbow_right[i,k])/2  \n");
     PostureMod << string("else	if ( j=4 ) then 	Wrist_right[i,k] \n");
     PostureMod << string("else	if ( j=5 ) then 	Wrist_right[i,k]+0.45*(Hand_right[i,k]-Wrist_right[i,k]) \n");
     PostureMod << string("else	if ( j=6 ) then 	Wrist_right[i,k]+0.75*(Hand_right[i,k]-Wrist_right[i,k]) \n");
     /*
     PostureMod << string("else	if ( j=7 ) then 	Finger1_1_right[i,k] \n");
     PostureMod << string("else	if ( j=8 ) then 	Finger2_1_right[i,k] \n");
     PostureMod << string("else	if ( j=9 ) then 	Finger3_1_right[i,k]\n");
     PostureMod << string("else	if ( j=10 ) then 	(Finger1_1_right[i,k]+Finger1_2_right[i,k])/2 \n");
     PostureMod << string("else	if ( j=11 ) then 	(Finger2_1_right[i,k]+Finger2_2_right[i,k])/2 \n");
     PostureMod << string("else	if ( j=12 ) then 	(Finger3_1_right[i,k]+Finger3_2_right[i,k])/2 \n");
     PostureMod << string("else	if ( j=13 ) then 	 Finger1_2_right[i,k] \n");
     PostureMod << string("else	if ( j=14 ) then 	 Finger2_2_right[i,k] \n");
     PostureMod << string("else	if ( j=15 ) then 	 Finger3_2_right[i,k] \n");
     PostureMod << string("else	if ( j=16 ) then 	(Finger1_2_right[i,k]+Finger1_tip_right[i,k])/2	 \n");
     PostureMod << string("else	if ( j=17 ) then 	(Finger2_2_right[i,k]+Finger2_tip_right[i,k])/2 \n");
     PostureMod << string("else	if ( j=18 ) then 	(Finger3_2_right[i,k]+Finger3_tip_right[i,k])/2 \n");
     PostureMod << string("else	if ( j=19 ) then 	Finger1_tip_right[i,k]\n");
     PostureMod << string("else	if ( j=20 ) then 	Finger2_tip_right[i,k] \n");
     PostureMod << string("else	if ( j=21 ) then 	Finger3_tip_right[i,k] \n");
     */
     PostureMod << string("else	if ( j=7 ) then 	Finger1_1_right[i,k] \n");
     PostureMod << string("else	if ( j=8 ) then 	Finger2_1_right[i,k] \n");
     PostureMod << string("else	if ( j=9 ) then 	Finger3_1_right[i,k]\n");
     PostureMod << string("else	if ( j=10 ) then 	 Finger1_2_right[i,k] \n");
     PostureMod << string("else	if ( j=11 ) then 	 Finger2_2_right[i,k] \n");
     PostureMod << string("else	if ( j=12 ) then 	 Finger3_2_right[i,k] \n");
     PostureMod << string("else	if ( j=13 ) then 	Finger1_tip_right[i,k]\n");
     PostureMod << string("else	if ( j=14 ) then 	Finger2_tip_right[i,k] \n");
     PostureMod << string("else	if ( j=15 ) then 	Finger3_tip_right[i,k] \n");

     if(n_s_dual==0)
     {
         if (obj_right_place){
             int j_init = 15;
             for(int i=1;i <= n_s_right;++i){
                 std::string i_str = to_string(i);
                 int j = j_init+i; std::string j_str = to_string(j);
                 PostureMod << string("else    if ( j=")+j_str+string(" ) then 	ObjRight2Transp_")+i_str+string("[i,k] \n");
             }
         }
     }else{
         if (obj_right_place){
             int j_init = 15;
             for(int i=1;i <= n_s_dual;++i){
                 std::string i_str = to_string(i);
                 int j = j_init+i; std::string j_str = to_string(j);
                 PostureMod << string("else    if ( j=")+j_str+string(" ) then 	Obj2Transp_")+i_str+string("[i,k] \n");
             }
         }
     }
     PostureMod << string("; \n\n");

     // --------- Points of the left arm --------------------------------------- //
     std::string n_str_left;
     if(obj_left_place && n_s_dual==0){
         n_str_left = to_string(15+n_s_left);
     }else if(obj_left_place && n_s_dual!=0){
         //n_str_left = to_string(15+n_s_dual);
         n_str_left = to_string(15);
     }else{
        n_str_left = to_string(15);
     }
      PostureMod << string("var Points_Arm_left {j in 1..")+n_str_left+string(", i in 1..4,k in Iterations} = \n");
      PostureMod << string("if ( j=1 ) then 	(Shoulder_left[i,k]+Elbow_left[i,k])/2  \n");
      PostureMod << string("else	if ( j=2 ) then 	Elbow_left[i,k] \n");
      PostureMod << string("else    if ( j=3 ) then 	(Wrist_left[i,k]+Elbow_left[i,k])/2  \n");
      PostureMod << string("else	if ( j=4 ) then 	Wrist_left[i,k] \n");
      PostureMod << string("else	if ( j=5 ) then 	Wrist_left[i,k]+0.45*(Hand_left[i,k]-Wrist_left[i,k]) \n");
      PostureMod << string("else	if ( j=6 ) then 	Wrist_left[i,k]+0.75*(Hand_left[i,k]-Wrist_left[i,k]) \n");
      /*
      PostureMod << string("else	if ( j=7 ) then 	Finger1_1_left[i,k] \n");
      PostureMod << string("else	if ( j=8 ) then 	Finger2_1_left[i,k] \n");
      PostureMod << string("else	if ( j=9 ) then 	Finger3_1_left[i,k]\n");
      PostureMod << string("else	if ( j=10 ) then 	(Finger1_1_left[i,k]+Finger1_2_left[i,k])/2 \n");
      PostureMod << string("else	if ( j=11 ) then 	(Finger2_1_left[i,k]+Finger2_2_left[i,k])/2 \n");
      PostureMod << string("else	if ( j=12 ) then 	(Finger3_1_left[i,k]+Finger3_2_left[i,k])/2 \n");
      PostureMod << string("else	if ( j=13 ) then 	 Finger1_2_left[i,k] \n");
      PostureMod << string("else	if ( j=14 ) then 	 Finger2_2_left[i,k] \n");
      PostureMod << string("else	if ( j=15 ) then 	 Finger3_2_left[i,k] \n");
      PostureMod << string("else	if ( j=16 ) then 	(Finger1_2_left[i,k]+Finger1_tip_left[i,k])/2	 \n");
      PostureMod << string("else	if ( j=17 ) then 	(Finger2_2_left[i,k]+Finger2_tip_left[i,k])/2 \n");
      PostureMod << string("else	if ( j=18 ) then 	(Finger3_2_left[i,k]+Finger3_tip_left[i,k])/2 \n");
      PostureMod << string("else	if ( j=19 ) then 	Finger1_tip_left[i,k]\n");
      PostureMod << string("else	if ( j=20 ) then 	Finger2_tip_left[i,k] \n");
      PostureMod << string("else	if ( j=21 ) then 	Finger3_tip_left[i,k] \n");
      */
      PostureMod << string("else	if ( j=7 ) then 	Finger1_1_left[i,k] \n");
      PostureMod << string("else	if ( j=8 ) then 	Finger2_1_left[i,k] \n");
      PostureMod << string("else	if ( j=9 ) then 	Finger3_1_left[i,k]\n");
      PostureMod << string("else	if ( j=10 ) then 	 Finger1_2_left[i,k] \n");
      PostureMod << string("else	if ( j=11 ) then 	 Finger2_2_left[i,k] \n");
      PostureMod << string("else	if ( j=12 ) then 	 Finger3_2_left[i,k] \n");
      PostureMod << string("else	if ( j=13 ) then 	Finger1_tip_left[i,k]\n");
      PostureMod << string("else	if ( j=14 ) then 	Finger2_tip_left[i,k] \n");
      PostureMod << string("else	if ( j=15 ) then 	Finger3_tip_left[i,k] \n");
      if(n_s_dual==0)
      {
          if (obj_left_place){
              int j_init = 15;
              for(int i=1;i <= n_s_left;++i){
                  std::string i_str = to_string(i);
                  int j = j_init+i; std::string j_str = to_string(j);
                  PostureMod << string("else    if ( j=")+j_str+string(" ) then  ObjLeft2Transp_")+i_str+string("[i,k] \n");
              }
          }
      }else{
          // to do
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
      PostureMod << string("subject to co_JointLimits {i in Iterations, j in nJoints}: llim[j] <= theta[i,j]  <= ulim[j]; \n");
      //PostureMod << string("# Right F1 and F2 move in synchrony \n");
      //PostureMod << string("subject to co_right_fingers: (theta_b[8] - theta_b[9])^2<=0.001; \n");
      //PostureMod << string("# Left F1 and F2 move in synchrony \n");
      //PostureMod << string("subject to co_left_fingers: (theta_b[17] - theta_b[18])^2<=0.001; \n\n");

      // hand constraints for approaching direction setting
      string n_steps_init_str;
      if(N_STEP_MIN>2){
          n_steps_init_str = boost::str(boost::format("%d") % (N_STEP_MIN-2));
      }else{
          n_steps_init_str = boost::str(boost::format("%d") % 1);
      }
      switch (dual_mov_type) {
      case 0: // dual pick
          // hand constraints for approaching direction settings
          /*
          if(approach_right && pre_post==1){
              PostureMod << string("# Right Hand approach orientation\n");
              PostureMod << string("subject to constr_hand_right_or {k in (Nsteps-")+n_steps_init_str+string(")..(Nsteps+1)}: ( sum{i in 1..3} (x_H_right[i,k] - x_t_right[i])^2)<= 0.01; #  x_H_right = x_t_right \n\n");
          }
          if(approach_left && pre_post==1){
              PostureMod << string("# Left Hand approach orientation\n");
              PostureMod << string("subject to constr_hand_left_or {k in (Nsteps-")+n_steps_init_str+string(")..(Nsteps+1)}: ( sum{i in 1..3} (x_H_left[i,k] - x_t_left[i])^2)<= 0.01; #  x_H_left = x_t_left \n\n");
          }
          */
          break;
      case 1: // dual place
          // hand constraints for approaching and retreating direction settings
          // TO DO
          /*
          if(approach && pre_post==1){
              PostureMod << string("# Hand approach orientation\n");
              PostureMod << string("subject to constr_hand_or {k in (Nsteps-")+n_steps_init_str+string(")..(Nsteps+1)}: ( sum{i in 1..3} (x_H[i,k] - z_t[i])^2 + sum{i in 1..3} (z_H[i,k] + y_t[i])^2 )<= 0.010; #  x_H = z_t  and z_H = -y_t \n\n");
          }
          */
          break;
      }

      if(target_avoidance && !obj_right_place && !move){
           // constraints with the targets
           MatrixXd tols_0 = tolsTarget_right.at(0);
           MatrixXd tols_1 = tolsTarget_right.at(1);
           MatrixXd tols_2 = tolsTarget_right.at(2);
           //xx1
           string txx1_0 = boost::str(boost::format("%.2f") % tols_0(0,0)); boost::replace_all(txx1_0,",",".");
           string txx1_1 = boost::str(boost::format("%.2f") % tols_1(0,0)); boost::replace_all(txx1_1,",",".");
           string txx1_2 = boost::str(boost::format("%.2f") % tols_2(0,0)); boost::replace_all(txx1_2,",",".");
           PostureMod << string("param tol_target_right_xx1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx1_2+string("; \n");
           //xx2
           string txx2_0 = boost::str(boost::format("%.2f") % tols_0(1,0)); boost::replace_all(txx2_0,",",".");
           string txx2_1 = boost::str(boost::format("%.2f") % tols_1(1,0)); boost::replace_all(txx2_1,",",".");
           string txx2_2 = boost::str(boost::format("%.2f") % tols_2(1,0)); boost::replace_all(txx2_2,",",".");

           PostureMod << string("param tol_target_right_xx2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx2_2+string("; \n");


           // xx3
           string txx3_0 = boost::str(boost::format("%.2f") % tols_0(2,0)); boost::replace_all(txx3_0,",",".");
           string txx3_1 = boost::str(boost::format("%.2f") % tols_1(2,0)); boost::replace_all(txx3_1,",",".");
           string txx3_2 = boost::str(boost::format("%.2f") % tols_2(2,0)); boost::replace_all(txx3_2,",",".");

           PostureMod << string("param tol_target_right_xx3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx3_2+string("; \n");

           // yy1
           string tyy1_0 = boost::str(boost::format("%.2f") % tols_0(0,1)); boost::replace_all(tyy1_0,",",".");
           string tyy1_1 = boost::str(boost::format("%.2f") % tols_1(0,1)); boost::replace_all(tyy1_1,",",".");
           string tyy1_2 = boost::str(boost::format("%.2f") % tols_2(0,1)); boost::replace_all(tyy1_2,",",".");

           PostureMod << string("param tol_target_right_yy1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy1_2+string("; \n");

           // yy2
           string tyy2_0 = boost::str(boost::format("%.2f") % tols_0(1,1)); boost::replace_all(tyy2_0,",",".");
           string tyy2_1 = boost::str(boost::format("%.2f") % tols_1(1,1)); boost::replace_all(tyy2_1,",",".");
           string tyy2_2 = boost::str(boost::format("%.2f") % tols_2(1,1)); boost::replace_all(tyy2_2,",",".");

           PostureMod << string("param tol_target_right_yy2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy2_2+string("; \n");


           // yy3
           string tyy3_0 = boost::str(boost::format("%.2f") % tols_0(2,1)); boost::replace_all(tyy3_0,",",".");
           string tyy3_1 = boost::str(boost::format("%.2f") % tols_1(2,1)); boost::replace_all(tyy3_1,",",".");
           string tyy3_2 = boost::str(boost::format("%.2f") % tols_2(2,1)); boost::replace_all(tyy3_2,",",".");

           PostureMod << string("param tol_target_right_yy3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy3_2+string("; \n");

           // zz1
           string tzz1_0 = boost::str(boost::format("%.2f") % tols_0(0,2)); boost::replace_all(tzz1_0,",",".");
           string tzz1_1 = boost::str(boost::format("%.2f") % tols_1(0,2)); boost::replace_all(tzz1_1,",",".");
           string tzz1_2 = boost::str(boost::format("%.2f") % tols_2(0,2)); boost::replace_all(tzz1_2,",",".");

           PostureMod << string("param tol_target_right_zz1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz1_2+string("; \n");

           // zz2
           string tzz2_0 = boost::str(boost::format("%.2f") % tols_0(1,2)); boost::replace_all(tzz2_0,",",".");
           string tzz2_1 = boost::str(boost::format("%.2f") % tols_1(1,2)); boost::replace_all(tzz2_1,",",".");
           string tzz2_2 = boost::str(boost::format("%.2f") % tols_2(1,2)); boost::replace_all(tzz2_2,",",".");

           PostureMod << string("param tol_target_right_zz2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz2_2+string("; \n");

           // zz3
           string tzz3_0 = boost::str(boost::format("%.2f") % tols_0(2,2)); boost::replace_all(tzz3_0,",",".");
           string tzz3_1 = boost::str(boost::format("%.2f") % tols_1(2,2)); boost::replace_all(tzz3_1,",",".");
           string tzz3_2 = boost::str(boost::format("%.2f") % tols_2(2,2)); boost::replace_all(tzz3_2,",",".");

           PostureMod << string("param tol_target_right_zz3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz3_2+string("; \n");


           // xy1
           string txy1_0 = boost::str(boost::format("%.2f") % tols_0(0,3)); boost::replace_all(txy1_0,",",".");
           string txy1_1 = boost::str(boost::format("%.2f") % tols_1(0,3)); boost::replace_all(txy1_1,",",".");
           string txy1_2 = boost::str(boost::format("%.2f") % tols_2(0,3)); boost::replace_all(txy1_2,",",".");

           PostureMod << string("param tol_target_right_xy1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy1_2+string("; \n");

           // xy2
           string txy2_0 = boost::str(boost::format("%.2f") % tols_0(1,3)); boost::replace_all(txy2_0,",",".");
           string txy2_1 = boost::str(boost::format("%.2f") % tols_1(1,3)); boost::replace_all(txy2_1,",",".");
           string txy2_2 = boost::str(boost::format("%.2f") % tols_2(1,3)); boost::replace_all(txy2_2,",",".");

           PostureMod << string("param tol_target_right_xy2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy2_2+string("; \n");

           // xy3
           string txy3_0 = boost::str(boost::format("%.2f") % tols_0(2,3)); boost::replace_all(txy3_0,",",".");
           string txy3_1 = boost::str(boost::format("%.2f") % tols_1(2,3)); boost::replace_all(txy3_1,",",".");
           string txy3_2 = boost::str(boost::format("%.2f") % tols_2(2,3)); boost::replace_all(txy3_2,",",".");

           PostureMod << string("param tol_target_right_xy3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy3_2+string("; \n");


           // xz1
           string txz1_0 = boost::str(boost::format("%.2f") % tols_0(0,4)); boost::replace_all(txz1_0,",",".");
           string txz1_1 = boost::str(boost::format("%.2f") % tols_1(0,4)); boost::replace_all(txz1_1,",",".");
           string txz1_2 = boost::str(boost::format("%.2f") % tols_2(0,4)); boost::replace_all(txz1_2,",",".");

           PostureMod << string("param tol_target_right_xz1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz1_2+string("; \n");

           // xz2
           string txz2_0 = boost::str(boost::format("%.2f") % tols_0(1,4)); boost::replace_all(txz2_0,",",".");
           string txz2_1 = boost::str(boost::format("%.2f") % tols_1(1,4)); boost::replace_all(txz2_1,",",".");
           string txz2_2 = boost::str(boost::format("%.2f") % tols_2(1,4)); boost::replace_all(txz2_2,",",".");

           PostureMod << string("param tol_target_right_xz2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz2_2+string("; \n");

           // xz3
           string txz3_0 = boost::str(boost::format("%.2f") % tols_0(2,4)); boost::replace_all(txz3_0,",",".");
           string txz3_1 = boost::str(boost::format("%.2f") % tols_1(2,4)); boost::replace_all(txz3_1,",",".");
           string txz3_2 = boost::str(boost::format("%.2f") % tols_2(2,4)); boost::replace_all(txz3_2,",",".");

           PostureMod << string("param tol_target_right_xz3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz3_2+string("; \n");

           // yz1
           string tyz1_0 = boost::str(boost::format("%.2f") % tols_0(0,5)); boost::replace_all(tyz1_0,",",".");
           string tyz1_1 = boost::str(boost::format("%.2f") % tols_1(0,5)); boost::replace_all(tyz1_1,",",".");
           string tyz1_2 = boost::str(boost::format("%.2f") % tols_2(0,5)); boost::replace_all(tyz1_2,",",".");

           PostureMod << string("param tol_target_right_yz1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz1_2+string("; \n");

           // yz2
           string tyz2_0 = boost::str(boost::format("%.2f") % tols_0(1,5)); boost::replace_all(tyz2_0,",",".");
           string tyz2_1 = boost::str(boost::format("%.2f") % tols_1(1,5)); boost::replace_all(tyz2_1,",",".");
           string tyz2_2 = boost::str(boost::format("%.2f") % tols_2(1,5)); boost::replace_all(tyz2_2,",",".");

           PostureMod << string("param tol_target_right_yz2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz2_2+string("; \n");


           // yz3
           string tyz3_0 = boost::str(boost::format("%.2f") % tols_0(2,5)); boost::replace_all(tyz3_0,",",".");
           string tyz3_1 = boost::str(boost::format("%.2f") % tols_1(2,5)); boost::replace_all(tyz3_1,",",".");
           string tyz3_2 = boost::str(boost::format("%.2f") % tols_2(2,5)); boost::replace_all(tyz3_2,",",".");

           PostureMod << string("param tol_target_right_yz3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz3_2+string("; \n");


           PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
           PostureMod << string("# \n");
           // in pick shorts movements (movements with N_STEP_MIN steps) collisions with the target are not considered
           int diff_steps = (int) (steps*BLANK_PERCENTAGE_DUAL_TAR);
           string n_steps_end_str = boost::str(boost::format("%d") % (diff_steps));
           PostureMod << string("subject to target_Arm_right{j in 4..15, l in 1..Nsteps-")+n_steps_end_str+("}:   \n");
           /*
           if(pre_post!=0){
               PostureMod << string("subject to target_Arm{j in 4..15, l in 1..Nsteps+1}:   \n");
           }else{
               int diff_steps = (int) (steps*BLANK_PERCENTAGE);
               string n_steps_end_str = boost::str(boost::format("%d") % (diff_steps));
               PostureMod << string("subject to target_Arm{j in 4..15, l in 1..Nsteps-")+n_steps_end_str+("}:   \n");
           }
           */
           PostureMod << string("(((x_t_right[1]*Points_Arm_right[j,1,l]+x_t_right[2]*Points_Arm_right[j,2,l]+x_t_right[3]*Points_Arm_right[j,3,l]\n");
           PostureMod << string("-ObjTar_right[1,1]*x_t_right[1]-ObjTar_right[1,2]*x_t_right[2]-ObjTar_right[1,3]*x_t_right[3])\n");
           PostureMod << string("/(ObjTar_right[1,4]+Points_Arm_right[j,4,l]+tol_target_right_xx1[l]))^2\n");
           PostureMod << string("+\n");
           PostureMod << string("((y_t_right[1]*Points_Arm_right[j,1,l]+y_t_right[2]*Points_Arm_right[j,2,l]+y_t_right[3]*Points_Arm_right[j,3,l]\n");
           PostureMod << string("-ObjTar_right[1,1]*y_t_right[1]-ObjTar_right[1,2]*y_t_right[2]-ObjTar_right[1,3]*y_t_right[3])\n");
           PostureMod << string("/(ObjTar_right[1,5]+Points_Arm_right[j,4,l]+tol_target_right_yy1[l]))^2\n");
           PostureMod << string("+\n");
           PostureMod << string("((z_t_right[1]*Points_Arm_right[j,1,l]+z_t_right[2]*Points_Arm_right[j,2,l]+z_t_right[3]*Points_Arm_right[j,3,l]\n");
           PostureMod << string("-ObjTar_right[1,1]*z_t_right[1]-ObjTar_right[1,2]*z_t_right[2]-ObjTar_right[1,3]*z_t_right[3])\n");
           PostureMod << string("/(ObjTar_right[1,6]+Points_Arm_right[j,4,l]+tol_target_right_zz1[l]))^2)\n");
           PostureMod << string(">= 1;\n");
           PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
           PostureMod << string("# \n");

           /*
           PostureMod << string("((Points_Arm_right[j,1,l]-ObjTar_right[1,1])^2)*( \n");
           PostureMod << string("(x_t_right[1])^2 / ((ObjTar_right[1,4]+Points_Arm_right[j,4,l]+tol_target_right_xx1[l])^2) + \n");
           PostureMod << string("(x_t_right[2])^2 / ((ObjTar_right[1,5]+Points_Arm_right[j,4,l]+tol_target_right_xx2[l])^2) + \n");
           PostureMod << string("(x_t_right[3])^2 / ((ObjTar_right[1,6]+Points_Arm_right[j,4,l]+tol_target_right_xx3[l])^2)) \n");
           PostureMod << string("+ \n");
           PostureMod << string("((Points_Arm_right[j,2,l]-ObjTar_right[1,2])^2)*(  \n");
           PostureMod << string("(y_t_right[1])^2 / ((ObjTar_right[1,4]+Points_Arm_right[j,4,l]+tol_target_right_yy1[l])^2) + \n");
           PostureMod << string("(y_t_right[2])^2 / ((ObjTar_right[1,5]+Points_Arm_right[j,4,l]+tol_target_right_yy2[l])^2) + \n");
           PostureMod << string("(y_t_right[3])^2 / ((ObjTar_right[1,6]+Points_Arm_right[j,4,l]+tol_target_right_yy3[l])^2)) \n");
           PostureMod << string("+ \n");
           PostureMod << string("((Points_Arm_right[j,3,l]-ObjTar_right[1,3])^2)*( \n");
           PostureMod << string("(z_t_right[1])^2 / ((ObjTar_right[1,4]+Points_Arm_right[j,4,l]+tol_target_right_zz1[l])^2) + \n");
           PostureMod << string("(z_t_right[2])^2 / ((ObjTar_right[1,5]+Points_Arm_right[j,4,l]+tol_target_right_zz2[l])^2) +  \n");
           PostureMod << string("(z_t_right[3])^2 / ((ObjTar_right[1,6]+Points_Arm_right[j,4,l]+tol_target_right_zz3[l])^2)) \n");
           PostureMod << string("+ \n");
           PostureMod << string("2*(Points_Arm_right[j,1,l]-ObjTar_right[1,1])*(Points_Arm_right[j,2,l]-ObjTar_right[1,2])* ( \n");
           PostureMod << string("(x_t_right[1]*y_t_right[1])/((ObjTar_right[1,4]+Points_Arm_right[j,4,l]+tol_target_right_xy1[l])^2) + \n");
           PostureMod << string("(x_t_right[2]*y_t_right[2])/((ObjTar_right[1,5]+Points_Arm_right[j,4,l]+tol_target_right_xy2[l])^2) + \n");
           PostureMod << string("(x_t_right[3]*y_t_right[3])/((ObjTar_right[1,6]+Points_Arm_right[j,4,l]+tol_target_right_xy3[l])^2)) \n");
           PostureMod << string("+ \n");
           PostureMod << string("2*(Points_Arm_right[j,1,l]-ObjTar_right[1,1])*(Points_Arm_right[j,3,l]-ObjTar_right[1,3])* ( \n");
           PostureMod << string("(x_t_right[1]*z_t_right[1])/((ObjTar_right[1,4]+Points_Arm_right[j,4,l]+tol_target_right_xz1[l])^2) + \n");
           PostureMod << string("(x_t_right[2]*z_t_right[2])/((ObjTar_right[1,5]+Points_Arm_right[j,4,l]+tol_target_right_xz2[l])^2) + \n");
           PostureMod << string("(x_t_right[3]*z_t_right[3])/((ObjTar_right[1,6]+Points_Arm_right[j,4,l]+tol_target_right_xz3[l])^2)) \n");
           PostureMod << string("+ \n");
           PostureMod << string("2*(Points_Arm_right[j,2,l]-ObjTar_right[1,2])*(Points_Arm_right[j,3,l]-ObjTar_right[1,3])* (\n");
           PostureMod << string("(y_t_right[1]*z_t_right[1])/((ObjTar_right[1,4]+Points_Arm_right[j,4,l]+tol_target_right_yz1[l])^2) + \n");
           PostureMod << string("(y_t_right[2]*z_t_right[2])/((ObjTar_right[1,5]+Points_Arm_right[j,4,l]+tol_target_right_yz2[l])^2) + \n");
           PostureMod << string("(y_t_right[3]*z_t_right[3])/((ObjTar_right[1,6]+Points_Arm_right[j,4,l]+tol_target_right_yz3[l])^2)) \n");
           PostureMod << string("-1 >=0; \n");
           PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
           PostureMod << string("# \n");
           */
      }

      if(target_avoidance && !obj_left_place && !move){

           // constraints with the targets
           MatrixXd tols_0 = tolsTarget_left.at(0);
           MatrixXd tols_1 = tolsTarget_left.at(1);
           MatrixXd tols_2 = tolsTarget_left.at(2);

           //xx1
           string txx1_0 = boost::str(boost::format("%.2f") % tols_0(0,0)); boost::replace_all(txx1_0,",",".");
           string txx1_1 = boost::str(boost::format("%.2f") % tols_1(0,0)); boost::replace_all(txx1_1,",",".");
           string txx1_2 = boost::str(boost::format("%.2f") % tols_2(0,0)); boost::replace_all(txx1_2,",",".");

           PostureMod << string("param tol_target_left_xx1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx1_2+string("; \n");

           //xx2
           string txx2_0 = boost::str(boost::format("%.2f") % tols_0(1,0)); boost::replace_all(txx2_0,",",".");
           string txx2_1 = boost::str(boost::format("%.2f") % tols_1(1,0)); boost::replace_all(txx2_1,",",".");
           string txx2_2 = boost::str(boost::format("%.2f") % tols_2(1,0)); boost::replace_all(txx2_2,",",".");

           PostureMod << string("param tol_target_left_xx2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx2_2+string("; \n");


           // xx3
           string txx3_0 = boost::str(boost::format("%.2f") % tols_0(2,0)); boost::replace_all(txx3_0,",",".");
           string txx3_1 = boost::str(boost::format("%.2f") % tols_1(2,0)); boost::replace_all(txx3_1,",",".");
           string txx3_2 = boost::str(boost::format("%.2f") % tols_2(2,0)); boost::replace_all(txx3_2,",",".");

           PostureMod << string("param tol_target_left_xx3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txx3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txx3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txx3_2+string("; \n");

           // yy1
           string tyy1_0 = boost::str(boost::format("%.2f") % tols_0(0,1)); boost::replace_all(tyy1_0,",",".");
           string tyy1_1 = boost::str(boost::format("%.2f") % tols_1(0,1)); boost::replace_all(tyy1_1,",",".");
           string tyy1_2 = boost::str(boost::format("%.2f") % tols_2(0,1)); boost::replace_all(tyy1_2,",",".");

           PostureMod << string("param tol_target_left_yy1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy1_2+string("; \n");

           // yy2
           string tyy2_0 = boost::str(boost::format("%.2f") % tols_0(1,1)); boost::replace_all(tyy2_0,",",".");
           string tyy2_1 = boost::str(boost::format("%.2f") % tols_1(1,1)); boost::replace_all(tyy2_1,",",".");
           string tyy2_2 = boost::str(boost::format("%.2f") % tols_2(1,1)); boost::replace_all(tyy2_2,",",".");

           PostureMod << string("param tol_target_left_yy2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy2_2+string("; \n");


           // yy3
           string tyy3_0 = boost::str(boost::format("%.2f") % tols_0(2,1)); boost::replace_all(tyy3_0,",",".");
           string tyy3_1 = boost::str(boost::format("%.2f") % tols_1(2,1)); boost::replace_all(tyy3_1,",",".");
           string tyy3_2 = boost::str(boost::format("%.2f") % tols_2(2,1)); boost::replace_all(tyy3_2,",",".");

           PostureMod << string("param tol_target_left_yy3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyy3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyy3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyy3_2+string("; \n");

           // zz1
           string tzz1_0 = boost::str(boost::format("%.2f") % tols_0(0,2)); boost::replace_all(tzz1_0,",",".");
           string tzz1_1 = boost::str(boost::format("%.2f") % tols_1(0,2)); boost::replace_all(tzz1_1,",",".");
           string tzz1_2 = boost::str(boost::format("%.2f") % tols_2(0,2)); boost::replace_all(tzz1_2,",",".");

           PostureMod << string("param tol_target_left_zz1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz1_2+string("; \n");

           // zz2
           string tzz2_0 = boost::str(boost::format("%.2f") % tols_0(1,2)); boost::replace_all(tzz2_0,",",".");
           string tzz2_1 = boost::str(boost::format("%.2f") % tols_1(1,2)); boost::replace_all(tzz2_1,",",".");
           string tzz2_2 = boost::str(boost::format("%.2f") % tols_2(1,2)); boost::replace_all(tzz2_2,",",".");

           PostureMod << string("param tol_target_left_zz2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz2_2+string("; \n");

           // zz3
           string tzz3_0 = boost::str(boost::format("%.2f") % tols_0(2,2)); boost::replace_all(tzz3_0,",",".");
           string tzz3_1 = boost::str(boost::format("%.2f") % tols_1(2,2)); boost::replace_all(tzz3_1,",",".");
           string tzz3_2 = boost::str(boost::format("%.2f") % tols_2(2,2)); boost::replace_all(tzz3_2,",",".");

           PostureMod << string("param tol_target_left_zz3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tzz3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tzz3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tzz3_2+string("; \n");


           // xy1
           string txy1_0 = boost::str(boost::format("%.2f") % tols_0(0,3)); boost::replace_all(txy1_0,",",".");
           string txy1_1 = boost::str(boost::format("%.2f") % tols_1(0,3)); boost::replace_all(txy1_1,",",".");
           string txy1_2 = boost::str(boost::format("%.2f") % tols_2(0,3)); boost::replace_all(txy1_2,",",".");

           PostureMod << string("param tol_target_left_xy1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy1_2+string("; \n");

           // xy2
           string txy2_0 = boost::str(boost::format("%.2f") % tols_0(1,3)); boost::replace_all(txy2_0,",",".");
           string txy2_1 = boost::str(boost::format("%.2f") % tols_1(1,3)); boost::replace_all(txy2_1,",",".");
           string txy2_2 = boost::str(boost::format("%.2f") % tols_2(1,3)); boost::replace_all(txy2_2,",",".");

           PostureMod << string("param tol_target_left_xy2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy2_2+string("; \n");

           // xy3
           string txy3_0 = boost::str(boost::format("%.2f") % tols_0(2,3)); boost::replace_all(txy3_0,",",".");
           string txy3_1 = boost::str(boost::format("%.2f") % tols_1(2,3)); boost::replace_all(txy3_1,",",".");
           string txy3_2 = boost::str(boost::format("%.2f") % tols_2(2,3)); boost::replace_all(txy3_2,",",".");

           PostureMod << string("param tol_target_left_xy3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txy3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txy3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txy3_2+string("; \n");


           // xz1
           string txz1_0 = boost::str(boost::format("%.2f") % tols_0(0,4)); boost::replace_all(txz1_0,",",".");
           string txz1_1 = boost::str(boost::format("%.2f") % tols_1(0,4)); boost::replace_all(txz1_1,",",".");
           string txz1_2 = boost::str(boost::format("%.2f") % tols_2(0,4)); boost::replace_all(txz1_2,",",".");

           PostureMod << string("param tol_target_left_xz1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz1_2+string("; \n");

           // xz2
           string txz2_0 = boost::str(boost::format("%.2f") % tols_0(1,4)); boost::replace_all(txz2_0,",",".");
           string txz2_1 = boost::str(boost::format("%.2f") % tols_1(1,4)); boost::replace_all(txz2_1,",",".");
           string txz2_2 = boost::str(boost::format("%.2f") % tols_2(1,4)); boost::replace_all(txz2_2,",",".");

           PostureMod << string("param tol_target_left_xz2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz2_2+string("; \n");

           // xz3
           string txz3_0 = boost::str(boost::format("%.2f") % tols_0(2,4)); boost::replace_all(txz3_0,",",".");
           string txz3_1 = boost::str(boost::format("%.2f") % tols_1(2,4)); boost::replace_all(txz3_1,",",".");
           string txz3_2 = boost::str(boost::format("%.2f") % tols_2(2,4)); boost::replace_all(txz3_2,",",".");

           PostureMod << string("param tol_target_left_xz3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+txz3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+txz3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+txz3_2+string("; \n");

           // yz1
           string tyz1_0 = boost::str(boost::format("%.2f") % tols_0(0,5)); boost::replace_all(tyz1_0,",",".");
           string tyz1_1 = boost::str(boost::format("%.2f") % tols_1(0,5)); boost::replace_all(tyz1_1,",",".");
           string tyz1_2 = boost::str(boost::format("%.2f") % tols_2(0,5)); boost::replace_all(tyz1_2,",",".");

           PostureMod << string("param tol_target_left_yz1 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz1_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz1_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz1_2+string("; \n");

           // yz2
           string tyz2_0 = boost::str(boost::format("%.2f") % tols_0(1,5)); boost::replace_all(tyz2_0,",",".");
           string tyz2_1 = boost::str(boost::format("%.2f") % tols_1(1,5)); boost::replace_all(tyz2_1,",",".");
           string tyz2_2 = boost::str(boost::format("%.2f") % tols_2(1,5)); boost::replace_all(tyz2_2,",",".");

           PostureMod << string("param tol_target_left_yz2 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz2_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz2_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz2_2+string("; \n");


           // yz3
           string tyz3_0 = boost::str(boost::format("%.2f") % tols_0(2,5)); boost::replace_all(tyz3_0,",",".");
           string tyz3_1 = boost::str(boost::format("%.2f") % tols_1(2,5)); boost::replace_all(tyz3_1,",",".");
           string tyz3_2 = boost::str(boost::format("%.2f") % tols_2(2,5)); boost::replace_all(tyz3_2,",",".");

           PostureMod << string("param tol_target_left_yz3 {i in 1..Nsteps+1} := \n");
           PostureMod << string("if 		(i <= (Nsteps+1)*0.8) then ")+tyz3_0+string("\n");
           PostureMod << string("else if (i>(Nsteps+1)*0.8 && i < (Nsteps+1)*0.95) then ")+tyz3_1+string("\n");
           PostureMod << string("else if (i>=(Nsteps+1)*0.95) then ")+tyz3_2+string("; \n");


           PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
           PostureMod << string("# \n");
           // in pick shorts movements (movements with N_STEP_MIN steps) collisions with the target are not considered
           int diff_steps = (int) (steps*BLANK_PERCENTAGE_DUAL_TAR);
           string n_steps_end_str = boost::str(boost::format("%d") % (diff_steps));
           PostureMod << string("subject to target_Arm_left{j in 4..15, l in 1..Nsteps-")+n_steps_end_str+("}:   \n");
           /*
           if(pre_post!=0){
               PostureMod << string("subject to target_Arm{j in 4..15, l in 1..Nsteps+1}:   \n");
           }else{
               int diff_steps = (int) (steps*BLANK_PERCENTAGE);
               string n_steps_end_str = boost::str(boost::format("%d") % (diff_steps));
               PostureMod << string("subject to target_Arm{j in 4..15, l in 1..Nsteps-")+n_steps_end_str+("}:   \n");
           }
           */

           PostureMod << string("(((x_t_left[1]*Points_Arm_left[j,1,l]+x_t_left[2]*Points_Arm_left[j,2,l]+x_t_left[3]*Points_Arm_left[j,3,l]\n");
           PostureMod << string("-ObjTar_left[1,1]*x_t_left[1]-ObjTar_left[1,2]*x_t_left[2]-ObjTar_left[1,3]*x_t_left[3])\n");
           PostureMod << string("/(ObjTar_left[1,4]+Points_Arm_left[j,4,l]+tol_target_left_xx1[l]))^2\n");
           PostureMod << string("+\n");
           PostureMod << string("((y_t_left[1]*Points_Arm_left[j,1,l]+y_t_left[2]*Points_Arm_left[j,2,l]+y_t_left[3]*Points_Arm_left[j,3,l]\n");
           PostureMod << string("-ObjTar_left[1,1]*y_t_left[1]-ObjTar_left[1,2]*y_t_left[2]-ObjTar_left[1,3]*y_t_left[3])\n");
           PostureMod << string("/(ObjTar_left[1,5]+Points_Arm_left[j,4,l]+tol_target_left_yy1[l]))^2\n");
           PostureMod << string("+\n");
           PostureMod << string("((z_t_left[1]*Points_Arm_left[j,1,l]+z_t_left[2]*Points_Arm_left[j,2,l]+z_t_left[3]*Points_Arm_left[j,3,l]\n");
           PostureMod << string("-ObjTar_left[1,1]*z_t_left[1]-ObjTar_left[1,2]*z_t_left[2]-ObjTar_left[1,3]*z_t_left[3])\n");
           PostureMod << string("/(ObjTar_left[1,6]+Points_Arm_left[j,4,l]+tol_target_left_zz1[l]))^2)\n");
           PostureMod << string(">= 1;\n");
           PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
           PostureMod << string("# \n");

           /*
           PostureMod << string("((Points_Arm_left[j,1,l]-ObjTar_left[1,1])^2)*( \n");
           PostureMod << string("(x_t_left[1])^2 / ((ObjTar_left[1,4]+Points_Arm_left[j,4,l]+tol_target_left_xx1[l])^2) + \n");
           PostureMod << string("(x_t_left[2])^2 / ((ObjTar_left[1,5]+Points_Arm_left[j,4,l]+tol_target_left_xx2[l])^2) + \n");
           PostureMod << string("(x_t_left[3])^2 / ((ObjTar_left[1,6]+Points_Arm_left[j,4,l]+tol_target_left_xx3[l])^2)) \n");
           PostureMod << string("+ \n");
           PostureMod << string("((Points_Arm_left[j,2,l]-ObjTar_left[1,2])^2)*(  \n");
           PostureMod << string("(y_t_left[1])^2 / ((ObjTar_left[1,4]+Points_Arm_left[j,4,l]+tol_target_left_yy1[l])^2) + \n");
           PostureMod << string("(y_t_left[2])^2 / ((ObjTar_left[1,5]+Points_Arm_left[j,4,l]+tol_target_left_yy2[l])^2) + \n");
           PostureMod << string("(y_t_left[3])^2 / ((ObjTar_left[1,6]+Points_Arm_left[j,4,l]+tol_target_left_yy3[l])^2)) \n");
           PostureMod << string("+ \n");
           PostureMod << string("((Points_Arm_left[j,3,l]-ObjTar_left[1,3])^2)*( \n");
           PostureMod << string("(z_t_left[1])^2 / ((ObjTar_left[1,4]+Points_Arm_left[j,4,l]+tol_target_left_zz1[l])^2) + \n");
           PostureMod << string("(z_t_left[2])^2 / ((ObjTar_left[1,5]+Points_Arm_left[j,4,l]+tol_target_left_zz2[l])^2) +  \n");
           PostureMod << string("(z_t_left[3])^2 / ((ObjTar_left[1,6]+Points_Arm_left[j,4,l]+tol_target_left_zz3[l])^2)) \n");
           PostureMod << string("+ \n");
           PostureMod << string("2*(Points_Arm_left[j,1,l]-ObjTar_left[1,1])*(Points_Arm_left[j,2,l]-ObjTar_left[1,2])* ( \n");
           PostureMod << string("(x_t_left[1]*y_t_left[1])/((ObjTar_left[1,4]+Points_Arm_left[j,4,l]+tol_target_left_xy1[l])^2) + \n");
           PostureMod << string("(x_t_left[2]*y_t_left[2])/((ObjTar_left[1,5]+Points_Arm_left[j,4,l]+tol_target_left_xy2[l])^2) + \n");
           PostureMod << string("(x_t_left[3]*y_t_left[3])/((ObjTar_left[1,6]+Points_Arm_left[j,4,l]+tol_target_left_xy3[l])^2)) \n");
           PostureMod << string("+ \n");
           PostureMod << string("2*(Points_Arm_left[j,1,l]-ObjTar_left[1,1])*(Points_Arm_left[j,3,l]-ObjTar_left[1,3])* ( \n");
           PostureMod << string("(x_t_left[1]*z_t_left[1])/((ObjTar_left[1,4]+Points_Arm_left[j,4,l]+tol_target_left_xz1[l])^2) + \n");
           PostureMod << string("(x_t_left[2]*z_t_left[2])/((ObjTar_left[1,5]+Points_Arm_left[j,4,l]+tol_target_left_xz2[l])^2) + \n");
           PostureMod << string("(x_t_left[3]*z_t_left[3])/((ObjTar_left[1,6]+Points_Arm_left[j,4,l]+tol_target_left_xz3[l])^2)) \n");
           PostureMod << string("+ \n");
           PostureMod << string("2*(Points_Arm_left[j,2,l]-ObjTar_left[1,2])*(Points_Arm_left[j,3,l]-ObjTar_left[1,3])* (\n");
           PostureMod << string("(y_t_left[1]*z_t_left[1])/((ObjTar_left[1,4]+Points_Arm_left[j,4,l]+tol_target_left_yz1[l])^2) + \n");
           PostureMod << string("(y_t_left[2]*z_t_left[2])/((ObjTar_left[1,5]+Points_Arm_left[j,4,l]+tol_target_left_yz2[l])^2) + \n");
           PostureMod << string("(y_t_left[3]*z_t_left[3])/((ObjTar_left[1,6]+Points_Arm_left[j,4,l]+tol_target_left_yz3[l])^2)) \n");
           PostureMod << string("-1 >=0; \n");
           PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
           PostureMod << string("# \n");
           */
      }

      if(obstacle_avoidance){
         // coinstraints with the obstacles
         MatrixXd tolsObs_0 = tolsObstacles_right.at(0);
         MatrixXd tolsObs_1 = tolsObstacles_right.at(1);
         //xx1
         string tbxx1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,0)); boost::replace_all(tbxx1_0,",",".");
         string tbxx1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,0)); boost::replace_all(tbxx1_1,",",".");
         PostureMod << string("param tol_obs_right_xx1 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx1_1+string("; \n");
         //xx2
         string tbxx2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,0)); boost::replace_all(tbxx2_0,",",".");
         string tbxx2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,0)); boost::replace_all(tbxx2_1,",",".");
         PostureMod << string("param tol_obs_right_xx2 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx2_1+string("; \n");
         // xx3
         string tbxx3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,0)); boost::replace_all(tbxx3_0,",",".");
         string tbxx3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,0)); boost::replace_all(tbxx3_1,",",".");
         PostureMod << string("param tol_obs_right_xx3 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx3_1+string("; \n");
         // yy1
         string tbyy1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,1)); boost::replace_all(tbyy1_0,",",".");
         string tbyy1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,1)); boost::replace_all(tbyy1_1,",",".");
         PostureMod << string("param tol_obs_right_yy1 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy1_1+string("; \n");
         // yy2
         string tbyy2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,1)); boost::replace_all(tbyy2_0,",",".");
         string tbyy2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,1)); boost::replace_all(tbyy2_1,",",".");
         PostureMod << string("param tol_obs_right_yy2 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy2_1+string("; \n");
         // yy3
         string tbyy3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,1)); boost::replace_all(tbyy3_0,",",".");
         string tbyy3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,1)); boost::replace_all(tbyy3_1,",",".");
         PostureMod << string("param tol_obs_right_yy3 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy3_1+string("; \n");
         // zz1
         string tbzz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,2)); boost::replace_all(tbzz1_0,",",".");
         string tbzz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,2)); boost::replace_all(tbzz1_1,",",".");
         PostureMod << string("param tol_obs_right_zz1 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz1_1+string("; \n");
         // zz2
         string tbzz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,2)); boost::replace_all(tbzz2_0,",",".");
         string tbzz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,2)); boost::replace_all(tbzz2_1,",",".");
         PostureMod << string("param tol_obs_right_zz2 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz2_1+string("; \n");
         // zz3
         string tbzz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,2)); boost::replace_all(tbzz3_0,",",".");
         string tbzz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,2)); boost::replace_all(tbzz3_1,",",".");
         PostureMod << string("param tol_obs_right_zz3 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz3_1+string("; \n");
         // xy1
         string tbxy1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,3)); boost::replace_all(tbxy1_0,",",".");
         string tbxy1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,3)); boost::replace_all(tbxy1_1,",",".");
         PostureMod << string("param tol_obs_right_xy1 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy1_1+string("; \n");
         // xy2
         string tbxy2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,3)); boost::replace_all(tbxy2_0,",",".");
         string tbxy2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,3)); boost::replace_all(tbxy2_1,",",".");
         PostureMod << string("param tol_obs_right_xy2 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy2_1+string("; \n");
         // xy3
         string tbxy3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,3)); boost::replace_all(tbxy3_0,",",".");
         string tbxy3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,3)); boost::replace_all(tbxy3_1,",",".");
         PostureMod << string("param tol_obs_right_xy3 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy3_1+string("; \n");
         // xz1
         string tbxz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,4)); boost::replace_all(tbxz1_0,",",".");
         string tbxz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,4)); boost::replace_all(tbxz1_1,",",".");
         PostureMod << string("param tol_obs_right_xz1 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz1_1+string("; \n");
         // xz2
         string tbxz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,4)); boost::replace_all(tbxz2_0,",",".");
         string tbxz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,4)); boost::replace_all(tbxz2_1,",",".");
         PostureMod << string("param tol_obs_right_xz2 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz2_1+string("; \n");
         // xz3
         string tbxz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,4)); boost::replace_all(tbxz3_0,",",".");
         string tbxz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,4)); boost::replace_all(tbxz3_1,",",".");
         PostureMod << string("param tol_obs_right_xz3 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz3_1+string("; \n");
         // yz1
         string tbyz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,5)); boost::replace_all(tbyz1_0,",",".");
         string tbyz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,5)); boost::replace_all(tbyz1_1,",",".");
         PostureMod << string("param tol_obs_right_yz1{i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz1_1+string("; \n");
         // yz2
         string tbyz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,5)); boost::replace_all(tbyz2_0,",",".");
         string tbyz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,5)); boost::replace_all(tbyz2_1,",",".");
         PostureMod << string("param tol_obs_right_yz2{i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz2_1+string("; \n");
         // yz3
         string tbyz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,5)); boost::replace_all(tbyz3_0,",",".");
         string tbyz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,5)); boost::replace_all(tbyz3_1,",",".");
         PostureMod << string("param tol_obs_right_yz3{i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz3_1+string("; \n");
         PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
         PostureMod << string("# \n");
         if(obj_right_place){
             // the object to place has to be considered
              PostureMod << string("subject to obst_Arm_right{j in 1..")+n_str_right+string(", i in 1..n_Obstacles_right, l in 1..Nsteps+1}:\n"); // approach stage is necessary
         }else if(move){
             // for the first number of diff_steps, no obstacle is considered because the movement is very short and the planner may get stuck
             int diff_steps = std::max(1,(int)(steps*BLANK_PERCENTAGE_DUAL_OBS));
             string n_steps_init_str = boost::str(boost::format("%d") % (diff_steps));
             PostureMod << string("subject to obst_Arm_right{j in 1..15, i in 1..(n_Obstacles_right), l in ")+n_steps_init_str+("..Nsteps+1}:\n");
         }else{
              PostureMod << string("subject to obst_Arm_right{j in 1..15, i in 1..(n_Obstacles_right), l in 1..Nsteps+1}:\n"); // pick movements
         }

         PostureMod << string("(((Rot_right[1,1,i]*Points_Arm_right[j,1,l]+Rot_right[2,1,i]*Points_Arm_right[j,2,l]+Rot_right[3,1,i]*Points_Arm_right[j,3,l]\n");
         PostureMod << string("-Obstacles_right[i,1]*Rot_right[1,1,i]-Obstacles_right[i,2]*Rot_right[2,1,i]-Obstacles_right[i,3]*Rot_right[3,1,i])\n");
         PostureMod << string("/(Obstacles_right[i,4]+Points_Arm_right[j,4,l]+tol_obs_right_xx1[l]))^2\n");
         PostureMod << string("+\n");
         PostureMod << string("((Rot_right[1,2,i]*Points_Arm_right[j,1,l]+Rot_right[2,2,i]*Points_Arm_right[j,2,l]+Rot_right[3,2,i]*Points_Arm_right[j,3,l]\n");
         PostureMod << string("-Obstacles_right[i,1]*Rot_right[1,2,i]-Obstacles_right[i,2]*Rot_right[2,2,i]-Obstacles_right[i,3]*Rot_right[3,2,i])\n");
         PostureMod << string("/(Obstacles_right[i,5]+Points_Arm_right[j,4,l]+tol_obs_right_yy1[l]))^2\n");
         PostureMod << string("+\n");
         PostureMod << string("((Rot_right[1,3,i]*Points_Arm_right[j,1,l]+Rot_right[2,3,i]*Points_Arm_right[j,2,l]+Rot_right[3,3,i]*Points_Arm_right[j,3,l]\n");
         PostureMod << string("-Obstacles_right[i,1]*Rot_right[1,3,i]-Obstacles_right[i,2]*Rot_right[2,3,i]-Obstacles_right[i,3]*Rot_right[3,3,i])\n");
         PostureMod << string("/(Obstacles_right[i,6]+Points_Arm_right[j,4,l]+tol_obs_right_zz1[l]))^2)\n");
         PostureMod << string(">= 1;\n");
         PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
         PostureMod << string("# \n");

         if((dual_mov_type==0 && pre_post!=2) || (dual_mov_type==1 && pre_post==2)){
             // pick movements (plan and approach stages)
             // OR
             // place movements (retreat stage)
             PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
             PostureMod << string("# \n");
             PostureMod << string("subject to objLeft_Arm_right{j in 1..")+n_str_right+string(", i in 1..n_ObjTar_left, l in 1..Nsteps+1}:  \n");
             PostureMod << string("(((Rot_obj_left[1,1]*Points_Arm_right[j,1,l]+Rot_obj_left[2,1]*Points_Arm_right[j,2,l]+Rot_obj_left[3,1]*Points_Arm_right[j,3,l]\n");
             PostureMod << string("-ObjTar_left[i,1]*Rot_obj_left[1,1]-ObjTar_left[i,2]*Rot_obj_left[2,1]-ObjTar_left[i,3]*Rot_obj_left[3,1])\n");
             PostureMod << string("/(ObjTar_left[i,4]+Points_Arm_right[j,4,l]+tol_obs_right_xx1[l]))^2\n");
             PostureMod << string("+\n");
             PostureMod << string("((Rot_obj_left[1,2]*Points_Arm_right[j,1,l]+Rot_obj_left[2,2]*Points_Arm_right[j,2,l]+Rot_obj_left[3,2]*Points_Arm_right[j,3,l]\n");
             PostureMod << string("-ObjTar_left[i,1]*Rot_obj_left[1,2]-ObjTar_left[i,2]*Rot_obj_left[2,2]-ObjTar_left[i,3]*Rot_obj_left[3,2])\n");
             PostureMod << string("/(ObjTar_left[i,5]+Points_Arm_right[j,4,l]+tol_obs_right_yy1[l]))^2\n");
             PostureMod << string("+\n");
             PostureMod << string("((Rot_obj_left[1,3]*Points_Arm_right[j,1,l]+Rot_obj_left[2,3]*Points_Arm_right[j,2,l]+Rot_obj_left[3,3]*Points_Arm_right[j,3,l]\n");
             PostureMod << string("-ObjTar_left[i,1]*Rot_obj_left[1,3]-ObjTar_left[i,2]*Rot_obj_left[2,3]-ObjTar_left[i,3]*Rot_obj_left[3,3])\n");
             PostureMod << string("/(ObjTar_left[i,6]+Points_Arm_right[j,4,l]+tol_obs_right_zz1[l]))^2)\n");
             PostureMod << string(">= 1;\n");
             PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
             PostureMod << string("#  \n");
         }

         /*
         PostureMod << string("((Points_Arm_right[j,1,l]-Obstacles_right[i,1])^2)*(\n");
         PostureMod << string("(Rot_right[1,1,i])^2 / ((Obstacles_right[i,4]+Points_Arm_right[j,4,l]+tol_obs_right_xx1[l])^2) +\n");
         PostureMod << string("(Rot_right[2,1,i])^2 / ((Obstacles_right[i,5]+Points_Arm_right[j,4,l]+tol_obs_right_xx2[l])^2) + \n");
         PostureMod << string("(Rot_right[3,1,i])^2 / ((Obstacles_right[i,6]+Points_Arm_right[j,4,l]+tol_obs_right_xx3[l])^2)) \n");
         PostureMod << string("+ \n");
         PostureMod << string("((Points_Arm_right[j,2,l]-Obstacles_right[i,2])^2)*( \n");
         PostureMod << string("(Rot_right[1,2,i])^2 / ((Obstacles_right[i,4]+Points_Arm_right[j,4,l]+tol_obs_right_yy1[l])^2) + \n");
         PostureMod << string("(Rot_right[2,2,i])^2 / ((Obstacles_right[i,5]+Points_Arm_right[j,4,l]+tol_obs_right_yy2[l])^2) + \n");
         PostureMod << string("(Rot_right[3,2,i])^2 / ((Obstacles_right[i,6]+Points_Arm_right[j,4,l]+tol_obs_right_yy3[l])^2)) \n");
         PostureMod << string("+ \n");
         PostureMod << string("((Points_Arm_right[j,3,l]-Obstacles_right[i,3])^2)*( \n");
         PostureMod << string("(Rot_right[1,3,i])^2 / ((Obstacles_right[i,4]+Points_Arm_right[j,4,l]+tol_obs_right_zz1[l])^2) + \n");
         PostureMod << string("(Rot_right[2,3,i])^2 / ((Obstacles_right[i,5]+Points_Arm_right[j,4,l]+tol_obs_right_zz2[l])^2) + \n");
         PostureMod << string("(Rot_right[3,3,i])^2 / ((Obstacles_right[i,6]+Points_Arm_right[j,4,l]+tol_obs_right_zz3[l])^2)) \n");
         PostureMod << string("+ \n");
         PostureMod << string("2*(Points_Arm_right[j,1,l]-Obstacles_right[i,1])*(Points_Arm_right[j,2,l]-Obstacles_right[i,2])* ( \n");
         PostureMod << string("(Rot_right[1,1,i]*Rot_right[1,2,i])/((Obstacles_right[i,4]+Points_Arm_right[j,4,l]+tol_obs_right_xy1[l])^2) + \n");
         PostureMod << string("(Rot_right[2,1,i]*Rot_right[2,2,i])/((Obstacles_right[i,5]+Points_Arm_right[j,4,l]+tol_obs_right_xy2[l])^2) + \n");
         PostureMod << string("(Rot_right[3,1,i]*Rot_right[3,2,i])/((Obstacles_right[i,6]+Points_Arm_right[j,4,l]+tol_obs_right_xy3[l])^2)) \n");
         PostureMod << string("+ \n");
         PostureMod << string("2*(Points_Arm_right[j,1,l]-Obstacles_right[i,1])*(Points_Arm_right[j,3,l]-Obstacles_right[i,3])* ( \n");
         PostureMod << string("(Rot_right[1,1,i]*Rot_right[1,3,i])/((Obstacles_right[i,4]+Points_Arm_right[j,4,l]+tol_obs_right_xz1[l])^2) + \n");
         PostureMod << string("(Rot_right[2,1,i]*Rot_right[2,3,i])/((Obstacles_right[i,5]+Points_Arm_right[j,4,l]+tol_obs_right_xz2[l])^2) + \n");
         PostureMod << string("(Rot_right[3,1,i]*Rot_right[3,3,i])/((Obstacles_right[i,6]+Points_Arm_right[j,4,l]+tol_obs_right_xz3[l])^2)) \n");
         PostureMod << string("+ \n");
         PostureMod << string("2*(Points_Arm_right[j,2,l]-Obstacles_right[i,2])*(Points_Arm_right[j,3,l]-Obstacles_right[i,3])* ( \n");
         PostureMod << string("(Rot_right[1,2,i]*Rot_right[1,3,i])/((Obstacles_right[i,4]+Points_Arm_right[j,4,l]+tol_obs_right_yz1[l])^2) + \n");
         PostureMod << string("(Rot_right[2,2,i]*Rot_right[2,3,i])/((Obstacles_right[i,5]+Points_Arm_right[j,4,l]+tol_obs_right_yz2[l])^2) + \n");
         PostureMod << string("(Rot_right[3,2,i]*Rot_right[3,3,i])/((Obstacles_right[i,6]+Points_Arm_right[j,4,l]+tol_obs_right_yz3[l])^2)) \n");
         PostureMod << string("-1 >=0; \n");
         PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
         PostureMod << string("# \n");
         */
      }

      if(obstacle_avoidance){
         // coinstraints with the obstacles
         MatrixXd tolsObs_0 = tolsObstacles_left.at(0);
         MatrixXd tolsObs_1 = tolsObstacles_left.at(1);
         //xx1
         string tbxx1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,0)); boost::replace_all(tbxx1_0,",",".");
         string tbxx1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,0)); boost::replace_all(tbxx1_1,",",".");
         PostureMod << string("param tol_obs_left_xx1 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx1_1+string("; \n");
         //xx2
         string tbxx2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,0)); boost::replace_all(tbxx2_0,",",".");
         string tbxx2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,0)); boost::replace_all(tbxx2_1,",",".");
         PostureMod << string("param tol_obs_left_xx2 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx2_1+string("; \n");
         // xx3
         string tbxx3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,0)); boost::replace_all(tbxx3_0,",",".");
         string tbxx3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,0)); boost::replace_all(tbxx3_1,",",".");
         PostureMod << string("param tol_obs_left_xx3 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxx3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxx3_1+string("; \n");
         // yy1
         string tbyy1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,1)); boost::replace_all(tbyy1_0,",",".");
         string tbyy1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,1)); boost::replace_all(tbyy1_1,",",".");
         PostureMod << string("param tol_obs_left_yy1 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy1_1+string("; \n");
         // yy2
         string tbyy2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,1)); boost::replace_all(tbyy2_0,",",".");
         string tbyy2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,1)); boost::replace_all(tbyy2_1,",",".");
         PostureMod << string("param tol_obs_left_yy2 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy2_1+string("; \n");
         // yy3
         string tbyy3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,1)); boost::replace_all(tbyy3_0,",",".");
         string tbyy3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,1)); boost::replace_all(tbyy3_1,",",".");
         PostureMod << string("param tol_obs_left_yy3 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyy3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyy3_1+string("; \n");
         // zz1
         string tbzz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,2)); boost::replace_all(tbzz1_0,",",".");
         string tbzz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,2)); boost::replace_all(tbzz1_1,",",".");
         PostureMod << string("param tol_obs_left_zz1 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz1_1+string("; \n");
         // zz2
         string tbzz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,2)); boost::replace_all(tbzz2_0,",",".");
         string tbzz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,2)); boost::replace_all(tbzz2_1,",",".");
         PostureMod << string("param tol_obs_left_zz2 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz2_1+string("; \n");
         // zz3
         string tbzz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,2)); boost::replace_all(tbzz3_0,",",".");
         string tbzz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,2)); boost::replace_all(tbzz3_1,",",".");
         PostureMod << string("param tol_obs_left_zz3 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbzz3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbzz3_1+string("; \n");
         // xy1
         string tbxy1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,3)); boost::replace_all(tbxy1_0,",",".");
         string tbxy1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,3)); boost::replace_all(tbxy1_1,",",".");
         PostureMod << string("param tol_obs_left_xy1 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy1_1+string("; \n");
         // xy2
         string tbxy2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,3)); boost::replace_all(tbxy2_0,",",".");
         string tbxy2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,3)); boost::replace_all(tbxy2_1,",",".");
         PostureMod << string("param tol_obs_left_xy2 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy2_1+string("; \n");
         // xy3
         string tbxy3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,3)); boost::replace_all(tbxy3_0,",",".");
         string tbxy3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,3)); boost::replace_all(tbxy3_1,",",".");
         PostureMod << string("param tol_obs_left_xy3 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxy3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxy3_1+string("; \n");
         // xz1
         string tbxz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,4)); boost::replace_all(tbxz1_0,",",".");
         string tbxz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,4)); boost::replace_all(tbxz1_1,",",".");
         PostureMod << string("param tol_obs_left_xz1 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz1_1+string("; \n");
         // xz2
         string tbxz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,4)); boost::replace_all(tbxz2_0,",",".");
         string tbxz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,4)); boost::replace_all(tbxz2_1,",",".");
         PostureMod << string("param tol_obs_left_xz2 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz2_1+string("; \n");
         // xz3
         string tbxz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,4)); boost::replace_all(tbxz3_0,",",".");
         string tbxz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,4)); boost::replace_all(tbxz3_1,",",".");
         PostureMod << string("param tol_obs_left_xz3 {i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbxz3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbxz3_1+string("; \n");
         // yz1
         string tbyz1_0 = boost::str(boost::format("%.2f") % tolsObs_0(0,5)); boost::replace_all(tbyz1_0,",",".");
         string tbyz1_1 = boost::str(boost::format("%.2f") % tolsObs_1(0,5)); boost::replace_all(tbyz1_1,",",".");
         PostureMod << string("param tol_obs_left_yz1{i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz1_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz1_1+string("; \n");
         // yz2
         string tbyz2_0 = boost::str(boost::format("%.2f") % tolsObs_0(1,5)); boost::replace_all(tbyz2_0,",",".");
         string tbyz2_1 = boost::str(boost::format("%.2f") % tolsObs_1(1,5)); boost::replace_all(tbyz2_1,",",".");
         PostureMod << string("param tol_obs_left_yz2{i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz2_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz2_1+string("; \n");
         // yz3
         string tbyz3_0 = boost::str(boost::format("%.2f") % tolsObs_0(2,5)); boost::replace_all(tbyz3_0,",",".");
         string tbyz3_1 = boost::str(boost::format("%.2f") % tolsObs_1(2,5)); boost::replace_all(tbyz3_1,",",".");
         PostureMod << string("param tol_obs_left_yz3{i in 1..Nsteps+1} :=  \n");
         PostureMod << string("if 		(i < (Nsteps/2)+1) then ")+tbyz3_0+string("\n");
         PostureMod << string("else if (i > (Nsteps/2)) then ")+tbyz3_1+string("; \n");
         PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
         PostureMod << string("# \n");
         if(obj_left_place){
             // the object to place has to be considered
              PostureMod << string("subject to obst_Arm_left{j in 1..")+n_str_left+string(", i in 1..n_Obstacles_left, l in 1..Nsteps+1}:\n"); // approach stage is necessary
         }else if(move){
             // for the first number of diff_steps, no obstacle is considered because the movement is very short and the planner may get stuck
             int diff_steps = std::max(1,(int)(steps*BLANK_PERCENTAGE_DUAL_OBS));
             string n_steps_init_str = boost::str(boost::format("%d") % (diff_steps));
             PostureMod << string("subject to obst_Arm_left{j in 1..15, i in 1..(n_Obstacles_left), l in ")+n_steps_init_str+("..Nsteps+1}:\n");
         }else{
              PostureMod << string("subject to obst_Arm_left{j in 1..15, i in 1..(n_Obstacles_left), l in 1..Nsteps+1}:\n"); // pick movements
         }

         PostureMod << string("(((Rot_left[1,1,i]*Points_Arm_left[j,1,l]+Rot_left[2,1,i]*Points_Arm_left[j,2,l]+Rot_left[3,1,i]*Points_Arm_left[j,3,l]\n");
         PostureMod << string("-Obstacles_left[i,1]*Rot_left[1,1,i]-Obstacles_left[i,2]*Rot_left[2,1,i]-Obstacles_left[i,3]*Rot_left[3,1,i])\n");
         PostureMod << string("/(Obstacles_left[i,4]+Points_Arm_left[j,4,l]+tol_obs_left_xx1[l]))^2\n");
         PostureMod << string("+\n");
         PostureMod << string("((Rot_left[1,2,i]*Points_Arm_left[j,1,l]+Rot_left[2,2,i]*Points_Arm_left[j,2,l]+Rot_left[3,2,i]*Points_Arm_left[j,3,l]\n");
         PostureMod << string("-Obstacles_left[i,1]*Rot_left[1,2,i]-Obstacles_left[i,2]*Rot_left[2,2,i]-Obstacles_left[i,3]*Rot_left[3,2,i])\n");
         PostureMod << string("/(Obstacles_left[i,5]+Points_Arm_left[j,4,l]+tol_obs_left_yy1[l]))^2\n");
         PostureMod << string("+\n");
         PostureMod << string("((Rot_left[1,3,i]*Points_Arm_left[j,1,l]+Rot_left[2,3,i]*Points_Arm_left[j,2,l]+Rot_left[3,3,i]*Points_Arm_left[j,3,l]\n");
         PostureMod << string("-Obstacles_left[i,1]*Rot_left[1,3,i]-Obstacles_left[i,2]*Rot_left[2,3,i]-Obstacles_left[i,3]*Rot_left[3,3,i])\n");
         PostureMod << string("/(Obstacles_left[i,6]+Points_Arm_left[j,4,l]+tol_obs_left_zz1[l]))^2)\n");
         PostureMod << string(">= 1;\n");
         PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
         PostureMod << string("# \n");

         if((dual_mov_type==0 && pre_post!=2) || (dual_mov_type==1 && pre_post==2)){
             // pick movements (plan and approach stages)
             // OR
             // place movements (retreat stage)
             PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
             PostureMod << string("# \n");
             PostureMod << string("subject to objRight_Arm_left{j in 1..")+n_str_right+string(", i in 1..n_ObjTar_right, l in 1..Nsteps+1}:  \n");
             PostureMod << string("(((Rot_obj_right[1,1]*Points_Arm_left[j,1,l]+Rot_obj_right[2,1]*Points_Arm_left[j,2,l]+Rot_obj_right[3,1]*Points_Arm_left[j,3,l]\n");
             PostureMod << string("-ObjTar_right[i,1]*Rot_obj_right[1,1]-ObjTar_right[i,2]*Rot_obj_right[2,1]-ObjTar_right[i,3]*Rot_obj_right[3,1])\n");
             PostureMod << string("/(ObjTar_right[i,4]+Points_Arm_right[j,4,l]+tol_obs_left_xx1[l]))^2\n");
             PostureMod << string("+\n");
             PostureMod << string("((Rot_obj_right[1,2]*Points_Arm_left[j,1,l]+Rot_obj_right[2,2]*Points_Arm_left[j,2,l]+Rot_obj_right[3,2]*Points_Arm_left[j,3,l]\n");
             PostureMod << string("-ObjTar_right[i,1]*Rot_obj_right[1,2]-ObjTar_right[i,2]*Rot_obj_right[2,2]-ObjTar_right[i,3]*Rot_obj_right[3,2])\n");
             PostureMod << string("/(ObjTar_right[i,5]+Points_Arm_left[j,4,l]+tol_obs_left_yy1[l]))^2\n");
             PostureMod << string("+\n");
             PostureMod << string("((Rot_obj_right[1,3]*Points_Arm_left[j,1,l]+Rot_obj_right[2,3]*Points_Arm_left[j,2,l]+Rot_obj_right[3,3]*Points_Arm_left[j,3,l]\n");
             PostureMod << string("-ObjTar_right[i,1]*Rot_obj_right[1,3]-ObjTar_right[i,2]*Rot_obj_right[2,3]-ObjTar_right[i,3]*Rot_obj_right[3,3])\n");
             PostureMod << string("/(ObjTar_right[i,6]+Points_Arm_left[j,4,l]+tol_obs_left_zz1[l]))^2)\n");
             PostureMod << string(">= 1;\n");
             PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
             PostureMod << string("#  \n");
         }

         /*
         PostureMod << string("((Points_Arm_left[j,1,l]-Obstacles_left[i,1])^2)*(\n");
         PostureMod << string("(Rot_left[1,1,i])^2 / ((Obstacles_left[i,4]+Points_Arm_left[j,4,l]+tol_obs_left_xx1[l])^2) +\n");
         PostureMod << string("(Rot_left[2,1,i])^2 / ((Obstacles_left[i,5]+Points_Arm_left[j,4,l]+tol_obs_left_xx2[l])^2) + \n");
         PostureMod << string("(Rot_left[3,1,i])^2 / ((Obstacles_left[i,6]+Points_Arm_left[j,4,l]+tol_obs_left_xx3[l])^2)) \n");
         PostureMod << string("+ \n");
         PostureMod << string("((Points_Arm_left[j,2,l]-Obstacles_left[i,2])^2)*( \n");
         PostureMod << string("(Rot_left[1,2,i])^2 / ((Obstacles_left[i,4]+Points_Arm_left[j,4,l]+tol_obs_left_yy1[l])^2) + \n");
         PostureMod << string("(Rot_left[2,2,i])^2 / ((Obstacles_left[i,5]+Points_Arm_left[j,4,l]+tol_obs_left_yy2[l])^2) + \n");
         PostureMod << string("(Rot_left[3,2,i])^2 / ((Obstacles_left[i,6]+Points_Arm_left[j,4,l]+tol_obs_left_yy3[l])^2)) \n");
         PostureMod << string("+ \n");
         PostureMod << string("((Points_Arm_left[j,3,l]-Obstacles_left[i,3])^2)*( \n");
         PostureMod << string("(Rot_left[1,3,i])^2 / ((Obstacles_left[i,4]+Points_Arm_left[j,4,l]+tol_obs_left_zz1[l])^2) + \n");
         PostureMod << string("(Rot_left[2,3,i])^2 / ((Obstacles_left[i,5]+Points_Arm_left[j,4,l]+tol_obs_left_zz2[l])^2) + \n");
         PostureMod << string("(Rot_left[3,3,i])^2 / ((Obstacles_left[i,6]+Points_Arm_left[j,4,l]+tol_obs_left_zz3[l])^2)) \n");
         PostureMod << string("+ \n");
         PostureMod << string("2*(Points_Arm_left[j,1,l]-Obstacles_left[i,1])*(Points_Arm_left[j,2,l]-Obstacles_left[i,2])* ( \n");
         PostureMod << string("(Rot_left[1,1,i]*Rot_left[1,2,i])/((Obstacles_left[i,4]+Points_Arm_left[j,4,l]+tol_obs_left_xy1[l])^2) + \n");
         PostureMod << string("(Rot_left[2,1,i]*Rot_left[2,2,i])/((Obstacles_left[i,5]+Points_Arm_left[j,4,l]+tol_obs_left_xy2[l])^2) + \n");
         PostureMod << string("(Rot_left[3,1,i]*Rot_left[3,2,i])/((Obstacles_left[i,6]+Points_Arm_left[j,4,l]+tol_obs_left_xy3[l])^2)) \n");
         PostureMod << string("+ \n");
         PostureMod << string("2*(Points_Arm_left[j,1,l]-Obstacles_left[i,1])*(Points_Arm_left[j,3,l]-Obstacles_left[i,3])* ( \n");
         PostureMod << string("(Rot_left[1,1,i]*Rot_left[1,3,i])/((Obstacles_left[i,4]+Points_Arm_left[j,4,l]+tol_obs_left_xz1[l])^2) + \n");
         PostureMod << string("(Rot_left[2,1,i]*Rot_left[2,3,i])/((Obstacles_left[i,5]+Points_Arm_left[j,4,l]+tol_obs_left_xz2[l])^2) + \n");
         PostureMod << string("(Rot_left[3,1,i]*Rot_left[3,3,i])/((Obstacles_left[i,6]+Points_Arm_left[j,4,l]+tol_obs_left_xz3[l])^2)) \n");
         PostureMod << string("+ \n");
         PostureMod << string("2*(Points_Arm_left[j,2,l]-Obstacles_left[i,2])*(Points_Arm_left[j,3,l]-Obstacles_left[i,3])* ( \n");
         PostureMod << string("(Rot_left[1,2,i]*Rot_left[1,3,i])/((Obstacles_left[i,4]+Points_Arm_left[j,4,l]+tol_obs_left_yz1[l])^2) + \n");
         PostureMod << string("(Rot_left[2,2,i]*Rot_left[2,3,i])/((Obstacles_left[i,5]+Points_Arm_left[j,4,l]+tol_obs_left_yz2[l])^2) + \n");
         PostureMod << string("(Rot_left[3,2,i]*Rot_left[3,3,i])/((Obstacles_left[i,6]+Points_Arm_left[j,4,l]+tol_obs_left_yz3[l])^2)) \n");
         PostureMod << string("-1 >=0; \n");
         PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
         PostureMod << string("# \n");
         */
      }

      // constraints with the body
      if(coll_body){
        this->writeDualBodyConstraints(PostureMod,false,true);
        this->writeDualBodyConstraints(PostureMod,false,false);
      }

      // constraints between the arms
      if(coll_arms){
          PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
          PostureMod << string("# \n");
          PostureMod << string("# Constraints between the arms \n");
          if(dual_mov_type==0 || dual_mov_type==2){ // dual pick or dual move
            PostureMod << string("subject to Arm_Arm{i1 in 4..9, i2 in 4..9,l in 1..Nsteps+1}:  \n");
          }else if(dual_mov_type==1){ // dual place
            PostureMod << string("subject to Arm_Arm{i1 in 4..")+n_str_left+string(", i2 in 4..")+n_str_right+string(",l in 1..Nsteps+1}:  \n");
          }
          PostureMod << string("(Points_Arm_left[i1,1,l] - Points_Arm_right[i2,1,l])^2 + \n");
          PostureMod << string("(Points_Arm_left[i1,2,l] - Points_Arm_right[i2,2,l])^2 + \n");
          PostureMod << string("(Points_Arm_left[i1,3,l] - Points_Arm_right[i2,3,l])^2 - \n");
          PostureMod << string("(Points_Arm_left[i1,4,l] + Points_Arm_right[i2,4,l])^2 >= 0; \n");
          PostureMod << string("# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n");
          PostureMod << string("# \n");
      }
      PostureMod << string("# *+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*# \n\n\n");

      // close the files
      PostureMod.close(); PostureDat.close();

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

int HUMPlanner::getSteps(std::vector<double> &maxLimits, std::vector<double> &minLimits, std::vector<double> &initPosture, std::vector<double> &finalPosture)
{

    int n_steps;
    VectorXd max = VectorXd::Map(maxLimits.data(), maxLimits.size());
    VectorXd min = VectorXd::Map(minLimits.data(), minLimits.size());
    double den = (max-min).norm();
    VectorXd init = VectorXd::Map(initPosture.data(), initPosture.size());
    VectorXd final = VectorXd::Map(finalPosture.data(), finalPosture.size());
    double num = (final-init).norm();

    n_steps = static_cast<int>(0.5 + (N_STEP_MIN+(N_STEP_MAX-N_STEP_MIN)*(num/den)));

    return n_steps;

}

/*
double HUMPlanner::getAlpha(int arm,std::vector<double>& posture)
{    
   double alpha;
   std::vector<double> shPos; this->getShoulderPos(arm,posture,shPos); Vector3d shoulder(shPos.data());
   std::vector<double> elPos; this->getElbowPos(arm,posture,elPos); Vector3d elbow(elPos.data());
   std::vector<double> wrPos; this->getWristPos(arm,posture,wrPos); Vector3d wrist(wrPos.data());
   std::vector<double> haPos; this->getHandPos(arm,posture,haPos); Vector3d hand(haPos.data());


   double Lu = (elbow-shoulder).norm();
   Vector3d v_se = (elbow-shoulder)/Lu;
   Vector3d v_sw = (wrist-shoulder)/((wrist-shoulder).norm());
   Vector3d u(v_sw(1), -v_sw(0), 0); u = u/u.norm(); //u _|_ v_sw
   //Vector3d v = u.cross(v_sw);
   Vector3d v = v_sw.cross(u);
   Vector3d C = shoulder + Lu*(v_sw.dot(v_se))*v_sw;
   Vector3d v_ce = (elbow-C)/((elbow-C).norm());

   if((elbow-C).norm()<0.001){ // [mm]
       // When the arm is completely stretched, the triangle SEW does not exsist.
       // Then, I consider as alpha the angle that the arm creates with the floor
       // floor_v = [1 -1 0]; floor_v = floor_v/ norm(floor_v);
       // alph = sign(d(3))*acos(d*floor_v');
       //Vector3d floor_v_prep(0,0,1);
       //alpha = std::asin(d.dot(floor_v_prep.transpose())); // it is negative if the arm is stretched down
       alpha = 0;
   }else{
      alpha=std::atan2(v_ce.dot(v),v_ce.dot(u));
   }

   return alpha;
}
*/

/*
int HUMPlanner::invKinematics(int arm, std::vector<double> &pose, double alpha, std::vector<double> &init_posture, std::vector<double> &posture)
{
    MatrixXd solutions(7,4);
    VectorXd solution_1(7); VectorXd solution_2(7); VectorXd solution_3(7); VectorXd solution_4(7);
    Matrix3d Rot_W_0;
    double Lu; double Ll; double Lh;
    double alpha_0; double alpha_1; double alpha_2; double alpha_3;
    std::vector<double> minLimits; std::vector<double> maxLimits;
    switch (arm) {
    case 0: // dual arm
        // TO DO
        break;
    case 1: // right arm
        Lu = this->DH_rightArm.d.at(2);
        Ll = this->DH_rightArm.d.at(4);
        Lh = this->DH_rightArm.d.at(6);
        alpha_0 = this->DH_rightArm.alpha.at(0);
        alpha_1 = this->DH_rightArm.alpha.at(1);
        alpha_2 = this->DH_rightArm.alpha.at(2);
        alpha_3 = this->DH_rightArm.alpha.at(3);
        minLimits = this->minRightLimits;
        maxLimits = this->maxRightLimits;
        Rot_W_0 = this->matWorldToRightArm.block<3,3>(0,0);
        break;
    case 2: // left arm
        Lu = this->DH_leftArm.d.at(2);
        Ll = this->DH_leftArm.d.at(4);
        Lh = this->DH_leftArm.d.at(6);
        alpha_0 = this->DH_leftArm.alpha.at(0);
        alpha_1 = this->DH_leftArm.alpha.at(1);
        alpha_2 = this->DH_leftArm.alpha.at(2);
        alpha_3 = this->DH_leftArm.alpha.at(3);
        minLimits = this->minLeftLimits;
        maxLimits = this->maxLeftLimits;
        Rot_W_0 = this->matWorldToLeftArm.block<3,3>(0,0);
        break;
    }
    // check if the target is inside the workspace
    double max_ext = Lh+Ll+Lu;
    std::vector<double> shPos; this->getShoulderPos(arm,init_posture,shPos);
    if(sqrt(pow(pose.at(0) - shPos.at(0),2)+
            pow(pose.at(1) - shPos.at(1),2)+
            pow(pose.at(2) - shPos.at(2),2))>= max_ext){
        return -1;
    }
    Vector3d pos_shoulder(shPos.data());
    Vector3d pos_hand(pose.at(0),pose.at(1),pose.at(2));
    std::vector<double> rpy_hand = {pose.at(3),pose.at(4),pose.at(5)};
    Matrix3d Rot_hand; this->RPY_matrix(rpy_hand,Rot_hand);
    Vector3d z_hand = Rot_hand.block<3,1>(0,2);
    Vector3d pos_wrist = pos_hand - z_hand*Lh;

    // joint of the elbow
    double theta_3 = -std::acos((-pow(Lu,2)-pow(Ll,2)+pow((pos_shoulder-pos_wrist).norm(),2))/(2*Lu*Ll));
    if((theta_3 > maxLimits.at(3))||(theta_3 < minLimits.at(3))){
        return -2;
    }
    // elbow position and orientation
    double L_sw = (pos_wrist-pos_shoulder).norm();
    Vector3d v_sw = (pos_wrist-pos_shoulder)/L_sw;
    Vector3d u(v_sw(1), -v_sw(0), 0); u = u/u.norm(); //u _|_ v_sw
    //Vector3d v = u.cross(v_sw);
    Vector3d v = v_sw.cross(u);
    double cos_beta = (pow(L_sw,2)+pow(Lu,2)-pow(Ll,2))/(2*L_sw*Lu);
    Vector3d C = pos_shoulder + cos_beta*Lu*v_sw;
    double R = Lu*sqrt(1-pow(cos_beta,2));
    Vector3d pos_elbow = C + R*(u*cos(alpha)+v*sin(alpha));
    Vector3d v_ew = (pos_wrist - pos_elbow)/(pos_wrist - pos_elbow).norm();
    Vector3d v_es = (pos_shoulder - pos_elbow)/(pos_shoulder - pos_elbow).norm();
    Vector3d v_ce = (pos_elbow - C)/(pos_elbow - C).norm();

    Vector3d x_el; Vector3d y_el; Vector3d z_el;

    //x_el = v_es;
    //y_el = (v_sw - v_sw.dot(x_el)*x_el)/(v_sw - v_sw.dot(x_el)*x_el).norm();
    //z_el = x_el.cross(y_el);
    //if((pos_elbow - C).norm()<0.001){
      //  z_el << 0, 0, -1;
    //}else{
        z_el = v_sw.cross(v_ce);
    //}
    y_el = v_ew;
    x_el = y_el.cross(z_el);

    Matrix3d Rot_el;
    Rot_el.block<3,1>(0,0) = x_el; Rot_el.block<3,1>(0,1) =y_el; Rot_el.block<3,1>(0,2) = z_el;

    // compute theta0 theta1 theta2
    Matrix3d Rot_3_4; this->RotMatrix(theta_3,alpha_3,Rot_3_4);
    Matrix3d Rot_W_3 = Rot_el*(Rot_3_4.transpose());

    double theta_0; double theta_1; double theta_2;
    theta_1 = std::atan2(sqrt(1-pow(Rot_W_3(1,2),2)),Rot_W_3(1,2));
    if(abs(theta_1)<0.001){ // theta_1 = 0
        theta_0=0;
        //if(abs(theta_3)<0.001){
          //  theta_2=0;
        //}else{
        theta_2=std::atan2(Rot_W_3(2,0),Rot_W_3(2,1));
        //}
        Matrix3d Rot_0_1; this->RotMatrix(theta_0,alpha_0,Rot_0_1);
        Matrix3d Rot_1_2; this->RotMatrix(theta_1,alpha_1,Rot_1_2);
        Matrix3d Rot_2_3; this->RotMatrix(theta_2,alpha_2,Rot_2_3);
        Matrix3d Rot_W_4 = Rot_W_0*Rot_0_1*Rot_1_2*Rot_2_3*Rot_3_4;
        // compute theta4 theta5 theta6
        double theta_4; double theta_5; double theta_6;
        Matrix3d Rot_4_7 = (Rot_W_4.transpose())*Rot_hand;
        // 1st solution
        theta_5 = std::atan2(sqrt(1-pow(Rot_4_7(1,2),2)),Rot_4_7(1,2));
        if(abs(theta_5)<0.001){
            theta_4 = 0;
            theta_6 = std::atan2(Rot_4_7(0,1),-Rot_4_7(2,1));
        }else{
            theta_4 = std::atan2(Rot_4_7(2,2)/sin(theta_5),-Rot_4_7(0,2)/sin(theta_5));
            theta_6 = std::atan2(Rot_4_7(1,1)/sin(theta_5),Rot_4_7(1,0)/sin(theta_5));
        }
        solution_1 << theta_0,theta_1,theta_2,theta_3,theta_4,theta_5,theta_6;
        // 2nd solution
        theta_5 = - theta_5;
        if (abs(theta_5) < 0.001){
            theta_4 = 0;
            theta_6 = std::atan2(Rot_4_7(0,1),-Rot_4_7(2,1));
        }else{
            theta_4 = std::atan2(Rot_4_7(2,2)/sin(theta_5),-Rot_4_7(0,2)/sin(theta_5));
            theta_6 = std::atan2(Rot_4_7(1,1)/sin(theta_5),Rot_4_7(1,0)/sin(theta_5));
        }
        solution_2 << theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6;
    }else{ // theta_1 != 0
        theta_0 = std::atan2(-Rot_W_3(2,2)/sin(theta_1),Rot_W_3(0,2)/sin(theta_1));
        theta_2 = std::atan2(Rot_W_3(1,1)/sin(theta_1),Rot_W_3(1,0)/sin(theta_1));

        Matrix3d Rot_0_1; this->RotMatrix(theta_0,alpha_0,Rot_0_1);
        Matrix3d Rot_1_2; this->RotMatrix(theta_1,alpha_1,Rot_1_2);
        Matrix3d Rot_2_3; this->RotMatrix(theta_2,alpha_2,Rot_2_3);
        Matrix3d Rot_W_4 = Rot_W_0*Rot_0_1*Rot_1_2*Rot_2_3*Rot_3_4;
        // compute theta4 theta5 theta6
        double theta_4; double theta_5; double theta_6;
        Matrix3d Rot_4_7 = (Rot_W_4.transpose())*Rot_hand;
        // 1st solution
        theta_5 = std::atan2(sqrt(1-pow(Rot_4_7(1,2),2)),Rot_4_7(1,2));
        if (abs(theta_5) < 0.001){
            theta_4 = 0;
            theta_6 = std::atan2(Rot_4_7(0,1),-Rot_4_7(2,1));
        }else{
            theta_4 = std::atan2(Rot_4_7(2,2)/sin(theta_5),-Rot_4_7(0,2)/sin(theta_5));
            theta_6 = std::atan2(Rot_4_7(1,1)/sin(theta_5),Rot_4_7(1,0)/sin(theta_5));
        }
        solution_1 << theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6;
        // 2nd solution
        theta_5 =  -theta_5;
        if (abs(theta_5) < 0.001){
            theta_4 = 0;
            theta_6 = std::atan2(Rot_4_7(0,1),-Rot_4_7(2,1));
        }else{
            theta_4 = std::atan2(Rot_4_7(2,2)/sin(theta_5),-Rot_4_7(0,2)/sin(theta_5));
            theta_6 = std::atan2(Rot_4_7(1,1)/sin(theta_5),Rot_4_7(1,0)/sin(theta_5));
        }
        solution_2 << theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6;
    }// theta_1

    theta_1 = -theta_1;
    if(abs(theta_1)<0.001){ // theta_1 =0
        theta_0 = 0;
        //if(abs(theta_3)<0.001){
           // theta_2 = 0;
        //}else{
        theta_2 = std::atan2(Rot_W_3(2,0),Rot_W_3(2,1));
        //}
        Matrix3d Rot_0_1; this->RotMatrix(theta_0,alpha_0,Rot_0_1);
        Matrix3d Rot_1_2; this->RotMatrix(theta_1,alpha_1,Rot_1_2);
        Matrix3d Rot_2_3; this->RotMatrix(theta_2,alpha_2,Rot_2_3);
        Matrix3d Rot_W_4 = Rot_W_0*Rot_0_1*Rot_1_2*Rot_2_3*Rot_3_4;
        // compute theta4 theta5 theta6
        double theta_4; double theta_5; double theta_6;
        Matrix3d Rot_4_7 = (Rot_W_4.transpose())*Rot_hand;
        // 3rd solution
        theta_5 = std::atan2(sqrt(1-pow(Rot_4_7(1,2),2)),-Rot_4_7(1,2));
        if(abs(theta_5)<0.001){
            theta_4 = 0;
            theta_6 = std::atan2(Rot_4_7(0,1),-Rot_4_7(2,1));
        }else{
            theta_4 = std::atan2(Rot_4_7(2,2)/sin(theta_5),-Rot_4_7(0,2)/sin(theta_5));
            theta_6 = std::atan2(Rot_4_7(1,1)/sin(theta_5),Rot_4_7(1,0)/sin(theta_5));
        }
        solution_3 << theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6;
        // 4th solution
        theta_5 = -theta_5;
        if(abs(theta_5)<0.001){
            theta_4 = 0;
            theta_6 = std::atan2(Rot_4_7(0,1),-Rot_4_7(2,1));
        }else{
            theta_4 = std::atan2(Rot_4_7(2,2)/sin(theta_5),-Rot_4_7(0,2)/sin(theta_5));
            theta_6 = std::atan2(Rot_4_7(1,1)/sin(theta_5),Rot_4_7(1,0)/sin(theta_5));
        }
        solution_4 << theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6;
    }else{// theta_1 !=0
        theta_0 = std::atan2(-Rot_W_3(2,2)/sin(theta_1),Rot_W_3(0,2)/sin(theta_1));
        theta_2 = std::atan2(Rot_W_3(1,1)/sin(theta_1),Rot_W_3(1,0)/sin(theta_1));

        Matrix3d Rot_0_1; this->RotMatrix(theta_0,alpha_0,Rot_0_1);
        Matrix3d Rot_1_2; this->RotMatrix(theta_1,alpha_1,Rot_1_2);
        Matrix3d Rot_2_3; this->RotMatrix(theta_2,alpha_2,Rot_2_3);
        Matrix3d Rot_W_4 = Rot_W_0*Rot_0_1*Rot_1_2*Rot_2_3*Rot_3_4;
        // compute theta4 theta5 theta6
        double theta_4; double theta_5; double theta_6;
        Matrix3d Rot_4_7 = (Rot_W_4.transpose())*Rot_hand;
        // 3rd solution
        theta_5 = std::atan2(sqrt(1-pow(Rot_4_7(1,2),2)),Rot_4_7(1,2));
        if(abs(theta_5)<0.001){
            theta_4 = 0;
            theta_6 = std::atan2(Rot_4_7(0,1),-Rot_4_7(2,1));
        }else{
            theta_4 = std::atan2(Rot_4_7(2,2)/sin(theta_5),-Rot_4_7(0,2)/sin(theta_5));
            theta_6 = std::atan2(Rot_4_7(1,1)/sin(theta_5),Rot_4_7(1,0)/sin(theta_5));
        }
        solution_3 << theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6;
        // 4th solution
        theta_5 = -theta_5;
        if(abs(theta_5)<0.001){
            theta_4 = 0;
            theta_6 = std::atan2(Rot_4_7(1,1),-Rot_4_7(1,0));
        }else{
            theta_4 = std::atan2(Rot_4_7(2,2)/sin(theta_5),-Rot_4_7(0,2)/sin(theta_5));
            theta_6 = std::atan2(Rot_4_7(1,1)/sin(theta_5),Rot_4_7(1,0)/sin(theta_5));
        }
        solution_4 << theta_0, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6;
    }// theta_1

    // check the solutions
    solutions.block<7,1>(0,0) = solution_1;
    solutions.block<7,1>(0,1) = solution_2;
    solutions.block<7,1>(0,2) = solution_3;
    solutions.block<7,1>(0,3) = solution_4;
    std::vector<double> costs;
    for(int i=0; i < solutions.cols();++i){
        VectorXd sol = solutions.block<7,1>(0,i);
        bool good_sol = true;
        double cost = 0;
        for(int j = 0; j<sol.size();++j){
            if((sol(j)>maxLimits.at(j))||(sol(j)<minLimits.at(j))){
                good_sol = false;
                break;
            }else{
                good_sol = true;
            }
            cost = cost + abs(sol(j)-init_posture.at(j));
        }
        if(good_sol){
           costs.push_back(cost);
        }else{
            costs.push_back(-1);
        }
    }
    double min = 1000; int index = -1;
    for(size_t i=0; i<costs.size(); ++i){
        double c = costs.at(i);
        if(c>=0){
            if(c<min){
               min=c;
               index=i;
            }
        }
    }
    if(index<0){
        return -3;
    }

    VectorXd sol_posture = solutions.col(index);
    posture.resize(sol_posture.size());
    VectorXd::Map(&posture[0], sol_posture.size()) = sol_posture;

    return 0;
}
*/

void HUMPlanner::RotMatrix(double theta, double alpha, Matrix3d &Rot)
{
    Rot(0,0) = cos(theta);              Rot(0,1) = -sin(theta);              Rot(0,2) = 0.0;
    Rot(1,0) = sin(theta)*cos(alpha);   Rot(1,1) = -cos(theta)*cos(alpha);   Rot(1,2) = -sin(alpha);
    Rot(2,0) = sin(theta)*sin(alpha);   Rot(2,1) = cos(theta)*sin(alpha);    Rot(2,2) = cos(alpha);
}

void HUMPlanner::transfMatrix(double alpha, double a, double d, double theta, Matrix4d &T)
{
    T = Matrix4d::Zero();

    T(0,0) = cos(theta);            T(0,1) = -sin(theta);            T(0,2) = 0.0;         T(0,3) = a;
    T(1,0) = sin(theta)*cos(alpha); T(1,1) = -cos(theta)*cos(alpha); T(1,2) = -sin(alpha); T(1,3) = -sin(alpha)*d;
    T(2,0) = sin(theta)*sin(alpha); T(2,1) = cos(theta)*sin(alpha);  T(2,2) = cos(alpha);  T(2,3) = cos(alpha)*d;
    T(3,0) = 0.0;                   T(3,1) = 0.0;                    T(3,2) = 0.0;         T(3,3) = 1.0;

}

bool HUMPlanner::getRPY(std::vector<double>& rpy, Matrix3d& Rot)
{
    if((Rot.cols()==3) && (Rot.rows()==3))
    {// the matrix is not empy
        rpy.resize(3,0);
        if((Rot(0,0)<1e-10) && (Rot(1,0)<1e-10))
        {// singularity
            rpy.at(0) = 0; // roll
            rpy.at(1) = std::atan2(-Rot(2,0),Rot(0,0)); // pitch
            rpy.at(2) = std::atan2(-Rot(1,2),Rot(1,1)); // yaw
            return false;
        }else{
            rpy.at(0) = std::atan2(Rot(1,0),Rot(0,0)); // roll
            double sp = std::sin(rpy.at(0)); double cp = std::cos(rpy.at(0));
            rpy.at(1) = std::atan2(-Rot(2,0),cp*Rot(0,0)+sp*Rot(1,0)); // pitch
            rpy.at(2) = std::atan2(sp*Rot(0,2)-cp*Rot(1,2),cp*Rot(1,1)-sp*Rot(0,1)); // yaw
            return true;
        }
    }else{
        return false;
    }
}

void HUMPlanner::directKinematicsSingleArm(int arm, std::vector<double>& posture)
{


    Matrix4d T;
    Matrix4d T_aux;
    Matrix4d mat_world;
    Matrix4d mat_hand;
    DHparameters m_DH_arm;
    this->shPose.clear(); this->elPose.clear(); this->wrPose.clear(); this->haPose.clear();

    vector<double> shoulderPos = vector<double>(3);
    Matrix3d shoulderOr;
    vector<double> elbowPos = vector<double>(3);
    Matrix3d elbowOr;
    vector<double> wristPos = vector<double>(3);
    Matrix3d wristOr;
    vector<double> handPos = vector<double>(3);
    Matrix3d handOr;

    switch (arm) {
    case 1: // right arm
        mat_world = this->matWorldToRightArm;
        mat_hand = this->matRightHand;
        m_DH_arm = this->DH_rightArm;
        break;
    case 2: //left arm
        mat_world = this->matWorldToLeftArm;
        mat_hand = this->matLeftHand;
        m_DH_arm = this->DH_leftArm;
        break;
    }

    T = mat_world;

    for (size_t i = 0; i < posture.size(); ++i){
        this->transfMatrix(m_DH_arm.alpha.at(i),m_DH_arm.a.at(i),m_DH_arm.d.at(i), posture.at(i),T_aux);
        T = T * T_aux;
        Vector3d v;

        if (i==0){
            // get the shoulder

            shoulderOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            //position
            shoulderPos[0] = v[0];
            shoulderPos[1] = v[1];
            shoulderPos[2] = v[2];
            this->shPose.push_back(shoulderPos[0]);
            this->shPose.push_back(shoulderPos[1]);
            this->shPose.push_back(shoulderPos[2]);
            //orientation
            std::vector<double> rpy; this->getRPY(rpy,shoulderOr);
            this->shPose.push_back(rpy[0]);
            this->shPose.push_back(rpy[1]);
            this->shPose.push_back(rpy[2]);

        }else if (i==2){

            // get the elbow
            elbowOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            //position
            elbowPos[0] = v[0];
            elbowPos[1] = v[1];
            elbowPos[2] = v[2];
            this->elPose.push_back(elbowPos[0]);
            this->elPose.push_back(elbowPos[1]);
            this->elPose.push_back(elbowPos[2]);
            //orientation
            std::vector<double> rpy; this->getRPY(rpy,elbowOr);
            this->elPose.push_back(rpy[0]);
            this->elPose.push_back(rpy[1]);
            this->elPose.push_back(rpy[2]);

        }else if (i==4){

            // get the wrist
            wristOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            // position
            wristPos[0] = v[0];
            wristPos[1] = v[1];
            wristPos[2] = v[2];
            this->wrPose.push_back(wristPos[0]);
            this->wrPose.push_back(wristPos[1]);
            this->wrPose.push_back(wristPos[2]);
            //orientation
            std::vector<double> rpy; this->getRPY(rpy,wristOr);
            this->wrPose.push_back(rpy[0]);
            this->wrPose.push_back(rpy[1]);
            this->wrPose.push_back(rpy[2]);


        } else if (i==6){

            //get the hand
            T = T * mat_hand;

            handOr = T.block(0,0,3,3);
            v = T.block(0,3,3,1);
            // position
            handPos[0] = v[0];
            handPos[1] = v[1];
            handPos[2] = v[2];
            this->haPose.push_back(handPos[0]);
            this->haPose.push_back(handPos[1]);
            this->haPose.push_back(handPos[2]);
            //orientation
            std::vector<double> rpy; this->getRPY(rpy,handOr);
            this->haPose.push_back(rpy[0]);
            this->haPose.push_back(rpy[1]);
            this->haPose.push_back(rpy[2]);

        }

    }

}

void HUMPlanner::getShoulderPos(int arm, vector<double> &posture, vector<double> &pos)
{
    std::vector<double> aux_posture(posture.begin(),posture.begin()+joints_arm);
    this->directKinematicsSingleArm(arm,aux_posture);
    pos = { this->shPose.at(0), this->shPose.at(1), this->shPose.at(2)};
}

void HUMPlanner::getShoulderOr(int arm, vector<double> &posture, vector<double> &orient)
{
    std::vector<double> aux_posture(posture.begin(),posture.begin()+joints_arm);
    this->directKinematicsSingleArm(arm,aux_posture);
    orient = { this->shPose.at(3), this->shPose.at(4), this->shPose.at(5)};
}

void HUMPlanner::getWristPos(int arm, vector<double> &posture, vector<double> &pos)
{
    std::vector<double> aux_posture(posture.begin(),posture.begin()+joints_arm);
    this->directKinematicsSingleArm(arm,aux_posture);
    pos = { this->wrPose.at(0), this->wrPose.at(1), this->wrPose.at(2)};
}

void HUMPlanner::getWristOr(int arm, vector<double> &posture, vector<double> &orient)
{
    std::vector<double> aux_posture(posture.begin(),posture.begin()+joints_arm);
    this->directKinematicsSingleArm(arm,aux_posture);
    orient = { this->wrPose.at(3), this->wrPose.at(4), this->wrPose.at(5)};
}

void HUMPlanner::getElbowPos(int arm, vector<double> &posture, vector<double> &pos)
{
    std::vector<double> aux_posture(posture.begin(),posture.begin()+joints_arm);
    this->directKinematicsSingleArm(arm,aux_posture);
    pos = { this->elPose.at(0), this->elPose.at(1), this->elPose.at(2)};
}

void HUMPlanner::getElbowOr(int arm, vector<double> &posture, vector<double> &orient)
{
    std::vector<double> aux_posture(posture.begin(),posture.begin()+joints_arm);
    this->directKinematicsSingleArm(arm,aux_posture);
    orient = { this->elPose.at(3), this->elPose.at(4), this->elPose.at(5)};
}

void HUMPlanner::getHandPos(int arm, vector<double> &posture, vector<double> &pos)
{
    std::vector<double> aux_posture(posture.begin(),posture.begin()+joints_arm);
    this->directKinematicsSingleArm(arm,aux_posture);
    pos = { this->haPose.at(0), this->haPose.at(1), this->haPose.at(2)};
}

void HUMPlanner::getHandOr(int arm, vector<double> &posture, vector<double> &orient)
{
    std::vector<double> aux_posture(posture.begin(),posture.begin()+joints_arm);
    this->directKinematicsSingleArm(arm,aux_posture);
    orient = { this->haPose.at(3), this->haPose.at(4), this->haPose.at(5)};
}

/*
bool HUMPlanner::singleArmInvKinematics(hump_params &params, std::vector<double> &init_posture, std::vector<double>& hand_pose, std::vector<double> &goal_posture)
{
    int arm = params.mov_specs.arm_code;
    double swivel_angle = this->getAlpha(arm,init_posture);
    int success = this->invKinematics(arm,hand_pose,swivel_angle,init_posture,goal_posture);

    return (success==0);

}
*/

} // namespace HUMotion
