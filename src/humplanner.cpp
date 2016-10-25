#include "../include/humplanner.hpp"


namespace HUMotion {


HUMPlanner::HUMPlanner(string name = string ("Default Planner")):
    scene(nullptr),task(nullptr)
{

    this->name = name;
}



HUMPlanner::HUMPlanner(const HUMPlanner &hp)
{

    this->name=hp.name;
}

HUMPlanner::~HUMPlanner()
{

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

void HUMPlanner::writeArmDHParams(std::vector<double> alpha, std::vector<double> a, std::vector<double> d, ofstream &stream, int k)
{
    // D-H Parameters of the Arm
    stream << string("# D-H PARAMETERS OF THE ARM \n");
    stream << string("param alpha := \n");
    string alpha_str;
    alpha_str =  boost::str(boost::format("%.2f") % (alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_str+string("\n");
    for(std::size_t i=1; i< joints_arm; ++i){
        alpha_str =  boost::str(boost::format("%.2f") % (k*alpha.at(i)));

        if (i == alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_str+string("\n");
        }
    }
    stream << string("param a := \n");
    string a_str;
    for(std::size_t i=0; i<joints_arm; ++i){
        a_str =  boost::str(boost::format("%.2f") % (a.at(i)));
        if (i == a.size()-1){
            stream << to_string(i+1)+string(" ")+a_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_str+string("\n");
        }
    }
    stream << string("param d := \n");
    string d_str;
    for(std::size_t i=0; i<joints_arm; ++i){
        d_str =  boost::str(boost::format("%.2f") % (k*d.at(i)));
        if (i == d.size()-1){
            stream << to_string(i+1)+string(" ")+d_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_str+string("\n");
        }
    }
}

void HUMPlanner::write_dHO(std::ofstream& stream, float dHO)
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

void HUMPlanner::writeHumanHandParams(human_finger index, human_finger ring, human_finger thumb, std::ofstream& stream, int k)
{
    stream << string("# PARAMETERS OF THE HAND \n");

    // Index finger
    stream << string("# Index Finger \n");

    stream << string("param u1x := ");
    string ux_fing1_str =  boost::str(boost::format("%.2f") % (index.ux));
    stream << ux_fing1_str+string(";\n");

    stream << string("param u1y := ");
    string uy_fing1_str =  boost::str(boost::format("%.2f") % (k*index.uy));
    stream << uy_fing1_str+string(";\n");

    stream << string("param u1z := ");
    string uz_fing1_str =  boost::str(boost::format("%.2f") % (index.uz));
    stream << uz_fing1_str+string(";\n");

    stream << string("param alpha_fing1 := \n");
    string alpha_fing1_str;
    alpha_fing1_str =  boost::str(boost::format("%.2f") % (k*index.alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_fing1_str+string("\n");
    for(std::size_t i=1; i<n_phalange+1; ++i){
        alpha_fing1_str =  boost::str(boost::format("%.2f") % (index.alpha.at(i)));
        if (i == index.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_fing1_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_fing1_str+string("\n");
        }
    }
    stream << string("param a_fing1 := \n");
    string a_fing1_str;
    for(std::size_t i=0; i<n_phalange+1; ++i){
        a_fing1_str =  boost::str(boost::format("%.2f") % (index.a.at(i)));
        if (i == index.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_fing1_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_fing1_str+string("\n");
        }
    }
    stream << string("param d_fing1 := \n");
    string d_fing1_str;
    for(std::size_t i=0; i<n_phalange+1; ++i){
        d_fing1_str =  boost::str(boost::format("%.2f") % (index.d.at(i)));
        if (i == index.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_fing1_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_fing1_str+string("\n");
        }
    }
    stream << string("param theta0_fing1 := ");
    string theta0_fing1_str;
    theta0_fing1_str =  boost::str(boost::format("%.2f") % (k*index.theta.at(0)));
    stream << theta0_fing1_str+string(";\n");

    // Ring finger
    stream << string("# Ring Finger \n");

    stream << string("param u3x := ");
    string ux_fing3_str =  boost::str(boost::format("%.2f") % (ring.ux));
    stream << ux_fing3_str+string(";\n");

    stream << string("param u3y := ");
    string uy_fing3_str =  boost::str(boost::format("%.2f") % (k*ring.uy));
    stream << uy_fing3_str+string(";\n");

    stream << string("param u3z := ");
    string uz_fing3_str =  boost::str(boost::format("%.2f") % (ring.uz));
    stream << uz_fing3_str+string(";\n");

    stream << string("param alpha_fing3 := \n");
    string alpha_fing3_str;
    alpha_fing3_str =  boost::str(boost::format("%.2f") % (k*ring.alpha.at(0)));
    stream << to_string(1)+string(" ")+alpha_fing3_str+string("\n");
    for(std::size_t i=1; i<n_phalange+1; ++i){
        alpha_fing3_str =  boost::str(boost::format("%.2f") % (ring.alpha.at(i)));
        if (i == ring.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_fing3_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_fing3_str+string("\n");
        }
    }
    stream << string("param a_fing3 := \n");
    string a_fing3_str;
    for(std::size_t i=0; i<n_phalange+1; ++i){
        a_fing3_str =  boost::str(boost::format("%.2f") % (ring.a.at(i)));
        if (i == ring.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_fing3_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_fing3_str+string("\n");
        }
    }
    stream << string("param d_fing3 := \n");
    string d_fing3_str;
    for(std::size_t i=0; i<n_phalange+1; ++i){
        d_fing3_str =  boost::str(boost::format("%.2f") % (ring.d.at(i)));
        if (i == ring.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_fing3_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_fing3_str+string("\n");
        }
    }
    stream << string("param theta0_fing3 := ");
    string theta0_fing3_str;
    theta0_fing3_str =  boost::str(boost::format("%.2f") % (k*ring.theta.at(0)));
    stream << theta0_fing3_str+string(";\n");

    // Thumb finger
    stream << string("# Thumb Finger \n");

    stream << string("param uTx := ");
    string uTx_fing2_str =  boost::str(boost::format("%.2f") % (thumb.ux));
    stream << uTx_fing2_str+string(";\n");

    stream << string("param uTy := ");
    string uTy_fing2_str =  boost::str(boost::format("%.2f") % (k*thumb.uy));
    stream << uTy_fing2_str+string(";\n");

    stream << string("param uTz := ");
    string uTz_fing2_str =  boost::str(boost::format("%.2f") % (thumb.uz));
    stream << uTz_fing2_str+string(";\n");

    stream << string("param alpha_thumb := \n");
    string alpha_thumb_str;
    if (k == 1){ // right hand
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb.alpha.at(0)));
    }else if(k==-1){// left hand
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb.alpha.at(0)-M_PI/2));
    }
    stream << to_string(1)+string(" ")+alpha_thumb_str+string("\n");
    for(std::size_t i=1; i<n_phalange+2; ++i){
        alpha_thumb_str =  boost::str(boost::format("%.2f") % (thumb.alpha.at(i)));
        if (i == thumb.alpha.size()-1){
            stream << to_string(i+1)+string(" ")+alpha_thumb_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+alpha_thumb_str+string("\n");
        }
    }

    stream << string("param a_thumb := \n");
    string a_thumb_str;
    for(std::size_t i=0; i<n_phalange+2; ++i){
        a_thumb_str =  boost::str(boost::format("%.2f") % (thumb.a.at(i)));
        if (i == thumb.a.size()-1){
            stream << to_string(i+1)+string(" ")+a_thumb_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+a_thumb_str+string("\n");
        }
    }

    stream << string("param d_thumb := \n");
    string d_thumb_str;
    for(std::size_t i=0; i<n_phalange+2; ++i){
        d_thumb_str =  boost::str(boost::format("%.2f") % (thumb.d.at(i)));
        if (i == thumb.d.size()-1){
            stream << to_string(i+1)+string(" ")+d_thumb_str+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+d_thumb_str+string("\n");
        }
    }

    stream << string("param theta0_thumb:= ");
    string theta0_thumb_str;
    theta0_thumb_str =  boost::str(boost::format("%.2f") % (thumb.theta.at(0)));
    stream << theta0_thumb_str+string(";\n");
}

void HUMPlanner::writeBarrettHandParams(double Aw, double A1, double A2, double A3,double D3, double phi2, double phi3,
                            std::vector<double> rk, std::vector<double> jk, std::ofstream& stream)

{


    // rk and jk parameters
    stream << string("# R and J parameters \n");
    stream << string("param rk := \n");

    for (size_t i=0; i < rk.size(); ++i){
        string rkstr =  boost::str(boost::format("%.2f") % (rk.at(i)));
        boost::replace_all(rkstr,",",".");
        if (i == rk.size()-1){
            stream << to_string(i+1)+string(" ")+rkstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+rkstr+string("\n");
        }
    }

    stream << string("param jk := \n");

    for (size_t i=0; i < jk.size(); ++i){
        string jkstr =  boost::str(boost::format("%.2f") % (jk.at(i)));
        boost::replace_all(jkstr,",",".");
        if (i == jk.size()-1){
            stream << to_string(i+1)+string(" ")+jkstr+string(";\n");
        }else{
            stream << to_string(i+1)+string(" ")+jkstr+string("\n");
        }

    }


    string Awstr =  boost::str(boost::format("%.2f") % (Aw));
    boost::replace_all(Awstr,",",".");
    stream << string("param Aw :=")+Awstr+string(";\n");

    string A1str =  boost::str(boost::format("%.2f") % (A1));
    boost::replace_all(A1str,",",".");
    stream << string("param A1 :=")+A1str+string(";\n");

    string A2str =  boost::str(boost::format("%.2f") % (A2));
    boost::replace_all(A2str,",",".");
    stream << string("param A2 :=")+A2str+string(";\n");

    string A3str =  boost::str(boost::format("%.2f") % (A3));
    boost::replace_all(A3str,",",".");
    stream << string("param A3 :=")+A3str+string(";\n");

    string D3str =  boost::str(boost::format("%.2f") % (D3));
    boost::replace_all(D3str,",",".");
    stream << string("param D3 :=")+D3str+string(";\n");

    string phi2str =  boost::str(boost::format("%.2f") % (phi2));
    boost::replace_all(phi2str,",",".");
    stream << string("param phi_2 :=")+phi2str+string(";\n");

    string phi3str =  boost::str(boost::format("%.2f") % (phi3));
    boost::replace_all(phi3str,",",".");
    stream << string("param phi_3 :=")+phi3str+string(";\n");

}

void HUMPlanner::writeInfoTarget(ofstream &stream, std::vector<double> pos, std::vector<double> xt, std::vector<double> yt, std::vector<double> zt)
{
    stream << string("# TARGET POSITION \n");
    stream << string("param Tar_pos := \n");

    string tarx =  boost::str(boost::format("%.2f") % (pos.at(0)));
    boost::replace_all(tarx,",",".");
    stream << to_string(1)+string(" ")+tarx+string("\n");

    string tary =  boost::str(boost::format("%.2f") % (pos.at(1));
    boost::replace_all(tary,",",".");
    stream << to_string(2)+string(" ")+tary+string("\n");

    string tarz =  boost::str(boost::format("%.2f") % (pos.at(2)));
    boost::replace_all(tarz,",",".");
    stream << to_string(3)+string(" ")+tarz+string(";\n");

    stream << string("# TARGET ORIENTATION \n");

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

} // namespace HUMotion
