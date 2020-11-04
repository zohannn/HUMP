#include <gtest/gtest.h>

#include "../../include/object.hpp"

using namespace HUMotion;

TEST(Norm, Norm_of_a_vector_Test)
{
    vector<double> poss{3.0,2.0,3.0};

    Object* obj = new Object();
    obj->setPos(poss);
    vector<double> gposs;	
    obj->getPos(gposs);
    EXPECT_TRUE((3.0-gposs.at(0)) <= 0.01);

}


int main(int argc, char* argv[])
{

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
