#include <gtest/gtest.h>

#include "../../include/point.hpp"

using namespace HUMotion;

TEST(Norm, Norm_of_a_vector_Test)
{
    pos poss;
    poss.Xpos = 3; poss.Ypos = 2; poss.Zpos = 3;

    Point* pt = new Point();
    pt->setPos(poss);
    EXPECT_TRUE((4.69-pt->getNorm()) <= 0.01);

}


int main(int argc, char* argv[])
{

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
