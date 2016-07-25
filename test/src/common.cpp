#include "../include/common.hpp"

using namespace CommonFunctions;
using namespace HUMotion;

TEST(Norm, Norm_of_a_vector_Test)
{
    pos poss;
    poss.Xpos = 3; poss.Ypos = 2; poss.Zpos = 3;
    EXPECT_TRUE((4.69-getNorm(poss)) <= 0.01);

}


int main(int argc, char* argv[]){

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
