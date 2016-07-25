#ifndef TARGET_H
#define TARGET_H

#include "HUMLconfig.hpp"
#include "common_functions.hpp"

namespace HUMotion {

using namespace std;

class Target
{
public:
    // constructors
    explicit Target();
    explicit Target(string name, pos ppos, orient oor);


    // copy constructor
    Target(const Target& tar);

    // destructor
    ~Target();

    //setters
    void setName(const std::string& name);
    void setPos(pos& ppos);
    void setOr(orient& oor);

    //getters
    string getName() const;
    pos getPos() const;
    orient getOr() const;
    void getXt(std::vector<float>& xt);
    void getYt(std::vector<float>& yt);
    void getZt(std::vector<float>& zt);
    float getNorm();
    void RPY_matrix(Matrix3f& Rot);


private:

    string m_name; // name of the target
    pos m_pos; // position of the target
    orient m_or; // orientation of the target



};

} // namespace HUMotion

#endif // TARGET_H
