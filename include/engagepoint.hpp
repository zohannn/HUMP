#ifndef ENGAGEPOINT_H
#define ENGAGEPOINT_H

#include "HUMLconfig.hpp"

namespace HUMotion {

using namespace std;

class EngagePoint
{
public:
    // constructors
    explicit EngagePoint();
    explicit EngagePoint(string name, pos ppos, orient oor);

    // copy constructor
    EngagePoint(const EngagePoint& eng);

    //destructor
    ~EngagePoint();


    // setters
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

    string m_name; // name of the engage point
    pos m_pos; // position of the target
    orient m_or; // orientation of the target




};

} // namespace HUMotion

#endif // ENGAGEPOINT_H
