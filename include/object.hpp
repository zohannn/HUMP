#ifndef HUMP_OBJECT_HPP
#define HUMP_OBJECT_HPP
#include <string>
#include <vector>

using namespace std;

namespace HUMotion {

/**
 * @brief The Object class
 */
class Object
{
public:

    /**
     * @brief Object, a constructor
     */
    Object();

    /**
     * @brief Object, a constructor
     * @param name
     */
    explicit Object(string name);

    /**
     * @brief Object
     * @param obj
     */
    Object(const Object& obj);

    /**
     * @brief ~Object, a destructor
     */
    ~Object();

    /**
     * @brief setName
     * @param name
     */
    void setName(string name);

    /**
     * @brief getName
     * @return
     */
    string getName();

    /**
     * @brief setPos
     * @param pos
     */
    void setPos(vector<double>& pos);

    /**
     * @brief getPos
     * @param pos
     */
    void getPos(vector<double>& pos);

    /**
     * @brief setOr
     * @param orr
     */
    void setOr(vector<double>& orr);

    /**
     * @brief getOr
     * @param orr
     */
    void getOr(vector<double>& orr);

    /**
     * @brief setSize
     * @param dim
     */
    void setSize(vector<double>& dim);

    /**
     * @brief getSize
     * @param dim
     */
    void getSize(vector<double>& dim);

    /**
     * @brief setParams
     * @param pos
     * @param orr
     * @param dim
     */
    void setParams(vector<double>& pos,vector<double>& orr, vector<double>& dim);

    /**
     * @brief getParams
     * @param pos
     * @param orr
     * @param dim
     */
    void getParams(vector<double>& pos,vector<double>& orr, vector<double>& dim);



private:

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

};


}// namespace HUMotion

#endif // HUMP_OBJECT_HPP
