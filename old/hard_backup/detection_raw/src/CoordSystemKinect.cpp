
#include "CoordSystemKinect.hpp"

/**THIS FILE CONTAINS USEFUL FUNCTIONS FOR USING KINECT DEPTH STREAM**/
/**written by: Leon Frickensmith leonfrickensmith@gmail.com**/

/**converts the strange units to meters**/
namespace csk
{
float RawDepthToMilli(int depthValue)
{
    const float mystery1 = -0.0030711016f;//these were obtained from the internet
    const float mystery2 = 3.3309495161f;
    return (1.0f / (((static_cast<float>(depthValue) * mystery1) + mystery2)))*1000;
}


/**screen style coordinates, 0,0 top left; 639,0 top right; 639,479 bottom right;**/
int GetCoord(int x, int y)
{
    return (x+y*dimX);
}


/**get the x and y angle of the ray(spherical coordinate system)**/
Vec2f GetAngle(int x, int y)
{
    Vec2f angles;
    angles.x = ((dimX2-x)/dimX2)*fovX2;
    angles.y = pi2-(((dimY2-y)/dimY2)*fovY2);
    return angles;
}


/**get the cartesian style coordinates for some ray in the depth frame**/
/**kinect measures distance from the viewing plane of the camera**/
Vec3f GetCartCoord(int x, int y, const uint16_t* pDepth)
{
    Vec3f cartesian;
    float distance = RawDepthToMilli(pDepth[GetCoord(x,y)]);//r in millimeters

    Vec2f azmuthPolar(GetAngle(x,y));//theta is CCW angle from +X, Phi is angle from ZY plane

    cartesian.x = distance;
    cartesian.y = distance*tanL(azmuthPolar.x);
    cartesian.z = distance*tanL(-azmuthPolar.y+pi2);

    return cartesian;
}


/**this gives a Vec that points down in coordinate system where forward is +X, left is +Y, up is +Z**/
Vec3f FindDown(int16_t accelerometer_x, int16_t accelerometer_y, int16_t accelerometer_z)
{
    return Vec3f(-accelerometer_z, -accelerometer_x, -accelerometer_y);
}


/**Finds a matrix to multiply INPUT by so that this vector lies in the -Z axis**/
Mat3f FindDownMatrix(const Vec3f& rVec)
{
    Vec3f a(rVec);//a is vector to rotate
    const Vec3f b(0,0,-1);//        b
    a.normalize();//                a
    const float c = a.dot(b);//     c

    const Vec3f v = a.cross(b);//   v
    const float s = v.length();//   s
    Mat3f vx;
    vx.setSkewSymCrossProd(v);//    vx
    const Mat3f vx2(vx*vx);//       vx2

    Mat3f I;//                      I
    I.x1 = 1;
    I.y2 = 1;
    I.z3 = 1;

    return I+vx+(vx2*((1-c)/(s*s)));
}
}
