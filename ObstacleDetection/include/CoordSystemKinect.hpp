/**THIS FILE CONTAINS USEFUL FUNCTIONS FOR USING KINECT DEPTH STREAM**/
/**written by: Leon Frickensmith leonfrickensmith@gmail.com**/

#ifndef COORDSYSTEMKINECT_HPP
#define COORDSYSTEMKINECT_HPP

#include "TrigLookup.hpp"//used for sinL
#include "Linear.hpp"
#include <stdint.h>//used for uint16_t

namespace csk
{

/**EVERTHING IS IN RADIANS**/
const float pi = 3.1415926;
const float pi2 = pi/2.0;

const float fovX = 57*pi/180.0;
const float fovY = 43*pi/180.0;
const float fovX2 = fovX/2.0;
const float fovY2 = fovY/2.0;

const int dimX = 640;
const int dimY = 480;
const float dimX2 = dimX/2.0;
const float dimY2 = dimY/2.0;


/**converts the strange units to meters**/
float RawDepthToMilli(int depthValue);

/**screen style coordinates, 0,0 top left; 639,0 top right; 639,479 bottom right;**/
int GetCoord(int x, int y);

/**get the x and y angle of the ray(used by spherical coordinate system)**/
Vec2f GetAngle(int x, int y);

/**get the cartesian style coordinates for some ray in the depth frame**/
Vec3f GetCartCoord(int x, int y, const uint16_t* pDepth);

/**this gives a Vec that points down in coordinate system where forward is +X, left is +Y, up is +Z**/
Vec3f FindDown(int16_t accelerometer_x, int16_t accelerometer_y, int16_t accelerometer_z);

/**Finds a matrix to multiply INPUT by so that this vector lies in the -Z axis**/
Mat3f FindDownMatrix(const Vec3f& rVec);
}
#endif // COORDSYSTEMKINECT_HPP










