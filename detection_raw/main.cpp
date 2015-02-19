/**LICENSE LOCATED AT BOTTOM; DO NOT MODIFY THIS**/
/** leonfrickensmith@gmail.com **/
/** IRIS at UIUC 2015 **/

#include <iostream> //cout
#include <pthread.h>//pthreads
#include <string.h>//strcpy
#include <vector> //for std::vector

#include "CoordSystemKinect.hpp"//Kinect Input
#include "libfreenect.hpp"//Kinect Input
#include "Linear.hpp"//Mat3
#include "Map.hpp"//Map<T>

using namespace std;
//csk namespace represents CoordinateSystemKinect
const int sizeDepth = 640*480*sizeof(uint16_t);//we need this much space to represent the depth data
const int sizeVideo = 640*480*3*sizeof(char);//we need this much for the video data

/**ASK IF THREADS HAVE STOPPED**/
static volatile bool main_stop = false;
static volatile bool threads_stop = false;

/**FOR THREADS**/
static volatile bool depth_used = true, video_used = true;
static uint16_t* pDepth = NULL;
static char* pVideo = NULL;
static Vec3f downDirection(0,0,0);//static to prevent other files from seeing this

/**LFN**/
static freenect_context* f_ctx;
freenect_device* f_dev;

/**MISC**/
static char userChoice = '\0';//


/**================================================================================**/
/**DEPTH SENSOR CALLBACK**/
/**================================================================================**/
void depth_cb(freenect_device* pDevice, void* v_depth, uint32_t timestamp)
{
    if(depth_used)
    {
        memcpy(pDepth, v_depth, sizeDepth);
        depth_used = false;
    }
}
/**================================================================================**/
/**RGB SENSOR CALLBACK**/
/**================================================================================**/
void video_cb(freenect_device* pDevice, void* v_video, uint32_t timestamp)
{
    if(video_used)
    {
        memcpy(pVideo, v_video, sizeVideo);
        video_used = false;
    }
}
/**================================================================================**/
/**DEPTH PROCESS THREAD**/
/**================================================================================**/
void* thread_depth(void* arg)
{
    while(not threads_stop)
    {
        if(not depth_used && pDepth != NULL)//make sure we don't take an image with bad accelerometer data
        {
            if(downDirection.z == 0)
                cout << "\nNo Data From Kinect Accelerometer!";

            const int pointCount = csk::dimX*csk::dimY;
            Map<float> gradient(Vec2i(80,80));
            Map<float> height(Vec2i(80,80));

            height.getPoint(Vec2i(0,0)).value = 9;
            vector<Vec3f> pointCloud;
            pointCloud.resize(csk::dimX*csk::dimY);//make our pointcloud large enough

            /**REMOVE INVALID POINTS FROM DEPTH DATA**/
            for(int i = 0; i<pointCount; ++i)
                if(csk::RawDepthToMilli(pDepth[i]) < 450)
                    pDepth[i] = 0;
            /**CREATE CARTESIAN POINT CLOUD**/
            for(int y = 0; y<csk::dimY; ++y)
                for(int x = 0; x<csk::dimX; ++x)
                    pointCloud[csk::GetCoord(x,y)] = csk::GetCartCoord(x, y, pDepth);
            /**POINT CLOUD ADJUSTED FOR PITCH AND ROLL**/
            /**NOTE THAT WE CANNOT KNOW YAW (How the kinect is turned in relation to another object)**/
            Mat3f pitchRoll = csk::FindDownMatrix(downDirection);//find the rotation matrix
            for(int i = 0; i<pointCount; ++i)//rotate the point cloud data appropriatly
            {
                pointCloud[i] = pitchRoll*pointCloud[i];
            }
            /**POINT CLOUD UNITS ADJUSTED FOR HUMAN VIEWING**/
            const float unitConvert = 1.0f/50.0f;//half decimeters (50 times larger than a millimeter is half a decimeter)
            for(int i = 0; i<pointCount; ++i)
            {
                pointCloud[i].z *= unitConvert;
                pointCloud[i].y *= unitConvert;
                pointCloud[i].x *= unitConvert;
            }
            /**CONVERT POINT CLOUD INTO HEIGHT MAP**/
            for(int i = 0; i<pointCount; ++i)
            {
                if(height.getPoint(Vec2i(pointCloud[i].x-40, pointCloud[i].y+40)).value < pointCloud[i].z)
                    height.getPoint(Vec2i(pointCloud[i].x-40, pointCloud[i].y+40)).value = pointCloud[i].z;
            }
            /**REMOVE STRANGE VALUES FROM MAP, PRINT TO A FILE, PAUSE PROGRAM**/
            height.makeGradient(gradient);
            gradient.minValue = -1;
            gradient.maxValue = 9;
            gradient.nullRep = '-';
            gradient.toFile("gradientMap.txt");
            //height.normalizeMap();
            //height.toFile("heightMap.txt");
            cout << "\n\tDepth Processed.";
            depth_used = true;
        }
    }
    return NULL;
}
/**================================================================================**/
/**VIDEO PROCESS THREAD**/
/**================================================================================**/
void* thread_video(void* arg)
{
    while(not threads_stop)
    {
        if(not video_used && pVideo != NULL)
        {
            sleep(1);//simulate process time
            cout << "\n\tVideo Processed.";
            video_used = true;
        }
    }
    return NULL;
}




/**================================================================================**/
/**KINECT UPDATE THREAD**/
/**================================================================================**/
void* thread_kinect(void* arg)
{
    /**MISC KINECT COMMANDS**/
    //freenect_set_tilt_degs(f_dev, -22);//set kinect angle
    //freenect_set_led(f_dev, static_cast<LED_COLOR>(3));//set kinect LED color, LED_RED, libfreenect.h

    /**SETUP VIDEO**/
    freenect_set_video_callback(f_dev, video_cb);
    freenect_frame_mode rgbMode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
    freenect_set_video_mode(f_dev, rgbMode);
    freenect_start_video(f_dev);//tell it to start reading rgb

    /**SETUP DEPTH**/
    freenect_set_depth_callback(f_dev, depth_cb);//set the function that will be called for each depth call
    freenect_frame_mode depthMode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT);
    freenect_set_depth_mode(f_dev, depthMode);
    freenect_start_depth(f_dev);//tell it to start reading depth


    while(not threads_stop && freenect_process_events(f_ctx) >= 0)/**this is primary loop for kinect stuff**/
    {
        double dx,dy,dz;
        freenect_raw_tilt_state* pState;
        freenect_update_tilt_state(f_dev);
        pState = freenect_get_tilt_state(f_dev);
        freenect_get_mks_accel(pState, &dx, &dy, &dz);
        downDirection = csk::FindDown(pState->accelerometer_x, pState->accelerometer_y, pState->accelerometer_z);
        //cout << "\nDown:\t" << downDirection.x << "\t" << downDirection.y << "\t" << downDirection.z;
    }

    /**SHUT DOWN STREAMS**/
    freenect_stop_video(f_dev);
    freenect_stop_depth(f_dev);
    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
    return NULL;
}





/**================================================================================**/
/**=================================  MAIN  =======================================**/
/**================================================================================**/
int main(int argc, char **argv)
{
    pthread_t freenect_thread;
    /**===================================================**/
    /**ALL ABOUT INITIALIZING THE CONNECTION WITH KINECT!!**/
    /**===================================================**/

    pDepth = static_cast<uint16_t*>(malloc(sizeDepth));//each point is a uint16_t for depth
    pVideo = static_cast<char*>(malloc(sizeVideo));//each point needs 3 chars to represent the color there (r255,g255,b255)

    if(freenect_init(&f_ctx, NULL) < 0)
    {
        cout << "\nFreenect_init() failed.(1)";
        return 1;
    }
    freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
    int nr_devices = freenect_num_devices(f_ctx);
    cout << "\nNumber of devices found: " << nr_devices;
    int user_device_number = 0;
    if(argc > 1)
        user_device_number = atoi(argv[1]);
    if(nr_devices < 1)
    {
        cout << "\nNo devices found.(2)";
        return 2;
    }
    if(freenect_open_device(f_ctx, &f_dev, user_device_number) < 0)
    {
        cout << "\nCould not open device.(3)";
        return 3;
    }
    else
        cout << "\nOpened a device.";



    /**THREADS TO SIMULTANEOUSLY RUN THE SENSOR INPUT AND COMPUTATION**/
    int kinect = pthread_create(&freenect_thread, NULL, thread_kinect, NULL);
    int map = pthread_create(&freenect_thread, NULL, thread_depth, NULL);
    int video = pthread_create(&freenect_thread, NULL, thread_video, NULL);


    if(kinect or map or video)
    {
        cout << "\nPThread_create failed.(5)";
        return 4;
    }


    while(not main_stop)//this loops while the other threads do things like depth callback
    {
        sleep(1);
        cout << "\nMain.";
        cin >> userChoice;
        if(userChoice == 's')
            threads_stop = true;
        if(userChoice == 'q')
            main_stop = true;
    }
    threads_stop = true;

    free(pDepth);
    free(pVideo);

    cout << "\nExit Success.(0)";
    return 0;
}
/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */
