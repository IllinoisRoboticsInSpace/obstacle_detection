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


/**OPENGL COMMANDS**/
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
void* thread_display(void* arg);
void gl_display();
void gl_init();
void gl_idle();
void gl_displayTexture(char* pTexture, int width, int height, int xStart, int yStart);
GLuint gl_rgb_tex;




using namespace std;
//csk namespace represents CoordinateSystemKinect
const int sizeDepth = 640*480*sizeof(uint16_t);//we need this much space to represent the depth data
const int sizeVideo = 640*480*3*sizeof(char);//we need this much for the video data

/**ASK IF THREADS HAVE STOPPED**/
static volatile bool main_stop = false;
static volatile bool threads_stop = false;

/**FOR THREADS**/
static volatile bool depth_used = true, video_used = true, depth_displayed = true, map_displayed = true;

static uint16_t* pDepth = NULL;
static char* pVideo = NULL;
static char* pDepthFeed = NULL;
static char* pMapFeed = NULL;
static uint16_t* pDepthDisplay = NULL;

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
    if(depth_displayed)
    {
        memcpy(pDepthDisplay, v_depth, sizeDepth);
        depth_displayed = false;
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
            {
                int milli = csk::RawDepthToMilli(pDepth[i]);
                if(milli < 450 || milli > 4000)
                    pDepth[i] = 0;

            }
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
                if(height.getPoint(Vec2i(pointCloud[i].x, pointCloud[i].y)).value < pointCloud[i].z)
                    height.getPoint(Vec2i(pointCloud[i].x, pointCloud[i].y)).value = pointCloud[i].z;
            }
            /**REMOVE STRANGE VALUES FROM MAP, PRINT TO A FILE, PAUSE PROGRAM**/
            height.makeGradient(gradient);
            gradient.minValue = -1;
            gradient.maxValue = 9;
            gradient.nullRep = '-';
            gradient.toFile("gradientMap.txt");
            //height.normalizeMap();
            //height.toFile("heightMap.txt");
            //cout << "\n\tDepth Processed.";
            const int xScale = 4;
            const int yScale = 3;
            const int xOff = 80;
            const int yOff = 80;

            if(map_displayed)
            {
                for(int i=0; i<sizeVideo; i+=3)
                {
                    int x = i/3;
                    float val = gradient.getPoint(Vec2i( (((x%640))/xScale-xOff), -((x/640)/yScale -yOff))).value;
                    if(val == -9999.0)
                    {
                        pMapFeed[i+0] = 0;
                        pMapFeed[i+1] = 0;
                        pMapFeed[i+2] = 0;
                    }
                    else if(val == 1)
                    {
                        pMapFeed[i+0] = 255;
                        pMapFeed[i+1] = 0;
                        pMapFeed[i+2] = 0;
                    }
                    else
                    {
                        pMapFeed[i+0] = 255;
                        pMapFeed[i+1] = 255;
                        pMapFeed[i+2] = 255;
                    }
                }
                map_displayed = false;
            }
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
    /*
    freenect_set_video_callback(f_dev, video_cb);
    freenect_frame_mode rgbMode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
    freenect_set_video_mode(f_dev, rgbMode);
    freenect_start_video(f_dev);//tell it to start reading rgb
    */
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
    pthread_t kinect_t;
    pthread_t depth_t;
    pthread_t video_t;
    pthread_t display_t;
    /**===================================================**/
    /**ALL ABOUT INITIALIZING THE CONNECTION WITH KINECT!!**/
    /**===================================================**/

    pDepthDisplay = static_cast<uint16_t*>(malloc(sizeDepth));
    pDepthFeed = static_cast<char*>(malloc(sizeVideo));//used to rgb display what the kinect sees
    pDepth = static_cast<uint16_t*>(malloc(sizeDepth));//each point is a uint16_t for depth
    pMapFeed = static_cast<char*>(malloc(sizeVideo));

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
    int kinect = pthread_create(&kinect_t, NULL, thread_kinect, NULL);
    int map = pthread_create(&depth_t, NULL, thread_depth, NULL);
    int video = pthread_create(&video_t, NULL, thread_video, NULL);
    int display = pthread_create(&display_t, NULL, thread_display, NULL);


    if(kinect or map or video or display)
    {
        cout << "\nPThread_create failed.(5)";
        return 4;
    }


    while(not main_stop)//this loops while the other threads do things like depth callback
    {
        sleep(1);
        cout << "\nMain.";
        //cin >> userChoice;
        if(userChoice == 's')
            threads_stop = true;
        if(userChoice == 'q')
            main_stop = true;
    }
    threads_stop = true;

    free(pDepthDisplay);
    free(pDepthFeed);
    free(pMapFeed);
    free(pDepth);
    free(pVideo);

    cout << "\nExit Success.(0)";
    return 0;
}


void* thread_display(void* arg)
{
    int myArgc = 1;
    char* myArgv[1];
    myArgv[0] = strdup("Obstacle Detection Display");

    glutInit(&myArgc, myArgv);
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(960, 480);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Viewing Window 0");
    gl_init();
    glutDisplayFunc(gl_display);
    glutMainLoop();
    return NULL;
}

void gl_display()
{
    if(not depth_displayed)
    {
        int x;
        float scale = 0;
        for(int i=0; i<(csk::dimX*csk::dimY); ++i)
        {
            x = 3*i;
            const float dist = csk::RawDepthToMilli(pDepthDisplay[i])/1000;
            pDepthFeed[x+0] = (3.5-2*(dist-0.4))*96;//Red
            pDepthFeed[x+1] = (dist)*768;
            pDepthFeed[x+2] = (dist)*96;//csk::RawDepthToMilli(pDepth[i])/1000*128;//Blue
        }

        gl_displayTexture(pDepthFeed, 640, 480, 0, 0);
        depth_displayed = true;
    }
    if(not map_displayed)
    {
        gl_displayTexture(pMapFeed, 640, 480, 480, 0);
        map_displayed = true;
    }


    glutSwapBuffers();
    glutPostRedisplay();
}
void gl_displayTexture(char* pTexture, int width, int height, int xStart, int yStart)
{
    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, pTexture);//you can pass an array here to display
    glBegin(GL_TRIANGLE_FAN);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glTexCoord2f(0, 0);
    glVertex3f(0+xStart,0+yStart,0);
    glTexCoord2f(1, 0);
    glVertex3f(480+xStart,0+yStart,0);
    glTexCoord2f(1, 1);
    glVertex3f(480+xStart,480+yStart,0);
    glTexCoord2f(0, 1);
    glVertex3f(0+xStart,480+yStart,0);
    glEnd();
}
void gl_init()
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0);
    glDepthFunc(GL_LESS);

    glEnable(GL_TEXTURE_2D);
    glDepthMask(GL_FALSE);
    glDisable(GL_ALPHA_TEST);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel(GL_SMOOTH);


    glGenTextures(1, &gl_rgb_tex);
    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);


    glViewport(0,0,960,480);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho (0, 960, 480, 0, -1.0f, 1.0f);
    glMatrixMode(GL_MODELVIEW);
}
void gl_idle()
{

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
