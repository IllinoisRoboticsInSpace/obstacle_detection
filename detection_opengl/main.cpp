/**LICENSE LOCATED AT BOTTOM; DO NOT MODIFY THIS**/
//  leonfrickensmith@gmail.com
/**GO THROUGH THESE INCLUDES AND REMOVE THE ONES WE DON'T NEED, MARK THE ONES WE DO!**/
#include <iostream>//cout
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>
#include <unistd.h>
#include <vector>//for std::vector


#include "libfreenect.hpp"//Kinect Input
#include "Linear.hpp"//Mat3
#include "CoordSystemKinect.hpp"//Kinect Input
#include "Map.hpp"//Map<T>

#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>


using namespace std;

static Vec3f downDirection(0,0,0);//static to prevent other files from seeing this
//static char userChoice = 't';

std::string leon;

const int resSize = 100;
float colorr[resSize][resSize];
float colorg[resSize][resSize];
float colorb[resSize][resSize];

void screenDraw(int x, int y, float red, float green, float blue)
{
    if(x < resSize && y < resSize)
    {
        colorr[x][y] = red;
        colorg[x][y] = green;
        colorb[x][y] = blue;
    }
}



pthread_t freenect_thread;
volatile int die = 0;
volatile bool depth_finished = false;
unsigned char* rgb_back;

unsigned char* rgb_feed;
unsigned char* bw_feed_last;
unsigned char* bw_feed;
unsigned char* depth_feed;

GLuint gl_rgb_tex;


freenect_context* f_ctx;
freenect_device* f_dev;
freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;
pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
uint16_t t_gamma[2048];









///FIX RES SIZE is display()








/**================================================================================**/
/**DEPTH SENSOR CALLBACK, a callback is called after every event of that type**/
/**================================================================================**/
void depth_cb(freenect_device* device, void* v_depth, uint32_t timestamp)
{
    uint16_t* pDepth = static_cast<uint16_t*>(v_depth);

    depth_finished = false;
  /*  for(int i = 0; i<(640*480*3); i+=3)
    {
        depth_feed[i+0] = 0;//(1-(RawDepthToMilli(v_depth[i])/1000.0f))*255;
        depth_feed[i+1] = (RawDepthToMilli(pDepth[i/3])/1000.0f)*255;
        depth_feed[i+2] = 0;//255;
    }*/


    if(downDirection.z != 0)//make sure we don't take an image with bad accelerometer data
    {

        const int pointCount = dimX*dimY;

        Map<float> gradient(Vec2i(80,80));
        Map<float> height(Vec2i(80,80));
        height.getPoint(Vec2i(0,0)).value = 9;
        vector<Vec3f> pointCloud;
        pointCloud.resize(dimX*dimY);//make our pointcloud large enough

        /**REMOVE INVALID POINTS FROM DEPTH DATA**/
        for(int i = 0; i<pointCount; ++i)
        {
            if(RawDepthToMilli(pDepth[i]) < 450)
            {
                pDepth[i] = 0;
            }
        }

        /**POINT CLOUD CREATED**/
        for(int y = 0; y<dimY; ++y)//populate point cloud
        {
            for(int x = 0; x<dimX; ++x)
            {
                pointCloud[GetCoord(x,y)] = GetCartCoord(x, y, pDepth);
            }
        }

        /**POINT CLOUD ADJUSTED FOR PITCH AND ROLL**/
        Mat3f pitchRoll = FindDownMatrix(downDirection);//find the rotation matrix
        for(int i = 0; i<pointCount; ++i)//rotate the point cloud data appropriatly
        {
            pointCloud[i] = pitchRoll*pointCloud[i];
        }
        /**STILL NEEDS ADJUSTMENT FOR YAW**/


        /**POINT CLOUD SHRUNK AND MANIPULATED FOR HUMAN VIEWING**/
        const float unitConvert = 1.0f/50.0f;//half decimeters
        for(int i = 0; i<pointCount; ++i)
        {
            pointCloud[i].z *= unitConvert;
            pointCloud[i].y *= unitConvert;
            pointCloud[i].x *= unitConvert;
        }
        for(int i = 0; i<pointCount; ++i)
        {
            pointCloud[i].z += 0;
        }


        /**CONVERT POINT CLOUD INTO HEIGHT MAP; NOTE: it has been manipulated for easy human viewing**/
        const int changex = 0;
        const int changey = 0;
        for(int i = 0; i<pointCount; ++i)
        {
            if(height.getPoint(Vec2i(pointCloud[i].x+changex, pointCloud[i].y+changey)).value < pointCloud[i].z)
                height.getPoint(Vec2i(pointCloud[i].x+changex, pointCloud[i].y+changey)).value = pointCloud[i].z;
        }

        /**REMOVE STRANGE VALUES FROM MAP, PRINT TO A FILE, PAUSE PROGRAM**/

        height.makeGradient(gradient);
        gradient.minValue = -1;
        gradient.maxValue = 9;
        gradient.nullRep = '-';
        gradient.toFile("gradientMap.txt");


        for(int x=0; x<resSize; x+=1)
        {
            for(int y=0; y<resSize; y+=1)
            {
                colorr[x][y] = 0;
                colorg[x][y] = 0;
                colorb[x][y] = 0;
            }
        }

        for(int i=0; i<(640*480*3); ++i)
        {
            depth_feed[i] = 0;
        }

        const int xScale = 4;
        const int yScale = 3;
        const int xOff = 80;
        const int yOff = 80;

        for(int i=0; i<(640*480*3); i+=3)
        {
            int x = i/3;
            float val = gradient.getPoint(Vec2i( (((x%640))/xScale-xOff), -((x/640)/yScale -yOff))).value;
            if(val == -9999.0)
            {
                depth_feed[i+0] = 255;
                depth_feed[i+1] = 255;
                depth_feed[i+2] = 255;
            }
            else if(val == 1)
            {
                depth_feed[i+0] = 255;
                depth_feed[i+1] = 0;
                depth_feed[i+2] = 0;
            }
        }
        depth_finished = true;

        /*
        for(int x=0; x<resSize; ++x)
        {
            for(int y=0; y<resSize; ++y)
            {
                float value = gradient.getPoint(Vec2i(x-10,y-40)).value;
                if(gradient.defaultValue == value)
                {

                }
                else
                {
                    screenDraw(x,y,1-value,value,0);
                }
            }
        }*/

        //cin >> userChoice;
    }

    pthread_cond_signal(&gl_frame_cond);
}
/**================================================================================**/
/**VIDEO CALLBACK**/
/**================================================================================**/
void video_cb(freenect_device* device, void* v_video, uint32_t timestamp)
{
    unsigned char* pVideo = (unsigned char*)v_video;



    for(int i=0; i<480*640; ++i)
    {
        bw_feed[i] = pVideo[i];
    }
    unsigned char *img1 = bw_feed_last, *img2 = bw_feed;


    int nFeatures = 1;
    int ncols = 640, nrows = 480;




    int i;
    //cin >> i;

    strncpy((char*)bw_feed_last, (const char*)bw_feed, 640*480*sizeof(char));
    strncpy((char*)rgb_feed, (const char*)pVideo, 640*480*3*sizeof(char));


}


/**================================================================================**/
/**THIS IS THE SECOND THREAD, effectivly a second main()**/
/**================================================================================**/
void* thread_opengl(void* arg)
{
    while(true)
    {
        //cout << "\nThread Running!";
    }
    return NULL;
}

void* freenect_threadfunc(void* arg)
{
    bool enableAccel = true;

    //freenect_set_tilt_degs(f_dev, -22);//set kinect angle
    freenect_set_led(f_dev, static_cast<LED_COLOR>(3));//set kinect LED color, LED_RED, libfreenect.h

    freenect_set_depth_callback(f_dev, depth_cb);//set the function that will be called for each depth call
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    freenect_start_depth(f_dev);//tell it to start reading depth

    freenect_set_video_callback(f_dev, video_cb);
    freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));
    freenect_set_video_buffer(f_dev, rgb_back);
    freenect_start_video(f_dev);//tell it to start reading rgb



    while (!die && freenect_process_events(f_ctx) >= 0 && enableAccel)/**this is primary loop for kinect stuff**/
    {
        double dx,dy,dz;
        freenect_raw_tilt_state* pState;
        freenect_update_tilt_state(f_dev);
        pState = freenect_get_tilt_state(f_dev);
        freenect_get_mks_accel(pState, &dx, &dy, &dz);
        downDirection = FindDown(pState->accelerometer_x, pState->accelerometer_y, pState->accelerometer_z);


        //cout << "\nDown:\t" << downDirection.x << "\t" << downDirection.y << "\t" << downDirection.z;

        if(requested_format != current_format)
        {
            freenect_stop_video(f_dev);
            freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, requested_format));
            freenect_start_video(f_dev);
            current_format = requested_format;
        }
    }

    freenect_stop_depth(f_dev);//shutting down streams..
    freenect_stop_video(f_dev);

    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);

    return NULL;
}









void display()
{
    glClear (GL_COLOR_BUFFER_BIT);

    while(not depth_finished)
    {
        //sleep(1);//wait for depth to get done drawing
    }

    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_feed);//you can pass an array here to display


    glBegin(GL_TRIANGLE_FAN);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glTexCoord2f(0, 0);
    glVertex3f(0,0,0);
    glTexCoord2f(1, 0);
    glVertex3f(480,0,0);
    glTexCoord2f(1, 1);
    glVertex3f(480,480,0);
    glTexCoord2f(0, 1);
    glVertex3f(0,480,0);
    glEnd();

    glutSwapBuffers();

    glutPostRedisplay();
}

void init()
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


/*
 *  Declare initial window size, position, and display mode
 *  (single buffer and RGBA).  Open window with "hello"
 *  in its title bar.  Call initialization routines.
 *  Register callback function to display graphics.
 *  Enter main loop and process events.
 */





/**================================================================================**/
/**=================================  MAIN  =======================================**/
/**================================================================================**/
int main(int argc, char **argv)
{
    ///PROOF THAT KLT CAN WORK IN THIS ENVIRONMENT

    /**===================================================**/
    /**ALL ABOUT INITIALIZING THE CONNECTION WITH KINECT!!**/
    /**===================================================**/
    int res;

    bw_feed = (uint8_t*)malloc(640*480);
    bw_feed_last = (uint8_t*)malloc(640*480);
    rgb_feed = (uint8_t*)malloc(640*480*3);
    depth_feed = (uint8_t*)malloc(640*480*3);

    int i;
    for (i=0; i<2048; i++)
    {
        float v = i/2048.0;//what does this do?
        v = powf(v, 3)* 6;
        t_gamma[i] = v*6*256;
    }
    if (freenect_init(&f_ctx, NULL) < 0)
    {
        printf("\n\nFreenect_init() failed.\n");
        return 1;
    }
    freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
    int nr_devices = freenect_num_devices (f_ctx);
    printf("\nNumber of devices found: %d.", nr_devices);
    int user_device_number = 0;
    if (argc > 1)
        user_device_number = atoi(argv[1]);
    if (nr_devices < 1)
    {
        printf("\n\nExit(1)\n");
        return 1;
    }
    if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0)
    {
        printf("\n\nCould not open device.\n");
        return 1;
    }
    else
        printf("\nOpened a device.");



    /**WE MUST HAVE OPENED A DEVICE, SO CREATE A NEW THREAD TO DEAL WITH IT**/
    res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
    int gl = pthread_create(&freenect_thread, NULL, thread_opengl, NULL);
    if(res)
    {
        printf("\n\nPThread_create failed.\n");
        return 1;
    }





    for(int x=0; x<resSize; x+=1)
    {
        for(int y=0; y<resSize; y+=1)
        {
            colorr[x][y] = 0;
            colorg[x][y] = 0;
            colorb[x][y] = 0;
        }
    }


    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(960, 480);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Viewing Window 0");
    init();
    glutDisplayFunc(display);
    glutMainLoop();

    delete bw_feed;
    delete bw_feed_last;
    delete rgb_back;
    delete rgb_feed;

    printf("\n\nExit(2).\n");
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
