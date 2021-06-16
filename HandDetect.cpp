/**
	@author Phillip ***
	@date 4/12/2020

	HandDetect.cpp

	This is the main file for initial test of detecting a hand.
	This project will only use static images. When the detector gets working,
	will implement code for real-time video
*/


#define _CRT_SECURE_NO_WARNINGS

//Detection box
#define MAX_DETECTION_BOX_COL 400
#define MAX_SUBDIVISION_NOTES 10 //use to determine how much area is divded per note
//Adaptive threshold parameters
#define ADAPT_BLOCK 9 //Must be Odd
#define ADAPT_C 7
#define ADAPT2_BLOCK 13 //Must be odd
#define ADAPT2_C 7

//Centroid Parameters
#define CENTROID_HISTORY_BUFF_SIZE 4
#define CENTROID_JUMP_THRESH 3 //Maybe use to determine when ump should be scaled
#define CENTROID_JUMP_MAX_THRESH 7 // Maybe used to determine ignored values since jump too large
//Contour detection
#define CONTOUR_THRESH 10 //Try to remove anything that might be detected
#define CONTOUR_AREA_THRESH 100 //Maybe also use to ignore artifacts that are small?

//HSV threshold for inRange
#define HSV_HUE_RANGE 20
#define HSV_SAT_RANGE 250
#define HSV_VAL_RANGE 250

#include <Windows.h>
#include <MMSystem.h>

#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"

#include <opencv2/video/background_segm.hpp> //Background subtraction

#include "../rtmidi-master/RtMidi.h" //This library will handle MIDI commands

using namespace std;
using namespace cv;


//get image size of video
Mat backgroundImg = imread("../pics/Hand_Background.bmp"); //Plain background for removal
Mat handImg = imread("../pics/Hand_Side.bmp"); // Hand image
//Mat handImg = imread("../pics/Hand.bmp"); // Hand image



int maxX = backgroundImg.size().width;
int maxY = backgroundImg.size().height;

// Global state variables

bool is_mouse_down;
Rect captureRect;// May actually use poly gon instead




//Use these to update what the background is to subtract
bool backgroundIsSet = false;
bool setBackground = false;

//Use these to (hopefully) better segment image based on HSV threshold (hand color)
bool applyColorThresh = false;
bool colorThreshSet = false;

//Mat for Template image
Mat tempImage;



//Function for handling Midi playing note
void playMidiNote(bool notePlaying, unsigned char oldNote, unsigned char newNote, vector<unsigned char>* theSignalBuffer, RtMidiOut* theMidiOut)
{
   // printf("Old note: %i | newNote: %i \n", oldNote, newNote);

    if (notePlaying && oldNote != newNote)
    {
        //cut off previous note
        //(*theSignalBuffer)[0] = 0x80;
        //(*theSignalBuffer)[1] = oldNote;
        //(*theSignalBuffer)[2] = 0x64; //Constant velocity for now

        //Using control change 'All notes off'
        (*theSignalBuffer)[0] = 0xb0;
        (*theSignalBuffer)[1] = 0x7b;
        (*theSignalBuffer)[2] = 0x00; //Constant velocity for now

        theMidiOut->sendMessage(theSignalBuffer);
    }
    //else if (!notePlaying)
    //{
        (*theSignalBuffer)[0] = 0x90;

        //(*theSignalBuffer)[1] = newNote;
        (*theSignalBuffer)[1] = newNote;

        (*theSignalBuffer)[2] = 0x64; //Constant velocity for now
        theMidiOut->sendMessage(theSignalBuffer);
   // }
    
    notePlaying = true;
}


// Mouse callback event
void MouseCallbackEvent(int event, int x, int y, int flags, void* userdata)
{
  
    if (event == EVENT_LBUTTONDOWN)
    {

        //Since pressing keyboard is wonky, lets just use mouse for now
        //to handle reseting background and such

        if (colorThreshSet) // restart everything
        {
           // backgroundIsSet = false;
           // setBackground = false;

            //applyColorThresh = false;
            //colorThreshSet = false;
        }
        




        

        
    }
    if (event == EVENT_LBUTTONUP)
    {
        if (backgroundIsSet == false)
            setBackground = true; //Reset the background for subtraction
        else if (colorThreshSet == false)
            applyColorThresh = true;
    }
}





int main()
{
  

    /*******Set up Midi device and any necessary variables********/
    RtMidiOut* midiOut = new RtMidiOut(); //the midi out dvice we'll use to send signals (hopefully) 
    bool midiActive;
    bool notePlaying;
    vector<unsigned char> midiSignal; //Use to send Midi signal
    unsigned char midiStatus, midiNote, midiVelocity; //Status for on off (0x9n/0x8n) with n = channel, midiNote note number (60 = middle C), velocity how hard pressed(volume) 0-127 
    midiStatus = 0x80; //status defaults to off
    midiNote = 0x3c; //someempty note value
    try 
    {   
       // midiOut = new RtMidiOut(); set later, getting erros in second try catch
        midiActive = true;
    }
    catch (RtMidiError &error)
    {
        //COuldn't create a Midi out device, continue for now, but set a boolean variable so know to ignore any midi related work
        error.printMessage();
        midiActive = false;
    }

    printf("Available ports \n ______________________\n");
   
    string portName;

    //Check available ports
    printf("available ports: %i \n", midiOut->getPortCount());
    
    for (int i = 0; i < midiOut->getPortCount(); i++)
    {
        try
        {
            portName = midiOut->getPortName(i);
        }
        catch (RtMidiError& error)
        {
            error.printMessage();
        }
        printf("Port%i: %s \n", portName);
    }

    if (midiOut->getPortCount() == 0)
    {
        //There is somehow no out ports, set bool to false
        midiActive = false;
    }






    //for debugging pruposes, go ahead and say midiActive is TRUE! Allows some connectivity to device
    midiActive = true;

    /*!!!!!!!!!!!!!!!!!!!! Change Midi Port here!!!!!!!!!!!!!!!!!!!
    */
    //Assuming here, means a port is available
    midiOut->openPort(2); //<<<<<<<<<<<<<<<<<<<<<<<< Doesn't work if no device!!!! Hence, be sure to remove above bool check for this!!!






    //Go ahead an set up an empty signal sound once to initialize vector
    midiSignal.push_back(0x80);
    midiSignal.push_back(0x3c);
    midiSignal.push_back(0x00);

   



    namedWindow("Output", 1);
    namedWindow("Mask", 1);
    namedWindow("MaskHSV",1);
    namedWindow("bgMask", 1);

    // Set the mouse callback
    setMouseCallback("Output", MouseCallbackEvent, NULL);


    Mat frameClone; //The base image w/ feature points drawn

    

    //************************* Main Processing *********************/


    //Colors
    Scalar rectColor = Scalar(0, 256, 0); // color green, re-used from making bounding box, now used for centroid
    Scalar highlightRect = Scalar(100, 100, 256);
    Scalar contourColor = Scalar(150, 200, 100); 
    Scalar boundingBoxColor = Scalar(34, 100, 220);


    //Threshold for HSV hand colors
    int minHueThresh;
    int minSatThresh;
    int minValThresh;
    int maxHueThresh;
    int maxSatThresh;
    int maxValThresh;

    Mat hsvSample; // sample taken to detect hsv values



    Mat regionOfBackground; //Set when mouse is clicked for bit_and
    Mat regionOfMask; //Maked region when doing bit_and

    Point2f theCentroid; // Used to determine center position of contourHull
    Point2f centroidHistory[CENTROID_HISTORY_BUFF_SIZE]; //A history of centroid position, used to try and tame jumps in position
    int centroidHistoryIter = -1 ; //Iterate through centroidHistory. Negative to signal not to start using Centroid



    /*Setting up webcam to get frames*/

    VideoCapture videoFeed;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if (!videoFeed.open(0))
        return 0;

    while(true) //run forever, baby
    {
        Mat frame;
        videoFeed >> frame;
        if (frame.empty()) 
            break; // nothing being shown, probably due to cut off feed
       

        frameClone = frame.clone(); //May be best to copy and preserve original.
        int maxFrameHeight = frameClone.rows; //get the height of the captured frame

        /* ????  THink about scaling down and such? Might not hurt computations  */

        // Copy a sub section of original to do processing
        Mat regionOfInterest = frameClone(Range::all(), Range(0,MAX_DETECTION_BOX_COL));

        if (setBackground)
        {
            //Get the static background for future reference

            regionOfBackground = regionOfInterest(Range::all(), Range(0, MAX_DETECTION_BOX_COL));
            cvtColor(regionOfBackground, regionOfBackground, COLOR_RGB2GRAY);
            GaussianBlur(regionOfBackground, regionOfBackground, Size(13, 13), 0, 0);

            /*   Thresholding on background. Keep for possible future experiments with double thresholding */

            //adaptiveThreshold(regionOfBackground, regionOfBackground, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, ADAPT_BLOCK, ADAPT_C);
            //threshold(regionOfBackground, regionOfBackground, 98, 255, THRESH_BINARY);


            //Setup centroid history buffer
            for (int i = 0; i < CENTROID_HISTORY_BUFF_SIZE; i++)
            {
                centroidHistory[i] = Point2f(0, 0); //Top left corner
            }
            theCentroid = Point2f(0, 0);
            //centroidHistoryIter = 0; //Will iterate in for loop, making it 0

            setBackground = false;
            backgroundIsSet = true;




            //applyColorThresh = true; //signal that we want to take sample of hand color

            

        }
        if (backgroundIsSet && !colorThreshSet)
        {
            //Draw rectangle to take sample color
            line(frameClone, Point(100, 100), Point(105, 100), rectColor, 1);
            line(frameClone, Point(105, 100), Point(105, 105), rectColor, 1);
            line(frameClone, Point(105, 105), Point(100, 105), rectColor, 1);
            line(frameClone, Point(100, 105), Point(100, 100), rectColor, 1);
        }
        if (applyColorThresh)
        {
            //Clone the small sample section, and convert to hsv
            hsvSample = frameClone(Range(99, 106), Range(99, 106));

            cvtColor(hsvSample, hsvSample, COLOR_BGR2HSV);

            //Maybe apply gaussian blur? even out pixels

            Vec3b hsvPoint;
            //calculate max,min threshold

            int avgHue = 0, avgValue = 0, avgSat = 0;
            for (int i = 0; i < 5; i++)
            {
                for (int j = 0; j < 5; j++)
                {
                    hsvPoint = hsvSample.at<Vec3b>(i+1, j+1);
                    avgHue += hsvPoint.val[0];
                    avgSat += hsvPoint.val[1];
                    avgValue += hsvPoint.val[2];
                }
            }
           
            avgHue = (avgHue / 25) + 180;
            avgSat = avgSat / 25;
            avgValue = avgValue / 25;

            //Assign threshold values to use later

            minHueThresh= avgHue - HSV_HUE_RANGE;
            minSatThresh= avgSat - HSV_SAT_RANGE;
            minValThresh= avgValue - HSV_VAL_RANGE;
            maxHueThresh= avgHue + HSV_HUE_RANGE;
            maxSatThresh= avgSat + HSV_SAT_RANGE;
            maxValThresh= avgValue + HSV_VAL_RANGE;

            

            applyColorThresh = false;
            colorThreshSet = true;

        }

        /*Main entry to hand detection. Starts when static background is set */
        if (backgroundIsSet && colorThreshSet)
        {

            //Draw boxes to show where notes are
            line(frameClone, Point(0, 0), Point(MAX_DETECTION_BOX_COL,0), rectColor, 2);
            line(frameClone, Point(MAX_DETECTION_BOX_COL,0), Point(MAX_DETECTION_BOX_COL, maxFrameHeight), rectColor, 2);
            line(frameClone, Point(MAX_DETECTION_BOX_COL, maxFrameHeight), Point(0,maxFrameHeight ), rectColor, 2);
            line(frameClone, Point(0, maxFrameHeight), Point(0,0), rectColor, 2);
            //Threshold line for where note on and off
            line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, 0), Point(MAX_DETECTION_BOX_COL / 2, maxFrameHeight), rectColor, 1);

            //Lines dividing each note (when played, will color different later)
            line(frameClone, Point(0, ((float)maxFrameHeight) * (9.0f / 10.0f)), Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (9.0f / 10.0f)), rectColor, 1);
            line(frameClone, Point(0, ((float)maxFrameHeight) * (8.0f / 10.0f)), Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (8.0f / 10.0f)), rectColor, 1);
            line(frameClone, Point(0, ((float)maxFrameHeight) * (7.0f / 10.0f)), Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (7.0f / 10.0f)), rectColor, 1);
            line(frameClone, Point(0, ((float)maxFrameHeight) * (6.0f / 10.0f)), Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (6.0f / 10.0f)), rectColor, 1);
            line(frameClone, Point(0, ((float)maxFrameHeight) * (5.0f / 10.0f)), Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (5.0f / 10.0f)), rectColor, 1);
            line(frameClone, Point(0, ((float)maxFrameHeight) * (4.0f / 10.0f)), Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (4.0f / 10.0f)), rectColor, 1);
            line(frameClone, Point(0, ((float)maxFrameHeight) * (3.0f / 10.0f)), Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (3.0f / 10.0f)), rectColor, 1);
            line(frameClone, Point(0, ((float)maxFrameHeight) * (2.0f / 10.0f)), Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (2.0f / 10.0f)), rectColor, 1);
            line(frameClone, Point(0, ((float)maxFrameHeight) * (1.0f / 10.0f)), Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (1.0f / 10.0f)), rectColor, 1);
            //line(frameClone, Point(0, ((float)maxFrameHeight)* (9.0f / 10.0f)), Point(MAX_DETECTION_BOX_COL, ((float)maxFrameHeight)* (9.0f / 10.0f)), rectColor, 1);
            




            centroidHistoryIter = (centroidHistoryIter+1)%CENTROID_HISTORY_BUFF_SIZE;
           

            //Get current frame in hand tracking region
            Mat greyRegionOfInterest = regionOfInterest.clone();
            
            //convert to greyscale, gaussian blur
            cvtColor(greyRegionOfInterest, greyRegionOfInterest, COLOR_BGR2GRAY);
            //GaussianBlur(greyRegionOfInterest, greyRegionOfInterest, Size(3, 3), 0, 0);

            regionOfMask = greyRegionOfInterest.clone(); //Copied to ensure same size and color
            
            //Threshold of foreground. !!!Note!!!! need find way to prevent hard threshold making things choppy, maybe something gradient like
            //adaptiveThreshold(greyRegionOfInterest, greyRegionOfInterest, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, ADAPT2_BLOCK, ADAPT2_C);
            //threshold(greyRegionOfInterest, greyRegionOfInterest, 100, 255, THRESH_BINARY_INV);
            //threshold(greyRegionOfInterest, greyRegionOfInterest, 10, 255, THRESH_BINARY);
            
            //Dialte and erode to do some repair
            //dilate(greyRegionOfInterest, greyRegionOfInterest, getStructuringElement(MORPH_DILATE, Size(7, 7)));
            //erode(greyRegionOfInterest, greyRegionOfInterest, getStructuringElement(MORPH_ERODE, Size(1, 1)));


            /*    Subtract background from foreground here   */

            //bitwise_and(greyRegionOfInterest, regionOfBackground, regionOfMask, noArray());

            GaussianBlur(greyRegionOfInterest, greyRegionOfInterest, Size(25, 25), 0, 0);
            bitwise_not(greyRegionOfInterest, greyRegionOfInterest, greyRegionOfInterest);
            bitwise_and(greyRegionOfInterest, regionOfBackground, regionOfMask, noArray());
            GaussianBlur(regionOfMask, regionOfMask, Size(13, 13), 0, 0);
            threshold(regionOfMask, regionOfMask,50, 255, THRESH_BINARY);

            Mat bgMaskClone = regionOfMask.clone();





            //!!!!!!!!!!!!! HSV Section   !!!!!!!!!!!!!!!!!!!!!!!!!
            Mat hsvHighlight, hsvShadows;


            Mat hsvMask;
            hsvMask = greyRegionOfInterest.clone();
            Mat hsvRegionOfInterest = regionOfInterest.clone();


            //Do inRange
            //inRange(hsvRegionOfInterest, Scalar(minHueThresh, minSatThresh, minValThresh), Scalar(maxHueThresh, maxSatThresh, maxValThresh), hsvMask);
           // inRange(hsvRegionOfInterest, Scalar(0, 0, 50), Scalar(60, 250, 70), hsvMask);
            inRange(hsvRegionOfInterest, Scalar(127, 50, 170), Scalar(180, 170, 220), hsvHighlight);
            inRange(hsvRegionOfInterest, Scalar(0, 28, 65), Scalar(58, 180, 190), hsvShadows);

            bitwise_or(hsvHighlight, hsvShadows, hsvMask, noArray());

            erode(hsvMask, hsvMask, getStructuringElement(MORPH_RECT, Size(5, 5)));
            dilate(hsvMask, hsvMask, getStructuringElement(MORPH_RECT, Size(7, 7)));
            //GaussianBlur(hsvMask, hsvMask, Size(3, 3), 0, 0);
            





            //bitwise_and the bgMask and hsvmask, hopefully overlap of just hand
            bitwise_and(regionOfMask, hsvMask, regionOfMask, noArray());







            /*  Start of calculating contours found in mask  */

            vector<vector<Point>> calcContour;
            Mat maskClone = regionOfMask.clone(); //cloned to prevent destructive computations
            findContours(maskClone, calcContour, RETR_TREE, CHAIN_APPROX_NONE, Point());


            //FInd contour with the largest area, should be the hand since most proiminent in screen
            int maxId = -1;// draw accepts index of -1, which will draw all (may need to default to something else)
            double maxArea = 0;
            vector<vector<Point>> convexList(calcContour.size());
            for (size_t i = 0; i < calcContour.size(); i++)
            {
                if (contourArea(calcContour[i]) > maxArea)
                {
                    maxArea = contourArea(calcContour[i]);
                    maxId = i;
                }
                convexHull(Mat(calcContour[i]), convexList[i], false); //Assigning convex hull as well

            }

            //Only do things if we found any contour
            if (calcContour.size() != 0 && maxId != -1)
            {
                //filter out the smaller contours and those that aren't very contoured (this ignores small noises, objects, and obviously round things)
                //!!!Play around with these thresholds to gett better filtering!!!!
                if (!(calcContour[maxId].size() < CONTOUR_THRESH) && !(contourArea(calcContour[maxId]) < CONTOUR_AREA_THRESH)) 
                {
                    //Calculate the centroid of from contour data
                    //Based off instructions from OpenCV tutorials
                    Moments contourMoments = moments(calcContour[maxId]);
                    Point2f theNewCentroid(contourMoments.m10 / contourMoments.m00, contourMoments.m01 / contourMoments.m00); //This is the new position of the centroid
                    theCentroid = Point2f(contourMoments.m10 / contourMoments.m00, contourMoments.m01 / contourMoments.m00);


                    //Check that the calculated Centroid isn't a sudden Jump
                    int distance = sqrt(((theCentroid.x - theNewCentroid.x) * (theCentroid.x - theNewCentroid.x)) + ((theCentroid.y - theNewCentroid.y) * (theCentroid.y - theNewCentroid.y)));
                    if (distance > CENTROID_JUMP_MAX_THRESH) //norm(theCentroid - theNewCentroid) should give the distance, but we'll do euclidean by hand since it may be faster
                    {
                        //Too high a jump, default to current centroid position
                        centroidHistory[centroidHistoryIter] = theCentroid;
                    }
                    else if (distance > CENTROID_JUMP_THRESH)
                    {
                        //Scale the position based on distance

                        centroidHistory[centroidHistoryIter] = theNewCentroid - (theNewCentroid * ((distance - CENTROID_JUMP_THRESH) / (CENTROID_JUMP_MAX_THRESH - CENTROID_JUMP_THRESH))) + (theCentroid * ((distance - CENTROID_JUMP_THRESH) / (CENTROID_JUMP_MAX_THRESH - CENTROID_JUMP_THRESH)));
                    }
                    else
                    {
                        //reasonable position, just add it
                        centroidHistory[centroidHistoryIter] = theNewCentroid;
                    }

                    //Trying if only update after a full buffer of inputs has been made
                    if (centroidHistoryIter == 0)
                    {
                        int tempX = 0, tempY = 0;
                        for (int i = 0; i < CENTROID_HISTORY_BUFF_SIZE; i++)
                        {
                            Point2f tempPoint = centroidHistory[i];
                            tempX += tempPoint.x;
                            tempY += tempPoint.y;
                        }
                        tempX = (tempX > MAX_DETECTION_BOX_COL) ? (MAX_DETECTION_BOX_COL - 1) / CENTROID_HISTORY_BUFF_SIZE : (tempX) / CENTROID_HISTORY_BUFF_SIZE;
                        tempY = (tempY > maxFrameHeight) ? (maxFrameHeight - 1) / CENTROID_HISTORY_BUFF_SIZE : (tempY) / CENTROID_HISTORY_BUFF_SIZE;
                        //printf("Averaged Centroid  position %f %f\n \n", theNewCentroid.x, theNewCentroid.y);


                        //Also try limiting jumps here
                        distance = sqrt(((theCentroid.x - tempX) * (theCentroid.x - tempX)) + ((theCentroid.y - tempY) * (theCentroid.y - tempY)));

                        if (distance > CENTROID_JUMP_MAX_THRESH) //norm(theCentroid - theNewCentroid) should give the distance, but we'll do euclidean by hand since it may be faster
                        {
                            //Too high a jump, default to current centroid position
                            //do nothing, or not update centroid
                        }
                        else if (distance > CENTROID_JUMP_THRESH)
                        {
                            //Scale the position based on distance

                            tempX = theCentroid.x - (theCentroid.x * ((distance - CENTROID_JUMP_THRESH) / (CENTROID_JUMP_MAX_THRESH - CENTROID_JUMP_THRESH))) + (tempX * ((distance - CENTROID_JUMP_THRESH) / (CENTROID_JUMP_MAX_THRESH - CENTROID_JUMP_THRESH)));
                            tempX = theCentroid.y - (theCentroid.y * ((distance - CENTROID_JUMP_THRESH) / (CENTROID_JUMP_MAX_THRESH - CENTROID_JUMP_THRESH))) + (tempY * ((distance - CENTROID_JUMP_THRESH) / (CENTROID_JUMP_MAX_THRESH - CENTROID_JUMP_THRESH)));
                            theCentroid = Point2f(tempX, tempY); //Assigning the average of all from buffer
                        }
                        else
                        {
                            //reasonable position, just add it
                            centroidHistory[centroidHistoryIter] = theNewCentroid;
                            theCentroid = Point2f(tempX, tempY); //Assigning the average of all from buffer
                        }


                    }


                    /*Here, convert centroid position into relative positions and such, then use to convert to MIDI, then send out*/
                    if (midiActive) //Figure if should put here or another spot
                    {
                        if (theCentroid.x != 0 && theCentroid.y != 0) //if in the initial 0,0 position, do nothing, for now
                        {
                            //use a horizontal position to determine volume or to play note at all
                            //for now, just do play note at all

                            if (theCentroid.x <= MAX_DETECTION_BOX_COL / 2)
                            {
                                notePlaying = true;
                                //3/4 to the left, go ahead and play a note

                                /*For now, use stepwise height for notes, using division*/
                                /*Hard coding for now, maybe later figure how to be variable based on different divisions*/

                                //low to high = large y to lower y
                                //printf("Max height: %i, centroidY: %f \n", maxFrameHeight, theCentroid.y);
                                if (theCentroid.y >= ((float)maxFrameHeight) * (9.0f / 10.0f) && midiNote!=0x3c) //
                                {
                                    //play c4
                                    //printf("C4 note being played!\n");
                                    playMidiNote(notePlaying, midiNote, 0x3c, &midiSignal, midiOut);
                                    midiNote = 0x3c;
                                    notePlaying = true;

                                    line(frameClone, Point(0, 0), Point(MAX_DETECTION_BOX_COL/2, 0), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL/2, 0), Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (9.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL/2, ((float)maxFrameHeight) * (9.0f / 10.0f)), Point(0, ((float)maxFrameHeight) * (9.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(0, ((float)maxFrameHeight) * (9.0f / 10.0f)), Point(0, 0), highlightRect, 3);


                                }
                                else if (theCentroid.y >= ((float)maxFrameHeight) * (8.0 / 10.0) && theCentroid.y < ((float)maxFrameHeight) * (9.0 / 10.0) && midiNote != 0x3e) //
                                {
                                    //play d4
                                   // printf("d4 note being played!\n");
                                    playMidiNote(notePlaying, midiNote, 0x3e, &midiSignal, midiOut);
                                    midiNote = 0x3e;
                                    notePlaying = true;

                                    line(frameClone, Point(0, ((float)maxFrameHeight) * (9.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (9.0 / 10.0)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (9.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (8.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (8.0f / 10.0f)), Point(0, ((float)maxFrameHeight) * (8.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(0, ((float)maxFrameHeight) * (8.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (9.0 / 10.0)), highlightRect, 3);

                                }
                                else if (theCentroid.y >= ((float)maxFrameHeight) * (7.0 / 10.0) && theCentroid.y < ((float)maxFrameHeight) * (8.0 / 10.0) && midiNote != 0x41)
                                {
                                    //play f4
                                   // printf("f4 note being played!\n");
                                    playMidiNote(notePlaying, midiNote, 0x41, &midiSignal, midiOut);
                                    midiNote = 0x41;
                                    notePlaying = true;

                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (8.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (8.0 / 10.0)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (8.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (7.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (7.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (7.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (7.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (8.0 / 10.0)), highlightRect, 3);

                                }
                                else if (theCentroid.y >= ((float)maxFrameHeight) * (6.0 / 10.0) && theCentroid.y < ((float)maxFrameHeight) * (7.0 / 10.0) && midiNote != 0x43)
                                {
                                    //play g4
                                   // printf("g4 note being played!\n");
                                    playMidiNote(notePlaying, midiNote, 0x43, &midiSignal, midiOut);
                                    midiNote = 0x43;
                                    notePlaying = true;

                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (7.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (7.0 / 10.0)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (7.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (6.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (6.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (6.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (6.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (7.0 / 10.0)), highlightRect, 3);

                                }
                                else if (theCentroid.y >= ((float)maxFrameHeight) * (5.0 / 10.0) && theCentroid.y < ((float)maxFrameHeight) * (6.0 / 10.0) && midiNote != 0x45)
                                {
                                    //play a4
                                   // printf("a4 note being played!\n");

                                    playMidiNote(notePlaying, midiNote, 0x45, &midiSignal, midiOut);
                                    midiNote = 0x45;
                                    notePlaying = true;

                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (6.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (6.0 / 10.0)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (6.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (5.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (5.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (5.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (5.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (6.0 / 10.0)), highlightRect, 3);

                                }
                                else if (theCentroid.y >= ((float)maxFrameHeight) * (4.0 / 10.0) && theCentroid.y < ((float)maxFrameHeight) * (5.0 / 10.0) && midiNote != 0x48)
                                {

                                    //play C5
                                   // printf("C5 note being played!\n");
                                    playMidiNote(notePlaying, midiNote, 0x48, &midiSignal, midiOut);
                                    midiNote = 0x48;
                                    notePlaying = true;

                                    line(frameClone, Point(0, ((float)maxFrameHeight) * (5.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (5.0 / 10.0)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (5.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (4.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (4.0f / 10.0f)), Point(0, ((float)maxFrameHeight) * (4.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(0, ((float)maxFrameHeight) * (4.0f / 10.0f)), Point(0, ((float)maxFrameHeight) * (5.0 / 10.0)), highlightRect, 3);

                                }
                                else if (theCentroid.y >= ((float)maxFrameHeight) * (3.0 / 10.0) && theCentroid.y < ((float)maxFrameHeight) * (4.0 / 10.0) && midiNote != 0x4a)
                                {
                                    //play d5
                                   // printf("d5 note being played!\n");
                                    playMidiNote(notePlaying, midiNote, 0x4a, &midiSignal, midiOut);
                                    midiNote = 0x4a;
                                    notePlaying = true;

                                    line(frameClone, Point(0, ((float)maxFrameHeight) * (4.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (4.0 / 10.0)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (4.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (3.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight) * (3.0f / 10.0f)), Point(0, ((float)maxFrameHeight) * (3.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(0, ((float)maxFrameHeight) * (3.0f / 10.0f)), Point(0, ((float)maxFrameHeight) * (4.0 / 10.0)), highlightRect, 3);
                                }
                                else if (theCentroid.y >= ((float)maxFrameHeight) * (2.0 / 10.0) && theCentroid.y < ((float)maxFrameHeight) * (3.0 / 10.0) && midiNote != 0x4d)
                                {
                                    //play f5
                                    //printf("f5 note being played!\n");
                                    playMidiNote(notePlaying, midiNote, 0x4d, &midiSignal, midiOut);
                                    midiNote = 0x4d;
                                    notePlaying = true;

                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (3.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (3.0 / 10.0)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (3.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (2.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (2.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (2.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (2.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (3.0 / 10.0)), highlightRect, 3);
                                }
                                else if (theCentroid.y >= (float)maxFrameHeight * (1.0 / 10.0) && theCentroid.y < (float)maxFrameHeight * (2.0 / 10.0) && midiNote != 0x4f)
                                {
                                    //play g5
                                    //printf("g5 note being played!\n");
                                    playMidiNote(notePlaying, midiNote, 0x4f, &midiSignal, midiOut);
                                    midiNote = 0x4f;
                                    notePlaying = true;
                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (2.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (2.0 / 10.0)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (2.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (1.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (1.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (1.0f / 10.0f)), highlightRect, 3);
                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (1.0f / 10.0f)), Point(0, ((float)maxFrameHeight)* (2.0 / 10.0)), highlightRect, 3);
                                }
                                else if (theCentroid.y < (float)maxFrameHeight * (1.0 / 10.0) && midiNote != 0x51)
                                {
                                    //play a5
                                    //printf("a5 note being played!\n");
                                    playMidiNote(notePlaying, midiNote, 0x51, &midiSignal, midiOut);
                                    midiNote = 0x51;
                                    notePlaying = true;

                                    line(frameClone, Point(0, ((float)maxFrameHeight)* (1.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* 1.0 / 10.0), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, ((float)maxFrameHeight)* (1.0 / 10.0)), Point(MAX_DETECTION_BOX_COL / 2,0), highlightRect, 3);
                                    line(frameClone, Point(MAX_DETECTION_BOX_COL / 2, 0), Point(0, 0), highlightRect, 3);
                                    line(frameClone, Point(0, 0), Point(0, ((float)maxFrameHeight)* (1.0 / 10.0)), highlightRect, 3);
                                }
                                else
                                {
                                    //somehow negative
                                    printf("negtaive y! too high a note!\n");

                                }


                            }
                            else
                            {
                                notePlaying = false;
                               // printf("Note not Playing... \n");

                                /*
                                //cut off previous note
                                midiSignal[0] = 0x80;

                                midiSignal[1] = midiNote;
                                //midiSignal[1] = 67;

                                midiSignal[2] = 0x64;
                                midiOut->sendMessage(&midiSignal);
                                */

                                //Using control change 'All notes off'
                                midiSignal[0] = 0xb0;
                                midiSignal[1] = 0x7b;
                                midiSignal[2] = 0x00; //Constant velocity for now

                                //dummy midi note value set
                                midiNote = 0;

                                midiOut->sendMessage(&midiSignal);
                            }
                        }
                    }


                    drawContours(frameClone, calcContour, maxId, contourColor);
                    drawContours(frameClone, convexList, maxId, boundingBoxColor);
                    circle(frameClone, theCentroid, 5, rectColor, 3, 8, 0);

                }

                
            }
            imshow("Output", frameClone);
            imshow("Mask", regionOfMask);
            imshow("bgMask", bgMaskClone);
            imshow("MaskHSV", hsvMask);


            waitKey(30);
        }
        else //Hand tracking not active, just play videos
        {
            //Just show video
            imshow("Output", frameClone);
            if (waitKey(10) == 27) break; // stop capturing by pressing ESC 
        }



    }
    // the camera will be closed automatically upon exit
    // cap.close();
    
    //clean up any allocated data. As of now, just the Midi device
    delete midiOut;

    return 0;
}

