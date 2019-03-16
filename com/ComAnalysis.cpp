#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <math.h>
#include <opencv2/opencv.hpp>

// COM location for each limb
#define SHIN_DISTAL 0.567
#define THIGH_DISTAL 0.433
#define TORSO_DISTAL 0.650
#define BICEP_DISTAL 0.564
#define FOREARM_DISTAL 0.682

// COM contribution of each body part
#define SHIN_COM 0.061
#define THIGH_COM 0.100
#define PELVIS_COM 0.142
#define TORSO_COM 0.355
#define BICEP_COM 0.028
#define FOREARM_COM 0.022
#define HEAD_COM 0.081

// Frame rate of video file
#define FRAME_RATE 60.0

// Frame jump for acceleration smoothing
#define VEL_JUMP 10
#define FRAME_JUMP 3

// Speed ranges for detecting athlete
#define SPEED_LOW -75
#define SPEED_HIGH -125

#define VELOCITY_DELAY 5

using namespace std;

class JointCoord
{
  private:
    double x;
    double y;
    double c;

  public:
    JointCoord()
    {
        x = 0;
        y = 0;
        c = 0;
    }

    void set_x(double X)
    {
        x = X;
    }

    void set_y(double Y)
    {
        y = Y;
    }

    void set_c(double C)
    {
        c = C;
    }

    double get_x()
    {
        return x;
    }

    double get_y()
    {
        return y;
    }

    double get_c()
    {
        return c;
    }
};

class LimbCoord
{
  private:
    // Low and high positions of joint, ex. rWrist and rElbow respectively for
    // rLowArm
    double lx;
    double ly;
    double hx;
    double hy;
    double comx;
    double comy;

  public:
    LimbCoord()
    {
        lx = 0;
        ly = 0;
        hx = 0;
        hy = 0;
        comx = 0;
        comy = 0;
    }

    LimbCoord(double LX, double LY, double HX, double HY)
    {
        lx = LX;
        ly = LY;
        hx = HX;
        hy = HY;
        comx = 0;
        comy = 0;
    }

    void set_lx(double LX)
    {
        lx = LX;
    }

    void set_hx(double HX)
    {
        hx = HX;
    }

    void set_ly(double LY)
    {
        ly = LY;
    }

    void set_hy(double HY)
    {
        hy = HY;
    }

    void set_comx(double COMX)
    {
        comx = COMX;
    }

    void set_comy(double COMY)
    {
        comy = COMY;
    }

    double get_lx()
    {
        return lx;
    }

    double get_hx()
    {
        return hx;
    }

    double get_ly()
    {
        return ly;
    }

    double get_hy()
    {
        return hy;
    }

    double get_comx()
    {
        return comx;
    }

    double get_comy()
    {
        return comy;
    }
};

class comData
{
  private:
    double x;
    double y;
    double velx;
    double vely;
    double accelx;
    double accely;
    double filtered_x;
    double filtered_y;

  public:
    comData()
    {
        x = 0;
        y = 0;
        velx = 0;
        vely = 0;
        accelx = 0;
        accely = 0;
        filtered_x = 0;
        filtered_y = 0;
    }

    void set_x(double X)
    {
        x = X;
    }

    void set_y(double Y)
    {
        y = Y;
    }

    void set_filtered_x(double X)
    {
        filtered_x = X;
    }

    void set_filtered_y(double Y)
    {
        filtered_y = Y;
    }

    void set_velx(double velX)
    {
        velx = velX;
    }

    void set_vely(double velY)
    {
        vely = velY;
    }

    void set_accelx(double accelX)
    {
        accelx = accelX;
    }

    void set_accely(double accelY)
    {
        accely = accelY;
    }

    double get_x()
    {
        return x;
    }

    double get_y()
    {
        return y;
    }

    double get_velx()
    {
        return velx;
    }

    double get_vely()
    {
        return vely;
    }

    double get_accelx()
    {
        return accelx;
    }

    double get_accely()
    {
        return accely;
    }
};

void get_joint_pose_data(string::iterator &it, vector<JointCoord> &joint)
{

    string curX;
    string curY;
    string curC;
    JointCoord curJoint;

    // Get pose data
    while (*it != ',')
    {
        curX += *it;
        it++;
    }
    it++;
    while (*it != ',')
    {
        curY += *it;
        it++;
    }
    it++;
    while (*it != ',')
    {
        curC += *it;
        it++;
    }
    it++;
    curJoint.set_x(stof(curX));
    curJoint.set_y(stof(curY));
    curJoint.set_c(stof(curC));
    joint.push_back(curJoint);
}

void get_frame_pose_data(
    vector<JointCoord> &nose,
    vector<JointCoord> &neck,
    vector<JointCoord> &rShoulder,
    vector<JointCoord> &rElbow,
    vector<JointCoord> &rWrist,
    vector<JointCoord> &lShoulder,
    vector<JointCoord> &lElbow,
    vector<JointCoord> &lWrist,
    vector<JointCoord> &midHip,
    vector<JointCoord> &rHip,
    vector<JointCoord> &rKnee,
    vector<JointCoord> &rAnkle,
    vector<JointCoord> &lHip,
    vector<JointCoord> &lKnee,
    vector<JointCoord> &lAnkle,
    vector<JointCoord> &rEye,
    vector<JointCoord> &lEye,
    vector<JointCoord> &rEar,
    vector<JointCoord> &lEar,
    vector<JointCoord> &lBigToe,
    vector<JointCoord> &lSmallToe,
    vector<JointCoord> &lHeel,
    vector<JointCoord> &rBigToe,
    vector<JointCoord> &rSmallToe,
    vector<JointCoord> &rHeel,
    string jsonData,
    int peopleNum)
{

    if (peopleNum == 1)
    {

        // Iterate to first number
        string::iterator it = jsonData.begin();
        int count = 0;
        while (count < 2)
        {
            if (*it == '[')
            {
                count++;
            }
            it++;
        }

        // Get pose data for each joint
        get_joint_pose_data(it, nose);
        get_joint_pose_data(it, neck);
        get_joint_pose_data(it, rShoulder);
        get_joint_pose_data(it, rElbow);
        get_joint_pose_data(it, rWrist);
        get_joint_pose_data(it, lShoulder);
        get_joint_pose_data(it, lElbow);
        get_joint_pose_data(it, lWrist);
        get_joint_pose_data(it, midHip);
        get_joint_pose_data(it, rHip);
        get_joint_pose_data(it, rKnee);
        get_joint_pose_data(it, rAnkle);
        get_joint_pose_data(it, lHip);
        get_joint_pose_data(it, lKnee);
        get_joint_pose_data(it, lAnkle);
        get_joint_pose_data(it, rEye);
        get_joint_pose_data(it, lEye);
        get_joint_pose_data(it, rEar);
        get_joint_pose_data(it, lEar);
        get_joint_pose_data(it, lBigToe);
        get_joint_pose_data(it, lSmallToe);
        get_joint_pose_data(it, lHeel);
        get_joint_pose_data(it, rBigToe);
        get_joint_pose_data(it, rSmallToe);
        get_joint_pose_data(it, rHeel);
    }
    else
    {
        // Set everything to zero
        JointCoord zeros;
        nose.push_back(zeros);
        neck.push_back(zeros);
        rShoulder.push_back(zeros);
        rElbow.push_back(zeros);
        rWrist.push_back(zeros);
        lShoulder.push_back(zeros);
        lElbow.push_back(zeros);
        lWrist.push_back(zeros);
        midHip.push_back(zeros);
        rHip.push_back(zeros);
        rKnee.push_back(zeros);
        rAnkle.push_back(zeros);
        lHip.push_back(zeros);
        lKnee.push_back(zeros);
        lAnkle.push_back(zeros);
        rEye.push_back(zeros);
        lEye.push_back(zeros);
        rEar.push_back(zeros);
        lEar.push_back(zeros);
        lBigToe.push_back(zeros);
        lSmallToe.push_back(zeros);
        lHeel.push_back(zeros);
        rBigToe.push_back(zeros);
        rSmallToe.push_back(zeros);
        rHeel.push_back(zeros);
    }
}

void approx_missing_data(vector<JointCoord> &joint)
{

    int lastFrame = joint.size();

    // Iterate to first data point for x
    int i = 0;
    while (joint[i].get_x() == 0 && i < lastFrame)
    {
        i++;
    }

    // Approximate all missing points for x
    while (i < lastFrame)
    {

        // Find the first area with missing data
        while (joint[i].get_x() != 0 && i < lastFrame)
        {
            i++;
        }
        int j = i - 1;

        if (i >= lastFrame)
            break;

        // Find other side with data
        while (joint[i].get_x() == 0 && i < lastFrame)
        {
            i++;
        }

        if (i >= lastFrame)
            break;

        // Make approximations
        double difference = joint[i].get_x() - joint[j].get_x();
        int numFrames = i - j;
        double incrementAmt = difference / (double)numFrames;
        int incrementNum = 1;
        for (int k = j + 1; k < i; k++)
        {
            joint[k].set_x(joint[j].get_x() + (incrementAmt * incrementNum));
            incrementNum++;
        }
    }

    // Iterate to first data point for y
    i = 0;
    while (joint[i].get_y() == 0 && i < lastFrame)
    {
        i++;
    }

    // Approximate all missing points for y
    while (i < lastFrame)
    {

        // Find the first area with missing data
        while (joint[i].get_y() != 0 && i < lastFrame)
        {
            i++;
        }
        int j = i - 1;

        if (i >= lastFrame)
            break;

        // Find other side with data
        while (joint[i].get_y() == 0 && i < lastFrame)
        {
            i++;
        }

        if (i >= lastFrame)
            break;

        // Make approximations
        double difference = joint[i].get_y() - joint[j].get_y();
        int numFrames = i - j;
        double incrementAmt = difference / (double)numFrames;
        int incrementNum = 1;
        for (int k = j + 1; k < i; k++)
        {
            joint[k].set_y(joint[j].get_y() + (incrementAmt * incrementNum));
            incrementNum++;
        }
    }
}

void joint_creation(vector<JointCoord> &lJoint, vector<JointCoord> &hJoint,
                    vector<LimbCoord> &curLimb)
{

    for (int i = 0; i < lJoint.size(); i++)
    {
        LimbCoord limb(lJoint[i].get_x(), lJoint[i].get_y(),
                       hJoint[i].get_x(), hJoint[i].get_y());
        curLimb.push_back(limb);
    }
}

void limb_com(vector<LimbCoord> &curLimb, double distal)
{

    for (int i = 0; i < curLimb.size(); i++)
    {
        curLimb[i].set_comx(curLimb[i].get_lx() * distal +
                            curLimb[i].get_hx() * (1 - distal));
        curLimb[i].set_comy(curLimb[i].get_ly() * distal +
                            curLimb[i].get_hy() * (1 - distal));
    }
}

void limb_com_pelvis(vector<LimbCoord> &rPelvis)
{

    for (int i = 0; i < rPelvis.size(); i++)
    {
        rPelvis[i].set_comx(rPelvis[i].get_hx());
        rPelvis[i].set_comy(rPelvis[i].get_hy());
    }
}

void limb_com_head(vector<LimbCoord> &head)
{

    for (int i = 0; i < head.size(); i++)
    {
        head[i].set_comx(head[i].get_hx());
        head[i].set_comy(head[i].get_hy());
    }
}

void com_calc(
    vector<LimbCoord> &rLowLeg,
    vector<LimbCoord> &lLowLeg,
    vector<LimbCoord> &rUpLeg,
    vector<LimbCoord> &lUpLeg,
    vector<LimbCoord> &torso,
    vector<LimbCoord> &rUpArm,
    vector<LimbCoord> &lUpArm,
    vector<LimbCoord> &rLowArm,
    vector<LimbCoord> &lLowArm,
    vector<LimbCoord> &rPelvis,
    vector<LimbCoord> &head,
    vector<comData> &com)
{

    for (int i = 0; i < head.size(); i++)
    {
        comData curCOM;
        curCOM.set_x(rLowLeg[i].get_comx() * SHIN_COM +
                     lLowLeg[i].get_comx() * SHIN_COM +
                     rUpLeg[i].get_comx() * THIGH_COM +
                     lUpLeg[i].get_comx() * THIGH_COM +
                     rPelvis[i].get_comx() * PELVIS_COM +
                     torso[i].get_comx() * TORSO_COM +
                     rUpArm[i].get_comx() * BICEP_COM +
                     lUpArm[i].get_comx() * BICEP_COM +
                     rLowArm[i].get_comx() * FOREARM_COM +
                     lLowArm[i].get_comx() * FOREARM_COM +
                     head[i].get_comx() * HEAD_COM);
        curCOM.set_y(rLowLeg[i].get_comy() * SHIN_COM +
                     lLowLeg[i].get_comy() * SHIN_COM +
                     rUpLeg[i].get_comy() * THIGH_COM +
                     lUpLeg[i].get_comy() * THIGH_COM +
                     rPelvis[i].get_comy() * PELVIS_COM +
                     torso[i].get_comy() * TORSO_COM +
                     rUpArm[i].get_comy() * BICEP_COM +
                     lUpArm[i].get_comy() * BICEP_COM +
                     rLowArm[i].get_comy() * FOREARM_COM +
                     lLowArm[i].get_comy() * FOREARM_COM +
                     head[i].get_comy() * HEAD_COM);
        com.push_back(curCOM);
    }
}

void moving_average_filtered_com(vector<comData> &com)
{
    double avrg_arr[5] = {0, 0, 0, 0, 0};
    for (int i = 0; i < (com.size() - 5); i++)
    {
        avrg_arr[0] = com[i].get_y();
        avrg_arr[1] = com[i + 1].get_y();
        avrg_arr[2] = com[i + 2].get_y();
        avrg_arr[3] = com[i + 3].get_y();
        avrg_arr[4] = com[i + 4].get_y();
        int sum = 0;
        for (int j = 0; j < 5; j++)
        {
            sum += avrg_arr[j];
        }
        double avrg = sum / 5;
        com[i].set_x(com[i].get_x());
        com[i].set_y((double)avrg);
        /*double distx = com[i].get_x() - com[i - VEL_JUMP].get_x();
        double disty = com[i].get_y() - com[i - VEL_JUMP].get_y();*/
        //com[i].set_velx(distx / (100.0 / (double)FRAME_RATE));
        //com[i].set_vely(disty / (100.0 / (double)FRAME_RATE));
    }
}

void com_lpf(vector<comData> &com)
{
    for (int i = 0; i < com.size(); i++)
    {
        double current = com[i].get_y();
        double prev = com[i - 1].get_y();
        if (i > 0 && (current > (prev + 100.0)))
        {
            com[i].set_y(((com[i + 10].get_y() + com[i - 1].get_y()) / 2));
        }
        if (i > 0 && (com[i].get_x() > (com[i].get_x() + 100.0)))
        {
            com[i].set_x(((com[i + 3].get_x() + com[i - 1].get_x()) / 2));
        }
        /*double distx = com[i].get_x() - com[i - VEL_JUMP].get_x();
        double disty = com[i].get_y() - com[i - VEL_JUMP].get_y();*/
        //com[i].set_velx(distx / (100.0 / (double)FRAME_RATE));
        //com[i].set_vely(disty / (100.0 / (double)FRAME_RATE));
    }
}

void stablize_first_frames(vector<comData> &com, int frames)
{
    double stable_x = com[frames].get_x();
    double stable_y = com[frames].get_y();
    for (int i = 0; i < frames; i++)
    {
        com[i].set_x(stable_x);
        com[i].set_y(stable_y);
    }
}

void com_vel(vector<comData> &com)
{
    for (int i = VEL_JUMP; i < com.size(); i++)
    {
        double distx = com[i].get_x() - com[i - VEL_JUMP].get_x();
        double disty = com[i].get_y() - com[i - VEL_JUMP].get_y();
        com[i].set_velx(distx / (50.0 / (double)FRAME_RATE));
        com[i].set_vely(disty / (50.0 / (double)FRAME_RATE));
    }
}

void com_accel(vector<comData> &com)
{
    /*for (int i = FRAME_JUMP; i < com.size(); i = i + FRAME_JUMP)
    {
        double deltax = com[i].get_velx() - com[i - FRAME_JUMP].get_velx();
        double deltay = com[i].get_vely() - com[i - FRAME_JUMP].get_vely();
        com[i].set_accelx(deltax / (50.0 / (double)FRAME_RATE));
        com[i].set_accely(deltay / (50.0 / (double)FRAME_RATE));
    }*/
    for (int i = FRAME_JUMP; i < com.size(); i = i + FRAME_JUMP)
    {
        double deltax = com[i].get_velx() - com[i - FRAME_JUMP].get_velx();
        double deltay = com[i].get_vely() - com[i - FRAME_JUMP].get_vely();
        com[i].set_accelx(deltax / (25.0 / (double)FRAME_RATE));
        com[i].set_accely(deltay / (25.0 / (double)FRAME_RATE));
    }
}

/*
 * To-do:
 * 
 * 
 * COM velocity and acceleration
 * drawing of everything
 * 
 * tweak take off point
 * com height percentage
 * 
 * Take off angle (detect take off point)
 * draw arrows for aceleration
 * pause video for a second at take off
 * save to a new video
 */
int main(int argc, char **argv)
{

    // Set up strings for dynamic file retrieval
    //const string prefix = "/Users/DavidChen/Desktop/output/120fps_";
    const string prefix = "/home/james/ece496/openpose/output/120fps_";
    const string suffix = "_keypoints.json";
    stringstream ss;
    string fileName;
    ifstream inFile;
    string jsonData;
    int curFrame = 0;

    /*
     * Vectors for storing pose data for each point
     * Vector data is coordinate object
     * Vector length is the number of frames recorded
     */
    vector<JointCoord> nose;      // 0
    vector<JointCoord> neck;      // 1
    vector<JointCoord> rShoulder; // 2
    vector<JointCoord> rElbow;    // 3
    vector<JointCoord> rWrist;    // 4
    vector<JointCoord> lShoulder; // 5
    vector<JointCoord> lElbow;    // 6
    vector<JointCoord> lWrist;    // 7
    vector<JointCoord> midHip;    // 8
    vector<JointCoord> rHip;      // 9
    vector<JointCoord> rKnee;     // 10
    vector<JointCoord> rAnkle;    // 11
    vector<JointCoord> lHip;      // 12
    vector<JointCoord> lKnee;     // 13
    vector<JointCoord> lAnkle;    // 14
    vector<JointCoord> rEye;      // 15
    vector<JointCoord> lEye;      // 16
    vector<JointCoord> rEar;      // 17
    vector<JointCoord> lEar;      // 18
    vector<JointCoord> lBigToe;   // 19
    vector<JointCoord> lSmallToe; // 20
    vector<JointCoord> lHeel;     // 21
    vector<JointCoord> rBigToe;   // 22
    vector<JointCoord> rSmallToe; // 23
    vector<JointCoord> rHeel;     // 24

    // Get first frame
    ss << setw(12) << setfill('0') << curFrame;
    fileName = prefix + ss.str() + suffix;
    inFile.open(fileName, ios::in);

    // Continue for each frame
    while (inFile)
    {
        //while (curFrame < 44) {

        // Get frame json data
        getline(inFile, jsonData);
        //cout << "getting JSON data for frame " << curFrame << endl;

        // Check how many people have been detected
        int peopleNum = 0;
        for (string::iterator it = jsonData.begin(); it != jsonData.end();
             it++)
        {
            if (*it == '{')
            {
                peopleNum++;
            }
        }
        peopleNum--; //number of people is 1 less than number of left brackets
        //cout << "people detected: " << peopleNum << endl;

        // Debugging purposes
        //if (peopleNum == 1) {
        //    cout << jsonData << endl;
        //}

        // Parse pose data into vectors
        get_frame_pose_data(nose, neck, rShoulder, rElbow, rWrist, lShoulder,
                            lElbow, lWrist, midHip, rHip, rKnee, rAnkle, lHip, lKnee,
                            lAnkle, rEye, lEye, rEar, lEar, lBigToe, lSmallToe, lHeel,
                            rBigToe, rSmallToe, rHeel, jsonData, peopleNum);

        // Get next frame
        inFile.close();
        ss.str(string());
        curFrame++;
        ss << setw(12) << setfill('0') << curFrame;
        fileName = prefix + ss.str() + suffix;
        inFile.open(fileName, ios::in);
    }

    //cout << "done" << endl;

    //cout << nose.size() << endl;
    //cout << neck.size() << endl;
    //cout << rHeel.size() << endl;

    //        for (int i = 0; i < neck.size(); i++) {
    //            cout << neck[i].get_x() << ", " << neck[i].get_y() << endl;
    //        }

    // Fill in missing point frames
    approx_missing_data(nose);      // 0
    approx_missing_data(neck);      // 1
    approx_missing_data(rShoulder); // 2
    approx_missing_data(rElbow);    // 3
    approx_missing_data(rWrist);    // 4
    approx_missing_data(lShoulder); // 5
    approx_missing_data(lElbow);    // 6
    approx_missing_data(lWrist);    // 7
    approx_missing_data(midHip);    // 8
    approx_missing_data(rHip);      // 9
    approx_missing_data(rKnee);     // 10
    approx_missing_data(rAnkle);    // 11
    approx_missing_data(lHip);      // 12
    approx_missing_data(lKnee);     // 13
    approx_missing_data(lAnkle);    // 14
    approx_missing_data(rEye);      // 15
    approx_missing_data(lEye);      // 16
    approx_missing_data(rEar);      // 17
    approx_missing_data(lEar);      // 18
    approx_missing_data(lBigToe);   // 19
    approx_missing_data(lSmallToe); // 20
    approx_missing_data(lHeel);     // 21
    approx_missing_data(rBigToe);   // 22
    approx_missing_data(rSmallToe); // 23
    approx_missing_data(rHeel);     // 24

    //    for (int i = 0; i < neck.size(); i++) {
    //        cout << neck[i].get_x() << ", " << neck[i].get_y() << endl;
    //    }

    // Create limbs
    vector<LimbCoord> head;
    vector<LimbCoord> rCollar;
    vector<LimbCoord> rUpArm;
    vector<LimbCoord> rLowArm;
    vector<LimbCoord> lCollar;
    vector<LimbCoord> lUpArm;
    vector<LimbCoord> lLowArm;
    vector<LimbCoord> torso;
    vector<LimbCoord> rPelvis;
    vector<LimbCoord> rUpLeg;
    vector<LimbCoord> rLowLeg;
    vector<LimbCoord> lPelvis;
    vector<LimbCoord> lUpLeg;
    vector<LimbCoord> lLowLeg;

    // Get limb values from joints
    joint_creation(neck, nose, head);
    joint_creation(rShoulder, neck, rCollar);
    joint_creation(rElbow, rShoulder, rUpArm);
    joint_creation(rWrist, rElbow, rLowArm);
    joint_creation(lShoulder, neck, lCollar);
    joint_creation(lElbow, lShoulder, lUpArm);
    joint_creation(lWrist, lElbow, lLowArm);
    joint_creation(midHip, neck, torso);
    joint_creation(rHip, midHip, rPelvis);
    joint_creation(rKnee, rHip, rUpLeg);
    joint_creation(rAnkle, rKnee, rLowLeg);
    joint_creation(lHip, midHip, lPelvis);
    joint_creation(lKnee, lHip, lUpLeg);
    joint_creation(lAnkle, lKnee, lLowLeg);

    //    for (int i = 0; i < head.size(); i++) {
    //        cout << head[i].get_lx() << ", " << head[i].get_ly() << ", " <<
    //                head[i].get_hx() << ", " << head[i].get_hy() << endl;
    //    }

    // Get COM positions for applicable joints
    limb_com(rLowLeg, SHIN_DISTAL);
    limb_com(lLowLeg, SHIN_DISTAL);
    limb_com(rUpLeg, THIGH_DISTAL);
    limb_com(lUpLeg, THIGH_DISTAL);
    limb_com(torso, TORSO_DISTAL);
    limb_com(rUpArm, BICEP_DISTAL);
    limb_com(lUpArm, BICEP_DISTAL);
    limb_com(rLowArm, FOREARM_DISTAL);
    limb_com(lLowArm, FOREARM_DISTAL);
    limb_com_pelvis(rPelvis);
    limb_com_head(head);

    //        for (int i = 0; i < rLowLeg.size(); i++) {
    //            cout << rLowLeg[i].get_comx() << ", " << rLowLeg[i].get_comy() << endl;
    //        }

    // Calculate total COM value
    vector<comData> com;
    com_calc(rLowLeg, lLowLeg, rUpLeg, lUpLeg, torso, rUpArm,
             lUpArm, rLowArm, lLowArm, rPelvis, head, com);

    cout << com.size() << endl;

    //stablize_first_frames(com, 75);

    //using moving average filter to smooth data
    moving_average_filtered_com(com);
    //com low pass filter for erroneous y values
    //com_lpf(com);
    // Get COM velocity and acceleration
    com_vel(com);
    com_accel(com);
    double maxX = 0;
    double maxY = 0;

    // Get begin point, takeoff point, and end point
    bool begin = false;
    int frameBegin;
    int frameEnd;
    bool takeOff = false;
    int takeOffFrame;
    for (int i = 0; i < com.size(); i++)
    {
        // Detecs when athlete enters the frame
        if (com[i].get_velx() < SPEED_LOW &&
            com[i + 1].get_velx() < SPEED_LOW &&
            com[i + 2].get_velx() < SPEED_LOW &&
            com[i + 3].get_velx() < SPEED_LOW &&
            com[i + 4].get_velx() < SPEED_LOW &&
            com[i].get_velx() > SPEED_HIGH &&
            com[i + 1].get_velx() > SPEED_HIGH &&
            com[i + 2].get_velx() > SPEED_HIGH &&
            com[i + 3].get_velx() > SPEED_HIGH &&
            com[i + 4].get_velx() > SPEED_HIGH &&
            begin == false)
        {
            begin = true;
            // Sets frame to begin drawing from point array, and aligns to frame jump
            frameBegin = i - (i % FRAME_JUMP);
            cout << "FRAMEBEGIN " << frameBegin << endl;
        }

        // Detects take off point
        if (com[i].get_accelx() > 0 &&
            com[i + FRAME_JUMP].get_accelx() > 0 &&
            //com[i + 2 * FRAME_JUMP].get_accelx() > 0 &&
            com[i].get_accely() < -30 &&
            com[i + FRAME_JUMP].get_accely() < -30 &&
            //com[i + 2 * FRAME_JUMP].get_accely() < -30 &&

            takeOff == false)
        {
            takeOff = true;
            takeOffFrame = i - (i % FRAME_JUMP);
            cout << "takeoff " << takeOffFrame << endl;
            frameEnd = takeOffFrame + 21;
        }
    }

    // PRINT VELOCITY VALUES FOR DEBUGGING
    for (int i = 0; i < com.size(); i++)
    {
        //cout << com[i].get_x() << ", " << com[i].get_y() << endl;
        //cout << com[i].get_velx() << ", " << com[i].get_vely() << endl;
        cout << com[i].get_accelx() << ", " << com[i].get_accely() << endl;

        if (com[i].get_accelx() > maxX)
        {
            maxX = com[i].get_accelx();
        }
        if (com[i].get_accely() > maxY)
        {
            maxY = com[i].get_accely();
        }
    }

    cout << "MAX X: " << maxX << endl;
    cout << "MAX Y: " << maxY << endl;

    // Open video for drawing
    //cv::VideoCapture video("/Users/DavidChen/Desktop/output/result.avi");
    cv::VideoCapture video("/home/james/ece496/openpose/input/120fps.mp4");
    cv::Mat frame;
    // Point arrays for persistant drawing
    vector<cv::Point2d> pointarray;
    vector<cv::Point2d> pointarray2;

    cv::Mat CurrentMat_RGB;
    cv::cvtColor(frame, CurrentMat_RGB, CV_GRAY2RGB);
    int i = 0;
    while (video.read(frame))
    {
        cv::Point2d point;
        cv::Point2d point2;

        if (i < frameEnd)
        {
            point.x = (double)com[i].get_x();
            point.y = (double)com[i].get_y();
            point2.x = point.x + (double)com[i].get_accelx();
            point2.y = point.y + (double)com[i].get_accely();
            pointarray.push_back(point);
            pointarray2.push_back(point2);
        }

        if (i > frameBegin)
        {
            //draw skeleton trace
            if(i < frameEnd){
                //draw head
                cv::Point2d skeleton_pt_l, skeleton_pt_h;
                skeleton_pt_l.x = (double)head[i].get_lx();
                skeleton_pt_l.y = (double)head[i].get_ly();
                skeleton_pt_h.x = (double)head[i].get_hx();
                skeleton_pt_h.y = (double)head[i].get_hy();
                cv::circle(frame, point, 5, (255, 0, 0), -1);
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw rcollar
                skeleton_pt_l.x = (double)rCollar[i].get_lx();
                skeleton_pt_l.y = (double)rCollar[i].get_ly();
                skeleton_pt_h.x = (double)rCollar[i].get_hx();
                skeleton_pt_h.y = (double)rCollar[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw rUpArm
                skeleton_pt_l.x = (double)rUpArm[i].get_lx();
                skeleton_pt_l.y = (double)rUpArm[i].get_ly();
                skeleton_pt_h.x = (double)rUpArm[i].get_hx();
                skeleton_pt_h.y = (double)rUpArm[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw rLowArm
                skeleton_pt_l.x = (double)rLowArm[i].get_lx();
                skeleton_pt_l.y = (double)rLowArm[i].get_ly();
                skeleton_pt_h.x = (double)rLowArm[i].get_hx();
                skeleton_pt_h.y = (double)rLowArm[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw lCollar
                skeleton_pt_l.x = (double)lCollar[i].get_lx();
                skeleton_pt_l.y = (double)lCollar[i].get_ly();
                skeleton_pt_h.x = (double)lCollar[i].get_hx();
                skeleton_pt_h.y = (double)lCollar[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw lUpArm
                skeleton_pt_l.x = (double)lUpArm[i].get_lx();
                skeleton_pt_l.y = (double)lUpArm[i].get_ly();
                skeleton_pt_h.x = (double)lUpArm[i].get_hx();
                skeleton_pt_h.y = (double)lUpArm[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw lLowArm
                skeleton_pt_l.x = (double)lLowArm[i].get_lx();
                skeleton_pt_l.y = (double)lLowArm[i].get_ly();
                skeleton_pt_h.x = (double)lLowArm[i].get_hx();
                skeleton_pt_h.y = (double)lLowArm[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw torso
                skeleton_pt_l.x = (double)torso[i].get_lx();
                skeleton_pt_l.y = (double)torso[i].get_ly();
                skeleton_pt_h.x = (double)torso[i].get_hx();
                skeleton_pt_h.y = (double)torso[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw rPelvis
                skeleton_pt_l.x = (double)rPelvis[i].get_lx();
                skeleton_pt_l.y = (double)rPelvis[i].get_ly();
                skeleton_pt_h.x = (double)rPelvis[i].get_hx();
                skeleton_pt_h.y = (double)rPelvis[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw rUpLeg
                skeleton_pt_l.x = (double)rUpLeg[i].get_lx();
                skeleton_pt_l.y = (double)rUpLeg[i].get_ly();
                skeleton_pt_h.x = (double)rUpLeg[i].get_hx();
                skeleton_pt_h.y = (double)rUpLeg[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw rLowLeg
                skeleton_pt_l.x = (double)rLowLeg[i].get_lx();
                skeleton_pt_l.y = (double)rLowLeg[i].get_ly();
                skeleton_pt_h.x = (double)rLowLeg[i].get_hx();
                skeleton_pt_h.y = (double)rLowLeg[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw lPelvis
                skeleton_pt_l.x = (double)lPelvis[i].get_lx();
                skeleton_pt_l.y = (double)lPelvis[i].get_ly();
                skeleton_pt_h.x = (double)lPelvis[i].get_hx();
                skeleton_pt_h.y = (double)lPelvis[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw lUpLeg
                skeleton_pt_l.x = (double)lUpLeg[i].get_lx();
                skeleton_pt_l.y = (double)lUpLeg[i].get_ly();
                skeleton_pt_h.x = (double)lUpLeg[i].get_hx();
                skeleton_pt_h.y = (double)lUpLeg[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);

                //draw lLowLeg
                skeleton_pt_l.x = (double)lLowLeg[i].get_lx();
                skeleton_pt_l.y = (double)lLowLeg[i].get_ly();
                skeleton_pt_h.x = (double)lLowLeg[i].get_hx();
                skeleton_pt_h.y = (double)lLowLeg[i].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, (10, 120, 0), 2, 8, 0);
            }

            if(i >= takeOffFrame){
                //draw head
                cv::Point2d skeleton_pt_l, skeleton_pt_h;
                skeleton_pt_l.x = (double)head[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)head[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)head[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)head[takeOffFrame].get_hy();
                cv::circle(frame, skeleton_pt_h, 10, cv::Scalar(0, 255, 0), -1);
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw rcollar
                skeleton_pt_l.x = (double)rCollar[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)rCollar[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)rCollar[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)rCollar[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw rUpArm
                skeleton_pt_l.x = (double)rUpArm[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)rUpArm[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)rUpArm[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)rUpArm[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw rLowArm
                skeleton_pt_l.x = (double)rLowArm[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)rLowArm[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)rLowArm[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)rLowArm[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw lCollar
                skeleton_pt_l.x = (double)lCollar[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)lCollar[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)lCollar[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)lCollar[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw lUpArm
                skeleton_pt_l.x = (double)lUpArm[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)lUpArm[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)lUpArm[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)lUpArm[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw lLowArm
                skeleton_pt_l.x = (double)lLowArm[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)lLowArm[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)lLowArm[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)lLowArm[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw torso
                skeleton_pt_l.x = (double)torso[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)torso[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)torso[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)torso[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw rPelvis
                skeleton_pt_l.x = (double)rPelvis[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)rPelvis[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)rPelvis[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)rPelvis[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw rUpLeg
                skeleton_pt_l.x = (double)rUpLeg[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)rUpLeg[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)rUpLeg[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)rUpLeg[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw rLowLeg
                skeleton_pt_l.x = (double)rLowLeg[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)rLowLeg[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)rLowLeg[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)rLowLeg[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw lPelvis
                skeleton_pt_l.x = (double)lPelvis[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)lPelvis[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)lPelvis[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)lPelvis[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw lUpLeg
                skeleton_pt_l.x = (double)lUpLeg[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)lUpLeg[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)lUpLeg[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)lUpLeg[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw lLowLeg
                skeleton_pt_l.x = (double)lLowLeg[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)lLowLeg[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)lLowLeg[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)lLowLeg[takeOffFrame].get_hy();
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 0), 2, 8, 0);

                //draw takeoff angle
                skeleton_pt_l.x = (double)lLowLeg[takeOffFrame].get_lx();
                skeleton_pt_l.y = (double)lLowLeg[takeOffFrame].get_ly();
                skeleton_pt_h.x = (double)lUpArm[takeOffFrame].get_hx();
                skeleton_pt_h.y = (double)lUpArm[takeOffFrame].get_hy();
                double x_diff = (skeleton_pt_h.x - skeleton_pt_l.x);
                double y_diff = (skeleton_pt_h.y - skeleton_pt_l.y);
                skeleton_pt_l.x = skeleton_pt_l.x - (x_diff/4);
                skeleton_pt_l.y = skeleton_pt_l.y - (y_diff/4);
                skeleton_pt_h.x = skeleton_pt_h.x + (x_diff/4);
                skeleton_pt_h.y = skeleton_pt_h.y + (y_diff/4);
                cv::line(frame, skeleton_pt_l, skeleton_pt_h, cv::Scalar(0, 255, 255), 2, 8, 0);
                double takeoff_angle = atan(y_diff/x_diff) * (180/3.14159265);
                cv::Point2d degPt;
                degPt.x = 50;
                degPt.y = 90;
                string rounded_deg = "";
                rounded_deg = to_string(roundf(takeoff_angle * -100) / 100);
                cv::putText(frame, rounded_deg + " Degrees", degPt, CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

            }

            //Draw athlete COM velocity every third frame
            double instant_vel;
            if (i % VELOCITY_DELAY == 0)
            {
                instant_vel = (sqrt(pow((com[i + 1].get_x() - com[i].get_x()), 2)) * 0.6);
            }
            cout.precision(10);
            cout << fixed << instant_vel << endl;
            cv::Point2d velPt;
            velPt.x = 50;
            velPt.y = 50;
            string rounded = "";
            rounded = to_string(roundf(instant_vel * 100) / 100);
            if(i <= frameEnd){
                cv::putText(frame, rounded + "m/s", velPt, CV_FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            }
            // Draw COM point
            if (i < frameEnd)
            {
                cv::circle(frame, point, 5, (0, 0, 255), -1);
            }
            //cv::line(frame, point, point2, 5, (0, 0, 255), -1);
            for (int j = frameBegin; j < pointarray.size(); j = j + FRAME_JUMP)
            //for (int i = 0; i < pointarray.size(); i++)
            {
                //low pass filtering the acceleration data
                if (sqrt(pow((pointarray2[j].x - pointarray[j].x), 2) + pow((pointarray2[j].y - pointarray[j].y), 2)) < 100)
                {
                    cv::line(frame, pointarray[j], pointarray2[j], cv::Scalar(0, 0, 255), 2, 8, 0);
                }
            }
        }
        //cv::line(frame, point, point2, (0, 0, 255), 3, 8, 0);

        // Display the frame
        cv::imshow("Video feed", frame);

        // For breaking the loop
        if (cv::waitKey(25) >= 0)
            break;

        i++;
    }

    return 0;
}