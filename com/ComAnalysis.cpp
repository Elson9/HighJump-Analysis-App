#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>

// COM location for each limb
#define SHIN_DISTAL 0.567
#define THIGH_DISTAL 0.433
#define TORSO_DISTAL 0.650
#define BICEP_DISTAL 0.564
#define FOREARM_DISTAL 0.682

//COM contribution of each body part
#define SHIN_COM 0.061
#define THIGH_COM 0.100
#define PELVIS_COM 0.142
#define TORSO_COM 0.355
#define BICEP_COM 0.028
#define FOREARM_COM 0.022
#define HEAD_COM 0.081

using namespace std;

class JointCoord
{
  private:
    float x;
    float y;
    float c;

  public:
    JointCoord()
    {
        x = 0;
        y = 0;
        c = 0;
    }

    void set_x(float X)
    {
        x = X;
    }

    void set_y(float Y)
    {
        y = Y;
    }

    void set_c(float C)
    {
        c = C;
    }

    float get_x()
    {
        return x;
    }

    float get_y()
    {
        return y;
    }

    float get_c()
    {
        return c;
    }
};

class LimbCoord
{
  private:
    // Low and high positions of joint, ex. rWrist and rElbow respectively for
    // rLowArm
    float lx;
    float ly;
    float hx;
    float hy;
    float comx;
    float comy;

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

    LimbCoord(float LX, float LY, float HX, float HY)
    {
        lx = LX;
        ly = LY;
        hx = HX;
        hy = HY;
        comx = 0;
        comy = 0;
    }

    void set_lx(float LX)
    {
        lx = LX;
    }

    void set_hx(float HX)
    {
        hx = HX;
    }

    void set_ly(float LY)
    {
        ly = LY;
    }

    void set_hy(float HY)
    {
        hy = HY;
    }

    void set_comx(float COMX)
    {
        comx = COMX;
    }

    void set_comy(float COMY)
    {
        comy = COMY;
    }

    float get_lx()
    {
        return lx;
    }

    float get_hx()
    {
        return hx;
    }

    float get_ly()
    {
        return ly;
    }

    float get_hy()
    {
        return hy;
    }

    float get_comx()
    {
        return comx;
    }

    float get_comy()
    {
        return comy;
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
        float difference = joint[i].get_x() - joint[j].get_x();
        int numFrames = i - j;
        float incrementAmt = difference / (float)numFrames;
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
        float difference = joint[i].get_y() - joint[j].get_y();
        int numFrames = i - j;
        float incrementAmt = difference / (float)numFrames;
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

void limb_com(vector<LimbCoord> &curLimb, float distal)
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
    vector<JointCoord> &com)
{

    for (int i = 0; i < head.size(); i++)
    {
        JointCoord curCOM;
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

/*
 * To-do:
 * 
 * test COM calculation
 * only do COM calculation when all joint COM values are recorded properly
 * create COMdata class for including velocity and accel data
 * 
 * detect the right person if more than one person detected
 * COM velocity and acceleration
 */
int main(int argc, char **argv)
{

    // Set up strings for dynamic file retrieval
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
    vector<JointCoord> com;
    com_calc(rLowLeg, lLowLeg, rUpLeg, lUpLeg, torso, rUpArm,
             lUpArm, rLowArm, lLowArm, rPelvis, head, com);

    cout << com.size() << endl;

    //for (int i = 0; i < com.size(); i++)
    //{
    //    cout << com[i].get_x() << ", " << com[i].get_y() << endl;
    //}

    // Get COM velocity and acceleration

    // Open video for drawing
    cv::VideoCapture video("/home/james/ece496/openpose/input/120fps.mp4");
    cv::Mat frame;

    int i = 0;
    while (video.read(frame))
    {

        cv::Point2d point;
        point.x = (double)com[i].get_x();
        point.y = (double)com[i].get_y();

        cv::circle(frame, point, 5, (0, 0, 255), -1);

        // Display the frame
        cv::imshow("Video feed", frame);

        // For breaking the loop
        if (cv::waitKey(25) >= 0)
            break;

        i++;
    }

    return 0;
}