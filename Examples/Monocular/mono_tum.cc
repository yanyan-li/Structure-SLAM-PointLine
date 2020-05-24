/**
Indoor SLAM.
Feed the pipeline with RGB and predicted normal frames.

Yanyan Li
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;


void LoadImages(const string &strFile, const string& gtFile, vector<string> &vstrImageFilenames,
                vector<string> &vstrNormalFilenames, vector<double> &vTimestamps,vector<cv::Mat> &gtPose);


int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<string> vstrNormalFilenames;
    vector<double> vTimestamps;
    vector<cv::Mat> vGtPose;
    string strFile = string(argv[3])+"/mono-normal.txt";
    string gtFile = string(argv[3])+"/groundtruth.txt";
    LoadImages(strFile,gtFile, vstrImageFilenames,vstrNormalFilenames, vTimestamps,vGtPose);


    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;



    // Main loop
    cv::Mat im;
    cv::Mat normal;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        //if(ni==1)ni+=10;
        //if((ni > 0) && (ni<20)) continue;

        //if(ni>140)sleep(2);
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        normal = cv::imread(string(argv[3])+"/"+vstrNormalFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty()||normal.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        cout<<"****This is the "<<ni<<"th image"<<", the name is "<<vTimestamps[ni]<<endl;
        SLAM.TrackMonocularWitNormal(im,normal,tframe);//, vGtPose[ni]);
        //cout<<vstrImageFilenames[ni]<< "normals_lrk2_exr/"+std::to_string(tframe)+".exr"<<endl;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e5);
    }

//    while (true) {
//        sleep(1000);
//    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("MonoTrajectory.txt");
    return 0;
}
cv::Mat setRotate(double qx, double qy,double qz, double qw,double tx, double ty,double tz) {
    double sqw = qw*qw;
    double sqx = qx*qx;
    double sqy = qy*qy;
    double sqz = qz*qz;

    cv::Mat Pose =cv::Mat::zeros(4,4,CV_32F);

    float m00 = sqx - sqy - sqz + sqw; // since sqw + sqx + sqy + sqz =1
    float m11 = -sqx + sqy - sqz + sqw;
    float m22 = -sqx - sqy + sqz + sqw;

    double tmp1 = qx*qy;
    double tmp2 = qz*qw;
    float m01 = 2.0 * (tmp1 + tmp2);
    float m10 = 2.0 * (tmp1 - tmp2);

    tmp1 = qx*qz;
    tmp2 = qy*qw;
    float m02 = 2.0 * (tmp1 - tmp2);
    float m20 = 2.0 * (tmp1 + tmp2);

    tmp1 = qy*qz;
    tmp2 = qx*qw;
    float m12 = 2.0 * (tmp1 + tmp2);
    float m21 = 2.0 * (tmp1 - tmp2);

    cv::Mat pose = (cv::Mat_<float>(4,4) << m00,m01,m02,tx,m10,m11,m12,ty,m20,m21,m22,tz,0,0,0,1);

    return pose;
}

void LoadImages(const string &strFile, const string& gtFile, vector<string> &vstrImageFilenames, vector<string> &vstrNormalFilenames, vector<double> &vTimestamps,vector<cv::Mat> &gtPose)
{
    ifstream f,gtF;
    f.open(strFile.c_str());
    gtF.open(gtFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);

        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sNormal;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
            ss >> sNormal;
            vstrNormalFilenames.push_back(sNormal);
        }
    }

    while(!gtF.eof())
    {
        string s;
        getline(gtF,s);

        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t,qw,qx,qy,qz,tx,ty,tz;
            ss >> t;
            ss >> tx;
            ss >>ty;
            ss >> tz;
            ss >>qx;
            ss >> qy;
            ss >>qz;
            ss >> qw;
            //convert qu  to Mat
            cv::Mat temPose= setRotate(qx,qy,qz,qw,tx,ty,tz);
            gtPose.push_back(temPose);
        }
    }
}
