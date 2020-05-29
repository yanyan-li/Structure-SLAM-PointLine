/**
 * This file is part of Structure-SLAM.
 * Yanyan Li <yanyan.li@tum.de> (Technical University of Munich)
 *
 *
*/

#include<iostream>
#include<algorithm>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>
using namespace std;


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<string> &vstrNormalFilenames, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./Structure-SLAM path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<string> vstrNormalFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/mono-normal.txt";
    LoadImages(strFile, vstrImageFilenames,vstrNormalFilenames, vTimestamps);


    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    StructureSLAM::System SLAM(argv[1],argv[2],StructureSLAM::System::MONOCULAR,true);

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

        // feed the images to the system
        cout<<"****This is the "<<ni<<"th image"<<", the name is "<<vTimestamps[ni]<<endl;
        SLAM.TrackMonocularWithPL(im,tframe);

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
            usleep((T-ttrack)*1e4);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("MonoTrajectory.txt");
    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<string> &vstrNormalFilenames, vector<double> &vTimestamps)
{
    ifstream f,gtF;
    f.open(strFile.c_str());
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
}
