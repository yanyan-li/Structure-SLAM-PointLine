/**
* This file is part of Structure-SLAM.
*
*
*/
/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Tracking.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include <iostream>
#include <mutex>

using namespace std;
namespace StructureSLAM
{
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    int img_width = fSettings["Camera.width"];
    int img_height = fSettings["Camera.height"];
    cout << "img_width = " << img_width << endl;
    cout << "img_height = " << img_height << endl;
    initUndistortRectifyMap(mK, mDistCoef, Mat_<double>::eye(3,3), mK, Size(img_width, img_height), CV_32F, mUndistX, mUndistY);
    cout << "mUndistX size = " << mUndistX.size << endl;
    cout << "mUndistY size = " << mUndistY.size << endl;
    mbf = fSettings["Camera.bf"];
    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageMonocularWithNormal(const cv::Mat &im,const cv::Mat &normal,  const double &timestamp)
{
    mImGray = im;
    mNormal = normal;
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }
    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,mNormal,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,mNormal,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    TrackWithNormal();
    return mCurrentFrame.mTcw.clone();
}

void Tracking::TrackWithNormal()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }
    bool bIni_Manhattan = true;
    bool bManhattanGood = false;
    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
    if(mState==NOT_INITIALIZED)
    {
        //Manhattan initialization
        MonocularInitializationWithNormal();
        mpFrameDrawer->Update(this);
        if(mState!=OK)
            return;
    }
    else  //after initialization
    {
        //Tracking: system is initialized
        bool bOK;
        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            cv::Mat relatedPose = cv::Mat::zeros(cv::Size(3,3),CV_32F);
            if(bIni_Manhattan)
            {
                // compute rotation based on the Manhattan
                cv ::Mat MF_can= cv::Mat::zeros(cv::Size(3,3),CV_32F);
                cv ::Mat MF_can_T= cv::Mat::zeros(cv::Size(3,3),CV_32F);
                cv ::Mat mRotation_wc1= cv::Mat::zeros(cv::Size(3,3),CV_32F);

                MF_can=TrackManhattanFrame(mLastRcm,mCurrentFrame.vSurfaceNormal,mCurrentFrame.mVF3DLines).clone();
                //MF_can_T = mLastRcm.t();
                //mRotation_wc=Rotation_cm*MF_can_T;
                //mRotation_wc1=mRotation_wc.t();  //R_k-1,1


                MF_can_T=MF_can.t();
                mRotation_wc=Rotation_cm*MF_can_T; //R_1,k
                mRotation_cw=mRotation_wc.t();   //R_k,1

                relatedPose=MF_can*mLastRcm.t();
                MF_can.copyTo(mLastRcm);
                //relatedPose = (mRotation_wc1*mRotation_wc).t(); //  R_k-1,k -->R_k,k-1

            }

            if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
            {
                //bOK = TrackReferenceKeyFrame();
            }
            else
            {
                if (bIni_Manhattan&&TranslationWithMotionModel(relatedPose))
                {
                    cout<<"\t translation only"<<endl;
                    bManhattanGood = true;
                }
                else
                {
                    bOK = TrackReferenceKeyFrame();
                    if(bOK)  bManhattanGood = true;
                }
                if(!bManhattanGood)
                {
                    bOK = Relocalization();
                    if(bOK) bManhattanGood= true;
                }
            }
            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();
                //compute translation
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    cout<<"tracking: in reference"<<endl;
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    {
                        if(!bManhattanGood)
                        bOK = TrackWithMotionModel();
                        //fix manhattan pose
                        //UpdateManhattanPose();
                    }
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        //pose refinement
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
                bOK=TrackLocalMap();
                if(!bOK)
                    bOK = TrackLocalMapWithLines();
            }
            else
            {
                bOK=Relocalization();
            }
        }
        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                // update mVelocity
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }
            for(int i=0; i<mCurrentFrame.NL; i++)
            {
                MapLine* pML = mCurrentFrame.mvpMapLines[i];
                if(pML)
                    if(pML->Observations()<1)
                    {
                        mCurrentFrame.mvbLineOutlier[i] = false;
                        mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
                    }
            }
            //cout<<" tracking: Clean VO matches2"<<endl;
            // Delete temporal MapPoints
            // 只用于双目或rgbd
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            for(list<MapLine*>::iterator lit = mlpTemporalLines.begin(), lend =  mlpTemporalLines.end(); lit!=lend; lit++)
            {
                MapLine* pML = *lit;
                delete pML;
            }
            mlpTemporalPoints.clear();
            mlpTemporalLines.clear();
            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
            {
                CreateNewKeyFrame();
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            // 剔除那些在BA中检测为outlier的3D map点
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }

            for(int i=0; i<mCurrentFrame.NL; i++)
            {
                if(mCurrentFrame.mvpMapLines[i] && mCurrentFrame.mvbLineOutlier[i])
                    mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
            }
        }
        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
        //cout<<"Tracking Tcr:"<<Tcr<<endl;
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}



void Tracking::MonocularInitializationWithNormal()
{
    int num=50;
    if(!mpInitializer)
    {
        if(mCurrentFrame.mvKeys.size()>num)
        {
            mInitialFrame=Frame(mCurrentFrame);
            mLastFrame=Frame(mCurrentFrame);

            //当前帧的 surface normal
            Rotation_cm=SeekManhattanFrame(mCurrentFrame.vSurfaceNormal,mCurrentFrame.mVF3DLines).clone();
            Rotation_cm.copyTo(mLastRcm);
            cv ::Mat MF_can= cv::Mat::zeros(cv::Size(3,3),CV_32F);
            cv ::Mat MF_can_T= cv::Mat::zeros(cv::Size(3,3),CV_32F);
            MF_can=TrackManhattanFrame(Rotation_cm,mCurrentFrame.vSurfaceNormal,mCurrentFrame.mVF3DLines).clone();
            MF_can.copyTo(mLastRcm);//.clone();
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0;i<mCurrentFrame.mvKeysUn.size();i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete  mpInitializer;
            mpInitializer=new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }
    }
    else
    {
        cv::Mat relatedPose = cv::Mat::zeros(cv::Size(3,3),CV_32F);
        if((int)mCurrentFrame.mvKeys.size()<num)
        {
            delete mpInitializer;
            mpInitializer= static_cast<Initializer*> (NULL);
            fill(mvIniMatches.begin(), mvIniMatches.end(),-1);
            return;
        }

        ORBmatcher matcher (0.9,true);
        LSDmatcher lmatcher;   //建立线特征之间的匹配
        int nmatches =matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);
        int nlineMatches = lmatcher.SerachForInitialize(mInitialFrame, mCurrentFrame, mvLineMatches);

        if(nmatches<20 )
        {
            delete  mpInitializer;
            cout<<"***********restart, because of less points"<<endl;
            mpInitializer= static_cast<Initializer*>(NULL);
            return;
        }

        cv ::Mat MF_can= cv::Mat::zeros(cv::Size(3,3),CV_32F);
        cv ::Mat MF_can_T= cv::Mat::zeros(cv::Size(3,3),CV_32F);
        MF_can=TrackManhattanFrame(mLastRcm,mCurrentFrame.vSurfaceNormal,mCurrentFrame.mVF3DLines).clone();
        cout<<"TRACKING:track MF mono:"<<MF_can<<endl;
        //cv::Mat R_cm=ClusterMMF(MF_can);

        relatedPose=MF_can*mLastRcm.t(); //R21
        MF_can.copyTo(mLastRcm);
        mRotation_cw = relatedPose.t();

        cout<<"Tracking: finish mf rotation"<<mRotation_wc<<endl;
        //获得 Rcw
        cv::Mat Rcw= cv::Mat::zeros(cv::Size(3,3),CV_32F);; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        mRotation_cw.copyTo(Rcw);

        //Rcw is R21
        if(mpInitializer->InitializeManhattanWorld(mInitialFrame,mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated, mvLineMatches, mvLineS3D, mvLineE3D, mvbLineTriangulated))
        {

            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    //cout<<vbTriangulated[i]<<endl;
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            // 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            // step6：将三角化得到的3D点包装成MapPoints
            //CreateInitialMapMonocular();
            CreateInitialMapMonoWithLine();
        }
        cout<<"Tracking: the second image"<<endl;
    }

}
bool Tracking::TranslationWithMotionModel(cv::Mat &relatedPose)
{
    ///cout<<"Tracking: with motion model"<<endl;
    bool reliableManhattanR_wc = true;
    bool bManhattanRotation = false;
    // --step1: 建立ORB特征点的匹配
    ORBmatcher matcher(0.9,true);
    LSDmatcher lmatcher;

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    // --step2: 更新上一帧的位姿
    UpdateLastFrame();

    // --step3:根据Const Velocity Model，估计当前帧的位姿
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    // --step4：根据上一帧特征点对应的3D点投影的位置缩小特征点匹配范围
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    cout<<"translation motion model：point matches1:"<<nmatches<<endl;
    int lmatches = lmatcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);//    int lmatches2=0;
//    if(mCurrentFrame.dealWithLine)
//    {
//        fill(mCurrentFrame.mvpMapLines.begin(),mCurrentFrame.mvpMapLines.end(), static_cast<MapLine*>(NULL));
//        vector<MapLine*> vpMapLineMatches;
//        lmatches2 = lmatcher.SearchByProjection(mpReferenceKF,mCurrentFrame,vpMapLineMatches);
//        mCurrentFrame.mvpMapLines=vpMapLineMatches;
//    }
    //vector<MapPoint*> vpMapPointMatches;

    //  int nmatches1 = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    //lmatcher.SearchByProjection(mCurrentFrame, mLastFrame.mvpMapLines, th);
    //    cout << "tracking points = " << nmatches << endl;
    //    cout << "tracking lmatches = " << lmatches << endl;

    //double lmatch_ratio = lmatches*1.0/mCurrentFrame.mvKeylinesUn.size();
    //    cout << "lmatch_ratio = " << lmatch_ratio << endl;

    // If few matches, uses a wider window search
    // 如果跟踪的点少，则扩大搜索半径再来一次
    if(nmatches<30)
    {
        cout<<"tracking: trans only: rematching"<<endl;
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,4*th,mSensor==System::MONOCULAR);
    }

//    if(mCurrentFrame.dealWithLine)
//    {
//        if(nmatches<10 &&lmatches2<3)  mCurrentFrame.SetPose(mpReferenceKF->GetPose());
//    }
//    else
//    {
//        if(nmatches<8 )
//            return false;
//    }

    // Optimize frame pose with all matches
    // --step5: 优化位姿
    cout<<"translation,pose before opti"<<mCurrentFrame.mTcw<<endl;

    //relatedPose
    if(bManhattanRotation)
        mRotation_cw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
    else
    {
        cout<<relatedPose<<endl;
        cout<<relatedPose.t()<<endl;
        cout<<mVelocity<<endl;
        cv::Mat poseFromLastFrame = relatedPose.t()*mLastFrame.mTcw.rowRange(0,3).colRange(0,3);
        poseFromLastFrame.copyTo(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
    }

    cout<<"translation  motion model：point matches:"<<nmatches<<endl;
    //if(mCurrentFrame.dealWithLine)
    Optimizer::PoseOptimization(&mCurrentFrame, true);
    //else

    float ratio = Optimizer::TranslationOptimization(&mCurrentFrame);
    cout<<"tracking: translation motion model,pose after opti"<<mCurrentFrame.mTcw<<endl;
    cout<<"tracking: translation ratio: "<<ratio<<endl;

    //check whether this pose is good nor not
    if(ratio<0.85)
    {
        //this pose cannot reliable
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
        //fix manhattan information
        reliableManhattanR_wc = false;
        return reliableManhattanR_wc;

    }
    else
    {
        //a reliable pose
        int nmatchesMap = 0;
        int nmatchesLineMap=0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    //mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

        for(int i=0; i<mCurrentFrame.NL;i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(mCurrentFrame.mvbLineOutlier[i])
                {
                    MapLine* pML=mCurrentFrame.mvpMapLines[i];
                    mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
                    mCurrentFrame.mvbLineOutlier[i]=false;
                    pML->mbTrackInView=false;
                    pML->mnLastFrameSeen=mCurrentFrame.mnId;
                    lmatches--;
                }
                else if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                    nmatchesLineMap++;
            }
        }
        mState = OK;
        return reliableManhattanR_wc;
    }
}

cv::Mat Tracking::SeekManhattanFrame(vector<SurfaceNormal> &vTempSurfaceNormal,vector<FrameLine> &vVanishingDirection)
{

    vector<cv::Mat> vRotaionMatrix;
    vector<cv::Mat> vRotaionMatrix_good;
    cv::RNG rnger(cv::getTickCount());
    vector<cv::Mat> vSN_good;
    vector<double> lambda_good;
    vector<cv::Point2d> m_j_selected;
    // R_cm_update matrix
    cv::Mat R_cm_update=cv::Mat::eye(cv::Size(3,3),CV_32F);
    cv::Mat R_cm_new=cv::Mat::eye(cv::Size(3,3),CV_32F);
// initialization with random matrix
#if 1
    cv::Mat qu = cv::Mat::zeros(cv::Size(4,1),CV_32F);
    rnger.fill(qu, cv::RNG::UNIFORM, cv::Scalar::all(0.01), cv::Scalar::all(1));
    Eigen::Quaterniond qnorm;
    Eigen::Quaterniond q(qu.at<float>(0,0),qu.at<float>(1,0),qu.at<float>(2,0),qu.at<float>(3,0));//=Eigen::MatrixXd::Random(1, 4);
    qnorm.x()=q.x()/q.norm();qnorm.y()=q.y()/q.norm();
    qnorm.z()=q.z()/q.norm();qnorm.w()=q.w()/q.norm();
    cv::eigen2cv(qnorm.matrix(),R_cm_update);//	eigen2cv(m, img);;*/
    //cout<<R_cm_update<<endl;
    cv::SVD svd; cv::Mat U,W,VT;
    svd.compute(R_cm_update,W,U,VT);
    R_cm_update=U*VT;
    //cout<<000<<R_cm_update<<endl;
    // R_cm_Rec matrix
    cv::Mat R_cm_Rec=cv::Mat::zeros(cv::Size(3,3),CV_32F);

    cv::Mat R_cm_initial;
    int  validMF=0;
    //cout<<R_cm_update<<endl;
    R_cm_new.at<float>(0,0) = R_cm_update.at<double>(0,0);
    R_cm_new.at<float>(0,1) = R_cm_update.at<double>(0,1);
    R_cm_new.at<float>(0,2) = R_cm_update.at<double>(0,2);
    R_cm_new.at<float>(1,0) = R_cm_update.at<double>(1,0);
    R_cm_new.at<float>(1,1) = R_cm_update.at<double>(1,1);
    R_cm_new.at<float>(1,2) = R_cm_update.at<double>(1,2);
    R_cm_new.at<float>(2,0) = R_cm_update.at<double>(2,0);
    R_cm_new.at<float>(2,1) = R_cm_update.at<double>(2,1);
    R_cm_new.at<float>(2,2) = R_cm_update.at<double>(2,2);
    //cout<<R_cm_new<<endl;
    //matTemp.convertTo(MatTemp2, CV_8U)
#endif
    //cout<<R_cm_update<<endl;
    R_cm_new = TrackManhattanFrame(R_cm_new, vTempSurfaceNormal,vVanishingDirection);
    return R_cm_new;//vRotaionMatrix_good[0];
}

ResultOfMS Tracking::ProjectSN2MF(int a,const cv::Mat &R_mc,const vector<SurfaceNormal> &vTempSurfaceNormal,vector<FrameLine> &vVanishingDirection,const int numOfSN)
{
    vector<cv::Point2d> m_j_selected;
    cv::Mat R_cm_Rec=cv::Mat::zeros(cv::Size(1,3),CV_32F);
    cv::Mat R_cm_NULL=cv::Mat::zeros(cv::Size(1,3),CV_32F);
    ResultOfMS RandDen;
    RandDen.axis=a;

    size_t sizeOfSurfaceNormal=vTempSurfaceNormal.size()+vVanishingDirection.size();
    m_j_selected.reserve(sizeOfSurfaceNormal);

    for(size_t i=0;i<sizeOfSurfaceNormal;i++)
    {
        //cv::Mat temp=cv::Mat::zeros(cv::Size(1,3),CV_32F);

        cv::Point3f n_ini;
        int tepSize=i-vTempSurfaceNormal.size();
        if(i>=vTempSurfaceNormal.size())
        {


            n_ini.x = R_mc.at<float>(0,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(0,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(0,2) * vVanishingDirection[tepSize].direction.z;
            n_ini.y = R_mc.at<float>(1,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(1,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(1,2) * vVanishingDirection[tepSize].direction.z;
            n_ini.z = R_mc.at<float>(2,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(2,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(2,2) * vVanishingDirection[tepSize].direction.z;
        }
        else{

            n_ini.x = R_mc.at<float>(0,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(0,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(0,2) * vTempSurfaceNormal[i].normal.z;
            n_ini.y = R_mc.at<float>(1,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(1,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(1,2) * vTempSurfaceNormal[i].normal.z;
            n_ini.z = R_mc.at<float>(2,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(2,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(2,2) * vTempSurfaceNormal[i].normal.z;
        }


        double lambda=sqrt(n_ini.x*n_ini.x+n_ini.y*n_ini.y);//at(k).y*n_a.at(k).y);
        //cout<<lambda<<endl;
        //inside the cone
        if(lambda<sin(0.2518)) //0.25
        {
            double tan_alfa=lambda/std::abs(n_ini.z);
            double alfa=asin(lambda);
            double m_j_x=alfa/tan_alfa*n_ini.x/n_ini.z;
            double m_j_y=alfa/tan_alfa*n_ini.y/n_ini.z;
            if(!std::isnan(m_j_x)&&!std::isnan(m_j_y))
                m_j_selected.push_back(cv::Point2d(m_j_x,m_j_y));
            if(i<vTempSurfaceNormal.size())
            {
                if(a==1)
                {
                    mCurrentFrame.vSurfaceNormalx.push_back(vTempSurfaceNormal[i].FramePosition);
                    mCurrentFrame.vSurfacePointx.push_back(vTempSurfaceNormal[i].cameraPosition);
                }
                else if(a==2)
                {
                    mCurrentFrame.vSurfaceNormaly.push_back(vTempSurfaceNormal[i].FramePosition);
                    mCurrentFrame.vSurfacePointy.push_back(vTempSurfaceNormal[i].cameraPosition);
                }
                else if(a==3)
                {
                    mCurrentFrame.vSurfaceNormalz.push_back(vTempSurfaceNormal[i].FramePosition);
                    mCurrentFrame.vSurfacePointz.push_back(vTempSurfaceNormal[i].cameraPosition);
                }
            }
            else
            {
                if(a==1)
                {
                    cv::Point2d endPoint=vVanishingDirection[tepSize].p;
                    cv::Point2d startPoint=vVanishingDirection[tepSize].q;
                    vector<cv::Point2d> pointPair(2);
                    pointPair.push_back(endPoint);pointPair.push_back(startPoint);
                    mCurrentFrame.vVanishingLinex.push_back(pointPair);
                    for(int k=0;k<vVanishingDirection[tepSize].rndpts3d.size();k++)
                        mCurrentFrame.vVaishingLinePCx.push_back(vVanishingDirection[tepSize].rndpts3d[k]);
                }
                else if(a==2)
                {
                    cv::Point2d endPoint=vVanishingDirection[tepSize].p;
                    cv::Point2d startPoint=vVanishingDirection[tepSize].q;
                    vector<cv::Point2d> pointPair(2);
                    pointPair.push_back(endPoint);pointPair.push_back(startPoint);
                    mCurrentFrame.vVanishingLiney.push_back(pointPair);
                    for(int k=0;k<vVanishingDirection[tepSize].rndpts3d.size();k++)
                        mCurrentFrame.vVaishingLinePCy.push_back(vVanishingDirection[tepSize].rndpts3d[k]);
                }
                else if(a==3)
                {
                    cv::Point2d endPoint=vVanishingDirection[tepSize].p;
                    cv::Point2d startPoint=vVanishingDirection[tepSize].q;
                    vector<cv::Point2d> pointPair(2);
                    pointPair.push_back(endPoint);pointPair.push_back(startPoint);
                    mCurrentFrame.vVanishingLinez.push_back(pointPair);
                    for(int k=0;k<vVanishingDirection[tepSize].rndpts3d.size();k++)
                        mCurrentFrame.vVaishingLinePCz.push_back(vVanishingDirection[tepSize].rndpts3d[k]);
                }
            }


        }
    }
    //cout<<"a=1:"<<mCurrentFrame.vSurfaceNormalx.size()<<",a =2:"<<mCurrentFrame.vSurfaceNormaly.size()<<", a=3:"<<mCurrentFrame.vSurfaceNormalz.size()<<endl;
    //cout<<"m_j_selected.push_back(temp)"<<m_j_selected.size()<<endl;

    if(m_j_selected.size()>numOfSN)
    {
        sMS tempMeanShift = MeanShift(m_j_selected);
        cv::Point2d s_j = tempMeanShift.centerOfShift;// MeanShift(m_j_selected);
        float s_j_density=tempMeanShift.density;
        //cout<<"tracking:s_j"<<s_j.x<<","<<s_j.y<<endl;
        float alfa=norm(s_j);
        float ma_x=tan(alfa)/alfa*s_j.x;
        float ma_y=tan(alfa)/alfa*s_j.y;
        cv::Mat temp1=cv::Mat::zeros(cv::Size(1,3),CV_32F);
        temp1.at<float>(0,0)=ma_x;
        temp1.at<float>(1,0)=ma_y;
        temp1.at<float>(2,0)=1;
        cv::Mat rtemp=R_mc.t();
        R_cm_Rec=rtemp*temp1;
        R_cm_Rec=R_cm_Rec/norm(R_cm_Rec); //列向量
        RandDen.R_cm_Rec=R_cm_Rec;
        RandDen.s_j_density=s_j_density;

        return RandDen;
    }
    RandDen.R_cm_Rec=R_cm_NULL;
    return RandDen;

}

axiSNV Tracking::ProjectSN2Conic(int a,const cv::Mat &R_mc,const vector<SurfaceNormal> &vTempSurfaceNormal,vector<FrameLine> &vVanishingDirection)
{
    int numInConic=0;
    vector<cv::Point2d> m_j_selected;
    cv::Mat R_cm_Rec=cv::Mat::zeros(cv::Size(3,3),CV_32F);
    cv::Mat R_cm_NULL=cv::Mat::zeros(cv::Size(1,3),CV_32F);
    //cv::Mat R_mc=cv::Mat::zeros(cv::Size(3,3),CV_32F);
    vector<SurfaceNormal> vSNCadidate;
    axiSNV tempaxiSNV;
    tempaxiSNV.axis=a;

    size_t sizeOfSurfaceNormal=vTempSurfaceNormal.size();//TODO 先去掉线 +vVanishingDirection.size();
    tempaxiSNV.SNVector.reserve(sizeOfSurfaceNormal);
    //cout<<"size of SN"<<sizeOfSurfaceNormal<<endl;
    for(size_t i=0;i<sizeOfSurfaceNormal;i++)
    {
        cv::Point3f n_ini;
        if(i<vTempSurfaceNormal.size())
        {
            n_ini.x = R_mc.at<float>(0,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(0,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(0,2) * vTempSurfaceNormal[i].normal.z;
            n_ini.y = R_mc.at<float>(1,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(1,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(1,2) * vTempSurfaceNormal[i].normal.z;
            n_ini.z = R_mc.at<float>(2,0) * vTempSurfaceNormal[i].normal.x + R_mc.at<float>(2,1) * vTempSurfaceNormal[i].normal.y + R_mc.at<float>(2,2) * vTempSurfaceNormal[i].normal.z;

            double lambda=sqrt(n_ini.x*n_ini.x+n_ini.y*n_ini.y);//at(k).y*n_a.at(k).y);
            //cout<<lambda<<endl;
            if(lambda<sin(0.2518)) //0.25
            {
                //vSNCadidate.push_back(vTempSurfaceNormal[i]);
                //numInConic++;
                tempaxiSNV.SNVector.push_back(vTempSurfaceNormal[i]);
            }
        }
        else
        {   //cout<<"vanishing"<<endl;
            int tepSize=i-vTempSurfaceNormal.size();
            //cout<<vVanishingDirection[tepSize].direction.x<<"vanishing"<<endl;

            n_ini.x = R_mc.at<float>(0,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(0,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(0,2) * vVanishingDirection[tepSize].direction.z;
            n_ini.y = R_mc.at<float>(1,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(1,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(1,2) * vVanishingDirection[tepSize].direction.z;
            n_ini.z = R_mc.at<float>(2,0) * vVanishingDirection[tepSize].direction.x + R_mc.at<float>(2,1) * vVanishingDirection[tepSize].direction.y + R_mc.at<float>(2,2) * vVanishingDirection[tepSize].direction.z;

            double lambda=sqrt(n_ini.x*n_ini.x+n_ini.y*n_ini.y);//at(k).y*n_a.at(k).y);
            //cout<<lambda<<endl;
            if(lambda<sin(0.1518)) //0.25
            {
                //vSNCadidate.push_back(vTempSurfaceNormal[i]);
                //numInConic++;
                tempaxiSNV.Linesvector.push_back(vVanishingDirection[tepSize]);
            }
        }
    }

    return tempaxiSNV;//numInConic;

}

cv::Mat Tracking::TrackManhattanFrame(cv::Mat &mLastRcm,vector<SurfaceNormal> &vSurfaceNormal,vector<FrameLine> &vVanishingDirection)
{
    //上一帧的 camera 到 manhattan的距离
    //cout<<"begin Tracking Manhattan frame"<<endl;
    //cout<<"mLastRcm0:"<<mLastRcm<<endl;
    cv::Mat R_cm_update=mLastRcm.clone();
    //cout<<"mLastRcm"<<mLastRcm<<endl;
    int isTracked = 0;
    vector<double>denTemp(3, 0.00001);
    for (int i = 0; i < 6; i++) {

        cv::Mat R_cm = R_cm_update;//cv::Mat::eye(cv::Size(3,3),CV_32FC1);  // 对角线为1的对角矩阵(3, 3, CV_32FC1);
        //cout<<"R_cm"<<R_cm<<endl;
        int directionFound1 = 0;
        int directionFound2 = 0;
        int directionFound3 = 0; //三个方向
        int numDirectionFound = 0;
        vector<axiSNVector>vaxiSNV(4);
        vector<int> numInCone = vector<int>(3, 0);
        vector<cv::Point2f> vDensity ;
        //chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for (int a = 1; a < 4; a++) {
            //在每个conic有多少 点
            cv::Mat R_mc=cv::Mat::zeros(cv::Size(3,3),CV_32F);
            int c1=(a+3)%3; int c2=(a+4)%3; int c3=(a+5)%3;
            //cout<<"R_cm a"<<R_cm<<endl;
            R_mc.at<float>(0,0)=R_cm.at<float>(0,c1);R_mc.at<float>(0,1)=R_cm.at<float>(0,c2);
            R_mc.at<float>(0,2)=R_cm.at<float>(0,c3);R_mc.at<float>(1,0)=R_cm.at<float>(1,c1);
            R_mc.at<float>(1,1)=R_cm.at<float>(1,c2);R_mc.at<float>(1,2)=R_cm.at<float>(1,c3);
            R_mc.at<float>(2,0)=R_cm.at<float>(2,c1);R_mc.at<float>(2,1)=R_cm.at<float>(2,c2);
            R_mc.at<float>(2,2)=R_cm.at<float>(2,c3);
            //cout<<"R_cm b"<<R_mc<<endl;
            cv::Mat R_mc_new=R_mc.t();
            //cout<<"R_mc_new"<<R_mc_new<<endl;
            vaxiSNV[a-1] = ProjectSN2Conic(a, R_mc_new, vSurfaceNormal,vVanishingDirection);
            numInCone[a - 1] = vaxiSNV[a-1].SNVector.size();
            //cout<<"2 a:"<<vaxiSNV[a-1].axis<<",vector:"<<numInCone[a - 1]<<endl;
        }
        //chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        //chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
        //cout << "first sN time: " << time_used.count() << endl;
        int minNumOfSN = vSurfaceNormal.size() / 20;
        //cout<<"minNumOfSN"<<minNumOfSN<<endl;
        //排序  a<b<c
        int a = numInCone[0];
        int b = numInCone[1];
        int c = numInCone[2];
        //cout<<"a:"<<a<<",b:"<<b<<",c:"<<c<<endl;
        int temp = 0;
        if (a > b) temp = a, a = b, b = temp;
        if (b > c) temp = b, b = c, c = temp;
        if (a > b) temp = a, a = b, b = temp;
        //cout<<"sequence  a:"<<a<<",b:"<<b<<",c:"<<c<<endl;
        //打印排序后的三个数
        if (b < minNumOfSN)
        {
            minNumOfSN = (b + a) / 2;
            //cout <<"thr"<<minNumOfSN<<endl;
        }

        //cout<<"new  minNumOfSN"<<minNumOfSN<<endl;
        //chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
        for (int a = 1; a < 4; a++)
        {
            cv::Mat R_mc=cv::Mat::zeros(cv::Size(3,3),CV_32F);
            int c1=(a+3)%3; int c2=(a+4)%3; int c3=(a+5)%3;
            R_mc.at<float>(0,0)=R_cm.at<float>(0,c1);R_mc.at<float>(0,1)=R_cm.at<float>(0,c2);
            R_mc.at<float>(0,2)=R_cm.at<float>(0,c3);R_mc.at<float>(1,0)=R_cm.at<float>(1,c1);
            R_mc.at<float>(1,1)=R_cm.at<float>(1,c2);R_mc.at<float>(1,2)=R_cm.at<float>(1,c3);
            R_mc.at<float>(2,0)=R_cm.at<float>(2,c1);R_mc.at<float>(2,1)=R_cm.at<float>(2,c2);
            R_mc.at<float>(2,2)=R_cm.at<float>(2,c3);
            cv::Mat R_mc_new=R_mc.t();
            vector<SurfaceNormal>* tempVVSN;
            vector<FrameLine>* tempLineDirection;
            for(int i=0;i<3;i++)
            {
                if(vaxiSNV[i].axis==a)
                {

                    tempVVSN=&vaxiSNV[i].SNVector;
                    tempLineDirection=&vaxiSNV[i].Linesvector;
                    //cout<<"2 a:"<<vaxiSNV[i].axis<<",vector:"<<tempVVSN->size()<<endl;
                    break;
                }

            }
            //cout<<"1 a:"<<a<<endl;

            ResultOfMS RD_temp  = ProjectSN2MF(a, R_mc_new, *tempVVSN,*tempLineDirection, minNumOfSN);

            //chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
            //chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t4-t3);
            //cout << "second SN time: " << time_used.count() << endl;

            //cout << "test projectSN2MF" << ra << endl;
            //如果 ra不为0
            if (sum(RD_temp.R_cm_Rec)[0] != 0) {
                numDirectionFound += 1;
                if (a == 1) directionFound1 = 1;//第一个轴
                else if (a == 2) directionFound2 = 1;
                else if (a == 3) directionFound3 = 1;
                R_cm_update.at<float>(0, a - 1) = RD_temp.R_cm_Rec.at<float>(0, 0);
                R_cm_update.at<float>(1, a - 1) = RD_temp.R_cm_Rec.at<float>(1, 0);
                R_cm_update.at<float>(2, a - 1) = RD_temp.R_cm_Rec.at<float>(2, 0);
                //RD_temp.s_j_density;

                vDensity.push_back(cv::Point2f(RD_temp.axis,RD_temp.s_j_density));

            }
        }
        //cout<<"numDirectionFound:"<<numDirectionFound<<endl;

        if (numDirectionFound < 2) {
            cout<<"oh, it has happened"<<endl;
            R_cm_update = R_cm;
            numDirectionFound = 0;
            isTracked = 0;
            directionFound1 = 0;
            directionFound2 = 0;
            directionFound3 = 0;
            break;
        } else if (numDirectionFound == 2) {
            if (directionFound1 && directionFound2) {
                cv::Mat v1 = R_cm_update.colRange(0, 1).clone();
                //cout << 1 << v1 << endl;
                cv::Mat v2 = R_cm_update.colRange(1, 2).clone();
                cv::Mat v3 = v1.cross(v2);
                R_cm_update.at<float>(0, 2) = v3.at<float>(0, 0);
                R_cm_update.at<float>(1, 2) = v3.at<float>(1, 0);
                R_cm_update.at<float>(2, 2) = v3.at<float>(2, 0);
                if (abs(cv::determinant(R_cm_update) + 1) < 0.5) {
                    R_cm_update.at<float>(0, 2) = -v3.at<float>(0, 0);
                    R_cm_update.at<float>(1, 2) = -v3.at<float>(1, 0);
                    R_cm_update.at<float>(2, 2) = -v3.at<float>(2, 0);
                }

            } else if (directionFound2 && directionFound3) {
                cv::Mat v2 = R_cm_update.colRange(1, 2).clone();
                //cout << 2 << v2 << endl;
                cv::Mat v3 = R_cm_update.colRange(2, 3).clone();
                cv::Mat v1 = v3.cross(v2);
                R_cm_update.at<float>(0, 0) = v1.at<float>(0, 0);
                R_cm_update.at<float>(1, 0) = v1.at<float>(1, 0);
                R_cm_update.at<float>(2, 0) = v1.at<float>(2, 0);
                if (abs(cv::determinant(R_cm_update) + 1) < 0.5) {
                    R_cm_update.at<float>(0, 0) = -v1.at<float>(0, 0);
                    R_cm_update.at<float>(1, 0) = -v1.at<float>(1, 0);
                    R_cm_update.at<float>(2, 0) = -v1.at<float>(2, 0);
                }
            } else if (directionFound1 && directionFound3) {
                cv::Mat v1 = R_cm_update.colRange(0, 1).clone();
                //cout << "3-v3" << v1 << endl;
                cv::Mat v3 = R_cm_update.colRange(2, 3).clone();
                //cout << "3-v3" << v3 << endl;
                cv::Mat v2 = v1.cross(v3);
                R_cm_update.at<float>(0, 1) = v2.at<float>(0, 0);
                R_cm_update.at<float>(1, 1) = v2.at<float>(1, 0);
                R_cm_update.at<float>(2, 1) = v2.at<float>(2, 0);
                if (abs(cv::determinant(R_cm_update) + 1) < 0.5) {
                    R_cm_update.at<float>(0, 1) = -v2.at<float>(0, 0);
                    R_cm_update.at<float>(1, 1) = -v2.at<float>(1, 0);
                    R_cm_update.at<float>(2, 1) = -v2.at<float>(2, 0);
                }

            }
        }

        //cout << "direction:" << directionFound1 << "," << directionFound2 << "," << directionFound3 << endl;
        //cout<<"svd before"<<R_cm_update<<endl;
        SVD svd;
        cv::Mat U, W, VT;


        svd.compute(R_cm_update, W, U, VT);

        R_cm_update = U * VT;
        //cout<<"svd after"<<R_cm_update<<endl;

        //validMF=1;
        //判断是否收敛
        vDensity.clear();
        if (acos((trace(R_cm.t() * R_cm_update)[0] - 1.0)) / 2 < 0.001)
        {cout<<"go outside"<<endl;break;}
    }
    isTracked=1;
    return R_cm_update.clone();
}

sMS Tracking::MeanShift(vector<cv::Point2d> & v2D)
{
    sMS tempMS;
    int numPoint=v2D.size();
    float density;
    cv::Point2d nominator;
    double denominator=0;
    double nominator_x=0;
    double nominator_y=0;
    for(int i=0;i<numPoint;i++)
    {
        double k = exp(-20*norm(v2D.at(i))*norm(v2D.at(i)));
        nominator.x+=k*v2D.at(i).x;
        nominator.y+=k*v2D.at(i).y;
        denominator+=k;
    }
    tempMS.centerOfShift=nominator/denominator;
    tempMS.density=denominator/numPoint;

    return  tempMS;
}

/**
 * @brief 为单目摄像头三角化生成带有线特征的Map，包括MapPoints和MapLine
 */
void Tracking::CreateInitialMapMonoWithLine()
{
        // step1:创建关键帧，即用于初始化的前两帧
        KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
        KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        // step2：将两个关键帧的描述子转为BoW，这里的BoW只有ORB的词袋
        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // step3：将关键帧插入到地图，凡是关键帧，都要插入地图
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);

        // step4：将特征点的3D点包装成MapPoints
        for(size_t i=0; i<mvIniMatches.size(); i++)
        {
            if(mvIniMatches[i]<0)
                continue;

            // Create MapPoint
            cv::Mat worldPos(mvIniP3D[i]);

            // step4.1：用3D点构造MapPoint
            MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);

            // step4.2：为该MapPoint添加属性：
            // a.观测到该MapPoint的关键帧
            // b.该MapPoint的描述子
            // c.该MapPoint的平均观测方向和深度范围

            // step4.3：表示该KeyFrame的哪个特征点对应到哪个3D点
            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            // a.表示该MapPoint可以被哪个KeyFrame观测到，以及对应的第几个特征点
            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            // b.从众多观测到该MapPoint的特征点中挑选出区分度最高的描述子
            pMP->UpdateNormalAndDepth();

            // Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            // Add to Map
            // step4.4：在地图中添加该MapPoint
            mpMap->AddMapPoint(pMP);
        }

        // step5：将特征线包装成MapLines
        for(size_t i=0; i<mvLineMatches.size(); i++)
        {
            if(!mvbLineTriangulated[i])
                continue;

            // Create MapLine
            Vector6d worldPos;
            worldPos << mvLineS3D[i].x, mvLineS3D[i].y, mvLineS3D[i].z, mvLineE3D[i].x, mvLineE3D[i].y, mvLineE3D[i].z;

            //step5.1：用线段的两个端点构造MapLine
            MapLine* pML = new MapLine(worldPos, pKFcur, mpMap);

            //step5.2：为该MapLine添加属性：
            // a.观测到该MapLine的关键帧
            // b.该MapLine的描述子
            // c.该MapLine的平均观测方向和深度范围？

            //step5.3：表示该KeyFrame的哪个特征点可以观测到哪个3D点
            pKFini->AddMapLine(pML,i);
            pKFcur->AddMapLine(pML,i);

            //a.表示该MapLine可以被哪个KeyFrame观测到，以及对应的第几个特征线
            pML->AddObservation(pKFini, i);
            pML->AddObservation(pKFcur, i);

            //b.MapPoint中是选取区分度最高的描述子，pl-slam直接采用前一帧的描述子,这里先按照ORB-SLAM的过程来
            pML->ComputeDistinctiveDescriptors();

            //c.更新该MapLine的平均观测方向以及观测距离的范围
            pML->UpdateAverageDir();

            // Fill Current Frame structure
            mCurrentFrame.mvpMapLines[i] = pML;
            mCurrentFrame.mvbLineOutlier[i] = false;

            // step5.4: Add to Map
            mpMap->AddMapLine(pML);
        }

        // step6：更新关键帧间的连接关系
        // 1.最初是在3D点和关键帧之间建立边，每一个边有一个权重，边的权重是该关键帧与当前关键帧公共3D点的个数
        // 2.加入线特征后，这个关系应该和特征线也有一定的关系，或者就先不加关系，只是单纯的添加线特征

        // step7：全局BA优化，这里需要再进一步修改优化函数，参照OptimizePose函数
        cout << "this Map created with " << mpMap->MapPointsInMap() << " points, and "<< mpMap->MapLinesInMap() << " lines." << endl;
        //Optimizer::GlobalBundleAdjustemnt(mpMap, 20, true); //true代表使用有线特征的BA

        // step8：将MapPoints的中值深度归一化到1，并归一化两帧之间的变换
        // Q：MapPoints的中值深度归一化为1，MapLine是否也归一化？
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f/medianDepth;

        cout << "medianDepth = " << medianDepth << endl;
        cout << "pKFcur->TrackedMapPoints(1) = " << pKFcur->TrackedMapPoints(1) << endl;

        if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<80)
        {
            cout << "Wrong initialization, reseting ... " << endl;
            Reset();
            return;
        }

        // Scale initial baseline
        cv::Mat Tc2w = pKFcur->GetPose();
        Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale Points
        vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); ++iMP)
        {
            if(vpAllMapPoints[iMP])
            {
                MapPoint* pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            }
        }

        // Scale Line Segments
        vector<MapLine*> vpAllMapLines = pKFini->GetMapLineMatches();
        for(size_t iML=0; iML < vpAllMapLines.size(); iML++)
        {
            if(vpAllMapLines[iML])
            {
                MapLine* pML = vpAllMapLines[iML];
                pML->SetWorldPos(pML->GetWorldPos()*invMedianDepth);
            }
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mvpLocalMapLines = mpMap->GetAllMapLines();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        mLastFrame = Frame(mCurrentFrame);

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
        mpMap->SetReferenceMapLines(mvpLocalMapLines);

        mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mState = OK;
    }


void Tracking::CheckReplacedInLastFrame()
{
        // points
        for(int i =0; i<mLastFrame.N; i++)
        {
            MapPoint* pMP = mLastFrame.mvpMapPoints[i];

            if(pMP)
            {
                MapPoint* pRep = pMP->GetReplaced();
                if(pRep)
                {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
        // lines
        for(int i=0; i<mLastFrame.NL; i++)
        {
            MapLine* pML = mLastFrame.mvpMapLines[i];

            if(pML)
            {
                MapLine* pReL = pML->GetReplaced();
                if(pReL)
                {
                    mLastFrame.mvpMapLines[i] = pReL;
                }
            }
        }
    }


bool Tracking::TrackReferenceKeyFrame()
{
        //cout<<"Tracking: with reference keyframe"<<endl;
        // Compute Bag of Words vector
        mCurrentFrame.ComputeBoW();
        int lmatches=0;
        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.9,true);
        LSDmatcher lmatcher;

        vector<MapPoint*> vpMapPointMatches;
        vector<MapLine*> vpMapLineMatches;
        vector<pair<int, int>> vLineMatches;

        int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
        cout<<"Tracking REFERENCE mmatches:"<<nmatches<<endl;

        if(mCurrentFrame.dealWithLine)
        {   lmatches = lmatcher.SearchByProjection(mpReferenceKF,mCurrentFrame,vpMapLineMatches);
            mCurrentFrame.mvpMapLines=vpMapLineMatches;
            cout<<"Tracking REFERENCE lmatches:"<<lmatches<<endl;
        }
        else
        {
            if(nmatches<15)
                return false;
        }

        mCurrentFrame.mvpMapPoints = vpMapPointMatches;

        for(size_t i=0;i<mCurrentFrame.mvpMapLines.size();i++)
        {
            MapLine * ml=mCurrentFrame.mvpMapLines[i];
            if(ml)
            {
                Eigen::Vector3d line_obs=mCurrentFrame.mvKeyLineFunctions[i];
            }
        }

        mCurrentFrame.SetPose(mLastFrame.mTcw);
        cout<<"tracking reference,pose before opti"<<mLastFrame.mTcw<<endl;
        // 通过优化3D-2D的重投影误差来获得位姿
        if(mCurrentFrame.dealWithLine)
            Optimizer::PoseOptimization(&mCurrentFrame, true);
        else
            Optimizer::PoseOptimization(&mCurrentFrame);
        cout<<"tracking reference,pose after opti"<<mCurrentFrame.mTcw<<endl;
        // Discard outliers
        // 剔除优化后的outlier匹配点（MapPoints）
        int nmatchesMap = 0;
        int nmatchesLineMap=0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

        //当 cMurrentFrame.dealWithLine为true，使用Pose去除错误的线
        for(int i=0;i<mCurrentFrame.NL;i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(mCurrentFrame.mvbLineOutlier[i])
                {
                    MapLine* pML=mCurrentFrame.mvpMapLines[i];
                    mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
                    mCurrentFrame.mvbLineOutlier[i]=false;
                    pML->mbTrackInView=false;
                    pML->mnLastFrameSeen=mCurrentFrame.mnId;
                    lmatches--;
                    cout<<"tracking,lmatches"<<lmatches<<endl;
                }
                else if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                    nmatchesLineMap++;
            }
        }
        if(mCurrentFrame.dealWithLine)
            return  nmatchesMap>=10||nmatchesLineMap>=5;
        else
            return nmatchesMap>=10;
}


void Tracking::UpdateLastFrame()
{
        // Update pose according to reference keyframe
        // step1: 更新最近一帧的位姿
        KeyFrame* pRef = mLastFrame.mpReferenceKF;
        cv::Mat Tlr = mlRelativeFramePoses.back();

        mLastFrame.SetPose(Tlr*pRef->GetPose());

        // 如果上一帧为关键帧，或者单目的情况，则退出
        if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for(int i=0; i<mLastFrame.N;i++)
        {
            float z = mLastFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }


        sort(vDepthIdx.begin(),vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for(size_t j=0; j<vDepthIdx.size();j++)
        {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint* pMP = mLastFrame.mvpMapPoints[i];
            if(!pMP)
                bCreateNew = true;
            else if(pMP->Observations()<1)
            {
                bCreateNew = true;
            }

            if(bCreateNew)
            {
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

                mLastFrame.mvpMapPoints[i]=pNewMP;

                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            }
            else
            {
                nPoints++;
            }

            if(vDepthIdx[j].first>mThDepth && nPoints>100)
                break;
        }


        // Create "visual odometry" MapLines
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float,int> > vLineDepthIdx;
        vLineDepthIdx.reserve(mLastFrame.NL);
        int nLines = 0;
        for(int i=0; i<mLastFrame.NL;i++)
        {
            float z = mLastFrame.mvDepthLine[i];
            if(z==1)
            {
                bool bCreateNew = false;
                vLineDepthIdx.push_back(make_pair(z,i));
                MapLine *pML = mLastFrame.mvpMapLines[i];
                if (!pML)
                    bCreateNew = true;
                else if (pML->Observations() < 1) {
                    bCreateNew = true;
                }
                if (bCreateNew) {
                    Vector6d line3D= mLastFrame.obtain3DLine(i);//mvLines3D[i];
                    MapLine *pNewML=new MapLine(line3D,mpMap,&mLastFrame,i);
                    //Vector6d x3D = mLastFrame.mvLines3D(i);
                    //MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

                    mLastFrame.mvpMapLines[i] = pNewML;

                    mlpTemporalLines.push_back(pNewML);
                    nLines++;
                } else {
                    nLines++;
                }

                if ( nLines> 30)
                    break;

            }
        }


}


bool Tracking::TrackWithMotionModel()
{
        cout<<"Tracking: with motion model"<<endl;
        // --step1: 建立ORB特征点的匹配
        ORBmatcher matcher(0.9,true);
        LSDmatcher lmatcher;

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        // --step2: 更新上一帧的位姿
        UpdateLastFrame();

        // --step3:根据Const Velocity Model，估计当前帧的位姿
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        // Project points seen in previous frame
        int th;
        if(mSensor!=System::STEREO)
            th=15;
        else
            th=7;
        // --step4：根据上一帧特征点对应的3D点投影的位置缩小特征点匹配范围
        int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);
        //int lmatches = lmatcher.SearchByProjection(mCurrentFrame, mLastFrame);
    int lmatches2=0;
    if(mCurrentFrame.dealWithLine)
    {
        fill(mCurrentFrame.mvpMapLines.begin(),mCurrentFrame.mvpMapLines.end(), static_cast<MapLine*>(NULL));
        vector<MapLine*> vpMapLineMatches;
        lmatches2 = lmatcher.SearchByProjection(mpReferenceKF,mCurrentFrame,vpMapLineMatches);
        mCurrentFrame.mvpMapLines=vpMapLineMatches;
    }

        // If few matches, uses a wider window search
        // 如果跟踪的点少，则扩大搜索半径再来一次
        if(nmatches<20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
        }

        if(mCurrentFrame.dealWithLine)
        {
            if(nmatches<20 &&lmatches2<6) return false;
        }
        else
        {
            if(nmatches<20 )
                return false;
        }

        // Optimize frame pose with all matches
        // --step5: 优化位姿
        cout<<"tracking motion model,pose before opti"<<mCurrentFrame.mTcw<<endl;
        if(mCurrentFrame.dealWithLine)
            Optimizer::PoseOptimization(&mCurrentFrame, true);
        else
            Optimizer::PoseOptimization(&mCurrentFrame);
        cout<<"tracking motion model,pose after opti"<<mCurrentFrame.mTcw<<endl;
        // Discard outliers
        // --step6：优化位姿后剔除outlier的mvpMapPoints
        int nmatchesMap = 0;
        int nmatchesLineMap=0;

        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                    nmatchesMap++;
            }
        }

        for(int i=0; i<mCurrentFrame.NL;i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(mCurrentFrame.mvbLineOutlier[i])
                {
                    MapLine* pML=mCurrentFrame.mvpMapLines[i];
                    mCurrentFrame.mvpMapLines[i]= static_cast<MapLine*>(NULL);
                    mCurrentFrame.mvbLineOutlier[i]=false;
                    pML->mbTrackInView=false;
                    pML->mnLastFrameSeen=mCurrentFrame.mnId;
                    lmatches2--;
                }
                else if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                    nmatchesLineMap++;
            }

        }

        if(mbOnlyTracking)
        {
            mbVO = nmatchesMap<10;
            return nmatches>20;
        }

        if(mCurrentFrame.dealWithLine)
        {
            return (nmatchesMap >= 10 || nmatchesLineMap >= 6);
        }
        else
        {
            return nmatchesMap>=10;
        }
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<10)
        return false;
    else
        return true;
}


bool Tracking::TrackLocalMapWithLines()
{
        UpdateLocalMap();
        thread threadPoints(&Tracking::SearchLocalPoints, this);
        thread threadLines(&Tracking::SearchLocalLines, this);
        threadPoints.join();
        threadLines.join();

        //cout<<"Tracking: start Poseoptimization"<<endl;
        // step4：更新局部所有MapPoints和MapLines后对位姿再次优化
        Optimizer::PoseOptimization(&mCurrentFrame);
        mnMatchesInliers = 0;
        mnLineMatchesInliers = 0;

        // Update MapPoints Statistics
        // step5：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if(!mbOnlyTracking)
                    {
                        if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                            mnMatchesInliers++;
                    }
                    else
                        mnMatchesInliers++;
                }
                else if(mSensor==System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
            }
        }
        // 更新MapLines Statistics
        // step6：更新当前帧的MapLines被观测程度，并统计跟踪局部地图的效果
        for(int i=0; i<mCurrentFrame.NL; i++)
        {
            if(mCurrentFrame.mvpMapLines[i])
            {
                if(!mCurrentFrame.mvbLineOutlier[i])
                {
                    mCurrentFrame.mvpMapLines[i]->IncreaseFound();
                    if(!mbOnlyTracking)
                    {
                        if(mCurrentFrame.mvpMapLines[i]->Observations()>0)
                            mnLineMatchesInliers++;
                    }
                    else
                        mnLineMatchesInliers++;
                }
                else if(mSensor==System::STEREO)
                    mCurrentFrame.mvpMapLines[i] = static_cast<MapLine*>(NULL);
            }
        }
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
            return false;

        if(mnMatchesInliers<30)
            return false;
        else
            return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
    // "total matches = matches to map + visual odometry matches"
    // Visual odometry matches will become MapPoints if we insert a keyframe.
    // This ratio measures how many MapPoints we could create if we insert a keyframe.
    int nMap = 0; //nTrackedClose
    int nTotal= 0;
    int nNonTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                nTotal++;
                if(mCurrentFrame.mvpMapPoints[i]&&!mCurrentFrame.mvbOutlier[i])
                        nMap++;
                else
                    nNonTrackedClose++;
            }
        }
    }
    else
    {
        // There are no visual odometry matches in the monocular case
        nMap=1;
        nTotal=1;
    }

    const float ratioMap = (float)nMap/fmax(1.0f,nTotal);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    float thMapRatio = 0.35f;
    if(mnMatchesInliers>300)
        thMapRatio = 0.20f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || ratioMap<0.3f) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| ratioMap<thMapRatio) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;
    if(!mCurrentFrame.dealWithLine)
    {
        if(mCurrentFrame.NL==0)
        {
            LineSegment* mpLineSegment;
            mpLineSegment->ExtractLineSegment(this->mImGray, mCurrentFrame.mvKeylinesUn, mCurrentFrame.mLdesc, mCurrentFrame.mvKeyLineFunctions);
            mCurrentFrame.mvDepthLine = vector<float>(mCurrentFrame.mvKeylinesUn.size(),-1.0f);
            mCurrentFrame.mvLines3D=vector<Vector6d>(mCurrentFrame.mvKeylinesUn.size(), static_cast<Vector6d>(NULL));
            for(int i=0; i<mCurrentFrame.mvKeylinesUn.size(); ++i)	{ // each line
                double len = cv::norm(mCurrentFrame.mvKeylinesUn[i].getStartPoint() - mCurrentFrame.mvKeylinesUn[i].getEndPoint());
                vector<cv::Point3d> pts3d;
                double numSmp = (double) min((int)len, 100); //number of line points sampled
                pts3d.reserve(numSmp);//预留数量

                for(int j=0; j<=numSmp; ++j) {
                    // use nearest neighbor to querry depth value
                    // assuming position (0,0) is the top-left corner of image, then the
                    // top-left pixel's center would be (0.5,0.5)
                    cv::Point2d pt = mCurrentFrame.mvKeylinesUn[i].getStartPoint() * (1-j/numSmp) + mCurrentFrame.mvKeylinesUn[i].getEndPoint()* (j/numSmp);
                    if(pt.x<0 || pt.y<0 || pt.x >= this->mImDepth.cols || pt.y >= this->mImDepth.rows ) continue;
                    int row, col; // nearest pixel for pt
                    if((floor(pt.x) == pt.x) && (floor(pt.y) == pt.y)) {// boundary issue
                        col = max(int(pt.x-1),0);
                        row = max(int(pt.y-1),0);
                    } else {
                        col = int(pt.x);
                        row = int(pt.y);
                    }
                    float d=-1;
                    if(this->mImGray.at<float>(row,col) <=0.01) { // no depth info
                        //cout<<"no depth info"<<endl;
                        continue;
                    }
                    else {
                        d = this->mImGray.ptr<float>(row)[col];
                    }
                    cv::Point3d p;

                    // 计算这个点的空间坐标
                    p.z = d;
                    p.x = (col - mCurrentFrame.cx) * p.z *mCurrentFrame.invfx;//K.at<float>(0,0);//invfx;
                    p.y = (row - mCurrentFrame.cy) * p.z *mCurrentFrame.invfy;//K.at<float>(1,1);//invfy;

                    pts3d.push_back(p);

                }
                //如果 点数量少于 10或者 ，就抛弃该线
                if (pts3d.size() < 10.0)//sysPara.ratio_of_collinear_pts
                    continue;

                RandomLine3d tmpLine;
#if 1
                vector<RandomPoint3d> rndpts3d;
                rndpts3d.reserve(pts3d.size());
                // compute uncertainty of 3d points
                for(int j=0; j<pts3d.size();++j) {
                    rndpts3d.push_back(compPt3dCov(pts3d[j], mCurrentFrame.mK, 1));
                }
                // using ransac to extract a 3d line from 3d pts
                tmpLine = extract3dline_mahdist(rndpts3d);
#else
                //计算这个线的参数
        tmpLine = extract3dline(pts3d,origPoints,optiPoints);
#endif
                if(tmpLine.pts.size()/len > 0.4/*sysPara.ratio_of_collinear_pts*/	&&
                   cv::norm(tmpLine.A - tmpLine.B) >0.02) { // sysPara.line3d_length_thresh
                    //this line is reliable
                    mCurrentFrame.mvDepthLine[i]= 1.0f;
                    cout<<"gaibian line 3d"<<endl;
                    mCurrentFrame.mvLines3D[i]<<tmpLine.A.x,tmpLine.A.y,tmpLine.A.z,tmpLine.B.x,tmpLine.B.y,tmpLine.B.z;
                }
            }

            mCurrentFrame.NL=mCurrentFrame.mvKeylinesUn.size();
            mCurrentFrame.mvpMapLines=vector<MapLine *>(mCurrentFrame.NL,static_cast<MapLine*>(NULL));
            mCurrentFrame.mvbLineOutlier=vector<bool>(mCurrentFrame.NL,false);
        }
    }

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        cout<<"Tracking: newKeyframe: RGBD"<<endl;
        mCurrentFrame.UpdatePoseMatrices();
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vector<pair<float, int>> vLineDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        for(int i=0;i<mCurrentFrame.NL;i++)
        {
            float isDepthGood= mCurrentFrame.mvDepthLine[i];
            //cout<<"Tracking: StereoInitialization:线是否存在:"<<isDepthGood<<endl;
            //如果存在， 执行程序
            if(isDepthGood==1)
            {
                vLineDepthIdx.push_back(make_pair(isDepthGood,i));
            }
        }

        if(!vLineDepthIdx.empty())
        {
            //sort(vLineDepthIdx.begin(),vLineDepthIdx.end());
            int nLines = 0;
            for(size_t j=0; j<vLineDepthIdx.size();j++)
            {
                int i = vLineDepthIdx[j].second;

                bool bCreateNew = false;

                MapLine* pMP = mCurrentFrame.mvpMapLines[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapLines[i] = static_cast<MapLine*>(NULL);
                }

                if(bCreateNew)
                {
                    Vector6d line3D= mCurrentFrame.obtain3DLine(i);//mvLines3D[i];
                    MapLine *pNewML=new MapLine(line3D,pKF,mpMap);
                    pNewML->AddObservation(pKF,i);
                    pKF->AddMapLine(pNewML,i);
                    pNewML->ComputeDistinctiveDescriptors();
                    pNewML->UpdateAverageDir();
                    mpMap->AddMapLine(pNewML);
                    mCurrentFrame.mvpMapLines[i]=pNewML;
                    nLines++;
                }
                else
                {
                    nLines++;
                }

                if(nLines>20)
                    break;
            }
        }
        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }
    mpLocalMapper->InsertKeyFrame(pKF);
    mpLocalMapper->SetNotStop(false);
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++; //将要match的
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::SearchLocalLines()
{
        // step1：遍历在当前帧的mvpMapLines，标记这些MapLines不参与之后的搜索，因为当前的mvpMapLines一定在当前帧的视野中
        for(vector<MapLine*>::iterator vit=mCurrentFrame.mvpMapLines.begin(), vend=mCurrentFrame.mvpMapLines.end(); vit!=vend; vit++)
        {
            MapLine* pML = *vit;
            if(pML)
            {
                if(pML->isBad())
                {
                    *vit = static_cast<MapLine*>(NULL);
                } else{
                    // 更新能观测到该线段的帧数加1
                    pML->IncreaseVisible();
                    // 标记该点被当前帧观测到
                    pML->mnLastFrameSeen = mCurrentFrame.mnId;
                    // 标记该线段将来不被投影，因为已经匹配过
                    pML->mbTrackInView = false;
                }
            }
        }

        int nToMatch = 0;

        // step2：将所有局部MapLines投影到当前帧，判断是否在视野范围内，然后进行投影匹配
        for(vector<MapLine*>::iterator vit=mvpLocalMapLines.begin(), vend=mvpLocalMapLines.end(); vit!=vend; vit++)
        {
            MapLine* pML = *vit;

            // 已经被当前帧观测到MapLine，不再判断是否能被当前帧观测到
            if(pML->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if(pML->isBad())
                continue;

            // step2.1：判断LocalMapLine是否在视野内
            if(mCurrentFrame.isInFrustum(pML, 0.6))
            {
                // 观察到该点的帧数加1，该MapLine在某些帧的视野范围内
                pML->IncreaseVisible();
                nToMatch++;
            }
        }
        if(nToMatch>0)
        {
            LSDmatcher matcher;
            int th = 1;

            if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
                th=5;

            matcher.SearchByProjection(mCurrentFrame, mvpLocalMapLines, th);
        }
    }


void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    mpMap->SetReferenceMapLines(mvpLocalMapLines);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
    UpdateLocalLines();
}

void Tracking::UpdateLocalLines()
{
        // step1：清空局部MapLines
        mvpLocalMapLines.clear();

        // step2：遍历局部关键帧mvpLocalKeyFrames
        for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            KeyFrame* pKF = *itKF;
            const vector<MapLine*> vpMLs = pKF->GetMapLineMatches();
            //step3：将局部关键帧的MapLines添加到mvpLocalMapLines
            for(vector<MapLine*>::const_iterator itML=vpMLs.begin(), itEndML=vpMLs.end(); itML!=itEndML; itML++)
            {
                MapLine* pML = *itML;
                if(!pML)
                    continue;
                if(pML->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                    continue;
                if(!pML->isBad())
                {
                    mvpLocalMapLines.push_back(pML);
                    pML->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
            }
        }
    }

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{
    mpViewer->RequestStop();

    cout << "System Reseting" << endl;
    while(!mpViewer->isStopped())
        usleep(3000);

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    //mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    mpViewer->Release();
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

} //namespace StructureSLAM
