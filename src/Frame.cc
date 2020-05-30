/**
* This file is part of Structure-SLAM.
*Copyright (C) 2020 Yanyan Li <yanyan.li at tum.de> (Technical University of Munich)
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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>
#include <include/LocalMapping.h>
namespace StructureSLAM
{

    long unsigned int Frame::nNextId=0;
    bool Frame::mbInitialComputations=true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

    Frame::Frame()
    {}

//Copy Constructor
Frame::Frame(const Frame &frame)
        :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
         mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
         mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
         mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
         mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
         mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
         mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
         mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
         mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
         mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
         mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
         mLdesc(frame.mLdesc), NL(frame.NL), mvKeylinesUn(frame.mvKeylinesUn), mvpMapLines(frame.mvpMapLines),  //线特征相关的类成员变量
         mvbLineOutlier(frame.mvbLineOutlier), mvKeyLineFunctions(frame.mvKeyLineFunctions),mvDepthLine(frame.mvDepthLine),
         mvLines3D(frame.mvLines3D),mv3DLineforMap(frame.mv3DLineforMap),dealWithLine(frame.dealWithLine),blurNumber(frame.blurNumber),vSurfaceNormal(frame.vSurfaceNormal),
         vVanishingDirection(frame.vVanishingDirection),mVF3DLines(frame.mVF3DLines), vSurfaceNormalx(frame.vSurfaceNormalx),vSurfaceNormaly(frame.vSurfaceNormaly), vSurfaceNormalz(frame.vSurfaceNormalz)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}

Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
        :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
         mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB and LSD extraction
    ExtractORB(0,imGray);
    ExtractLSD(imGray);
    dealWithLine = true;
    N = mvKeys.size();
    NL = mvKeylinesUn.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();


    //TODO  vanishing direction of lines

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    mvpMapLines = vector<MapLine*>(NL,static_cast<MapLine*>(NULL));
    mvbLineOutlier = vector<bool>(NL,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractLSD(const cv::Mat &im)
{
    mpLineSegment->ExtractLineSegment(im, mvKeylinesUn, mLdesc, mvKeyLineFunctions);
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::featureSelect(const cv::Mat &im)
{
    Mat gray1;
    double m1,sd1;
    Mat tmp_m1,tmp_sd1;
    Laplacian(im, gray1, CV_16S,3);
    //计算均值和方差
    meanStdDev(gray1,tmp_m1, tmp_sd1);
    m1 = tmp_m1.at<double>(0, 0);		//均值
    sd1 = tmp_sd1.at<double>(0, 0);		//标准差
    //cout << "原图像：" << endl;
    //cout << "均值: " << m1 << " , 方差: " << sd1<< endl;
    blurNumber=sd1*sd1;
    if (sd1*sd1 < 1200 || N<50)
    {
        cout << "blur,sd1^2="<<sd1*sd1 << endl;
        dealWithLine=true;
    }
    else
    {
        cout << "not blur,sd1^2="<<sd1*sd1 << endl;
    }


}

// line descriptor MAD
void Frame::lineDescriptorMAD( vector<vector<DMatch>> line_matches, double &nn_mad, double &nn12_mad) const
{
    vector<vector<DMatch>> matches_nn, matches_12;
    matches_nn = line_matches;
    matches_12 = line_matches;
//    cout << "Frame::lineDescriptorMAD——matches_nn = "<<matches_nn.size() << endl;

    // estimate the NN's distance standard deviation
    double nn_dist_median;
    sort( matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist());
    nn_dist_median = matches_nn[int(matches_nn.size()/2)][0].distance;

    for(unsigned int i=0; i<matches_nn.size(); i++)
        matches_nn[i][0].distance = fabsf(matches_nn[i][0].distance - nn_dist_median);
    sort(matches_nn.begin(), matches_nn.end(), compare_descriptor_by_NN_dist());
    nn_mad = 1.4826 * matches_nn[int(matches_nn.size()/2)][0].distance;

    // estimate the NN's 12 distance standard deviation
    double nn12_dist_median;
    sort( matches_12.begin(), matches_12.end(), conpare_descriptor_by_NN12_dist());
    nn12_dist_median = matches_12[int(matches_12.size()/2)][1].distance - matches_12[int(matches_12.size()/2)][0].distance;
    for (unsigned int j=0; j<matches_12.size(); j++)
        matches_12[j][0].distance = fabsf( matches_12[j][1].distance - matches_12[j][0].distance - nn12_dist_median);
    sort(matches_12.begin(), matches_12.end(), compare_descriptor_by_NN_dist());
    nn12_mad = 1.4826 * matches_12[int(matches_12.size()/2)][0].distance;
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

bool Frame::isInFrustum(MapLine *pML, float viewingCosLimit)
{
    pML->mbTrackInView = false;

    Vector6d P = pML->GetWorldPos();

    cv::Mat SP = (Mat_<float>(3,1) << P(0), P(1), P(2));
    cv::Mat EP = (Mat_<float>(3,1) << P(3), P(4), P(5));

    // 两个端点在相机坐标系下的坐标
    const cv::Mat SPc = mRcw*SP + mtcw;
    const float &SPcX = SPc.at<float>(0);
    const float &SPcY = SPc.at<float>(1);
    const float &SPcZ = SPc.at<float>(2);

    const cv::Mat EPc = mRcw*EP + mtcw;
    const float &EPcX = EPc.at<float>(0);
    const float &EPcY = EPc.at<float>(1);
    const float &EPcZ = EPc.at<float>(2);

    // 检测两个端点的Z值是否为正
    if(SPcZ<0.0f || EPcZ<0.0f)
        return false;

    // V-D 1) 将端点投影到当前帧上，并判断是否在图像内
    const float invz1 = 1.0f/SPcZ;
    const float u1 = fx * SPcX * invz1 + cx;
    const float v1 = fy * SPcY * invz1 + cy;

    if(u1<mnMinX || u1>mnMaxX)
        return false;
    if(v1<mnMinY || v1>mnMaxY)
        return false;

    const float invz2 = 1.0f/EPcZ;
    const float u2 = fx*EPcX*invz2 + cx;
    const float v2 = fy*EPcY*invz2 + cy;

    if(u2<mnMinX || u2>mnMaxX)
        return false;
    if(v2<mnMinY || v2>mnMaxY)
        return false;

    // V-D 3)计算MapLine到相机中心的距离，并判断是否在尺度变化的距离内
    const float maxDistance = pML->GetMaxDistanceInvariance();
    const float minDistance = pML->GetMinDistanceInvariance();
    // 世界坐标系下，相机到线段中点的向量，向量方向由相机指向中点
    const cv::Mat OM = 0.5*(SP+EP) - mOw;
    const float dist = cv::norm(OM);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    // V-D 2)计算当前视角和平均视角夹角的余弦值，若小于cos(60°),即夹角大于60°则返回
    Vector3d Pn = pML->GetNormal();
    cv::Mat pn = (Mat_<float>(3,1) << Pn(0), Pn(1), Pn(2));
    const float viewCos = OM.dot(pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    // V-D 4) 根据深度预测尺度（对应特征在一层）
    const int nPredictedLevel = pML->PredictScale(dist, mfLogScaleFactor);

    // Data used by the tracking
    // 标记该特征将来要被投影
    pML->mbTrackInView = true;
    pML->mTrackProjX1 = u1;
    pML->mTrackProjY1 = v1;
    pML->mTrackProjX2 = u2;
    pML->mTrackProjY2 = v2;
    pML->mnTrackScaleLevel = nPredictedLevel;
    pML->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

vector<size_t> Frame::GetLinesInArea(const float &x1, const float &y1, const float &x2, const float &y2, const float &r,
                                     const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;

    vector<KeyLine> vkl = this->mvKeylinesUn;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>0);

//    for(vector<KeyLine>::iterator vit=vkl.begin(), vend=vkl.end(); vit!=vend; vit++)
    for(size_t i=0; i<vkl.size(); i++)
    {
        KeyLine keyline = vkl[i];

        // 1.对比中点距离
        float distance = (0.5*(x1+x2)-keyline.pt.x)*(0.5*(x1+x2)-keyline.pt.x)+(0.5*(y1+y2)-keyline.pt.y)*(0.5*(y1+y2)-keyline.pt.y);
        if(distance > r*r)
            continue;

        // 2.比较斜率，KeyLine的angle就是代表斜率
        float slope = (y1-y2)/(x1-x2)-keyline.angle;
        if(slope > r*0.01)
            continue;

        // 3.比较金字塔层数
        if(bCheckLevels)
        {
            if(keyline.octave<minLevel)
                continue;
            if(maxLevel>=0 && keyline.octave>maxLevel)
                continue;
        }

        vIndices.push_back(i);
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

Vector6d Frame::obtain3DLine(const int &i)
{
    Vector6d Lines3D=mvLines3D[i];
    cv::Mat Ac =(Mat_<float>(3,1) <<Lines3D(0),Lines3D(1),Lines3D(2));
    cv::Mat A= mRwc*Ac+mOw;
    cv::Mat Bc = (Mat_<float>(3,1) << Lines3D(3),Lines3D(4),Lines3D(5));
    cv::Mat B= mRwc*Bc+mOw;
    Lines3D<<A.at<float>(0,0),A.at<float>(1,0),A.at<float>(2,0),B.at<float>(0,0),B.at<float>(1,0),B.at<float>(2,0);
    return Lines3D;
}


} //namespace StructureSLAM