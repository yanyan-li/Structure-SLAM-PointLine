/**
* This file is part of Structure-SLAM.
*
*
*/
/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
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

#ifndef ORB_SLAM2_LSDMATCHER_H
#define ORB_SLAM2_LSDMATCHER_H

#include "MapLine.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace StructureSLAM
{
    class LSDmatcher
    {
    public:
        static const int TH_HIGH, TH_LOW;

        LSDmatcher(float nnratio=0.6, bool checkOri=true);

        int SearchByDescriptor(KeyFrame* pKF, Frame &currentF, std::vector<MapLine*> &vpMapLineMatches);
        int SearchByDescriptor(KeyFrame* pKF, KeyFrame *pKF2, std::vector<MapLine*> &vpMapLineMatches);
        int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);
        int SearchByProjection(KeyFrame* pKF,Frame &currentF, vector<MapLine*> &vpMapLineMatches);
        int SearchByProjection(Frame &F, const std::vector<MapLine*> &vpMapLines, const float th=3);
        int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapLine*> &vpLines, std::vector<MapLine*> &vpMatched, int th);
        int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapLine *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
        int SerachForInitialize(Frame &InitialFrame, Frame &CurrentFrame, std::vector<std::pair<int,int>> &LineMatches);
        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<std::pair<size_t, size_t>> &vMatchedPairs);

        // Project MapLines into KeyFrame and search for duplicated MapLines
        int Fuse(KeyFrame* pKF, const vector<MapLine *> &vpMapLines, const float th=3.0);

        int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapLine*> &vpLines, float th, vector<MapLine *> &vpReplaceLine);

        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    protected:
        float RadiusByViewingCos(const float &viewCos);
        float mfNNratio;
        bool mbCheckOrientation;
    };
}


#endif //ORB_SLAM2_LSDMATCHER_H
