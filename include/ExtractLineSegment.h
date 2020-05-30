/**
* This file is part of Structure-SLAM.
* Copyright (C) 2020 Yanyan Li <yanyan.li at tum.de> (Technical University of Munich)
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

#ifndef ORB_SLAM2_LINEFEATURE_H
#define ORB_SLAM2_LINEFEATURE_H

#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <opencv2/features2d/features2d.hpp>
// Thirdparty中的#include <line_descriptor_custom.hpp>
//#include <line_descriptor/descriptor_custom.hpp>
//

#include <eigen3/Eigen/Core>
#include "auxiliar.h"

using namespace std;
using namespace cv;
//using namespace line_descriptor;      // Thirdparty中的

namespace StructureSLAM
{
    class LineSegment
    {
    public:
        // constructor
        LineSegment();

        ~LineSegment(){}

        // 提取线特征,计算线特征描述子
        void ExtractLineSegment(const Mat &img, vector<KeyLine> &keylines, Mat &ldesc, vector<Vector3d> &keylineFunctions, int scale = 1.2, int numOctaves = 1);

        // 线特征匹配
        void LineSegmentMatch(Mat &ldesc_1, Mat &ldesc_2);

        // 线特征的描述子距离中位值
        void lineDescriptorMAD();

        // 求线特征的观测线段和重投影线段的重合率
        double lineSegmentOverlap(double spl_obs, double epl_obs, double spl_proj, double epl_proj  );

    protected:
        vector<vector<DMatch>> line_matches;
        double nn_mad, nn12_mad;
    };
}


#endif //ORB_SLAM2_LINEFEATURE_H
