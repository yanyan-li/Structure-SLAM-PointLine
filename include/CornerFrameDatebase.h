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
#ifndef ORB_SLAM2_CORNERFRAMEDATEBASE_H
#define ORB_SLAM2_CORNERFRAMEDATEBASE_H

#include "Frame.h"
#include <math.h>
#include "MapPoint.h"
#include <mutex>
namespace StructureSLAM{
class Frame;

class CornerFrameDatebase{

public:
    CornerFrameDatebase();
    //添加帧
    void addFrame(Frame pF);
    //释放一帧
    void eraseFrame();

    //清除
    void clear();

    //计算角度
    double computeCornerAngle();
    //获得当前帧数
    int getSizeofCornerFrames();
    vector<Frame> getCornerFrame();
    //五张图的mappoint
    vector<MapPoint *> getCornerMappoint();

    int cornerObservation();
    void prepareCornerInformation();



protected:
    std::vector<Frame> cornerFrameDatabase;
    std::list<Frame> mqcornerFrameDatabase;

    vector<MapPoint *> vpCornerMP;
    // Mutex
    std::mutex mMutex;
    Frame * tmpFrame;

};

}


#endif //ORB_SLAM2_CORNERFRAMEDATEBASE_H
