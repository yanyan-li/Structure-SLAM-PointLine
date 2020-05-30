/**
* This file is part of Structure-SLAM.
* Copyright (C) 2020 Yanyan Li <yanyan.li at tum.de> (Technical University of Munich)
*
*/

#ifndef ORB_SLAM2_LINEEDGE_H
#define ORB_SLAM2_LINEEDGE_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

using namespace g2o;
namespace types_six_dof_expmap {
    void init();
}
using namespace Eigen;

typedef Matrix<double, 6, 6> Matrix6d;

/**
 * @brief Line Segment point vertex, XYZ
 */
//class VertexLinePointXYZ : public BaseVertex<3, Vector3d>
//{
//public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    VertexLinePointXYZ();
//    virtual bool read(std::istream& is);
//    virtual bool write(std::ostream& os) const ;
//
//    virtual void setToOriginImpl(){
//        _estimate.fill(0.);
//    }
//
//    virtual void oplusImpl(const double* update)
//    {
//        Eigen::Map<const Vector3d> v(update);
//        _estimate += v;
//    }
//};

class EdgeLineProjectXYZOnlyPose : public BaseUnaryEdge<3, Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeLineProjectXYZOnlyPose() {}

    virtual void computeError()
    {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        Vector3d obs = _measurement;    //线段所在直线参数
        Vector2d proj = cam_project(v1->estimate().map(Xw));    //MapLine端点在像素平面上的投影
        //cout<<"proj"<<proj(0)<<", "<<proj(1)<<endl;
        _error(0) = obs(0) * proj(0) + obs(1) * proj(1) + obs(2);  //线段投影端点到观测直线距离
        //cout<<"error(0)"<<_error(0)<<endl;
    }
    double chiline()
    {
        return  _error(0)*_error(0);
    }

    virtual void linearizeOplus()
    {
        VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
        Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz * invz;

        double lx = _measurement(0);
        double ly = _measurement(1);

        // 1*6 jacobian
//        _jacobianOplusXi(0,0) = fy*ly + fx*lx*x*y*invz_2 + fy*ly*y*y*invz_2;
//        _jacobianOplusXi(0,1) = -fx*lx - fx*lx*x*x*invz_2 - fy*ly*x*y*invz_2;
//        _jacobianOplusXi(0,2) = fx*lx*y*invz - fy*ly*x*invz;
//        _jacobianOplusXi(0,3) = -fx*lx*invz;
//        _jacobianOplusXi(0.4) = -fy*ly*invz;
//        _jacobianOplusXi(0,5) = (fx*lx*x+fy*ly*y)*invz_2;

        _jacobianOplusXi(0,0) = -fy*ly - fx*lx*x*y*invz_2 - fy*ly*y*y*invz_2;
        _jacobianOplusXi(0,1) = fx*lx + fx*lx*x*x*invz_2 + fy*ly*x*y*invz_2;
        _jacobianOplusXi(0,2) = -fx*lx*y*invz + fy*ly*x*invz;
        _jacobianOplusXi(0,3) = fx*lx*invz;
        _jacobianOplusXi(0.4) = fy*ly*invz;
        _jacobianOplusXi(0,5) = -(fx*lx*x+fy*ly*y)*invz_2;
    }

    bool read(std::istream& is)
    {
        for(int i=0; i<3; i++)
        {
            is >> _measurement[i];
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                is >> information()(i, j);
                if(i!=j)
                    information()(j,i) = information()(i,j);
            }
        }
        return true;
    }

    bool write(std::ostream& os) const
    {
        for(int i=0; i<3; i++)
        {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                os << " " << information()(i,j);
            }
        }
        return os.good();
    }

    Vector2d project2d(const Vector3d& v)
    {
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }

    Vector2d cam_project(const Vector3d& trans_xyz)
    {
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    Vector3d Xw;    //MapLine的一个端点在世界坐标系的位置
    double fx, fy, cx, cy;  //相机内参数
};

/**
 * 线段端点和相机位姿之间的边
 */
//class EdgeLineProjectXYZ : public BaseBinaryEdge<3, Vector3d, g2o::VertexSE3Expmap, VertexLinePointXYZ>
//{
//public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//    EdgeLineProjectXYZ() {}
//
//    virtual void computeError()
//    {
//        const VertexSE3Expmap* v1 = static_cast<VertexSE3Expmap *>(_vertices[0]);
//        const VertexLinePointXYZ* v2 = static_cast<VertexLinePointXYZ*>(_vertices[1]);
//
//        Vector3d obs = _measurement;    //线段所在直线参数
//        Vector2d proj = cam_project(v1->estimate().map(Xw));
//        _error(0) = obs(0) * proj(0) + obs(1)*proj(1) + obs(2);
//    }
//
//    virtual void linearizeOplus()
//    {
//        // 位姿顶点
//        VertexSE3Expmap *vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
//        SE3Quat T(vi->estimate());
//        Vector3d xyz_trans = vi->estimate().map(Xw);    //线段端点的世界坐标系转换到相机坐标系下
//
//        // 线段端点顶点
//        VertexLinePointXYZ *vj = static_cast<VertexLinePointXYZ *>(_vertices[1]);
//        Vector3d xyz = vj->estimate();
//
//        double x = xyz_trans[0];
//        double y = xyz_trans[1];
//        double invz = 1.0/xyz_trans[2];
//        double invz_2 = invz * invz;
//
//        double lx = _measurement(0);
//        double ly = _measurement(1);
//
//        // 1*6 jacobian
//        _jacobianOplusXi(0,0) = fy*ly + fx*lx*x*y*invz_2 + fy*ly*y*y*invz_2;
//        _jacobianOplusXi(0,1) = -fx*lx - fx*lx*x*x*invz_2 - fy*ly*x*y*invz_2;
//        _jacobianOplusXi(0,2) = fx*lx*y*invz - fy*ly*x*invz;
//        _jacobianOplusXi(0,3) = -fx*lx*invz;
//        _jacobianOplusXi(0.4) = -fy*ly*invz;
//        _jacobianOplusXi(0,5) = (fx*lx*x+fy*ly*y)*invz_2;
//
//        Matrix<double, 1, 3> tmp;
//        tmp(0,0) = fx*lx;
//        tmp(0,1) = fy*ly;
//        tmp(0,2) = -(fx*lx*x+fy*ly*y)*invz;
//
//        Matrix<double, 3, 3> R;
//        R = T.rotation().toRotationMatrix();
//
//        _jacobianOplusXj = -1. * invz * R;
////        _jacobianOplusXj = -1. * invz * tmp * R;
//    }
//
//    bool read(std::istream& is)
//    {
//        for(int i=0; i<3; i++)
//        {
//            is >> _measurement[i];
//        }
//
//        for (int i = 0; i < 3; ++i) {
//            for (int j = i; j < 3; ++j) {
//                is >> information()(i, j);
//                if(i!=j)
//                    information()(j,i) = information()(i,j);
//            }
//        }
//        return true;
//    }
//
//    bool write(std::ostream& os) const
//    {
//        for(int i=0; i<3; i++)
//        {
//            os << measurement()[i] << " ";
//        }
//
//        for (int i = 0; i < 3; ++i) {
//            for (int j = i; j < 3; ++j) {
//                os << " " << information()(i,j);
//            }
//        }
//        return os.good();
//    }
//
//    Vector2d project2d(const Vector3d& v)
//    {
//        Vector2d res;
//        res(0) = v(0)/v(2);
//        res(1) = v(1)/v(2);
//        return res;
//    }
//
//    Vector2d cam_project(const Vector3d& trans_xyz)
//    {
//        Vector2d proj = project2d(trans_xyz);
//        Vector2d res;
//        res[0] = proj[0]*fx + cx;
//        res[1] = proj[1]*fy + cy;
//        return res;
//    }
//
//    Vector3d Xw;    //MapLine的一个端点在世界坐标系的位置
//    double fx, fy, cx, cy;  //相机内参数
//};

/**
 * 线段端点和相机位姿之间的边，线段端点的类型仍然采用g2o中的3D坐标类型
 */
class EdgeLineProjectXYZ : public BaseBinaryEdge<3, Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeLineProjectXYZ() {}

    virtual void computeError()
    {
        const VertexSE3Expmap* v1 = static_cast<VertexSE3Expmap *>(_vertices[0]);
        const VertexSBAPointXYZ* v2 = static_cast<VertexSBAPointXYZ*>(_vertices[1]);

        Vector3d obs = _measurement;    //线段所在直线参数
        Vector2d proj = cam_project(v1->estimate().map(Xw));
        _error(0) = obs(0) * proj(0) + obs(1)*proj(1) + obs(2);
    }

    virtual void linearizeOplus()
    {
        // 位姿顶点
        VertexSE3Expmap *vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
        SE3Quat T(vj->estimate());
        Vector3d xyz_trans = T.map(Xw);    //线段端点的世界坐标系转换到相机坐标系下

        // 线段端点顶点
//        VertexLinePointXYZ *vj = static_cast<VertexLinePointXYZ *>(_vertices[1]);
        VertexSBAPointXYZ *vi = static_cast<VertexSBAPointXYZ *>(_vertices[0]);
        Vector3d xyz = vi->estimate();

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz * invz;

        double lx = _measurement(0);
        double ly = _measurement(1);

        // 3*6 jacobian
        // 1.这是最开始推导雅克比时，有负号的
//        _jacobianOplusXj(0,0) = fy*ly + fx*lx*x*y*invz_2 + fy*ly*y*y*invz_2;
//        _jacobianOplusXj(0,1) = -fx*lx - fx*lx*x*x*invz_2 - fy*ly*x*y*invz_2;
//        _jacobianOplusXj(0,2) = fx*lx*y*invz - fy*ly*x*invz;
//        _jacobianOplusXj(0,3) = -fx*lx*invz;
//        _jacobianOplusXj(0.4) = -fy*ly*invz;
//        _jacobianOplusXj(0,5) = (fx*lx*x+fy*ly*y)*invz_2;

        // 雅克比没有负号的
        _jacobianOplusXj(0,0) = -fy*ly - fx*lx*x*y*invz_2 - fy*ly*y*y*invz_2;
        _jacobianOplusXj(0,1) = fx*lx + fx*lx*x*x*invz_2 + fy*ly*x*y*invz_2;
        _jacobianOplusXj(0,2) = -fx*lx*y*invz + fy*ly*x*invz;
        _jacobianOplusXj(0,3) = fx*lx*invz;
        _jacobianOplusXj(0.4) = fy*ly*invz;
        _jacobianOplusXj(0,5) = -(fx*lx*x+fy*ly*y)*invz_2;

        Matrix<double, 3, 3, Eigen::ColMajor> tmp;
        tmp = Eigen::Matrix3d::Zero();
        tmp(0,0) = fx*lx;
        tmp(0,1) = fy*ly;
        tmp(0,2) = -(fx*lx*x+fy*ly*y)*invz;

        Matrix<double, 3, 3> R;
        R = T.rotation().toRotationMatrix();

//        _jacobianOplusXi = -1. * invz * tmp * R;
        _jacobianOplusXi = 1. * invz * tmp * R;
    }

    bool read(std::istream& is)
    {
        for(int i=0; i<3; i++)
        {
            is >> _measurement[i];
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                is >> information()(i, j);
                if(i!=j)
                    information()(j,i) = information()(i,j);
            }
        }
        return true;
    }

    bool write(std::ostream& os) const
    {
        for(int i=0; i<3; i++)
        {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                os << " " << information()(i,j);
            }
        }
        return os.good();
    }

    Vector2d project2d(const Vector3d& v)
    {
        Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }

    Vector2d cam_project(const Vector3d& trans_xyz)
    {
        Vector2d proj = project2d(trans_xyz);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    Vector3d Xw;    //MapLine的一个端点在世界坐标系的位置
    double fx, fy, cx, cy;  //相机内参数
};

#endif //ORB_SLAM2_LINEEDGE_H
