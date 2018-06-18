


#ifndef DRAWER_HPP
#define DRAWER_HPP
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include "detectconelane.hpp"




class Drawer{
    public:
        Drawer(std::map<std::string,std::string> commandlineArgs, DetectConeLane &detectconelane);
        void drawPoses();
        void drawCones();
        void drawSurfaces(bool,bool);
        void drawGraph();

    private:
        DetectConeLane& detectconelane;
        std::vector<opendlv::logic::perception::GroundSurfaceArea> m_surfaces = {};
        Eigen::MatrixXd m_cones = {};
        std::vector<Eigen::Vector3d> m_poses = {};
        Eigen::Vector3d m_pose = {};
        void drawPath(opendlv::logic::perception::GroundSurfaceArea surface);
        void drawFinalCones(opendlv::logic::perception::GroundSurfaceArea surface);
        Eigen::MatrixXd Spherical2Cartesian(double, double, double);
        const double DEG2RAD = 0.017453292522222; // PI/180.0

};
#endif