#ifndef MEC_TRANSFORMATION
#define MEC_TRANSFORMATION
#include <string>
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#define PI 3.141592652
#define G 9.80665
namespace mec_transformation {
   typedef struct
   {
      double Lat;
      double Lon;
      double High;
   }llh_InitTypeDef;
   typedef struct
   {
      double xx;
      double yy;
      double zz;
   }xyz_InitTypeDef;
   typedef struct
   {
      double e;
      double n;
      double u;
   }enu_InitTypeDef;
   typedef struct
   {
      double n;
      double e;
      double d;
   }ned_InitTypeDef;
   typedef struct
   {
      double roll;
      double pitch;
      double yaw;
   }rpy_InitTypeDef;

   double deg2rad(double degrees);
   double rad2deg(double radians);

   xyz_InitTypeDef Radii_of_curvature(double Lat);
   llh_InitTypeDef lever_arm(double L1, double L2, double H1,double roll,double pitch,double yaw,double lever_arm_x,double lever_arm_y,double lever_arm_z);
   xyz_InitTypeDef lla2xyz(double L1, double L2, double H1);
   enu_InitTypeDef lla2enu(double lat0,double lon0,double alt0,double lat1,double lon1,double alt1);
   ned_InitTypeDef lla2ned(double lat0,double lon0,double alt0,double lat1,double lon1,double alt1);
   Eigen::Matrix4d rotation_matrix_eular( rpy_InitTypeDef rpy);
   xyz_InitTypeDef enu2ecef(double lat0, double lon0, double h0, double east, double north, double up);
   llh_InitTypeDef ecef2llh(const xyz_InitTypeDef& ecef);
   llh_InitTypeDef enu2llh(const enu_InitTypeDef& enu_A,const llh_InitTypeDef& llh_0);

}
#endif