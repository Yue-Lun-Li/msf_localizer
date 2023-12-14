#include <msf_localizer/mec_transformation.h>

mec_transformation::xyz_InitTypeDef mec_transformation::Radii_of_curvature(double Lat){
    mec_transformation::xyz_InitTypeDef Rmn;

    double R_0 = 6378137;
    double e = 0.0818191908425;
    double temp = 1 - pow((e * sin(Lat*PI/180)),2); 

    Rmn.xx = R_0 * (1 - e*e) / pow(temp,1.5);//RM
    Rmn.yy = R_0 / sqrt(temp);//Rn
    return Rmn;
}

mec_transformation::llh_InitTypeDef mec_transformation::lever_arm(double L1, double L2, double H1,double roll,double pitch,double yaw,double lever_arm_x,double lever_arm_y,double lever_arm_z){
    mec_transformation::llh_InitTypeDef LLA;
    static mec_transformation::xyz_InitTypeDef radii = Radii_of_curvature(L1);

    roll = -roll;
    pitch = - pitch;
    yaw = -yaw;

    Eigen::Vector3f b2a_in_base(lever_arm_x, lever_arm_y, lever_arm_z); //base_link(IMU) to antenna vector in vehicle-frame(base_link)
    Eigen::Matrix3f tf_baseton;
    Eigen::Matrix3f D_inv;
    D_inv << 1/(radii.xx + H1),                               0.0,   0.0,
                        0.0,  1/(radii.yy + H1)/cos(L1*PI/180),   0.0,
                        0.0,                               0.0,  -1.0;
    Eigen::Vector3f b2a_in_n; //base_link(IMU) to antenna vector in navigation-frame

    Eigen::AngleAxisf rot_x((roll + 180)*PI/180, Eigen::Vector3f::UnitX());    
    Eigen::AngleAxisf rot_y(pitch*PI/180, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(yaw*PI/180, Eigen::Vector3f::UnitZ());

    tf_baseton = (rot_x*rot_y*rot_z).matrix(); //the angle between vehicle-frame(base_link) and map-frame along z-axis
    b2a_in_n = D_inv * tf_baseton * b2a_in_base;

    LLA.Lat = L1 - b2a_in_n(0,0) * 180/PI;
    LLA.Lon = L2 - b2a_in_n(1,0) * 180/PI;
    LLA.High = H1 - b2a_in_n(2,0); 

    return (LLA);
}

mec_transformation::xyz_InitTypeDef mec_transformation::lla2xyz(double L1, double L2, double H1){
    mec_transformation::xyz_InitTypeDef xyzC;
    double a, b, e;
    double sinphi, cosphi, coslam, sinlam, tan2phi;
    double tmp, tmpden, tmp2;

    L1=(L1*PI)/180;
    L2=(L2*PI)/180;

    a = 6378137.0000;
    b = 6356752.3142;
    e = sqrt(1-(b/a)*(b/a));  

    sinphi = sin(L1);
    cosphi = cos(L1);
    coslam = cos(L2);
    sinlam = sin(L2);
    tan2phi = (tan(L1))*(tan(L1));
    tmp = 1 - e*e;
    tmpden = sqrt( 1 + tmp*tan2phi );

    xyzC.xx = (a*coslam)/tmpden + H1*coslam*cosphi;

    xyzC.yy = (a*sinlam)/tmpden + H1*sinlam*cosphi;

    tmp2 = sqrt(1 - e*e*sinphi*sinphi);
    xyzC.zz = (a*tmp*sinphi)/tmp2 + H1*sinphi;

    return (xyzC);
}

mec_transformation::enu_InitTypeDef mec_transformation::lla2enu(double lat0,double lon0,double alt0,double lat1,double lon1,double alt1){
    mec_transformation::xyz_InitTypeDef xe2 = lla2xyz(lat0,lon0,alt0);
    mec_transformation::xyz_InitTypeDef xe1 = lla2xyz(lat1,lon1,alt1);

    mec_transformation::enu_InitTypeDef enuC;
    double a, b, c;
    double phi, lam, sinphi, cosphi, sinlam, coslam;

    a=xe1.xx-xe2.xx;
    b=xe1.yy-xe2.yy;
    c=xe1.zz-xe2.zz;

    phi=(lat0*PI)/180;
    lam=(lon0*PI)/180;
    sinphi=sin(phi);
    cosphi=cos(phi);
    sinlam=sin(lam);
    coslam=cos(lam);

    enuC.e=(-sinlam)*a+(coslam)*b+(0)*c;
    enuC.n=(-sinphi*coslam)*a+(-sinphi*sinlam)*b+(cosphi)*c;
    enuC.u=(cosphi*coslam)*a+(cosphi*sinlam)*b+(sinphi)*c;

    return (enuC);
}

mec_transformation::ned_InitTypeDef mec_transformation::lla2ned(double lat0,double lon0,double alt0,double lat1,double lon1,double alt1){
    mec_transformation::xyz_InitTypeDef xe2 = lla2xyz(lat0,lon0,alt0);
    mec_transformation::xyz_InitTypeDef xe1 = lla2xyz(lat1,lon1,alt1);  
    mec_transformation::ned_InitTypeDef nedC;
    double a, b, c;
    double phi, lam, sinphi, cosphi, sinlam, coslam;

    a=xe1.xx-xe2.xx;
    b=xe1.yy-xe2.yy;
    c=xe1.zz-xe2.zz;

    phi=(lat0*PI)/180;
    lam=(lon0*PI)/180;
    sinphi=sin(phi);
    cosphi=cos(phi);
    sinlam=sin(lam);
    coslam=cos(lam);

    nedC.e=(-sinlam)*a+(coslam)*b+(0)*c;
    nedC.n=(-sinphi*coslam)*a+(-sinphi*sinlam)*b+(cosphi)*c;
    nedC.d=-((cosphi*coslam)*a+(cosphi*sinlam)*b+(sinphi)*c);

    return (nedC);
}

Eigen::Matrix4d mec_transformation::rotation_matrix_eular( mec_transformation::rpy_InitTypeDef rpy){
    double cos_roll,cos_pitch,cos_yaw,sin_roll,sin_pitch,sin_yaw;
    cos_roll = cos(rpy.roll*PI/180.0);
    cos_pitch = cos(rpy.pitch*PI/180.0);
    cos_yaw = cos(rpy.yaw*PI/180.0);
    sin_roll = sin(rpy.roll*PI/180.0);
    sin_pitch = sin(rpy.pitch*PI/180.0);
    sin_yaw = sin(rpy.yaw*PI/180.0);

    Eigen::Matrix4d TF_b2n_rot_x, TF_b2n_rot_y, TF_b2n_rot_z;       
    TF_b2n_rot_x << 1, 0,        0,         0, 
                    0, cos_roll, -sin_roll, 0,
                    0, sin_roll, cos_roll,  0,
                    0, 0,        0,         1;

    TF_b2n_rot_y << cos_pitch,  0, sin_pitch, 0, 
                    0,          1, 0,         0,
                    -sin_pitch, 0, cos_pitch, 0,
                    0,          0, 0,         1;

    TF_b2n_rot_z << cos_yaw, -sin_yaw, 0, 0, 
                    sin_yaw, cos_yaw,  0, 0,
                    0,       0,        1, 0,
                    0,       0,        0, 1;
    return TF_b2n_rot_z*TF_b2n_rot_y*TF_b2n_rot_x;
}

double mec_transformation::deg2rad(double degrees) {
   return degrees * PI / 180.0;
}

double mec_transformation::rad2deg(double radians) {
   return radians * 180.0/PI;
}

mec_transformation::xyz_InitTypeDef mec_transformation::enu2ecef(double lat0, double lon0, double h0, double east, double north, double up) {
   mec_transformation::xyz_InitTypeDef xyz;
   // Constants for WGS84 ellipsoid
   const double WGS84_A = 6378137.0;         // Semi-major axis
   const double WGS84_E_SQ = 6.69437999014e-3; // Square of eccentricity
   
   // Compute some intermediate values
   mec_transformation::xyz_InitTypeDef xyz0 = mec_transformation::lla2xyz(lat0,lon0,h0);
   const double phi=(lat0*PI)/180;
   const double lam=(lon0*PI)/180;
   const double sinphi=sin(phi);
   const double cosphi=cos(phi);
   const double sinlam=sin(lam);
   const double coslam=cos(lam);

   // Compute ECEF coordinates
   xyz.xx = -sinlam * east - coslam * sinphi * north + coslam * cosphi * up + xyz0.xx;
   xyz.yy = coslam * east - sinlam * sinphi * north + sinlam * cosphi * up + xyz0.yy;
   xyz.zz = cosphi * north + sinphi * up + xyz0.zz;

//    std::cout << " enu2ecef : E:" << east << " N:" << north << " U:" << up << " , X:" << xyz.xx << " Y:" << xyz.yy << " Z:" << xyz.zz << std::endl;

   return xyz;
}

mec_transformation::llh_InitTypeDef mec_transformation::ecef2llh(const mec_transformation::xyz_InitTypeDef& ecef) {
    // output variables
    mec_transformation::llh_InitTypeDef llh;

    // Constants for WGS84 ellipsoid
    const double WGS84_A = 6378137.0;         // Semi-major axis
    const double WGS84_E_SQ = 6.69437999014e-3; // Square of eccentricity
    const double WGS84_F = 1.0/298.25722563;  // Flattening

    // middle term in computing process
    const double b = WGS84_A*(1-WGS84_F);  // semi-minor axis
    const double e = sqrt(WGS84_E_SQ);  // eccentricity
    const double p = sqrt(ecef.xx*ecef.xx+ecef.yy*ecef.yy); // length of ecef project on X-Y plane
    static double h_h = 0.0;  // height hat (previous heigh or guess heigh)
    h_h = 0.0; //every time initialize height & guess height from 0
    static int h_count = 0;
    h_count = 0;

    // first guess with h = 0
    double phi = atan2(ecef.zz,p*(1.0-WGS84_E_SQ)); //latitude
    double cosphi = cos(phi);
    double sinphi = sin(phi);
    double R_phi = (WGS84_A)/sqrt(1-WGS84_E_SQ*sinphi*sinphi); // R(phi),radius of earth accroding to latitude
    double h = p/cosphi-R_phi; //comput height from ground level (equation on p.10 of "Lecture Note on Strapdown Inertial Navigation and Integrated Navigation" author by Prof. Juang)
    // std::cout << " first ecef2llh : lat:" << mec_transformation::rad2deg(phi) 
    //     << " lon:" << mec_transformation::rad2deg(atan2(ecef.yy,ecef.xx)) << " R_phi:" 
    //     << R_phi << " h:"<< h << " , X:" << ecef.xx << " Y:" << ecef.yy << " Z:" << ecef.zz 
    //     << std::endl;

    while (abs(h-h_h)>1.0e-6 && h_count<=100 ){ // height error in 30cm
        h_h = h;
        phi = atan2(ecef.zz,p*(1-WGS84_E_SQ*(R_phi)/(R_phi+h)));
        cosphi = cos(phi);
        sinphi = sin(phi);
        R_phi = (WGS84_A)/sqrt(1-WGS84_E_SQ*sinphi*sinphi);
        h = p/cosphi-R_phi;
        h_count ++;
        // std::cout << " count: " << h_count <<", ecef2llh : lat:" << mec_transformation::rad2deg(phi) 
        //     << " lon:" << mec_transformation::rad2deg(atan2(ecef.yy,ecef.xx)) << " R_phi:" 
        //     << R_phi << " h:"<< h <<std::endl;
    }

    // compute llh
    llh.Lat = mec_transformation::rad2deg(phi);
    llh.Lon = mec_transformation::rad2deg(atan2(ecef.yy,ecef.xx));
    llh.High = h;

    // std::cout << " ecef2llh : lat:" << llh.Lat << " lon:" << llh.Lon << " heigh:" << llh.High << " , X:" << ecef.xx << " Y:" << ecef.yy << " Z:" << ecef.zz << std::endl;

    return llh;
}

mec_transformation::llh_InitTypeDef mec_transformation::enu2llh(const mec_transformation::enu_InitTypeDef& enu_A,const mec_transformation::llh_InitTypeDef& llh_0){
   mec_transformation::xyz_InitTypeDef ecef_A = mec_transformation::enu2ecef(llh_0.Lat,llh_0.Lon,llh_0.High,enu_A.e,enu_A.n,enu_A.u);
   mec_transformation::llh_InitTypeDef llh_A = mec_transformation::ecef2llh(ecef_A);
   return llh_A;
}