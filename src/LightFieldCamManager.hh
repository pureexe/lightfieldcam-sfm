#ifndef VLL_LIGHTFIELDCAM_MANAGER_H_
#define VLL_LIGHTFIELDCAM_MANAGER_H_

#include <vector>
#include <string>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace std;
class Intrinsic {
  public:
    Intrinsic(int id) : id(id) {}
    Eigen::VectorXd focal, distortion;
    Eigen::Vector2d center;
    int id;
    template<typename T> void setFocal(T focals){
        focal = Eigen::Map<Eigen::VectorXd>((double*) focals);
    }
};
class Extrinsic {
  public:
    Extrinsic(int id) : id(id) {}
    template<typename T> Extrinsic(T rot, T tran){
      rotation = Eigen::Map<Eigen::Vector3d>((double*) rot);
      translation = Eigen::Map<Eigen::Vector3d>((double*) tran);
    }

    Eigen::Vector3d apply(Eigen::Vector3d x) {
      Eigen::Vector3d x_previous = x;
      ceres::AngleAxisRotatePoint(this->rotation.data(), x_previous.data(), x.data());
      return x + this->translation;
    }

    Eigen::Vector3d getPosition() {
      return this->applyInverse(Eigen::Vector3d(0, 0, 0));
    }

    Eigen::Vector3d applyInverse(Eigen::Vector3d x) {
      Eigen::Vector3d tmp, x_previous, inv = -this->rotation;
      x_previous = x;
      ceres::AngleAxisRotatePoint(inv.data(), x_previous.data(), x.data());
      ceres::AngleAxisRotatePoint(inv.data(), this->translation.data(), tmp.data());
      return x - tmp;
    }

    template<typename T> void setTranslation(T tran){
        translation = Eigen::Map<Eigen::Vector3d>((double*) tran);
    }

    template<typename T> void setRotation(T rot){
        rotation = Eigen::Map<Eigen::Vector3d>((double*) rot);
    }

    Eigen::Vector3d rotation, translation;
    int id;
};

class Keypoint {
  public:
    Keypoint(int extrinsic_id, int point_id, double x, double y) {
      this->extrinsic_id = extrinsic_id;
      this->point_id = point_id;
      this->pixel = Eigen::Vector2d(x, y);
    };
    int extrinsic_id, point_id;
    Eigen::Vector2d pixel;
};

typedef Eigen::Matrix<double, 6, 1> Point3d;

class LightFieldCamManager{
    public: 
        bool read(string filename);
        void resetCamera();
        void writePly(string filename);
        vector<double*> getParamBlocks(int kid);
        void filterReprojection(double threshold);
        void filterSeenByMany(int numCam); 
        void filterZvalue(double zval);
        void writeColmapText(string directory_path);

        int extrinsic_size, intrinsic_size, image_width, image_height;
        string imagestxt_format;
        vector<Intrinsic> intrinsics;
        vector<Extrinsic> extrinsics;
        vector<Point3d> points;
        vector<int> used;
        vector<Keypoint> keypoints;
        vector< vector< pair<int,int> > > point2kpt;
        bool point3d_black, colmap_track;
    private: 
        void readKeypoints(ifstream &file, int size);
        void readIntrinsic(ifstream &file, int size);
        void readExtrinsic(ifstream &file, int size);
        void readPoint3d(ifstream &file, int size);
        void writeCamerasText(string directory_path);
        void writeImagesText(string directory_path);
        void writePoints3dText(string directory_path);
        string getCameraModel(Intrinsic &intrinsic);
};
#endif