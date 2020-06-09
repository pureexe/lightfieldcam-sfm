#ifndef VLL_DEEPARC_MANAGER_H_
#define VLL_DEEPARC_MANAGER_H_

#include <vector>
#include <map>
#include <utility>
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

    Eigen::Vector3d rotation, translation;
    int id;
};

class Keypoint {
  public:
    Keypoint(int arc, int ring, int point_id, double x, double y) {
      this->arc = arc;
      this->ring = ring;
      this->point_id = point_id;
      this->pixel = Eigen::Vector2d(x, y);
    };

    int arc, ring;
    int point_id;
    Eigen::Vector2d pixel;
};

typedef Eigen::Matrix<double, 6, 1> Point3d;
class DeepArcManager {
  public:
    bool read(string filename);
    void writePly(string filename);
    void writeColmapText(string directory_path);
    int getRingId(int ring);
    vector<double*> getParamBlocks(int kid);
    void filterHemisphere(Eigen::Vector3d center, double radius, double percent=0.75); 
    void filterReprojection(double threshold);
    void filterSeenByMany(int numCam); 
    
    int arc_size, ring_size, intrinsic_size, image_width, image_height;
    string imagestxt_format;
    vector<Intrinsic> intrinsics;
    vector<Extrinsic> extrinsics;
    vector<Point3d> points;
    vector<int> used;
    vector<Keypoint> keypoints;
    vector< vector< pair<int,int> > > point2kpt;
    bool point3d_black, colmap_track, share_extrinsic;

    void writeCamerasText(string directory_path);
    void writeImagesText(string directory_path);
    void writePoints3dText(string directory_path);
    string getCameraModel(Intrinsic &intrinsic);
    Eigen::Vector3d getComposeTranslation(int arc_position, int ring_position);
    Eigen::Vector3d getComposeRotation(int arc_position, int ring_position);
    int colmapImageId(int arc_position, int ring_position);

  private:
    void readKeypoints(ifstream &file, int size);
    void readExtrinsic(ifstream &file, int size);
    void readIntrinsic(ifstream &file, int size);
    void readPoint3d(ifstream &file, int size);

};
#endif