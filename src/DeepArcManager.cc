#include <fstream>
#include <string>
#include <algorithm>
#include <iomanip>
#include <set>
#include <vector>
#include <cstdio>
#include <ceres/rotation.h>
#include <fmt/format.h>

#include "DeepArcManager.hh"
#include "snavely_reprojection_error.hh"

using namespace std;
void DeepArcManager::readKeypoints(std::ifstream &file, int size){
  set<int> arcs,rings;
  for(int i = 0; i < size; i++){
    int arc, ring, pid;
    double x, y;
    file >> arc >> ring >> pid >> x >> y;
    this->keypoints.push_back(Keypoint(arc, ring, pid, x, y));
  }
}

void DeepArcManager::readIntrinsic(std::ifstream &file, int size){
  for (int i = 0; i < size; i++){
    int elem_size;
    Intrinsic ins(i);
    file >> ins.center(0) >> ins.center(1);

    file >> elem_size; //read focal length
    ins.focal = Eigen::VectorXd(elem_size);
    for (int j = 0; j < elem_size; j++)
      file >> ins.focal(j);
    
    file >> elem_size; //read distortion
    cout << elem_size << endl;
    ins.distortion = Eigen::VectorXd(elem_size);
    for (int j = 0; j < elem_size; j++)
      file >> ins.distortion(j);

    this->intrinsics.push_back(ins);

    cout << "center " << ins.center << endl;
    cout << "focal " << ins.focal << endl;
    cout << "dist " << ins.distortion << endl;
  }
}

void DeepArcManager::readExtrinsic(std::ifstream &file, int size){
  for (int i = 0; i < size; i++) {
    int elem_size;
    double rotation[9];
    Extrinsic ext(i);
    file >> ext.translation(0) >> ext.translation(1) >> ext.translation(2);

    file >> elem_size; //read rotation
    for (int j = 0; j < elem_size; j++)
      file >> rotation[j];

    if (elem_size == 3) 
      ext.rotation << rotation[0], rotation[1], rotation[2];
    else if (elem_size == 9)
      ceres::RotationMatrixToAngleAxis(rotation, ext.rotation.data());
    else if (elem_size == 4)
      ceres::QuaternionToAngleAxis(rotation, ext.rotation.data());

    this->extrinsics.push_back(ext);
  }
}

void DeepArcManager::readPoint3d(std::ifstream &file, int size) {
  this->points.resize(size);
  this->used.resize(size);
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < 6; j++) 
      file >> (this->points[i])(j);
    this->used[i] = 1;
  }
}

int DeepArcManager::getRingId(int ring) {
  return ring + (ring > 0 ? this->arc_size-1 : 0);
}

void DeepArcManager::writePly(std::string filename){
  std::ofstream of(filename);
  int point_size = 0;
  for (int i = 0; i < (int) points.size(); i++) 
    point_size += used[i];

  int camera_size = this->arc_size * this->ring_size;

  int vertex_size = point_size + (camera_size * 5);
  of << "ply"
    << '\n' << "format ascii 1.0"
    << '\n' << "element vertex " << vertex_size
    << '\n' << "property float x"
    << '\n' << "property float y"
    << '\n' << "property float z"
    << '\n' << "property uchar red"
    << '\n' << "property uchar green"
    << '\n' << "property uchar blue"
    << '\n' << "element edge " << camera_size * 8
    << '\n' << "property int vertex1"
    << '\n' << "property int vertex2"
    << '\n' << "property uchar red"
    << '\n' << "property uchar green"
    << '\n' << "property uchar blue"
    << '\n' << "end_header" << std::endl;
  //Write Camera Position
  vector<int> lines;
  int vcount = 0;
  for (int arc = 0; arc < this->arc_size; arc++) {
    for (int ring = 0; ring < this->ring_size; ring++) {
      Extrinsic &e_arc = this->extrinsics[arc];
      Extrinsic &e_ring = this->extrinsics[this->getRingId(ring)];

      double s = 0.1;
      int w = this->image_width, h = this->image_height;
      double f = this->intrinsics[arc].focal(0);
      Eigen::Vector3d corners[5] = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(-w * s / f, -h * s / f, s),
        Eigen::Vector3d(-w * s / f, h * s / f, s),
        Eigen::Vector3d(w * s / f, h * s / f, s),
        Eigen::Vector3d(w * s / f, -h * s / f, s)
      };

      for (int i = 0; i < 5; i++) {
        auto output = e_ring.applyInverse(e_arc.applyInverse(corners[i]));
        for (int j = 0; j < 3; j++) 
          of << output(j) << " ";
        if (i > 0) 
          of << "128 128 128\n";
        else if (arc == 0 && ring == 1)
          of << "255 0 0\n";
        else if (arc == 0 && ring == 40)
          of << "0 0 255\n" ;
        else if (arc == 0 && ring == 2)
          of << "0 0 0\n" ;
        else if (arc == 0 || ring == 0)
          of << "0 255 0\n";
        else
          of << "255 0 255\n";
      }
      for (int i = 0; i < 4; i++) {
        lines.push_back(vcount); 
        lines.push_back(vcount+i+1);
      }
      for (int i = 0; i < 4; i++) {
        lines.push_back(vcount + (i + 1));
        lines.push_back(vcount + (i + 1) % 4 + 1);
      }
      vcount += 5;
    }
  }

  //Write Point3d
  for (int i = 0; i < (int) this->points.size(); i++) {
    if (!this->used[i]) continue;
    for (int j = 0; j < 6; j++) 
      of << this->points[i](j) << ' ';
    of << endl;
  }

  for (int i = 0; i < (int) lines.size(); i+=2) {
    of << lines[i] << " " << lines[i+1] << " 128 128 128\n";
  }
  of.close();
}

bool DeepArcManager::read(std::string filename){
  std::ifstream file(filename);
  if (file.fail()){
    std::cout << "Cannot read " << filename << std::endl;
    throw "Cannot read input file";
  }
  double version;
  int paramsblock_size, intrinsic_size, extrinsic_arc_size,
      extrinsic_ring_size, point3d_size, extrinsic_size;
  // read version number
  file >> version;
  // read size header;
  file >> paramsblock_size >> intrinsic_size >> extrinsic_arc_size 
    >> extrinsic_ring_size >> point3d_size;

  this->arc_size = extrinsic_arc_size;
  this->ring_size = extrinsic_ring_size;
  if(extrinsic_ring_size != 0){
    this->share_extrinsic = true;
    extrinsic_size = extrinsic_arc_size + extrinsic_ring_size - 1;
    cout << "share extrinsic " << this->arc_size << " " << this->ring_size << endl;
  }else{
    this->share_extrinsic = false;
    extrinsic_size = extrinsic_arc_size;
    cout << "standard extrinsic " << this->arc_size << " " << this->ring_size << endl;
    this->ring_size = 1;
  }

  cout << extrinsic_size << endl;
  cout << paramsblock_size << endl;

  this->readKeypoints(file, paramsblock_size);
  this->readIntrinsic(file, intrinsic_size);
  this->readExtrinsic(file, extrinsic_size);
  this->readPoint3d(file, point3d_size);

  file.close();

  return true;
}

vector<double*> DeepArcManager::getParamBlocks(int kid) {
  const Keypoint &kp = keypoints[kid];
  const Intrinsic &intrinsic = this->intrinsics[kp.arc];
  const Extrinsic &e_ring = this->extrinsics[this->getRingId(kp.ring)];
  const Extrinsic &e_arc = this->extrinsics[kp.arc];

  vector<double *> params = {
    this->points[kp.point_id].data(),
    (double*)intrinsic.center.data(),
    (double*)intrinsic.focal.data(),
    (double*)intrinsic.distortion.data(),
  };

  if (kp.arc == 0) {
    params.push_back((double*)e_ring.rotation.data());
    params.push_back((double*)e_ring.translation.data());
  } else if (kp.ring == 0) {
    params.push_back((double*)e_arc.rotation.data());
    params.push_back((double*)e_arc.translation.data());
  } else {
    params.push_back((double*)e_arc.rotation.data());
    params.push_back((double*)e_arc.translation.data());
    params.push_back((double*)e_ring.rotation.data());
    params.push_back((double*)e_ring.translation.data());
  }

  return params;
}

void DeepArcManager::filterHemisphere(Eigen::Vector3d center, double radius, double percent) {
  for (int i = 0; i < (int) this->points.size(); i++)
    if ((this->points[i].head(3) - center).norm() > radius * percent) 
      this->used[i] = 0;
}

void DeepArcManager::filterSeenByMany(int numCam) {
  vector<int> count(this->used.size(), 0);
  for (int i = 0; i < (int) this->keypoints.size(); i++) {
    const Keypoint &kp = this->keypoints[i];
    count[kp.point_id]++;
  }
  for (int i = 0; i < (int) this->used.size(); i++) {
    if (count[i] < numCam) {
      this->used[i] = 0;
    }
  }
}

void DeepArcManager::filterReprojection(double threshold) {
  for (int i = 0; i < (int) this->keypoints.size(); i++) {
    const Keypoint &kp = this->keypoints[i];
    if (!this->used[kp.point_id]) continue;
    
    vector<double*> param_blocks = this->getParamBlocks(i);
    SnavelyReprojectionError projector(
        kp.pixel(0),
        kp.pixel(1),
        this->intrinsics[kp.arc].focal.size(),
        this->intrinsics[kp.arc].distortion.size(),
        param_blocks.size() > 6);

    Eigen::Vector2d res;
    projector(param_blocks.data(), res.data());
    if (res.norm() > threshold) {
      this->used[kp.point_id] = 0;
    }
  }
}

string DeepArcManager::getCameraModel(Intrinsic &intrinsic){
  int focal_size = intrinsic.focal.size();
  int distortion_size = intrinsic.distortion.size();
  if(focal_size == 1 &&  distortion_size== 0){
    return "SIMPLE_PINHOLE";
  }else if(focal_size == 2 && distortion_size == 0){
    return "PINEHOLE";
  }else if(focal_size == 1 && distortion_size == 1){
    return "SIMPLE_RADIAL";
  }else if(focal_size == 2 && distortion_size == 1){
    return "RADIAL";
  }
  return "";
}

int DeepArcManager::colmapImageId(int arc_position, int ring_position){
  return arc_position*this->ring_size+ring_position+1;
}

void DeepArcManager::writeColmapText(string directory_path){;
  writeCamerasText(directory_path);
  writeImagesText(directory_path);
  writePoints3dText(directory_path);
}

void DeepArcManager::writeCamerasText(string directory_path){
  ofstream of(directory_path+"/cameras.txt");
  of << "# Camera list with one line of data per camera: \n"
    << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n"
    << "# Number of cameras: " << to_string(this->intrinsics.size()) << "\n";
  for(int i = 0; i < (int) this->intrinsics.size(); i++){
    Intrinsic &intrinsic =  this->intrinsics[i];
    of << (intrinsic.id + 1)
      << " "
      << this->getCameraModel(intrinsic)
      << " "
      << this->image_width
      << " " 
      << this->image_height;
    double* focal = intrinsic.focal.data();
    for(int j = 0; j < intrinsic.focal.size(); j++){
      of << " " << focal[j];
    }
    double* center = intrinsic.center.data();
    of << " " << center[0] << " " << center[1];
    double* distortion = intrinsic.distortion.data();
    for(int j = 0; j < intrinsic.distortion.size(); j++){
      of << " " << distortion[j];
    }
    of << "\n";
  }
  of.close();
}
/*
* calcurate R_n (new Rotation) and t_n (new translation) 
* from R_a / t_a (arc's extrinsic) and R_r / T_r (ring's extrinsic) 
* R_n P + t_n =  R_a(R_r P +  t_r) + t_a
* R_n P + t_n = (R_a R_r) P + ((R_a t_r) + t_a)
* then, R_n = (R_a R_r) and t_n = (R_a t_r) + t_a
*/
Eigen::Vector3d DeepArcManager::getComposeTranslation(int arc_position, int ring_position){
  if(ring_position == 0){
    return this->extrinsics[arc_position].translation;
  }else if(arc_position == 0){
    return this->extrinsics[this->getRingId(ring_position)].translation;
  }else{
    //calcurate  t_n = (R_a t_r) + t_a
    //let t_r = P, then t_n = R_a P + t_a, this is RotatePoint.
    return this->extrinsics[arc_position].apply(
      this->extrinsics[this->getRingId(ring_position)].translation
    );
  }
}

Eigen::Vector3d DeepArcManager::getComposeRotation(int arc_position, int ring_position){
  if(ring_position == 0){
    return this->extrinsics[arc_position].rotation;
  }else if(arc_position == 0){
    return this->extrinsics[this->getRingId(ring_position)].rotation;
  }else{
    double arc_rot[9], ring_rot[9], result_vec[3];
    ceres::AngleAxisToRotationMatrix(
      this->extrinsics[arc_position].rotation.data(), arc_rot
    );
    ceres::AngleAxisToRotationMatrix(
      this->extrinsics[this->getRingId(ring_position)].rotation.data(), ring_rot
    );
     //calculate R_n = (R_a R_r) by using ceres rotation
    Eigen::Map<Eigen::Matrix3d> rotation_arc(arc_rot), rotation_ring(ring_rot);
    Eigen::Matrix3d rotation_new = rotation_arc * rotation_ring;
    ceres::RotationMatrixToAngleAxis(rotation_new.data(),result_vec);
    Eigen::Vector3d result(result_vec);
    return result;
  }
}


void DeepArcManager::writeImagesText(string directory_path){
  int image_size = this->arc_size * this->ring_size;
  ofstream of(directory_path+"/images.txt");
  // build point2kpt
  vector< vector<int> > image2kpt;
  if(this->colmap_track){
    this->point2kpt.clear();
    this->point2kpt.resize(this->keypoints.size());
    image2kpt.resize(this->arc_size*this->ring_size);
    for(int kpt_id = 0; kpt_id < (int) this->keypoints.size(); kpt_id++){
      Keypoint &kpt = this->keypoints[kpt_id];
      int colmapId = this->colmapImageId(kpt.arc,kpt.ring);
      this->point2kpt[kpt.point_id].push_back(
        pair<int,int>(colmapId,image2kpt[colmapId-1].size())
      );
      image2kpt[colmapId-1].push_back(kpt_id);
    }
  }
  of << "# Image list with two lines of data per image:\n"
    << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n"
    << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n"
    << "# Number of images: " << image_size << ", mean observations per image: " << 0 << "\n";
  for(int arc_pos = 0; arc_pos < this->arc_size; arc_pos++){
    for(int ring_pos = 0; ring_pos < this->ring_size; ring_pos++){
      Eigen::Vector3d rotation = this->getComposeRotation(arc_pos,ring_pos);
      Eigen::Vector3d translation = this->getComposeTranslation(arc_pos,ring_pos);
      double qvec[4], *tvec;
      ceres::AngleAxisToQuaternion(rotation.data(),qvec);
      tvec = translation.data();
      string file_name = fmt::format(
        this->imagestxt_format,
        fmt::arg("arc", arc_pos),
        fmt::arg("ring", ring_pos)
      );
      int colmapImageId = this->colmapImageId(arc_pos,ring_pos);
      int image2kptId = colmapImageId-1;
      of << colmapImageId << " "
        << qvec[0] << " " << qvec[1] << " " << qvec[2] << " " << qvec[3] << " "
        << tvec[0] << " " << tvec[1] << " " << tvec[2] << " "
        << (arc_pos + 1) << " " //camera_id
        << file_name << "\n";
      //keypoint
      if(this->colmap_track){
        for(int id = 0; id < (int) image2kpt[image2kptId].size() ; id++){
          Keypoint &kpt = this->keypoints[image2kpt[image2kptId][id]];
          of << kpt.pixel[0] << " " << kpt.pixel[1];
          if(this->used[kpt.point_id]){
            of << " " << (kpt.point_id+1) << " ";
          }else{
            of << " -1 ";
          }
        } 
      }
      of << "\n";
    }
  }
  of.close();
}

void DeepArcManager::writePoints3dText(string directory_path){
  ofstream of(directory_path+"/points3D.txt");
  of << "# 3D point list with one line of data per point:\n"
    << "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n"
    << "# Number of points: 0, mean track length: 0\n"; //temporary set 0
  for(int i = 0; i < (int) this->points.size(); i++){
    if(this->used[i] == 0) continue;
    double* point = this->points[i].data();
    of << (i+1) << " " << point[0] << " " << point[1] << " " << point[2] << " ";
    if(this->point3d_black){
      of << "0 0 0 "; //black color for debug
    }else{
      of << (int)point[3] << " " << (int)point[4] << " " << (int)point[5] << " ";
    }
    of << "0"; //temporary set 0, and no track
    if(this->colmap_track){
      vector<pair<int,int > > &kpt_ids = this->point2kpt[i];
      for(int j = 0; j < (int) kpt_ids.size(); j++){
        of << " " << kpt_ids[j].first << " " << kpt_ids[j].second;
      }
    }
    of << "\n";
  }
  of.close();
}