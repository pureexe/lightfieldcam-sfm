#include "LightFieldCamManager.hh"
#include "snavely_reprojection_error.hh"

#include <fmt/format.h>

#include <iostream>
#include <fstream>

using namespace std;

void LightFieldCamManager::readKeypoints(std::ifstream &file, int size){
  for(int i = 0; i < size; i++){
    int intrinsic_id, extrinsic_id, point_id;
    double x, y;
    file >> intrinsic_id >> extrinsic_id >> point_id >> x >> y;
    this->keypoints.push_back(Keypoint(extrinsic_id, point_id, x, y));
  }
}

void LightFieldCamManager::readIntrinsic(std::ifstream &file, int size){
  for (int i = 0; i < size; i++){
    int elem_size;
    Intrinsic ins(i);
    file >> ins.center(0) >> ins.center(1);

    file >> elem_size; //read focal length
    ins.focal = Eigen::VectorXd(elem_size);
    for (int j = 0; j < elem_size; j++)
      file >> ins.focal(j);
    
    file >> elem_size; //read distortion
    ins.distortion = Eigen::VectorXd(elem_size);
    for (int j = 0; j < elem_size; j++)
      file >> ins.distortion(j);
    this->intrinsics.push_back(ins);
  }
}

void LightFieldCamManager::readExtrinsic(std::ifstream &file, int size){
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

void LightFieldCamManager::readPoint3d(std::ifstream &file, int size) {
  this->points.resize(size);
  this->used.resize(size);
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < 6; j++) 
      file >> (this->points[i])(j);
    this->used[i] = 1;
  }
}

bool LightFieldCamManager::read(std::string filename){
  std::ifstream file(filename);
  if (file.fail()){
    std::cout << "Cannot read " << filename << std::endl;
    throw "Cannot read input file";
  }
  double version;
  int paramsblock_size, intrinsic_size,
      extrinsic_ring_size, point3d_size, extrinsic_size;
  // read version number
  file >> version;
  // read size header;
  file >> paramsblock_size >> intrinsic_size >> extrinsic_size 
    >> extrinsic_ring_size >> point3d_size;

  this->readKeypoints(file, paramsblock_size);
  this->readIntrinsic(file, intrinsic_size);
  this->readExtrinsic(file, extrinsic_size);
  this->readPoint3d(file, point3d_size);
  return true;
}


void LightFieldCamManager::resetCamera(){
  // reset focal length distrotion roration and translation here
  double zeros[3] = {0.0, 0.0, 0.0};
  double zval = 1.0;
  for(int i = 0; i < 8; i++){
    for(int j = 0; j < 8; j++){
      int id = i*8 + j;
      double translation[3] = {((double) i)* 0.25 -1.0,((double) j) * 0.25 - 1.0, zval};
      this->extrinsics[id].setRotation(zeros);
      this->extrinsics[id].setTranslation(translation);
    }
  }
  //this->intrinsics[0].focal(0) = 100.0;
}

vector<double*> LightFieldCamManager::getParamBlocks(int kid) {
  const Keypoint &kp = keypoints[kid];
  const Intrinsic &intrinsic = this->intrinsics[0];
  const Extrinsic &extrinsic = this->extrinsics[kp.extrinsic_id];
  vector<double *> params = {
    this->points[kp.point_id].data(),
    (double*)intrinsic.center.data(),
    (double*)intrinsic.focal.data(),
    (double*)intrinsic.distortion.data(),
  };
  params.push_back((double*)extrinsic.rotation.data());
  params.push_back((double*)extrinsic.translation.data());
  return params;
}

void LightFieldCamManager::writePly(std::string filename){
  std::ofstream of(filename);
  int point_size = 0;
  for (int i = 0; i < (int) points.size(); i++) 
    point_size += used[i];

  int camera_size = this->extrinsics.size();

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
  for (int extrinsic_id = 0; extrinsic_id < camera_size; extrinsic_id++) {
      Extrinsic &current_e = this->extrinsics[extrinsic_id];

      double s = 0.1;
      int w = this->image_width, h = this->image_height;
      double f = this->intrinsics[0].focal(0);
      Eigen::Vector3d corners[5] = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(-w * s / f, -h * s / f, s),
        Eigen::Vector3d(-w * s / f, h * s / f, s),
        Eigen::Vector3d(w * s / f, h * s / f, s),
        Eigen::Vector3d(w * s / f, -h * s / f, s)
      };

      for (int i = 0; i < 5; i++) {
        auto output = (current_e.applyInverse(corners[i]));
        for (int j = 0; j < 3; j++) 
          of << output(j) << " ";
        if (i > 0) 
          of << "128 128 128\n";
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

void LightFieldCamManager::filterReprojection(double threshold) {
  for (int i = 0; i < (int) this->keypoints.size(); i++) {
    const Keypoint &kp = this->keypoints[i];
    if (!this->used[kp.point_id]) continue;
    
    vector<double*> param_blocks = this->getParamBlocks(i);
    SnavelyReprojectionError projector(
        kp.pixel(0),
        kp.pixel(1),
        this->intrinsics[0].focal.size(),
        this->intrinsics[0].distortion.size()
      );

    Eigen::Vector2d res;
    projector(param_blocks.data(), res.data());
    if (res.norm() > threshold) {
      this->used[kp.point_id] = 0;
    }
  }
}

void LightFieldCamManager::filterSeenByMany(int numCam) {
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

void LightFieldCamManager::filterZvalue(double zval){
  for (int i = 0; i < (int) this->used.size(); i++) {
    if (this->points[i](2) < zval ||this->points[i](2) > 10000) {
      this->used[i] = 0;
    }
  }
}

string LightFieldCamManager::getCameraModel(Intrinsic &intrinsic){
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

void LightFieldCamManager::writeColmapText(string directory_path){;
  this->writeCamerasText(directory_path);
  this->writeImagesText(directory_path);
  this->writePoints3dText(directory_path);
}

void LightFieldCamManager::writeCamerasText(string directory_path){
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

void LightFieldCamManager::writeImagesText(string directory_path){
  int image_size = this->extrinsics.size();
  ofstream of(directory_path+"/images.txt");
  // build point2kpt
  vector< vector<int> > image2kpt;
  if(this->colmap_track){
    this->point2kpt.clear();
    this->point2kpt.resize(this->keypoints.size());
    image2kpt.resize(image_size);
    for(int kpt_id = 0; kpt_id < (int) this->keypoints.size(); kpt_id++){
      Keypoint &kpt = this->keypoints[kpt_id];
      int colmapId = kpt.extrinsic_id + 1;
      this->point2kpt[kpt.point_id].push_back(
        pair<int,int>(colmapId,image2kpt[kpt.extrinsic_id].size())
      );
      image2kpt[kpt.extrinsic_id].push_back(kpt_id);
    }
  }
  of << "# Image list with two lines of data per image:\n"
    << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n"
    << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n"
    << "# Number of images: " << image_size << ", mean observations per image: " << 0 << "\n";
  for(int row_pos = 0; row_pos < 8; row_pos++){
    for(int col_pose = 0; col_pose < 8; col_pose++){
      Extrinsic extrinsic = this->extrinsics[row_pos*8+col_pose];
      Eigen::Vector3d rotation = extrinsic.rotation;
      Eigen::Vector3d translation = extrinsic.rotation;
      double qvec[4], *tvec;
      ceres::AngleAxisToQuaternion(rotation.data(),qvec);
      tvec = translation.data();
      string file_name = fmt::format(
        this->imagestxt_format,
        fmt::arg("row", row_pos),
        fmt::arg("col", col_pose)
      );
      int colmapImageId = row_pos*8 + col_pose + 1;
      int image2kptId = colmapImageId - 1;
      of << colmapImageId << " "
        << qvec[0] << " " << qvec[1] << " " << qvec[2] << " " << qvec[3] << " "
        << tvec[0] << " " << tvec[1] << " " << tvec[2] << " "
        << 1 << " " //camera_id
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

void LightFieldCamManager::writePoints3dText(string directory_path){
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