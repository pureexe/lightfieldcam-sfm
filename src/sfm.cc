#include <iostream>
#include <cmath>
#include <fmt/format.h>
#include "LightFieldCamManager.hh"
#include "snavely_reprojection_error.hh"

using namespace std;

DEFINE_string(input, "../data/car.deeparc", "deeparc input");
DEFINE_int32(solver_max_thread, 16, "solver max thread");
DEFINE_int32(solver_max_iteration, 100, "solver max iteration");

DEFINE_int32(filter_reprojection, 3, "reprojection error threshold");
DEFINE_int32(filter_seen_by, 5, "points need to be seen by these many cameras");

DEFINE_int32(image_width, 541, "Image width");
DEFINE_int32(image_height, 376, "Image height");
DEFINE_string(imagestxt_format, "Cars_{row:02d}_{col:02d}.png", "format string for colmap's images.txt file name");
DEFINE_string(output, "../output", "output directory");
DEFINE_string(final_ply, "", "provide final_ply name for output final ply for visualization");
DEFINE_bool(step_ply, true, "output ply every step");
DEFINE_bool(point3d_black, true, "force writeColmapText output point3d in black color for easier to view in colmap gui");
DEFINE_bool(colmap_track, true, "force writeColmapText to write out track. please enable if you want to watch result in colmap gui");
DEFINE_double(cauchy_value, 0.5, "cauchy loss value in solver");

void setSolverOption(ceres::Solver::Options &options){
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = FLAGS_solver_max_iteration; 
  options.num_threads = FLAGS_solver_max_thread; 
}

void solve(LightFieldCamManager &lfm,
  bool freeze_point3d = false,
  bool freeze_intrinsics = true,
  bool freeze_extrinsics = true
  ){
  ceres::Problem problem;
  for (int i = 0; i < (int)lfm.keypoints.size(); i++) {
    const Keypoint &kp = lfm.keypoints[i];
    if(!lfm.used[kp.point_id]) continue;
    vector<double*> param_blocks = lfm.getParamBlocks(i);
    SnavelyCostFunction* cost_fn = SnavelyReprojectionError::Create(
      kp.pixel(0), kp.pixel(1),
      lfm.intrinsics[0].focal.size(),
      lfm.intrinsics[0].distortion.size()
    );
    problem.AddResidualBlock(cost_fn, new ceres::CauchyLoss(FLAGS_cauchy_value), param_blocks);
    if(freeze_point3d){
      problem.SetParameterBlockConstant(param_blocks[0]);    
    }
    if (freeze_extrinsics) {
      for(int j = 4; j < (int)param_blocks.size(); j++) 
        problem.SetParameterBlockConstant(param_blocks[j]);
    } 
    if (freeze_intrinsics) { 

      //problem.SetParameterBlockConstant(param_blocks[1]); // principle point
      problem.SetParameterBlockConstant(param_blocks[2]); // focal length
      problem.SetParameterBlockConstant(param_blocks[3]); // distortion
    }
  }
  ceres::Solver::Options options;
  setSolverOption(options);
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);

  if(FLAGS_input == ""){
    cout << "Please specify file in -input" << endl;
    return -1;
  }
  if(FLAGS_output == ""){
    cout << "Please specify output directory in -output" <<endl;
    return -2;
  }
  LightFieldCamManager lfm;

  lfm.image_height = FLAGS_image_height;
  lfm.image_width = FLAGS_image_width;
  lfm.point3d_black = FLAGS_point3d_black;
  lfm.colmap_track = FLAGS_colmap_track;
  lfm.imagestxt_format = FLAGS_imagestxt_format;

  lfm.read(FLAGS_input);
  lfm.resetCamera();

  if(FLAGS_step_ply) lfm.writePly("0_after_load.ply");

  solve(lfm);
  /*
  lfm.filterReprojection(FLAGS_filter_reprojection);
  lfm.filterSeenByMany(FLAGS_filter_seen_by);
  lfm.filterZvalue(1.0);
  */
  
  if(FLAGS_step_ply) lfm.writePly("1_first_refine.ply");
  lfm.writeColmapText(FLAGS_output);

  return 0;
}