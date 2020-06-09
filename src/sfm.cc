#include <iostream>
#include <cmath>
#include <fmt/format.h>
#include "LightFieldCamManager.hh"
#include "snavely_reprojection_error.hh"

using namespace std;

DEFINE_string(input, "../data/teabottle_green_bfs_distroted.deeparc", "deeparc input");
DEFINE_int32(solver_max_thread, 16, "solver max thread");
DEFINE_int32(solver_max_iteration, 200, "solver max iteration");

DEFINE_int32(filter_reprojection, 3, "reprojection error threshold");
DEFINE_int32(filter_seen_by, 5, "points need to be seen by these many cameras");
DEFINE_bool(fix_intrinsic, true, "whether to fix the intrinsics");

DEFINE_int32(image_width, 541, "Image width");
DEFINE_int32(image_height, 736, "Image height");
DEFINE_string(imagestxt_format, "Flower1_{row:02d}_{col:05d}.png", "format string for colmap's images.txt file name");
DEFINE_string(output, "", "output directory");
DEFINE_string(final_ply, "", "provide final_ply name for output final ply for visualization");
DEFINE_bool(step_ply, false, "output ply every step");
DEFINE_bool(point3d_black, false, "force writeColmapText output point3d in black color for easier to view in colmap gui");
DEFINE_bool(colmap_track, true, "force writeColmapText to write out track. please enable if you want to watch result in colmap gui");
DEFINE_double(cauchy_value, 0.5, "cauchy loss value in solver");

void setSolverOption(ceres::Solver::Options &options){
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = FLAGS_solver_max_iteration; 
  options.num_threads = FLAGS_solver_max_thread; 
}

/*
void solve(DeepArcManager &dm,
           bool freeze_extrinsics,
           bool robust_loss = false,
           bool freeze_intrinsics = true,
           bool freeze_point3d = false) {

  ceres::Problem problem;
  for (int i = 0; i < (int)dm.keypoints.size(); i++) {
    const Keypoint &kp = dm.keypoints[i];
    if(!dm.used[kp.point_id]) continue;

    vector<double*> param_blocks = dm.getParamBlocks(i);
    SnavelyCostFunction* cost_fn = SnavelyReprojectionError::Create(
        kp.pixel(0), kp.pixel(1),
        dm.intrinsics[kp.arc].focal.size(),
        dm.intrinsics[kp.arc].distortion.size(),
        param_blocks.size() > 6);

    if (robust_loss)
      problem.AddResidualBlock(cost_fn, new ceres::CauchyLoss(FLAGS_cauchy_value), param_blocks);
    else
      problem.AddResidualBlock(cost_fn, new ceres::CauchyLoss(FLAGS_cauchy_value), param_blocks);

    if(freeze_point3d){
      problem.SetParameterBlockConstant(param_blocks[0]);    
    }
    if (kp.arc == 0 && kp.ring == 0) {
      problem.SetParameterBlockConstant(param_blocks[4]);
      problem.SetParameterBlockConstant(param_blocks[5]);    
    }
    if (freeze_extrinsics) {
      for(int j = 4; j < (int)param_blocks.size(); j++) 
        problem.SetParameterBlockConstant(param_blocks[j]);
    } 
    if (freeze_intrinsics) { 
      for (int j = 1; j < 4; j++) 
        problem.SetParameterBlockConstant(param_blocks[j]); 
    }
  }
  
  ceres::Solver::Options options;
  setSolverOption(options);
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
}
*/

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
  /*
  DeepArcManager dm;
  dm.imagestxt_format = FLAGS_imagestxt_format;
  dm.image_height = FLAGS_image_height;
  dm.image_width = FLAGS_image_width;
  dm.point3d_black = FLAGS_point3d_black;
  dm.colmap_track = FLAGS_colmap_track;
  dm.read(FLAGS_input);
  
  Eigen::Vector3d center;
  center.setZero();
  double radius = 4;

  findHemiCenter(dm, center, radius);
  if(FLAGS_step_ply) dm.writePly("0_after_load_hemisphere.ply");
  dm.filterSeenByMany(FLAGS_filter_seen_by);
  dm.filterHemisphere(center, radius);
  if(FLAGS_step_ply) dm.writePly("1_after_first_filter.ply");

  solve(dm, true, false); // Fix cameras, solve for points
  dm.filterHemisphere(center, radius);
  if(FLAGS_step_ply) dm.writePly("2_after_optimized_points.ply");

  solve(dm, false, true, FLAGS_fix_intrinsic); // Solve cameras + points, Robust loss
  dm.filterReprojection(FLAGS_filter_reprojection);
  dm.filterHemisphere(center, radius);
  if(FLAGS_step_ply) dm.writePly("3_after_joint_robust.ply");

  solve(dm, false, false, FLAGS_fix_intrinsic); // Solve cameras + points, L2 loss
  dm.filterHemisphere(center, radius);

  if(FLAGS_step_ply) dm.writePly("4_output.ply");
  if(FLAGS_final_ply != "") dm.writePly(FLAGS_final_ply);

  dm.writeColmapText(FLAGS_output);
  */
  return 0;
}