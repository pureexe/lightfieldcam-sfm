#ifndef VLL_SNAVELY_REPROJECTION_ERROR_H_
#define VLL_SNAVELY_REPROJECTION_ERROR_H_

#define SNAVELY_REPROJECTION_KSTRIDE 10

#include "ceres/ceres.h"
#include "ceres/rotation.h"

struct SnavelyReprojectionError;

typedef ceres::DynamicAutoDiffCostFunction<
      SnavelyReprojectionError,
      SNAVELY_REPROJECTION_KSTRIDE
    > SnavelyCostFunction;

struct SnavelyReprojectionError {
  double observed_x, observed_y;
  bool two_extrinsics;
  int num_dist, num_focal;

  SnavelyReprojectionError(
      double x,
      double y,
      int num_focal,
      int num_dist,
      bool two_extrinsics
    ) {
      this->observed_x = x;
      this->observed_y = y;
      this->num_focal = num_focal;
      this->num_dist = num_dist;
      this->two_extrinsics = two_extrinsics;
  }
  template <typename T>
  bool projectPoint(
    T const* const* params, 
    const T* p,
    T* residuals) const {
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes
    const T* principal = params[1];
    const T* focal = params[2];
    const T* dist = params[3];

    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    //handle forcal length
    T focal_x, focal_y;
    focal_x = focal[0];
    focal_y = (num_focal == 2) ? focal[1] : focal[0];

    // Apply second and fourth order radial distortion.
    T distortion = T(1.0);
    T r2 = xp * xp + yp * yp;
    //fourth order raidal distrotion
    if (num_dist == 2) {
      distortion = 1.0 + r2 * (dist[0] + dist[1] * r2);
    }
    //second order radial distrotion
    if (num_dist == 1) {
      distortion = 1.0 + r2 * dist[0];
    }
    // Compute final projected point position.
    // instrinsic[0] is principal point px and instrinsic[1] is py.
    T predicted_x = focal_x * distortion * xp + principal[0];
    T predicted_y = focal_y * distortion * yp + principal[1];

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
  }

  template <typename T>
  void rotatePoint(
    const T* rotation,
    const T* translation,
    const  T* point,
    T* p) const {
      // rotate point by using angle-axis rotation.
      ceres::AngleAxisRotatePoint(rotation, point, p);
      p[0] += translation[0];
      p[1] += translation[1];
      p[2] += translation[2];
  }

  template <typename T>
  bool operator()(T const* const* params, T* residuals) const {                  
    T p[3],p2[3];
    if (this->two_extrinsics) {
      rotatePoint(
        params[6], //ring_rotation
        params[7], //ring_translation
        params[0], //point3d
        p2
      );
      rotatePoint(
        params[4], //arc_rotation
        params[5], //arc_translation
        p2,
        p
      );
    } else {
      rotatePoint(
        params[4], //rotation
        params[5], //translation
        params[0], //point3d
        p
      );
    }
    return projectPoint(params, p, residuals);
  }

  // Factory to hide the construction of the CostFunction object 
  static SnavelyCostFunction* Create(
      double x,
      double y,
      int focal,
      int distortion,
      bool two_extrinsics) {    
    SnavelyReprojectionError *projector = new SnavelyReprojectionError(x, y, focal, distortion, two_extrinsics);
    SnavelyCostFunction* cost_fn = new SnavelyCostFunction(projector);
    cost_fn->AddParameterBlock(3);           // 0: point3d
    cost_fn->AddParameterBlock(2);           // 1: principal point
    cost_fn->AddParameterBlock(focal);       // 2: focal length
    cost_fn->AddParameterBlock(distortion);  // 3: distortion
    cost_fn->AddParameterBlock(3);           // 4: rotation (arc)
    cost_fn->AddParameterBlock(3);           // 5: translation (arc)
    if (two_extrinsics) {
      cost_fn->AddParameterBlock(3);         // 6: rotation (ring)
      cost_fn->AddParameterBlock(3);         // 7: translation (ring)
    }
    cost_fn->SetNumResiduals(2);
    return cost_fn;
  }

};
#endif