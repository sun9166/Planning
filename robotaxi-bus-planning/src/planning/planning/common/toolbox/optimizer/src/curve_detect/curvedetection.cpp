#include "common/toolbox/optimizer/include/curvedetection.h"

namespace optimizer {

CurveDetection::CurveDetection() {
  fit_curve_smoothlen_ = 5;
  current_path_direction_ = 0;
  fit_curve_len = 1.0;
  fit_curve_reverse_len = 0.5;
  curve_end_ = 0;
  fit_curve_minlen_ = 5.0;
  total_length_ = 0.0;
  curve_limit_ = 0.5;          // radius is 2m
  curve_limit_max_ = 1 / 1.4;  // radius is 1.4m
}

double CurveDetection::RunDetection(geometry::SiteVec& c_path, int pathdirection) {
  int path_size = c_path.size();
  if (path_size == 0) return 10000.0;

  geometry::SiteVec path = c_path;
  std::vector<double> length_vec;
  length_vec.clear();
  GetTotalLength(c_path, length_vec);
  int smooth_path_halflen =
      std::min(fit_curve_smoothlen_, static_cast<int>(path_size / 2));
  double fitlength;
  current_path_direction_ = pathdirection;
  fitlength = GetFitLength(current_path_direction_);

  // curvature culculating
  double calclength = 0.0;
  geometry::Site samplepoint[3];
  int i; //j;
  int fitstart = 0;
  int fitend = 0;
  samplepoint[0] = path[0];
  for (i = 0; i < path_size; ++i) {
    // calclength =
    //     std::hypot(samplepoint[0].x - path[i].x, samplepoint[0].y - path[i].y);
    calclength = length_vec[i] - length_vec[0];
    if (calclength >= fitlength) {
      samplepoint[1] = path[i];
      fitstart = i;
      break;
    }
  }
  if (fitstart == 0) {
    for (i = 0; i < path_size; ++i) {
      if (c_path[i].curvature < 0) {
        c_path[i].curvature = curve_end_;
      }
    }
    return 10000.0;
  }
  for (i = fitstart; i < path_size; ++i) {
    int m; //n;
    samplepoint[1] = path[i];  // the middle point of vector
    // calculate the left point of vector
    for (m = i; m > 0; --m) {
      // calclength = std::hypot(samplepoint[1].x - path[m].x,
      //                         samplepoint[1].y - path[m].y);
      calclength = length_vec[i] - length_vec[m];
      if (calclength >= fitlength) {
        samplepoint[0] = path[m];
        break;
      }
    }
    if (m == 0) {
      samplepoint[0] = path[0];
    }
    for (m = i; m < path_size; ++m) {
      // calclength = std::hypot(samplepoint[1].x - path[m].x,
      //                         samplepoint[1].y - path[m].y);
      calclength = length_vec[m] - length_vec[i];
      if (calclength >= fitlength) {
        samplepoint[2] = path[m];
        break;
      }
    }
    if (m == path_size) {
      fitend = i;
      if (c_path[fitend - 1].curvature >= 0) {
        curve_end_ = c_path[fitend - 1].curvature;
      }
      break;
    }
    // calculate point and curvature
    geometry::Site vecA, vecB, vecC;
    // calculate the vec A,B and C
    vecA.x = samplepoint[0].x - samplepoint[1].x;
    vecA.y = samplepoint[0].y - samplepoint[1].y;
    vecB.x = samplepoint[2].x - samplepoint[1].x;
    vecB.y = samplepoint[2].y - samplepoint[1].y;
    vecC.x = vecA.x - vecB.x;
    vecC.y = vecA.y - vecB.y;

    double lenA  = sqrt( vecA.x * vecA.x + vecA.y * vecA.y );
    double lenB  = sqrt( vecB.x * vecB.x + vecB.y * vecB.y );
    double lenC = sqrt( vecC.x*vecC.x + vecC.y*vecC.y );
    double S = vecA.x * vecB.y - vecA.y * vecB.x;

    double abc = lenA * lenB * lenC;
    double Curve = 0.0;
    if(abc > 0.0001){   // TBD 0.0001
        Curve = fabs(2*S /( abc ));
    } else {
        // AINFO << "lenth of a, b or c is too small.";
    }
    c_path[i].curvature  = Curve;
  }
  if (fitstart == fitend) {
    if (c_path[fitstart].curvature < 0) {
      c_path[fitstart].curvature = curve_end_;
    }
  }
  for (i = 0; i < fitstart; ++i) {
    if (c_path[i].curvature < 0) {
      for (int j = i; j <= fitstart; ++j) {
        if (c_path[j].curvature >= 0) {
          c_path[i].curvature = c_path[j].curvature;
          break;
        }
      }
    }
  }
  if (total_length_ < fit_curve_minlen_) {
    for (i = fitend; i < path_size; ++i) {
      if (c_path[i].curvature < 0) {
        c_path[i].curvature = curve_end_;
      }
    }
  }
  return PointCheck(c_path);
}

double CurveDetection::GlobalCurveFitting(geometry::SiteVec& c_path, int pathdirection) {
  int path_size = c_path.size();
  if (path_size == 0) return 10000.0;

  geometry::SiteVec path = c_path;
  std::vector<double> length_vec;
  GetTotalLength(c_path, length_vec, true);
  int smooth_path_halflen =
      std::min(fit_curve_smoothlen_, static_cast<int>(path_size / 2));
  double fitlength;
  current_path_direction_ = pathdirection;
  fitlength = GetFitLength(current_path_direction_);

  // curvature culculating
  double calclength = 0.0;
  geometry::Site samplepoint[3];
  int i; //j;
  int fitstart = 0;
  int fitend = 0;
  samplepoint[0] = path[0];
  for (i = 0; i < path_size; ++i) {
    // calclength =
    //     std::hypot(samplepoint[0].xg - path[i].xg, samplepoint[0].yg - path[i].yg);
    calclength = length_vec[i] - length_vec[0];
    if (calclength >= fitlength) {
      samplepoint[1] = path[i];
      fitstart = i;
      break;
    }
  }
  if (fitstart == 0) {
    for (i = 0; i < path_size; ++i) {
      if (c_path[i].curvature < 0) {
        c_path[i].curvature = curve_end_;
      }
    }
    return 10000.0;
  }
  for (i = fitstart; i < path_size; ++i) {
    int m; //n;
    samplepoint[1] = path[i];  // the middle point of vector
    // calculate the left point of vector
    for (m = i; m > 0; --m) {
      // calclength = std::hypot(samplepoint[1].xg - path[m].xg,
      //                         samplepoint[1].yg - path[m].yg);
      calclength = length_vec[i] - length_vec[m];
      if (calclength >= fitlength) {
        samplepoint[0] = path[m];
        break;
      }
    }
    if (m == 0) {
      samplepoint[0] = path[0];
    }
    for (m = i; m < path_size; ++m) {
      // calclength = std::hypot(samplepoint[1].xg - path[m].xg,
      //                         samplepoint[1].yg - path[m].yg);
      calclength = length_vec[m] - length_vec[i];
      if (calclength >= fitlength) {
        samplepoint[2] = path[m];
        break;
      }
    }
    if (m == path_size) {
      fitend = i;
      if (c_path[fitend - 1].curvature >= 0) {
        curve_end_ = c_path[fitend - 1].curvature;
      }
      break;
    }
    // calculate point and curvature
    geometry::Site vecA, vecB, vecC;
    // calculate the vec A,B and C
    vecA.xg = samplepoint[0].xg - samplepoint[1].xg;
    vecA.yg = samplepoint[0].yg - samplepoint[1].yg;
    vecB.xg = samplepoint[2].xg - samplepoint[1].xg;
    vecB.yg = samplepoint[2].yg - samplepoint[1].yg;
    vecC.xg = vecA.xg - vecB.xg;
    vecC.yg = vecA.yg - vecB.yg;

    double lenA  = sqrt( vecA.xg * vecA.xg + vecA.yg * vecA.yg );
    double lenB  = sqrt( vecB.xg * vecB.xg + vecB.yg * vecB.yg );
    double lenC = sqrt( vecC.xg*vecC.xg + vecC.yg*vecC.yg );
    double S = vecA.xg * vecB.yg - vecA.yg * vecB.xg;

    double abc = lenA * lenB * lenC;
    double Curve = 0.0;
    if(abc > 0.0001){   // TBD 0.0001
        Curve = fabs(2*S /( abc ));
    } else {
        std::cout<<"lenth of a, b or c is too small."<<std::endl;
    }
    c_path[i].curvature  = Curve;
  }
  if (fitstart == fitend) {
    if (c_path[fitstart].curvature < 0) {
      c_path[fitstart].curvature = curve_end_;
    }
  }
  for (i = 0; i < fitstart; ++i) {
    if (c_path[i].curvature < 0) {
      for (int j = i; j <= fitstart; ++j) {
        if (c_path[j].curvature >= 0) {
          c_path[i].curvature = c_path[j].curvature;
          break;
        }
      }
    }
  }
  if (total_length_ < fit_curve_minlen_) {
    for (i = fitend; i < path_size; ++i) {
      if (c_path[i].curvature < 0) {
        c_path[i].curvature = curve_end_;
      }
    }
  }
  return PointCheck(c_path);
}

double CurveDetection::GetFitLength(int direction) {
  return direction == 0 ? fit_curve_len : fit_curve_reverse_len;
}

bool CurveDetection::GetTotalLength(geometry::SiteVec& path, std::vector<double> &length_vec, bool use_global) {
  total_length_ = 0.0;
  double single_len = 0.0;
  if (path.size() < 1) return false;
  if (path.size() == 1) {
    length_vec.push_back(0.0);
    return false;
  }
  length_vec.push_back(0.0);
  for (int i = 1; i < path.size(); ++i) {
    if(!use_global){
      single_len =
          std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    } else {
      single_len =
          std::hypot(path[i].xg - path[i - 1].xg, path[i].yg - path[i - 1].yg);
    }
    total_length_ += single_len;
    length_vec.push_back(total_length_);
  }
  return true;
}
double CurveDetection::PointCheck(geometry::SiteVec& path) {
  double max_curvature = 0.0;
  if (path.size()<=10) return 2;
  for (int i = 0; i < path.size(); i++) {
    if (i < 10 || i >= path.size()-10) continue;
    max_curvature = std::max(max_curvature, path[i].curvature);
  }
  if (max_curvature < 1e-4) return 10000.0; //fixed by zhangzhuo [0713]
  double curve_radius = 1.0 / max_curvature;
  return curve_radius;
  // if (max_curvature > curve_limit_max_) {
  //   return 2;
  // } else if (max_curvature > curve_limit_) {
  //   return 1;
  // }
  // return 0;
}

}  // optimizer
