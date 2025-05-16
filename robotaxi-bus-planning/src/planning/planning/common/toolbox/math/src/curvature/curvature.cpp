
#include "curvature/curvature.h"
#include <iostream>

namespace acu {
namespace planning {
namespace math {

using geometry::Site;
using geometry::SiteVec;

Curvature::Curvature() {
  curve_end_ = 0.0;
}

Curvature::~Curvature() {}

int Curvature::Curve(SiteVec& list) {
  int list_size = list.size();
  if (list_size < 5) return -1;
  if (list.back().length <= 0) {
    CalculateLengthOfPoint(list); 
  }
  SiteVec smooth_list = list;
  int half_length = std::min(5, (int)(list_size / 2)); //窗口大小为 half_length*2+1
  double fit_length = 2.0; //取的 路点 向量的模长为 3.0m
  //1.路径坐标点窗口平滑处理
  for(int i = 0; i < list_size; ++i){
    int j = 0;
    double x_sum = 0;
    double y_sum = 0;
    int smooth_points_cnt = 0;
    if(i < half_length){ //路径前几个路点处理
      for(j = 0; j <= 2 * i; ++j){
        x_sum += list[j].x;
        y_sum += list[j].y;
      }
      smooth_points_cnt = j;
    }else if(i >= list_size - half_length){ //路径末尾几个路点处理
      for(j = i - (list_size - 1 - i); j < list_size; ++j){
        x_sum += list[j].x;
        y_sum += list[j].y;
      }
      smooth_points_cnt = list_size - (i - (list_size - 1 - i));
    }else{//正常平滑处理
      for(j = i - half_length; j <= i + half_length; ++j){
        x_sum += list[j].x;
        y_sum += list[j].y;
      }
      smooth_points_cnt = 2 * half_length + 1;
    }
    smooth_points_cnt = std::max(1,smooth_points_cnt);
    smooth_list[i].x = x_sum / smooth_points_cnt;
    smooth_list[i].y = y_sum / smooth_points_cnt;
  }
  //2.根据截取路径的 两个向量夹角及模长，计算曲率
  double cal_length = 0;
  Site sample_point_0, sample_point_1, sample_point_2;
  int i, j;
  int fit_start_index = 0;
  int fit_end_index = 0;
  // 获得向量的第一个起点
  sample_point_0 = smooth_list[0];
  for(i = 0; i < list_size; ++i){
    cal_length = std::hypot(sample_point_0.x - smooth_list[i].x,
                            sample_point_0.y - smooth_list[i].y);
    if(cal_length >= fit_length){
      sample_point_1 = smooth_list[i];
      fit_start_index = i;
      break;
    }
  }
  if(fit_start_index == 0){
    for(i = 0; i < list_size; ++i){
      if (fabs(list[i].curvature) > 10)  //- FLAGS_curvature_init_value )<0.01)
        list[i].curvature = curve_end_;
    }
    return -1;
  }
  //开始正常提取向量
  for(i = fit_start_index; i < list_size; ++i){
    int m, n;
    //向量中间点
    sample_point_1 = smooth_list[i];
    //计算左向量终点
    for(m = i; m > 0; --m){
      cal_length = std::hypot(sample_point_1.x - smooth_list[m].x,
                              sample_point_1.y - smooth_list[m].y);
      if(cal_length >= fit_length){
        sample_point_0 = smooth_list[m];
        break;
      }
    }
    if(m == 0) sample_point_0 = smooth_list[0];
    //计算右向量终点
    for(m = i; m < list_size; ++m){
      cal_length = std::hypot(sample_point_1.x - smooth_list[m].x,
                              sample_point_1.y - smooth_list[m].y);
      if(cal_length >= fit_length){
        sample_point_2 = smooth_list[m];
        break;
      }
    }

    if(m == list_size) {
      fit_end_index = i;
      if (fabs(list[fit_end_index-1].curvature) > 10) {//-FLAGS_curvature_init_value)>0.01 ){
        curve_end_ = list[fit_end_index-1].curvature;
      }
      break;
    }

    //计算 路点 曲率
    Site vecA, vecB,vecAB;
    double AngleAB = 0;
    //根据三点坐标，计算A、B向量
    vecA.x = sample_point_0.x - sample_point_1.x;
    vecA.y = sample_point_0.y - sample_point_1.y;
    vecB.x = sample_point_2.x - sample_point_1.x;
    vecB.y = sample_point_2.y - sample_point_1.y;
    vecAB.x = vecA.x - vecB.x;
    vecAB.y = vecA.y - vecB.y;
    //method 1
    //1)计算A、B向量的夹角
    // double acosData = (vecA.x * vecB.x + vecA.y * vecB.y) / ///(fit_length*fit_length));
    //                   (sqrt((vecA.x * vecA.x + vecA.y * vecA.y) * (vecB.x * vecB.x + vecB.y * vecB.y)));
    // if(acosData > 1)acosData = 1;
    // else if(acosData < -1)acosData = -1;
    // AngleAB = acos(acosData);
    // //2)根据夹角及向量模长计算曲率
    // double Curve = 2 * cos(AngleAB / 2) / fit_length;
    //method 2 
    double vector_OA_OB =  vecA.x * vecB.y - vecA.y * vecB.x;  //OAXOB(叉乘)其值为以OA和OB向量构成的平行四边形的面积。
    double module_vecA = sqrt(vecA.x * vecA.x + vecA.y * vecA.y);//向量OA的模
    double module_vecB = sqrt(vecB.x * vecB.x + vecB.y * vecB.y);//向量OB的模
    double module_vecAB = sqrt(vecAB.x * vecAB.x + vecAB.y * vecAB.y);//向量AB的模
    double Curve = 2 * vector_OA_OB/(module_vecA * module_vecB * module_vecAB);//三角形外切圆半径R=abc/4S,abc为三角形三边长，S为三角形面积。所以kappa=1/R=4S/(abc)
    // Curve = fabs(Curve);//现在先用绝对值，后面需要正负值时再放开。
    Curve = -Curve;//@pqg 因后面的规划层算出来的曲率为左正右负，这里不取反，则为左负右正 
    if (fabs(list[i].curvature) > 10) //- FLAGS_curvature_init_value) <0.01)
      list[i].curvature = Curve;
  }
  if (fit_start_index == fit_end_index && fabs(list[fit_start_index].curvature) > 10){
    list[fit_start_index].curvature = curve_end_;
  }

  for(i = 0 ; i < fit_start_index; ++i){
    if (fabs(list[i].curvature) >10) //- FLAGS_curvature_init_value)<0.01 )
      list[i].curvature = list[fit_start_index].curvature;
  }

  if (list.back().length < 40 ){
    for(i = fit_end_index ; i < list_size; ++i){
      if (fabs(list[i].curvature) > 10) //- FLAGS_curvature_init_value)<0.01 )
        list[i].curvature = curve_end_;
    }
  }  

  return 0;  
}
int Curvature::CurveUTM(SiteVec& list) {
//使用全局坐标进行计算曲率
  if (list.empty()) return -1;
  if (list.back().length <= 0) {
    CalculateLengthOfPoint(list); 
  }
  geometry::SiteVec path_points_smooth = list;
  int path_point_size = list.size();
  int half_length = std::min(5, int(path_point_size / 2)); //窗口大小为 half_length*2+1
  if (!list.empty()) {
    double fit_length = 2.0; //取的 路点 向量的模长为 3.0m
    //1.路径坐标点窗口平滑处理
    for(int i = 0; i < path_point_size; ++i){
        int j = 0;
        double x_sum = 0;
        double y_sum = 0;
        int smooth_points_cnt = 0;
        if(i < half_length){ //路径前几个路点处理
            for(j = 0; j <= 2 * i; ++j){
                x_sum += list[j].xg;
                y_sum += list[j].yg;
            }
            smooth_points_cnt = j;
        }else if(i >= path_point_size - half_length){ //路径末尾几个路点处理
            for(j = i - (path_point_size - 1 - i); j < path_point_size; ++j){
                x_sum += list[j].xg;
                y_sum += list[j].yg;
            }
            smooth_points_cnt = path_point_size - (i - (path_point_size - 1 - i));
        }else{//正常平滑处理
            for(j = i - half_length; j <= i + half_length; ++j){
                x_sum += list[j].xg;
                y_sum += list[j].yg;
            }
            smooth_points_cnt = 2 * half_length + 1;
        }
        smooth_points_cnt = std::max(1,smooth_points_cnt);
        path_points_smooth[i].xg = x_sum / smooth_points_cnt;
        path_points_smooth[i].yg = y_sum / smooth_points_cnt;
    }
    //2.根据截取路径的 两个向量夹角及模长，计算曲率
    double cal_length = 0;
    geometry::Site sample_points[3];
    int i, j;
    int fit_start_index = 0;
    int fit_end_index = 0;
    // 获得向量的第一个起点
    sample_points[0] = path_points_smooth[0];
    for(i = 0; i < path_point_size; ++i){
        cal_length = std::hypot( sample_points[0].xg - path_points_smooth[i].xg,
                                 sample_points[0].yg - path_points_smooth[i].yg);
        if(cal_length >= fit_length){
            sample_points[1] = path_points_smooth[i];
            fit_start_index = i;
            break;
        }
    }
    if(fit_start_index == 0){
        for(i = 0; i < path_point_size; ++i){
            if (fabs(list[i].curvature) > 10)// - FLAGS_curvature_init_value )<0.01)
                list[i].curvature = curve_end_;
        }
        return -1;
    }
    //开始正常提取向量
    for(i = fit_start_index; i < path_point_size; ++i){
        int m, n;
        //向量中间点
        sample_points[1] = path_points_smooth[i];
        //计算左向量终点
        for(m = i; m > 0; --m){
            cal_length = std::hypot( sample_points[1].xg - path_points_smooth[m].xg,
                                     sample_points[1].yg - path_points_smooth[m].yg);
            if(cal_length >= fit_length){
                sample_points[0] = path_points_smooth[m];
                break;
            }
        }
        if(m == 0)sample_points[0] = path_points_smooth[0];
        //计算右向量终点
        for(m = i; m < path_point_size; ++m){
            cal_length = std::hypot( sample_points[1].xg - path_points_smooth[m].xg,
                                     sample_points[1].yg - path_points_smooth[m].yg);

            if(cal_length >= fit_length){
                sample_points[2] = path_points_smooth[m];
                break;
            }
        }

        if(m == path_point_size) {
            fit_end_index = i;
            if( fabs(list[fit_end_index-1].curvature) > 10) {//-FLAGS_curvature_init_value)>0.01 ){
                curve_end_ = list[fit_end_index-1].curvature;
            }
            break;
        }

        //计算 路点 曲率
        geometry::Site vecA, vecB,vecAB;
        double AngleAB = 0;
        //根据三点坐标，计算A、B向量
        vecA.xg = sample_points[0].xg - sample_points[1].xg;
        vecA.yg = sample_points[0].yg - sample_points[1].yg;
        vecB.xg = sample_points[2].xg - sample_points[1].xg;
        vecB.yg = sample_points[2].yg - sample_points[1].yg;
        vecAB.xg = vecA.xg - vecB.xg;
        vecAB.yg = vecA.yg - vecB.yg;
        double vector_OA_OB =  vecA.xg * vecB.yg - vecA.yg * vecB.xg;  //OAXOB(叉乘)其值为以OA和OB向量构成的平行四边形的面积。
        double module_vecA = sqrt(vecA.xg * vecA.xg + vecA.yg * vecA.yg);//向量OA的模
        double module_vecB = sqrt(vecB.xg * vecB.xg + vecB.yg * vecB.yg);//向量OB的模
        double module_vecAB = sqrt(vecAB.xg * vecAB.xg + vecAB.yg * vecAB.yg);//向量AB的模
        double Curve = 2 * vector_OA_OB/(module_vecA*module_vecB*module_vecAB);//三角形外切圆半径R=abc/4S,abc为三角形三边长，S为三角形面积。所以kappa=1/R=4S/(abc)
        Curve = -Curve;//@pqg 因后面的规划层算出来的曲率为左正右负，这里不取反，则为左负右正 
        if(fabs(list[i].curvature) > 10) //- FLAGS_curvature_init_value) <0.01)
            list[i].curvature = Curve;
    }
    if(fit_start_index == fit_end_index){
        if (fabs(list[fit_start_index].curvature) > 10)// - FLAGS_curvature_init_value)<0.01 )
            list[fit_start_index].curvature = curve_end_;
    }

    for(i = 0 ; i < fit_start_index; ++i){
        if (fabs(list[i].curvature) > 10) //- FLAGS_curvature_init_value)<0.01 )
            list[i].curvature = list[fit_start_index].curvature;
    }

    if (list[path_point_size - 1].length < 40 || 1){
        for(i = fit_end_index ; i < path_point_size; ++i){
            if (fabs(list[i].curvature) > 10) //- FLAGS_curvature_init_value)<0.01 )
                list[i].curvature = curve_end_;
        }
    }
  }
  return 0;   
} 


void Curvature::CalculateLengthOfPoint(SiteVec &list){
  if (list.empty()) return;
  int points_size = list.size();
  double length_accumulate = 0.0;
  for (int i = 0; i < points_size; ++i){
    if (i == 0){
      list[i].length = 0.0;
      continue;
    } 
    double distance = std::hypot(list[i].xg - list[i-1].xg, 
                                 list[i].yg - list[i-1].yg);
    length_accumulate = length_accumulate + distance;
    list[i].length = length_accumulate;
  }
}




} // math
} // planning
} // acu

