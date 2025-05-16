/**
 * @file path_bound_generator.h
 **/

#pragma once
#include "../base_generator/path_bound_generator.h"

namespace acu {
namespace planning {
class RegularBoundGenerator : public PathBoundGenerator {
 public:
  RegularBoundGenerator(Frame* frame, ReferenceLineInfo* reference_line_info, 
  	const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state);
  ~RegularBoundGenerator() = default;

  common::Status Generate(std::vector<PathBoundary>& candidate_path_boundaries) override;

 protected:


 private:
  
}; 

}  // namespace planning
}  // namespace acu