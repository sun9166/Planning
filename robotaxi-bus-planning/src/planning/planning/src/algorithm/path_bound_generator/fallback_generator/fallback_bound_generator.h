/**
 * @file path_bound_generator.h
 **/

#pragma once
#include "../base_generator/path_bound_generator.h"

namespace acu {
namespace planning {
class FallbackBoundGenerator : public PathBoundGenerator {
 public:
  FallbackBoundGenerator(Frame* frame, ReferenceLineInfo* reference_line_info, 
  	   const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state);
  ~FallbackBoundGenerator() = default;

  const std::string& Name() const override{
  	return "FallbackBoundGenerator";
  }

common::Status Generate(std::vector<PathBoundary>& candidate_path_boundaries) override;

 protected:


 private:
  
}; 
}  // namespace planning
}  // namespace acu