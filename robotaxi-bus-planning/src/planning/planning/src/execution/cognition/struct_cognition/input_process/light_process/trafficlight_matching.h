#include "locperception_input.h"
#include "mapengine_input.h"

namespace acu{
namespace planning {


class TrafficLightMatching {
 public:
	void MatchTrafficLights(const PerceptionData& perception_data,
							const LocalizationData &localization,
							vector<StructTrafficlightInfo> &matched_lights);
 private:
 	void Convert(const TrafficLight& perception_light, 
				 StructTrafficlightInfo &output_light,
				 double timestamp);
};

}
}