```c++
#include "geometry/curves/dubins/dubins.h"
#include "geometry/points/geoheader.h"

using namespace geometry;
using geometry::dubins::Dubins;

int main(int argc, char* argv[]) {
  Dubins dubins;
  SiteVec list;
  dubins.GetPath(list, Site(1, 1, -45), Site(10, 10, -45));

  std::ofstream out("data");
  for (auto& p : list) {
    out << p.x << "," << p.y << std::endl;
  }
  out.close();

  return 0;
}
```