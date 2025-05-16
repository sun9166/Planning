
#include "common/toolbox/geometry/include/bezier.h"

namespace geometry {

/*
calculate Bezier Curve's position and angel
input SiteVec& curve
output int: (0: sucess, 1: angle failed, -1: failed)
*/
int Bezier::CalculateCurve(SiteVec& curve)
{
	int rtvalue = -1;
	curve.clear();
	float t, u, t2, u2, u3, t3;
	for(t = 0.0; t <= 1.0; t += _reselution)
	{
		u = 1.0 - t;
		t2 = t * t;
		u2 = u * u;
		u3 = u2 * u;
		t3 = t2 * t;

		Site pt;
		pt.x = u3*_p0.x + 3*u2*t*_p1.x + 3*u*t2*_p2.x + t3*_p3.x;
		pt.y = u3*_p0.y + 3*u2*t*_p1.y + 3*u*t2*_p2.y + t3*_p3.y;

		curve.emplace_back(pt);
		rtvalue = 1;
	}
 //assign the angel for Site
	if(curve.empty()) {return rtvalue;}

    auto iter = curve.begin();
	for(; iter!=curve.end() && std::next(iter)!=curve.end(); ++iter)
	{
		iter->angle = ((*std::next(iter))- (*iter)).inerangle();
	}
	curve.back().angle = _p3.angle;
	rtvalue = 0;
	return rtvalue;
}

/*
calculate Bezier Curve's control points
input 
output int: (0: sucess, -1: failed)
*/
int Bezier::CalculateCtrlPoints()
{
	int rtvalue = -1;
	float dis_p0_p3 = std::hypot(_p0.x - _p3.x, _p0.y - _p3.y);

	_reselution = 0.05/dis_p0_p3;

	if(fabs(dis_p0_p3)<1e-3) {return rtvalue;}

	float move_dis = 1.0/3.0 * dis_p0_p3;

	_p1.x = _p0.x + move_dis * std::cos(_p0.angle * M_PI / 180.0);
	_p1.y = _p0.y + move_dis * std::sin(_p0.angle * M_PI / 180.0);

	_p2.x = _p3.x - move_dis * std::cos(_p3.angle * M_PI / 180.0);
	_p2.y = _p3.y - move_dis * std::sin(_p3.angle * M_PI / 180.0);

	rtvalue = 0;
	return rtvalue;
}



/*
calculate Bezier Curve API
input Site: start,end, pre: precision
output int: (0: sucess, -1: failed) curve: result
*/
int Bezier::GenerateBezierCurve(const Site &start,const Site &end,SiteVec& curve)
    {
        int rtvalue = -1;
        _p0=start;
        _p3=end;

        auto calc_ctrl_pt_rt = CalculateCtrlPoints();
        if(calc_ctrl_pt_rt != 0)
        {//calc ctrl points failed
            return rtvalue;
        }

        auto cal_curve_rt = CalculateCurve(curve);
        if(cal_curve_rt != 0)
        {//calc curve failed
            return rtvalue;
        }

        rtvalue = 0;
        return rtvalue;
    }

}//namespace