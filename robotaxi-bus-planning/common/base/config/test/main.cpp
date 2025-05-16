#include <iostream>
#include "include/param_config_manager.h"
#include "config_schema.pb.h"


using namespace std;
using namespace acu::common::config;

int main(int argc, char const *argv[])
{


	ParamConfigManager param_config_manager;
	ModelConfig *pmodel_config_;// = new ModelConfig();
	param_config_manager.ModuleConfigLoad("test_config/control_param.conf");

	if (!param_config_manager.GetModelConfig("latcontrol", &pmodel_config_))
	{
		cout << "Error get lon parameter" << endl ;
		return 0;//Status(ErrorCode::CONTROL_INIT_ERROR, "Error get lon parameter");
	}
	double s;
	pmodel_config_->GetValue("lat_period", &s);
	pmodel_config_->ChangeValue("lat_period", 10001234.0);
	param_config_manager.SaveConfig("test_config/control_param.conf");
	cout << "OK : " << s << endl;

	/* code */
	return 0;
}