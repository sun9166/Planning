#include "../include/monitor_api.h"


int main(int argc, char const *argv[])
{
	auto monitor_api = avos::common::MonitorApi::Instance();
	monitor_api->InitMonitorTable("monitor_table.conf");
	monitor_api->InitMonitor("test", 1000);

	int base = 0;

	int counter = 0;
	while (1) {
		monitor_api->HeartBeat();

		for (int i = 0; i < 3; i++) {
			monitor_api->SetFaultInfo(0, base + i, "error " + std::to_string(i));
		}

		std::cout << "set:" << base << std::endl;
		monitor_api->Print();
		std::cout << "set self_drive fault_level:" << monitor_api->GetFaultTableLevel("self_drive") << std::endl;
		std::cout << "set monitor fault_level:" << monitor_api->GetFaultTableLevel("monitor") << std::endl;
		sleep(1);
		for (int i = 0; i < 3; i++) {
			monitor_api->ResetFaultInfo(0, base + i);
		}
		std::cout << "reset:" << std::endl;
		monitor_api->Print();

		std::cout << "reset self_drive fault_level:" << monitor_api->GetFaultTableLevel("self_drive") << std::endl;
		std::cout << "reset monitor fault_level:" << monitor_api->GetFaultTableLevel("monitor") << std::endl;
		std::cout << "-----------------------" << std::endl;
		base += 3;


		usleep(1000 * 100);
		counter++;
		if (counter > 8) break;
	}

	/* code */
	return 0;
}

