#include "FeedBack.h"

SMSBL sm;

FeedBack feedback[6];



void MyFeedBack(char * serial_name)
{
	if (!sm.begin(1000000, serial_name))
	{
		printf("Failed to read serial\n");
		return;
	}
	std::thread timer_thread(refresh_timer);
	timer_thread.detach();
	// sm.end();
	return;
}

void update_data()
{
	for (int i = 0; i < 6; i++)
	{
		if (sm.FeedBack(i) == -1)
		{
			break;
			// std::cout << ("Read Error!\n") << std::endl;
			// for (int j = 0; j < 5; j++)
			// 	std::cout << ("\n") << std::endl;
		}
		feedback[i].Joint = i;

		feedback[i].Pos = sm.ReadPos(-1);

		feedback[i].Speed = sm.ReadSpeed(-1);

		feedback[i].Load = sm.ReadLoad(-1);

		feedback[i].Temper = sm.ReadTemper(-1);

		feedback[i].Move = sm.ReadMove(-1);


		// std::cout << ("#####Joint: %d#####", i) << std::endl;
		// std::cout << ("Pos: ");
		// std::cout << ("%d", sm.ReadPos(-1)) << std::endl;
		// std::cout << ("Speed: ");
		// std::cout << ("%d", sm.ReadSpeed(-1)) << std::endl;
		// std::cout << ("Load: ");
		// std::cout << ("%d", sm.ReadLoad(-1)) << std::endl;
		// std::cout << ("Temper: ");
		// std::cout << ("%d", sm.ReadTemper(-1)) << std::endl;
		// std::cout << ("Move: ");
		// std::cout << ("%d", sm.ReadMove(-1)) << std::endl;


	}
	// feedback[2].Pos = feedback[1].Pos + feedback[2].Pos;

}

void refresh_timer()
{
	while (true)
	{
		update_data();
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
}