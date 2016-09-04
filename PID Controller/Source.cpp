#include <iostream>
#include <sstream>
#include "File.h"
#include "PID.hpp"

void main () {

	ControlLib::PID<int> Controller;
	Controller.SetKp (10.0f);
	Controller.SetKi (1.0f);
	Controller.SetZeroPoint (0);
	Controller.Initialize ();

	CSV_File output;
	output.CreateFile ("PID_Test_01.csv");

	unsigned int DeltaTime = 0x1F0;
	int SV = 0;
	int PV = 0;
	for (int i = 0; i < 500; i++) {

		std::vector<std::string> Line;
		if (i < 100) {
			Line.push_back ("0");
		}
		else {
			Line.push_back ("50");
			SV = 50;
		}

		PV = Controller.Update ((SV), DeltaTime);

		std::string str_output = std::to_string (PV);
		Line.push_back (str_output);

		output.WriteLine (Line);
	}
}