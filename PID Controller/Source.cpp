#include <iostream>
#include <sstream>
#include "File.h"
#include "PID.hpp"

void main () {

	ControlLib::PID<float> Controller;
	Controller.SetKp (10.0f);
	Controller.SetKi (0.01f);
	Controller.SetKd (0.0f);
	Controller.SetN (9);
	Controller.SetZeroPoint (0);
	Controller.Initialize ();

	CSV_File output;
	output.CreateFile ("PID_Test_01.csv");

	unsigned int DeltaTime = 0x1F0;
	float SV = 0;
	float PV = 0;
	for (int i = 0; i < 500; i++) {

		std::vector<std::string> Line;
		if (i < 100) {
		}
		else if (i == 100) {
			SV = 50.0f;
		}
		else if (i == 150) {
			SV = 0.0f;
		}
		Line.push_back (std::to_string (SV));
		PV = Controller.Update ((SV), DeltaTime);

		std::string str_output = std::to_string (PV);
		Line.push_back (str_output);

		output.WriteLine (Line);
	}
}