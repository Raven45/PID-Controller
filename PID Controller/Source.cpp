#include <iostream>							//standard template library.
#include <sstream>							//standard template library.
#include "File.h"							//CSV file definitions.
#include "PID.hpp"							//PID-Controller library.

void main () {

	ControlLib::PID<float> Controller;		//Create controller instance.
	Controller.SetKp (10.0f);				//Proportional gain.
	Controller.SetKi (0.1f);				//Turn integral off.
	Controller.SetKd (0.0f);				//Turn derivative off.
	Controller.SetZeroPoint (0);			//Define zero.
	Controller.Initialize ();				//Initialize the controller.

	CSV_File output;						//Define output.
	output.CreateFile ("PID_Test_01.csv");	//Create output file.

	unsigned int DeltaTime = 0x1F0;			//Define constant time.
	float SV = 0;							//Define set point.
	float PV = 0;							//Define measured point.
	float PID_Out = 0;

	//Loop through fifty test values.
	for (int i = 0; i < 50; i++) {

		std::vector<std::string> Line;
		if (i == 10) {
			SV = 50.0f;						//Sudden change in the set point.
		}
		else if (i == 15) {
			SV = 0.0f;						//Sudden change in the set point.
		}

		//Write setpoint to file.
		Line.push_back (std::to_string (SV));

		//Call to the PID controller.
		PID_Out = Controller.Update ((SV), DeltaTime);

		//Write output to file.
		std::string str_output = std::to_string (PID_Out);
		Line.push_back (str_output);
		output.WriteLine (Line);
	}
}