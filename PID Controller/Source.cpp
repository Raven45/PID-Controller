#include <iostream>							//standard template library.
#include <sstream>							//standard template library.
#include "File.h"							//CSV file definitions.
#include "PID.hpp"							//PID-Controller library.
#include "TransferFunction.hpp"				//Class for implementing the plant
											//in discreet time.

void main () {

	std::vector<float> A;
	std::vector<float> B;
	A.push_back(0.0f);						//Define initial conditions for plant
	B.push_back(0.0f);						//Define initial conditions for plant
	A.push_back (0.0f);						//Define initial conditions for plant
	B.push_back (0.0f);						//Define initial conditions for plant
	TransferFunction <float> Plant(A,B);	//Define the plant.

	ControlLib::PID<float> Controller;		//Create controller instance.
	Controller.SetKp (35.0f);				//Proportional gain.
	Controller.SetKi (0.1f);				//Turn integral off.
	Controller.SetKd (0.0f);				//Turn off derivative action
	Controller.SetZeroPoint (0.0f);			//Define zero.
	Controller.Initialize ();				//Initialize the controller.

	CSV_File output;						//Define output.
	output.CreateFile ("PID_Test_01.csv");	//Create output file.

	unsigned int DeltaTime = 1;			//Define constant time.
	float SV = 0;							//Define set point.
	float PV = 0;							//Define measured point.
	float PID_Out = 0;
	float Kv = 954.93f;

	//Loop through fifty test values.
	for (int i = 0; i < 50; i++) {

		std::vector<std::string> Line;
		if (i == 10) {
			//set to 3000 rpm
			SV = 3000.0f;					//Sudden change in the set point.
		}

		//Write setpoint to file.
		Line.push_back (std::to_string (i));
		Line.push_back (std::to_string (SV));

		float Error = SV - PV; //in rpm
		float ErrorVolts = Error / Kv; //in volts

		//Call to the PID controller.
		PID_Out = Controller.Update (ErrorVolts, DeltaTime);

		//Calculate output from the plant.
		PV = 9.55f * Plant.Update (PID_Out, DeltaTime);

		//Write output to file.
		Line.push_back (std::to_string (PID_Out));
		Line.push_back (std::to_string (PV));
		output.WriteLine (Line);
	}
}