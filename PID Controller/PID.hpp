#pragma once

namespace ControlLib {

	static const float KP_DEFAULT = 1.0f;
	static const float KI_DEFAULT = 0.0f;
	static const float KS_DEFAULT = 1.0f;
	static const float KD_DEFAULT = 0.0f;
	static const unsigned int N_DEFAULT = 3;

	template <class DataType>
	class PID {

	public:
		/**********************************************************************
		Class Constructors
		**********************************************************************/
		PID ();
		PID (float P);
		PID (float P, float I);
		PID (float P, float I, float D);

		/**********************************************************************
		Function name: SetKp
		Input:		Kp (float): the specified gain for the internal proportional 
					controller.
		Outputs:	None.
		Description: Sets the gain for the internal proportional controller to
		the specified input Kp (float).
		**********************************************************************/
		void SetKp (float Kp);

		/**********************************************************************
		Function name: SetKi
		Input:		Ki (float): the specified gain for the internal integral
					controller.
		Outputs:	None.
		Description: Sets the gain for the internal integral controller to
		the specified input Ki (float).
		**********************************************************************/
		void SetKi (float Ki);

		/**********************************************************************
		Function name: SetKd
		Input:		Kd (float): the specified gain for the internal derivative
		controller.
		Outputs:	None.
		Description: Sets the gain for the internal derivative controller to
		the specified input Kd (float).
		**********************************************************************/
		void SetKd (float Kd);
		void SetZeroPoint (DataType ZeroPoint);

		/**********************************************************************
		Function name: SetN
		Input:		N (unsigned int): the number of interations to use within
					the 3/8ths rule.
		Outputs:	None.
		Description: Sets the number of iterations to use when integrating
					 the input Error. Must be a multiple of 3. This value
					 sets the "N" value in Simpson's 3/8ths rule formula.
					 If a value that is not a multiple of three is supplied, 
					 then no action will be taken.
		**********************************************************************/
		void SetN (unsigned int N);

		/**********************************************************************
		Function name: SetSaturationHighLimit
		Input:			SatFilterHigh (DataType): The high limit for the
						controller's saturation filter.
		Outputs:		None.
		Description:	Sets the upper (high) limit for the controller's
					saturation filter. When the saturation filter is enabled,
					the controller ouput shall not exceed this limit. This
					should be called with a valid value before enabling the
					saturation filter in order to garuntee a defined output
					from the controller.
		**********************************************************************/
		void SetSaturationHighLimit (DataType SatFilterHigh);

		/**********************************************************************
		Function name:	SetSaturationLowLimit
		Input:			SatFilterLow (DataType): The low limit for the
						controller's saturation filter.
		Outputs:		None.
		Description:	Sets thelower limit for the controller's
					saturation filter. When the saturation filter is enabled,
					the controller ouput shall not exceed this limit. This
					should be called with a valid value before enabling the
					saturation filter in order to garuntee a defined output
					from the controller.
		**********************************************************************/
		void SetSaturationLowLimit (DataType SatFilterLow);

		/**********************************************************************
		Function name: GetKp
		Input:		None.
		Outputs:	The proportional gain of the internal P controller.
		**********************************************************************/
		float GetKp () const;

		/**********************************************************************
		Function name: GetKi
		Input:		None.
		Outputs:	The proportional gain of the internal I controller.
		**********************************************************************/
		float GetKi () const;

		/**********************************************************************
		Function name: GetKd
		Input:		None.
		Outputs:	The proportional gain of the internal D controller.
		**********************************************************************/
		float GetKd () const;
		DataType GetZeroPoint () const;

		/**********************************************************************
		Function name: GetN
		Input:		None.
		Outputs:	The N value of the 3/8ths formula.
		**********************************************************************/
		unsigned int GetN ();

		/**********************************************************************
		Function name: GetSaturationHighLimit
		Input:		None.
		Outputs:	The upper (high) limit for the controller's saturation
		filter.
		**********************************************************************/
		DataType GetSaturationHighLimit () const;

		/**********************************************************************
		Function name: GetSaturationLowLimit
		Input:		None.
		Outputs:	The lower limit for the controller's saturation filter.
		**********************************************************************/
		DataType GetSaturationLowLimit () const;

		/**********************************************************************
		Function name: Update
		Input:		Error (DataType): The error point for the PID controller.
					DeltaTime (unsigned int): The amount of time since the
						last time the Update function was called. 
		Outputs:	Output from the PID controller.
		Description: This will take an error and the amount of time since the
			last run and calculate the output of the internal PID controller.
			The error needs to be pre-calculated before being passed to the PID
			controller.
		**********************************************************************/
		DataType Update (DataType Error, unsigned int DeltaTime);

		void Initialize ();

		void EnableSaturationFilter ();
		void DisableSaturationFilter ();

	protected:
		float Kp;							//Proportional gain.
		float Ki;							//Integral gain.
		float Ks;							//Anti-windup gain.
		float Kd;							//Derivative gain.
		DataType Shortcoming;				//Saturation discrepancy. 
		DataType PreviousError;				//Previous error point.
		DataType SumErrors;					//Sum of all errors.
		DataType ZeroPoint;					//The number zero.
		DataType SaturationLimitHigh;		//Upper limit for sat filter.
		DataType SaturationLimitLow;		//Lower limit for sat filter.
		unsigned int IntegrationNValue;		//Variable 'n' in 3/8th's rule.
		bool SaturationFilterEnabled;		//Control flag for saturation filter.

		/**********************************************************************
		Function name: CalculateP
		Input:		Error (DataType): The error point for the PID controller.
		Outputs:	Output from the proportional controller.
		Description: Will take a pre-calculated error and calculate the output
			of the internal proportional controller.
		**********************************************************************/
		DataType CalculateP (DataType Error);

		/**********************************************************************
		Function name: CalculateI
		Input:		Error (DataType): The error point for the PID controller.
		Outputs:	Output from the integral controller.
		Description: Will take a pre-calculated error and calculate the output
		of the internal integral controller.
		**********************************************************************/
		DataType CalculateI (DataType Error, unsigned int DeltaTime);

		/**********************************************************************
		Function name: CalculateD
		Input:		Error (DataType): The error point for the PID controller.
		Outputs:	Output from the derivative controller.
		Description: Will take a pre-calculated error and calculate the output
		of the internal derivative controller.
		**********************************************************************/
		DataType CalculateD (DataType Error, unsigned int DeltaTime);

		DataType SaturationFilter (DataType Error);
	};

	template<class DataType>
	inline PID<DataType>::PID () {
		this->Kp = KP_DEFAULT;
		this->Ki = KI_DEFAULT;
		this->Kd = KD_DEFAULT;
		this->IntegrationNValue = N_DEFAULT;
		this->SaturationFilterEnabled = false;
	}

	template<class DataType>
	inline PID<DataType>::PID (float P) {
		this->Kp = P;
		this->Ki = KI_DEFAULT;
		this->Kd = KD_DEFAULT;
		this->IntegrationNValue = N_DEFAULT;
		this->SaturationFilterEnabled = false;
	}

	template<class DataType>
	inline PID<DataType>::PID (float P, float I) {
		this->Kp = P;
		this->Ki = I;
		this->Kd = KD_DEFAULT;
		this->IntegrationNValue = N_DEFAULT;
		this->SaturationFilterEnabled = false;
	}

	template<class DataType>
	inline PID<DataType>::PID (float P, float I, float D) {
		this->Kp = P;
		this->Ki = I;
		this->Kd = D;
		this->IntegrationNValue = N_DEFAULT;
		this->SaturationFilterEnabled = false;
	}

	template<class DataType>
	inline void PID<DataType>::SetKp (float Kp) {
		this->Kp = Kp;
	}

	template<class DataType>
	inline void PID<DataType>::SetKi (float Ki) {
		this->Ki = Ki;
	}

	template<class DataType>
	inline void PID<DataType>::SetKd (float Kd) {
		this->Kd = Kd;
	}

	template<class DataType>
	inline void PID<DataType>::SetZeroPoint (DataType ZeroPoint) {
		this->ZeroPoint = ZeroPoint;
	}

	template<class DataType>
	inline void PID<DataType>::SetN (unsigned int N) {

		if (N % 3 == 0) {
			this->IntegrationNValue = N;
		}
	}

	template<class DataType>
	inline void PID<DataType>::SetSaturationHighLimit (DataType SatFilterHigh) {
		this->SaturationLimitHigh = SatFilterHigh;
	}

	template<class DataType>
	inline void PID<DataType>::SetSaturationLowLimit (DataType SatFilterLow) {
		this->SaturationLimitLow = SatFilterLow;
	}

	template<class DataType>
	inline float PID<DataType>::GetKp () const {
		return this->Kp;
	}

	template<class DataType>
	inline float PID<DataType>::GetKi () const {
		return this->Ki;
	}

	template<class DataType>
	inline float PID<DataType>::GetKd () const {
		return this->Kd;
	}

	template<class DataType>
	inline DataType PID<DataType>::GetZeroPoint () const {
		return this->ZeroPoint;
	}

	template<class DataType>
	inline unsigned int PID<DataType>::GetN () {
		return IntegrationNValue;
	}

	template<class DataType>
	inline DataType PID<DataType>::GetSaturationHighLimit () const {
		return this->SaturationLimitHigh;
	}

	template<class DataType>
	inline DataType PID<DataType>::GetSaturationLowLimit () const {
		return this->SaturationLimitLow;
	}

	/**********************************************************************
	Function name: Update
	Input:		Error (DataType): The error point for the PID controller.
	DeltaTime (unsigned int): The amount of time since the
	last time the Update function was called.
	Outputs:	Output from the PID controller.
	Description: This will take an error and the amount of time since the
	last run and calculate the output of the internal PID controller.
	The error needs to be pre-calculated before being passed to the PID
	controller.
	History:	9/3/2016: Initial Development
	**********************************************************************/
	template<class DataType>
	inline DataType PID<DataType>::Update (DataType Error, unsigned int DeltaTime) {

		DataType Output;

		//Calculate proportional portion of the output.
		Output = CalculateP (Error);
		
		//Calculate the integral portion if enabled.
		if (Ki != 0.0f) {
			Output += CalculateI (Error, DeltaTime);
		}

		//Calculate the derivative portion if enabled.
		if (Kd != 0.0f) {
			Output += CalculateD (Error, DeltaTime);
		}

		//Run the controller output through the saturation filter.
		if (SaturationFilterEnabled) {

			//Capture the ideal output for anti-windup algorithm.
			DataType IdealOutput = Output;

			//Calculate the output due to saturaiton.
			Output = SaturationFilter (Output);

			//Capture the discrepency between the clamped and ideal outputs.
			Shortcoming = IdealOutput - Output;
		}

		PreviousError = Error;

		return Output;
	}

	template<class DataType>
	inline void PID<DataType>::Initialize () {
		SumErrors = ZeroPoint;
		PreviousError = ZeroPoint;
	}

	template<class DataType>
	inline void PID<DataType>::EnableSaturationFilter () {
		this->SaturationFilterEnabled = true;
	}

	template<class DataType>
	inline void PID<DataType>::DisableSaturationFilter () {
		this->SaturationFilterEnabled = false;
	}

	/**********************************************************************
	Function name: CalculateP
	Input:		Error (DataType): The error point for the PID controller.
	Outputs:	Output from the proportional controller.
	Description: Will take a pre-calculated error and calculate the output
	of the internal proportional controller.
	History:	9/3/2016: Initial Development
	**********************************************************************/
	template<class DataType>
	inline DataType PID<DataType>::CalculateP (DataType Error) {
		
		DataType Output = Kp * Error;
		return Output;
	}

	/**********************************************************************
	Function name: CalculateI
	Input:		Error (DataType): The error point for the PID controller.
	Outputs:	Output from the integral controller.
	Description:	Will take a pre-calculated error and calculate the output
					of the internal integral controller.
	History:	9/3/2016: Initial Development.
				9/4/2016: Implemented the 3/8th's rule for integration.
	**********************************************************************/
	template<class DataType>
	inline DataType PID<DataType>::CalculateI (DataType Error, unsigned int DeltaTime) {

		DataType Output;

		if (SaturationFilterEnabled) {
			SumErrors += Ki*(Error - (Ks * Shortcoming));
		}
		else {
			SumErrors += Ki*Error;
		}

		DataType Sum = SumErrors;
	
		unsigned int n = IntegrationNValue;
		unsigned int H = DeltaTime / n;

		for (int i = 1; i < n; i++) {
			if (i % 3 == 0) {
				Sum += 2 * SumErrors;
			}
			else {
				Sum += 3 * SumErrors;
			}
		}

		Sum += SumErrors;

		//0.375f = 3/8. Optimized to eliminate a division for MCUs.
		Output = 0.375f * H * Sum;

		return Output;
	}

	/**********************************************************************
	Function name: CalculateD
	Input:		Error (DataType): The error point for the PID controller.
	Outputs:	Output from the derivative controller.
	Description: Will take a pre-calculated error and calculate the output
	of the internal derivative controller.
	History:	9/3/2016: Initial Development.
	**********************************************************************/
	template<class DataType>
	inline DataType PID<DataType>::CalculateD (DataType Error, unsigned int DeltaTime) {
		
		DataType Output;
		Output = (Kd * (Error - PreviousError)) / DeltaTime;
		return Output;
	}

	template<class DataType>
	inline DataType PID<DataType>::SaturationFilter (DataType Error) {
		
		if (Error > SaturationLimitLow && Error < SaturationLimitHigh) {
			return Error;
		}
		else if (Error <= SaturationLimitLow) {
			return SaturationLimitLow;
		}
		else if (Error >= SaturationLimitHigh) {
			return SaturationLimitHigh;
		}
	}


}

