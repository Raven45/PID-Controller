#pragma once

namespace ControlLib {

	static const float KP_DEFAULT = 1.0f;
	static const float KI_DEFAULT = 0.0f;
	static const float KD_DEFAULT = 0.0f;

	template <class DataType>
	class PID {

	public:
		PID ();
		PID (float P);
		PID (float P, float I);
		PID (float P, float I, float D);

		void SetKp (float Kp);
		void SetKi (float Ki);
		void SetKd (float Kd);
		void SetZeroPoint (DataType ZeroPoint);

		float GetKp () const;
		float GetKi () const;
		float GetKd () const;
		DataType GetZeroPoint () const;

		DataType Update (DataType Error, unsigned int DeltaTime);

	protected:
		float Kp;
		float Ki;
		float Kd;
		DataType PreviousError;
		DataType SumErrors;
		DataType ZeroPoint;

		DataType CalculateP (DataType Error);
		DataType CalculateI (DataType Error, unsigned int DeltaTime);
		DataType CalculateD (DataType Error, unsigned int DeltaTime);
	};

	template<class DataType>
	inline PID<DataType>::PID () {
		this->Kp = KP_DEFAULT;
		this->Ki = KI_DEFAULT;
		this->Kd = KD_DEFAULT;
	}

	template<class DataType>
	inline PID<DataType>::PID (float P) {
		this->Kp = P;
		this->Ki = KI_DEFAULT;
		this->Kd = KD_DEFAULT;
	}

	template<class DataType>
	inline PID<DataType>::PID (float P, float I) {
		this->Kp = P;
		this->Ki = I;
		this->Kd = KD_DEFAULT;
	}

	template<class DataType>
	inline PID<DataType>::PID (float P, float I, float D) {
		this->Kp = P;
		this->Ki = I;
		this->Kd = D;
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
	inline DataType PID<DataType>::Update (DataType Error, unsigned int DeltaTime) {
		
		return CalculateP (Error) + CalculateD (Error, DeltaTime) + CalculateI (Error, DeltaTime);
	}

	template<class DataType>
	inline DataType PID<DataType>::CalculateP (DataType Error) {
		
		return Kp * Error;
	}

	template<class DataType>
	inline DataType PID<DataType>::CalculateI (DataType Error, unsigned int DeltaTime) {

		DataType Result = ZeroPoint;

		SumErrors += Ki * Error;
	
		return SumErrors * DeltaTime;
	}

	template<class DataType>
	inline DataType PID<DataType>::CalculateD (DataType Error, unsigned int DeltaTime) {
		
		DataType Output;
		Output = (Kd * (Error - PreviousError)) / DeltaTime;
		return Output;
	}


}

