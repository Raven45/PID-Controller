#pragma once
#include <vector>

template <class DataType>
class TransferFunction {

public:

	TransferFunction (std::vector<DataType> Aa, std::vector<DataType> B);
	DataType Update (DataType Input, float DeltaTime);

protected:

	std::vector<DataType> A;
	std::vector<DataType> B;

};

template<class DataType>
inline TransferFunction<DataType>::TransferFunction (std::vector<DataType> A, std::vector<DataType> B) {

	this->A = A;
	this->B = B;
}

template<class DataType>
inline DataType TransferFunction<DataType>::Update (DataType Input, float DeltaTime) {
	
	DataType Output = 0.25*Input + 0.5f*B[0] + 0.25f*B[1];
	Output -= -1.0f*A[0] + 0.0025f*A[1];

	B[1] = B[0];
	A[1] = A[0];
	B[0] = Input;
	A[0] = Output;

	return Output ;
}
