#pragma once
#include "StdAfx.h"

namespace CibraryEngine
{
	struct NeuralNet
	{
		unsigned int num_inputs;					// columns in matrix
		unsigned int num_outputs;					// rows in matrix

		float* matrix;								// indices are in reading order

		NeuralNet(unsigned int num_inputs, unsigned int num_outputs);
		NeuralNet(const NeuralNet& other);
		~NeuralNet();

		void operator =(const NeuralNet& other);

		void Multiply(float* inputs, float* outputs);
		void ClampInputs(float* inputs);							// clamp inputs to the range [-1, 1]
		void ClampOutputs(float* outputs);							// clamp outputs to the range [-1, 1]

		void SigmoidOutputs(float* outputs);						// like ClampOutputs, but smoothly
	};
}