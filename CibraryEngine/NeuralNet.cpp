#include "StdAfx.h"
#include "NeuralNet.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * NeuralNet methods
	 */
	NeuralNet::NeuralNet(unsigned int num_inputs, unsigned int num_outputs) :
		num_inputs(num_inputs),
		num_outputs(num_outputs),
		matrix(new float[num_inputs * num_outputs])
	{
		for(float *ptr = matrix, *end = &matrix[num_inputs * num_outputs]; ptr != end; ++ptr)
			*ptr = 0.0f;
	}

	NeuralNet::NeuralNet(const NeuralNet& other) :
		num_inputs(other.num_inputs),
		num_outputs(other.num_outputs),
		matrix(new float[num_inputs * num_outputs])
	{
		float* my_end = &matrix[num_inputs * num_outputs];
		for(float *my_ptr = matrix, *other_ptr = other.matrix; my_ptr != my_end; ++my_ptr, ++other_ptr)
			*my_ptr = *other_ptr;
	}

	NeuralNet::~NeuralNet() { delete[] matrix; matrix = NULL; }

	void NeuralNet::operator =(const NeuralNet& other)
	{
		if(&other != this)
		{
			delete[] matrix;

			num_inputs = other.num_inputs;
			num_outputs = other.num_outputs;
			matrix = new float[num_inputs * num_outputs];

			float* my_end = &matrix[num_inputs * num_outputs];
			for(float *my_ptr = matrix, *other_ptr = other.matrix; my_ptr != my_end; ++my_ptr, ++other_ptr)
				*my_ptr = *other_ptr;
		}
	}

	void NeuralNet::Multiply(float* inputs, float* outputs)
	{
		// if the input and output arrays overlap we have to do some extra work
		if(outputs <= inputs + num_inputs && outputs + num_outputs >= inputs)
		{
			float* temp = new float[num_outputs];

			float* input_end = &inputs[num_inputs];
			float* temp_end = &temp[num_outputs];

			float* matrix_ptr = matrix;
			for(float* temp_ptr = temp; temp_ptr != temp_end; ++temp_ptr)
			{
				float output = 0.0f;

				float* input_ptr = inputs;
				while(input_ptr != input_end)
					output += *(matrix_ptr++) * *(input_ptr++);

				*temp_ptr = output;
			}

			for(float *temp_ptr = temp, *out_ptr = outputs; temp_ptr != temp_end; ++out_ptr, ++temp_ptr)
				*out_ptr = *temp_ptr;

			delete[] temp;
		}
		else
		{
			float* input_end = &inputs[num_inputs];
			float* output_end = &outputs[num_outputs];

			float* matrix_ptr = matrix;
			for(float* output_ptr = outputs; output_ptr != output_end; ++output_ptr)
			{
				float output = 0.0f;

				float* input_ptr = inputs;
				while(input_ptr != input_end)
					output += *(matrix_ptr++) * *(input_ptr++);

				*output_ptr = output;
			}
		}
	}

	void NeuralNet::ClampInputs(float* inputs)
	{
		float* end = &inputs[num_inputs];
		for(float* ptr = inputs; ptr != end; ++ptr)
			*ptr = max(-1.0f, min(1.0f, *ptr));
	}

	void NeuralNet::ClampOutputs(float* outputs)
	{
		float* end = &outputs[num_outputs];
		for(float* ptr = outputs; ptr != end; ++ptr)
			*ptr = max(-1.0f, min(1.0f, *ptr));
	}
}
