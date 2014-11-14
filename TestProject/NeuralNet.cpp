#include "StdAfx.h"
#include "NeuralNet.h"

namespace Test
{
	using namespace CibraryEngine;



	/*
	 * NeuralNet methods
	 */
	// private constructor & destructor
	NeuralNet::NeuralNet(unsigned int num_inputs, unsigned int num_outputs, unsigned int num_middles, float* data_end, unsigned int last_size) :
		num_inputs (num_inputs),
		num_outputs(num_outputs),
		num_middles(num_middles),
		top_matrix_size   (num_inputs  * num_middles),
		bottom_matrix_size(num_middles * num_outputs),
		data(data_end),
		inputs          (NextSection(num_inputs,         last_size, data_end)),
		middle_sums     (NextSection(num_middles,        last_size, data_end)),
		middles         (NextSection(num_middles,        last_size, data_end)),
		output_sums     (NextSection(num_outputs,        last_size, data_end)),
		outputs         (NextSection(num_outputs,        last_size, data_end)),
		correct_outputs (NextSection(num_outputs,        last_size, data_end)),
		errors          (NextSection(num_outputs,        last_size, data_end)),
		phiprime_mids   (NextSection(num_middles,        last_size, data_end)),
		phiprime_outs   (NextSection(num_outputs,        last_size, data_end)),
		top_matrix      (NextSection(top_matrix_size,    last_size, data_end)),
		temp_top        (NextSection(top_matrix_size,    last_size, data_end)),
		bottom_matrix   (NextSection(bottom_matrix_size, last_size, data_end)),
		temp_bottom     (NextSection(bottom_matrix_size, last_size, data_end))
	{
	}

	NeuralNet::~NeuralNet() { }


	// public static constructor-ish and destructor-ish functions
	NeuralNet* NeuralNet::New(unsigned int num_inputs, unsigned int num_outputs, unsigned int num_middles)
	{
		unsigned int needed_size = sizeof(NeuralNet) + GetNumFloats(num_inputs, num_outputs, num_middles) * sizeof(float);

		NeuralNet* nn_ptr = (NeuralNet*)malloc(needed_size);

		return new(nn_ptr) NeuralNet(num_inputs, num_outputs, num_middles, (float*)(nn_ptr + 1), 0);
	}

	void NeuralNet::Delete(NeuralNet* nn) { free(nn); }


	// public methods (there are some more public methods besides these; their implementations are in the header)
	void NeuralNet::Randomize(float scale)
	{
		for(float *ptr = top_matrix, *end = ptr + top_matrix_size; ptr != end; ++ptr)
			*ptr = Random3D::Rand(-scale, scale);
		for(float *ptr = bottom_matrix, *end = ptr + bottom_matrix_size; ptr != end; ++ptr)
			*ptr = Random3D::Rand(-scale, scale);
	}

	float NeuralNet::Train(float learning_rate)
	{
		float initial_error = EvaluateAndScore();

		float* ppo_end = phiprime_outs + num_outputs;
		float* ppm_end = phiprime_mids + num_middles;
		for(float *ppo_ptr = phiprime_outs, *outputs_ptr = outputs; ppo_ptr != ppo_end; ++ppo_ptr, ++outputs_ptr)
			*ppo_ptr = 1.0f - *outputs_ptr * *outputs_ptr;
		for(float *ppm_ptr = phiprime_mids, *middles_ptr = middles; ppm_ptr != ppm_end; ++ppm_ptr, ++middles_ptr)
			*ppm_ptr = 1.0f - *middles_ptr * *middles_ptr;

		float* temp_bottom_ptr = temp_bottom;
		float* bottom_ptr      = bottom_matrix;
		float* middles_end     = middles + num_middles;
		for(float *ppo_ptr = phiprime_outs, *ppo_end = ppo_ptr + num_outputs, *err_ptr = errors; ppo_ptr != ppo_end; ++ppo_ptr, ++err_ptr)
			for(float* mid_ptr = middles; mid_ptr != middles_end; ++mid_ptr, ++temp_bottom_ptr, ++bottom_ptr)
				*temp_bottom_ptr = *bottom_ptr - learning_rate * 2.0f * *err_ptr * *ppo_ptr * *mid_ptr;

		float* temp_top_ptr = temp_top;
		float* top_ptr      = top_matrix;
		float* inputs_end   = inputs + num_inputs;
		for(float* input_ptr = inputs; input_ptr != inputs_end; ++input_ptr)
			for(float* top_plus_middles = top_ptr + num_middles; top_ptr != top_plus_middles; ++top_ptr, ++temp_top_ptr)
			{
				bottom_ptr = bottom_matrix;

				float derrordtop = 0.0f;
				for(float *ppo_ptr = phiprime_outs, *err_ptr = errors; ppo_ptr != ppo_end; ++ppo_ptr, ++err_ptr)
				{
					float doutputdtop = 0.0f;
					for(float* ppm_ptr = phiprime_mids; ppm_ptr != ppm_end; ++ppm_ptr, ++bottom_ptr)
						doutputdtop += *bottom_ptr * *ppm_ptr * *input_ptr;
					derrordtop += *err_ptr * doutputdtop * *ppo_ptr;
				}
				derrordtop *= 2.0f;
				*temp_top_ptr = *top_ptr - learning_rate * derrordtop;
			}

		memcpy(bottom_matrix, temp_bottom, num_middles * num_outputs * sizeof(float));
		memcpy(top_matrix,    temp_top,    num_inputs  * num_middles * sizeof(float));

		return initial_error;
	}



	unsigned int NeuralNet::Write(ostream& s) const
	{	
		stringstream nnss;
		WriteUInt32(num_inputs, nnss);
		WriteUInt32(num_outputs, nnss);
		WriteUInt32(num_middles, nnss);

		unsigned int num_floats = GetNumFloats(num_inputs, num_outputs, num_middles);

		WriteUInt32(num_floats,  nnss);					// write this to act as sort of a checksum
		for(unsigned int i = 0; i < num_floats; ++i)
			WriteSingle(data[i], nnss);

		BinaryChunk chunk("NEURALNT");
		chunk.data = nnss.str();
		chunk.Write(s);

		return 0;
	}

	unsigned int NeuralNet::Read(istream& s, NeuralNet*& result)
	{
		BinaryChunk chunk;
		chunk.Read(s);

		if(chunk.GetName() != "NEURALNT")
			return 1;

		istringstream nnss(chunk.data);

		unsigned int num_inputs  = ReadUInt32(nnss);
		unsigned int num_outputs = ReadUInt32(nnss);
		unsigned int num_middles = ReadUInt32(nnss);
		unsigned int num_floats  = ReadUInt32(nnss);

		if(num_floats != GetNumFloats(num_inputs, num_outputs, num_middles))
			return 2;

		result = New(num_inputs, num_outputs, num_middles);
		for(float *data_ptr = result->data, *data_end = data_ptr + num_floats; data_ptr != data_end; ++data_ptr)
			*data_ptr = ReadSingle(nnss);

		return 0;
	}
}
