#include "StdAfx.h"
#include "ScaledIOBrain.h"

#include "NeuralNet.h"

namespace Test
{
	using namespace CibraryEngine;



	/*
	 * ScaledIOBrain methods
	 */
	ScaledIOBrain::ScaledIOBrain(NeuralNet* nn) : nn(nn), input_scales(nn->num_inputs), output_scales(nn->num_outputs) { }
	ScaledIOBrain::~ScaledIOBrain() { NeuralNet::Delete(nn); nn = NULL; }

	unsigned int ScaledIOBrain::AbortRead(NeuralNet* nn, unsigned int code) { input_scales.clear(); output_scales.clear(); NeuralNet::Delete(nn); nn = NULL; return code; }

	unsigned int ScaledIOBrain::Read(istream& s)
	{
		BinaryChunk whole;
		whole.Read(s);

		if(whole.GetName() != "BRAINDAT")
			return 1;
		else
		{
			istringstream ss(whole.data);

			NeuralNet* temp;
			if(unsigned int error = NeuralNet::Read(ss, temp))			// can be as high as 2
				return 1 + error;

			unsigned int num_inputs = ReadUInt32(ss);
			if(num_inputs != temp->num_inputs)
				return AbortRead(temp, 4);

			input_centers.resize(num_inputs);
			input_scales.resize(num_inputs);
			for(unsigned int i = 0; i < num_inputs; ++i)
			{
				input_centers[i] = ReadSingle(ss);
				input_scales[i] = ReadSingle(ss);
			}

			if(ss.bad())
				return AbortRead(temp, 5);

			unsigned int num_outputs = ReadUInt32(ss);
			if(num_outputs != temp->num_outputs)
				return AbortRead(temp, 6);

			output_centers.resize(num_outputs);
			output_scales.resize(num_outputs);
			for(unsigned int i = 0; i < num_outputs; ++i)
			{
				output_centers[i] = ReadSingle(ss);
				output_scales[i] = ReadSingle(ss);
			}

			if(ss.bad())
				return AbortRead(temp, 7);

			unsigned int sevens = ReadUInt32(ss);
			if(sevens != 7777777)
				return AbortRead(temp, 8);

			nn = temp;
			return 0;
		}
	}

	unsigned int ScaledIOBrain::Write(ostream& s)
	{
		BinaryChunk whole("BRAINDAT");

		stringstream ss;
		nn->Write(ss);

		WriteUInt32(input_centers.size(), ss);
		for(unsigned int i = 0; i < input_scales.size(); ++i)
		{
			WriteSingle(input_centers[i], ss);
			WriteSingle(input_scales[i], ss);
		}

		WriteUInt32(output_centers.size(), ss);
		for(unsigned int i = 0; i < output_scales.size(); ++i)
		{
			WriteSingle(output_centers[i], ss);
			WriteSingle(output_scales[i], ss);
		}

		WriteUInt32(7777777, ss);

		whole.data = ss.str();
		whole.Write(s);

		return 0;
	}
}
