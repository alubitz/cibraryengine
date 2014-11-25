#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;

	class NeuralNet;

	class ScaledIOBrain
	{
		private:

			unsigned int AbortRead(NeuralNet* nn, unsigned int code);

		public:

			NeuralNet* nn;
			vector<float> input_centers, input_scales;
			vector<float> output_centers, output_scales;

			ScaledIOBrain() : nn(NULL) { }
			ScaledIOBrain(NeuralNet* nn);

			~ScaledIOBrain();

			unsigned int Read(istream& s);
			unsigned int Write(ostream& s);
	};
}
