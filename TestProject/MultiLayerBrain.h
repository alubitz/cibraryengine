#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	struct MultiLayerBrain
	{
		private:

		public:

			vector<unsigned int> layer_sizes;
			vector<vector<float>> matrices;

			MultiLayerBrain(const vector<unsigned int>& layer_sizes);
			MultiLayerBrain(const MultiLayerBrain& other) : layer_sizes(other.layer_sizes), matrices(other.matrices) { }
			~MultiLayerBrain();

			void SetZero();

			void Evaluate(const vector<float>& inputs, vector<float>& outputs, vector<float>& scratch) const;
			float AddGradient(unsigned int num_target_outputs, const vector<float>& inputs, vector<float>& outputs, vector<float>& derrdin, vector<float>& scratch, const vector<float>& derrdout, MultiLayerBrain& gradient) const;

			void Train(const MultiLayerBrain& gradient, float learning_rate);

			void Randomize(float scale);
			void RandomizeOne(float scale);

			void GetCoeffTotals(float& tot, float& sqtot) const;

			unsigned int Write(ostream& s);
			static unsigned int Read(istream& s, MultiLayerBrain*& result, unsigned int id);

		private:

			float ComputeDerrorDoutputs(unsigned int count, const float* outputs, const float* correct_outputs, float* derrdout, const float* scratch_end) const;

			MultiLayerBrain(const vector<unsigned int>& layer_sizes, const vector<vector<float>>& matrices) : layer_sizes(layer_sizes), matrices(matrices) { }
	};
}
