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
			~MultiLayerBrain();

			void SetZero();

			void Evaluate(const vector<float>& inputs, vector<float>& outputs, vector<float>& scratch) const;
			float AddGradient(const vector<float>& inputs, vector<float>& outputs, vector<float>& scratch, const vector<float>& correct_outputs, MultiLayerBrain& gradient) const;

			void Train(const MultiLayerBrain& gradient, float learning_rate);

			void Randomize(float scale);
			void RandomizeOne(float scale);

			void GetCoeffTotals(float& tot, float& sqtot) const;
	};
}
