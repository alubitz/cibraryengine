#pragma once
#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct NeuralNet
	{
		unsigned int num_inputs;					// columns in matrix
		unsigned int num_outputs;					// rows in matrix

		float* matrix;								// indices are in reading order

		NeuralNet(unsigned int num_inputs, unsigned int num_outputs);
		NeuralNet(const NeuralNet& other);
		~NeuralNet();

		void operator =(const NeuralNet& other);

		void Multiply(const float* inputs, float* outputs);

		void SigmoidOutputs(float* outputs);						// applies a function to outputs to make them range from [-1, 1], smoothly
	};

	struct MultiLayerPerceptron
	{
		vector<NeuralNet*> neural_nets;

		MultiLayerPerceptron();
		MultiLayerPerceptron(unsigned int num_layers, unsigned int* layer_sizes);			// num_layers should be >= 2
		MultiLayerPerceptron(const MultiLayerPerceptron& other);

		~MultiLayerPerceptron();

		void operator =(const MultiLayerPerceptron& other);

		void Process(const float* inputs, float* outputs);										// use this once the MLP is trained

		void Train(const float* inputs, float* correct_outputs, float learning_rate);
	};
}