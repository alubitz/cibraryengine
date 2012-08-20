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
		for(float *ptr = matrix, *end = matrix + num_inputs * num_outputs; ptr != end; ++ptr)
			*ptr = 0.0f;
	}

	NeuralNet::NeuralNet(const NeuralNet& other) :
		num_inputs(other.num_inputs),
		num_outputs(other.num_outputs),
		matrix(new float[num_inputs * num_outputs])
	{
		float* my_end = matrix + num_inputs * num_outputs;
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

			float* my_end = matrix + num_inputs * num_outputs;
			for(float *my_ptr = matrix, *other_ptr = other.matrix; my_ptr != my_end; ++my_ptr, ++other_ptr)
				*my_ptr = *other_ptr;
		}
	}

	void NeuralNet::Multiply(const float* inputs, float* outputs)
	{
		// if the input and output arrays overlap we have to do some extra work
		if(outputs <= inputs + num_inputs && outputs + num_outputs >= inputs)
		{
			float* temp = new float[num_outputs];

			const float* input_end = inputs + num_inputs;
			float* temp_end = temp + num_outputs;

			float* matrix_ptr = matrix;
			for(float* temp_ptr = temp; temp_ptr != temp_end; ++temp_ptr)
			{
				float output = 0.0f;

				const float* input_ptr = inputs;
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
			const float* input_end = inputs + num_inputs;
			float* output_end = outputs + num_outputs;

			float* matrix_ptr = matrix;
			for(float* output_ptr = outputs; output_ptr != output_end; ++output_ptr)
			{
				float output = 0.0f;

				const float* input_ptr = inputs;
				while(input_ptr != input_end)
					output += *(matrix_ptr++) * *(input_ptr++);

				*output_ptr = output;
			}
		}
	}

	void NeuralNet::SigmoidOutputs(float* outputs)
	{
		float* end = outputs + num_outputs;
		for(float* ptr = outputs; ptr != end; ++ptr)
			*ptr = tanhf(*ptr);
	}




	/*
	 * MultiLayerPerceptron methods
	 */
	MultiLayerPerceptron::MultiLayerPerceptron() : neural_nets() { }
	MultiLayerPerceptron::MultiLayerPerceptron(unsigned int num_layers, unsigned int* layer_sizes) : neural_nets()
	{
		for(unsigned int i = 1; i < num_layers; ++i)
			neural_nets.push_back(new NeuralNet(layer_sizes[i - 1], layer_sizes[i]));
	}

	MultiLayerPerceptron::MultiLayerPerceptron(const MultiLayerPerceptron& other) : neural_nets()
	{
		for(vector<NeuralNet*>::const_iterator iter = other.neural_nets.begin(); iter != other.neural_nets.end(); ++iter)
			neural_nets.push_back(new NeuralNet(**iter));
	}

	MultiLayerPerceptron::~MultiLayerPerceptron()
	{
		for(vector<NeuralNet*>::iterator iter = neural_nets.begin(); iter != neural_nets.end(); ++iter)
			delete *iter;
		neural_nets.clear();
	}

	void MultiLayerPerceptron::operator =(const MultiLayerPerceptron& other)
	{
		if(&other != this)
		{
			for(vector<NeuralNet*>::iterator iter = neural_nets.begin(); iter != neural_nets.end(); ++iter)
				delete *iter;

			neural_nets.clear();
			for(vector<NeuralNet*>::const_iterator iter = other.neural_nets.begin(); iter != other.neural_nets.end(); ++iter)
				neural_nets.push_back(new NeuralNet(**iter));
		}
	}

	void MultiLayerPerceptron::Process(const float* inputs, float* outputs)
	{
		if(unsigned int nets = neural_nets.size())
		{
			if(nets == 1)
			{
				neural_nets[0]->Multiply(inputs, outputs);
				neural_nets[0]->SigmoidOutputs(outputs);
			}
			else
			{
				unsigned int biggest = neural_nets[0]->num_inputs;
				for(unsigned int i = 1; i < nets; ++i)
					biggest = max(biggest, neural_nets[i]->num_inputs);

				float* temp = new float[biggest];

				neural_nets[0]->Multiply(inputs, temp);
				neural_nets[0]->SigmoidOutputs(temp);

				unsigned int nm1 = nets - 1;

				for(unsigned int i = 1; i < nm1; ++i)
				{
					neural_nets[i]->Multiply(temp, temp);
					neural_nets[i]->SigmoidOutputs(temp);
				}

				neural_nets[nm1]->Multiply(temp, outputs);
				neural_nets[nm1]->SigmoidOutputs(outputs);

				delete[] temp;
			}
		}
	}

	void MultiLayerPerceptron::Train(const float* inputs, float* correct_outputs, float learning_rate)
	{
		if(unsigned int nets = neural_nets.size())
		{
			unsigned int row_size = neural_nets[0]->num_inputs;				// size of the biggest row of values
			for(unsigned int i = 0; i < nets; ++i)
				row_size = max(row_size, neural_nets[i]->num_outputs);

			float* computed_values = new float[(nets + 1) * row_size];

			// first row of computed values = inputs 
			for(unsigned int i = 0; i < neural_nets[0]->num_inputs; ++i)
				computed_values[i] = inputs[i];

			// process each layer, outputting results to levels of outputs array
			{			// curly braces just for scope
				float* my_inputs = computed_values + row_size;
				for(unsigned int i = 0; i < nets; ++i)
				{
					float* my_outputs = my_inputs + row_size;

					neural_nets[i]->Multiply(my_inputs, my_outputs);
					neural_nets[i]->SigmoidOutputs(my_outputs);						// not sure about this!

					my_inputs = my_outputs;
				}
			}

			// backpropagation time!
			vector<NeuralNet*> nu_networks;

			for(unsigned int layer = nets - 1; layer >= 0; --layer)
			{
				float* my_inputs = computed_values + row_size * layer;
				float* my_outputs = my_inputs + row_size;

				NeuralNet* net = neural_nets[layer];
				NeuralNet* nu_net = new NeuralNet(*net);

				float* input_ptr = my_inputs;
				float* mat_ptr = net->matrix;
				float* numat_ptr = nu_net->matrix;
				for(unsigned int i = 0; i < net->num_outputs; ++i)
					for(unsigned int j = 0; j < net->num_inputs; ++j, ++numat_ptr, ++mat_ptr, ++input_ptr)
					{
						float weight = *mat_ptr;

						// TODO: compute partial derivative of error wrt weighted sum of inputs (it's not supposed to be zero)
						float partial_derivative = 0.0f;

						*numat_ptr -= learning_rate * partial_derivative * *input_ptr;
					}

				nu_networks.push_back(nu_net);				// yes it's in reverse order; we'll take care of that after this loop finishes

				// TODO: recompute outputs between here and final output layer? (if it affects the partial derivatives)
			}
		
			for(vector<NeuralNet*>::iterator iter = neural_nets.begin(); iter != neural_nets.end(); ++iter)
				delete *iter;
			neural_nets.assign(nu_networks.rbegin(), nu_networks.rend());

			delete[] computed_values;
		}
	}
}
