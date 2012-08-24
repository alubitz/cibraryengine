#include "StdAfx.h"
#include "NeuralNet.h"

#include "DebugLog.h"
#include "Random3D.h"

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

	void NeuralNet::Randomize()
	{
		for(float *ptr = matrix, *end = matrix + num_inputs * num_outputs; ptr != end; ++ptr)
			*ptr = Random3D::Rand(-2, 2);
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

	void MultiLayerPerceptron::Train(const float* inputs, const float* correct_outputs, float learning_rate)
	{
		if(unsigned int nets = neural_nets.size())
		{
			unsigned int nm1 = nets - 1;

			unsigned int row_size = neural_nets[0]->num_inputs;				// size of the biggest row of values
			for(unsigned int i = 0; i < nets; ++i)
				row_size = max(row_size, neural_nets[i]->num_outputs);

			float* weighted_sums = new float[nets * row_size];
			float* computed_values = new float[(nets + 1) * row_size];
			float* derrordsum = new float[nets * row_size];

			// first row of computed values = inputs 
			for(unsigned int i = 0; i < neural_nets[0]->num_inputs; ++i)
				computed_values[i] = inputs[i];

			// process each layer, outputting results to levels of outputs array
			{			// curly braces just for scope
				float* my_inputs = computed_values;
				float* my_sums = weighted_sums;
				for(unsigned int i = 0; i < nets; ++i)
				{
					NeuralNet* net = neural_nets[i];

					float* my_outputs = my_inputs + row_size;

					net->Multiply(my_inputs, my_sums);

					// copy from my_sums to my_outputs
					for(float	*from_ptr = my_sums,						
								*from_end = my_sums + net->num_outputs,
								*to_ptr = my_outputs;					from_ptr != from_end;	++from_ptr, ++to_ptr)
					{
						*to_ptr = *from_ptr;
					}
					net->SigmoidOutputs(my_outputs);

					my_inputs = my_outputs;
					my_sums += row_size;
				}
			}

			// backpropagation time!
			vector<NeuralNet*> nu_nets(nets);
			for(int layer = nm1; layer >= 0; --layer)
			{
				NeuralNet* net = neural_nets[layer];
				NeuralNet* nu_net = nu_nets[layer] = new NeuralNet(*net);

				NeuralNet* net_below = layer == nm1 ? NULL : neural_nets[layer + 1];
				float* numat_ptr = nu_net->matrix;
				unsigned int layer_rows = row_size * layer;

				for(unsigned int i = 0; i < net->num_outputs; ++i)
				{
					for(float	*input_ptr = computed_values + layer_rows,
								*input_end = input_ptr + net->num_inputs,
								*sum_ptr = weighted_sums + layer_rows,
								*deds_ptr = derrordsum + layer_rows;		input_ptr != input_end;		++numat_ptr, ++input_ptr, ++sum_ptr, ++deds_ptr)
					{
						float sum = *sum_ptr;

						float actf_val = tanhf(sum);
						float dactf = 1.0f - actf_val * actf_val;

						float other_term;			// the term in the partial derivative formula which isn't the derivative of the activation function
						if(layer == nm1)
							other_term = computed_values[nets * row_size + i] - correct_outputs[i];
						else
						{
							other_term = 0;			// start a running total
							unsigned below_inputs = net_below->num_inputs, below_outputs = net_below->num_outputs;
							for(float	*ch_deds = derrordsum + layer_rows + row_size,
										*ch_deds_end = ch_deds + below_outputs,
										*bmat_ptr = net_below->matrix + i;				ch_deds != ch_deds_end;		++ch_deds, bmat_ptr += below_inputs)
							{
								other_term += (*ch_deds) * (*bmat_ptr);
							}
						}
						*deds_ptr = dactf * other_term;

						*numat_ptr -= learning_rate * (*deds_ptr) * (*input_ptr);
					}
				}
			}

			for(vector<NeuralNet*>::iterator iter = neural_nets.begin(); iter != neural_nets.end(); ++iter)
				delete *iter;
			neural_nets.assign(nu_nets.begin(), nu_nets.end());

			delete[] computed_values;
			delete[] weighted_sums;
			delete[] derrordsum;
		}
	}
}
