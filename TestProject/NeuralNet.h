#pragma once

#include "StdAfx.h"

namespace Test
{
	class NeuralNet
	{
		private:

			NeuralNet(unsigned int num_inputs, unsigned int num_outputs, unsigned int num_middles, float* data_ptr, unsigned int last_size);
			~NeuralNet();

			static float* NextSection(unsigned int sz, unsigned int& last, float*& ptr) { ptr += last; last = sz; return ptr; }

			void Multiply(const float* matrix, const float* step_inputs, float* step_outputs, unsigned int num_in, unsigned int num_out)
			{
				const float* in_end  = step_inputs  + num_in;
				const float* out_end = step_outputs + num_out;
				const float* mat_ptr = matrix;

				float value;

				for(float* out_ptr = step_outputs; out_ptr != out_end; ++out_ptr)
				{
					value = 0.0f;
					for(const float* in_ptr = step_inputs; in_ptr != in_end; ++in_ptr, ++mat_ptr)
						value += *mat_ptr * *in_ptr;
					*out_ptr = value;
				}
			}
			void Sigmoid(const float* inputs, float* outputs, unsigned int count)
			{
				for(const float* in_end = inputs + count; inputs != in_end; ++inputs, ++outputs)
					*outputs = tanh(*inputs);
			}
			void MultiplyAndSigmoid(const float* matrix, const float* step_inputs, float* step_sums, float* step_outputs, unsigned int num_in, unsigned int num_out)
			{
				Multiply(matrix, step_inputs, step_sums, num_in, num_out);
				Sigmoid(step_sums, step_outputs, num_out);
			}

		public:

			unsigned int num_inputs, num_outputs, num_middles;
			unsigned int top_matrix_size, bottom_matrix_size;

			float* data;					// consolidate float arrays all in one place

			float* inputs;

			float* middle_sums;
			float* middles;
			
			float* output_sums;
			float* outputs;
			float* correct_outputs;
			float* errors;

			float* phiprime_mids;
			float* phiprime_outs;

			float* top_matrix;
			float* temp_top;
			float* bottom_matrix;
			float* temp_bottom;

			void Randomize(float scale);

			void Evaluate()
			{
				MultiplyAndSigmoid(top_matrix,    inputs,  middle_sums, middles, num_inputs,  num_middles);
				MultiplyAndSigmoid(bottom_matrix, middles, output_sums, outputs, num_middles, num_outputs);
			}

			float CheckOutput()
			{
				float tot = 0.0f;
				for(float *out_ptr = outputs, *correct_ptr = correct_outputs, *err_ptr = errors, *out_end = out_ptr + num_outputs; out_ptr != out_end; ++out_ptr, ++correct_ptr, ++err_ptr)
				{
					*err_ptr = *out_ptr - *correct_ptr;
					tot += *err_ptr * *err_ptr;
				}
				return tot;
			}

			float EvaluateAndScore() { Evaluate(); return CheckOutput(); }

			float Train(float learning_rate);


			// constructor/destructor-ish
			static NeuralNet* New(unsigned int num_inputs, unsigned int num_outputs, unsigned int num_middles);

			static void Delete(NeuralNet* nn);
	};
}
