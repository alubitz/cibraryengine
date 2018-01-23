#include "StdAfx.h"
#include "MultiLayerBrain.h"

namespace Test
{
	/*
	 * MultiLayerBrain methods
	 */
	MultiLayerBrain::MultiLayerBrain(const vector<unsigned int>& layer_sizes) : layer_sizes(layer_sizes), matrices()
	{
		for(unsigned int i = 1; i < layer_sizes.size(); ++i)
			matrices.push_back(vector<float>(layer_sizes[i] * layer_sizes[i - 1]));
	}

	MultiLayerBrain::~MultiLayerBrain()
	{
		for(unsigned int i = 0; i < matrices.size(); ++i)
			matrices[i].clear();
		matrices.clear();
		layer_sizes.clear();
	}

	void MultiLayerBrain::SetZero()
	{
		for(unsigned int i = 0; i < matrices.size(); ++i)
			matrices[i] = vector<float>(matrices[i].size());
	}

	void MultiLayerBrain::Evaluate(const vector<float>& inputs, vector<float>& outputs, vector<float>& scratch) const
	{
		int num_outputs = layer_sizes[layer_sizes.size() - 1];

		unsigned int needed_size = 0;
		for(unsigned int i = 0; i < layer_sizes.size(); ++i)
			needed_size += layer_sizes[i];

		scratch.assign(needed_size, 0);

		memcpy(scratch.data(), inputs.data(), sizeof(float) * min(inputs.size(), layer_sizes[0]));

		const float* ibegin = scratch.data();
		float* optr = scratch.data() + layer_sizes[0];

		for(unsigned int i = 0; i < matrices.size(); ++i)
		{
			const float* iend = ibegin + layer_sizes[i];
			const float* mptr = matrices[i].data();
			for(float* oend = optr + layer_sizes[i + 1]; optr != oend; ++optr)
			{
				float tot = 0.0f;
				for(const float* iptr = ibegin; iptr != iend; ++mptr, ++iptr)
					tot += *mptr * *iptr;
				*optr = tanhf(tot);
			}

			assert(mptr == matrices[i].data() + matrices[i].size());

			ibegin = iend;
		}

		ibegin += num_outputs;

		assert(scratch.data() + scratch.size() == ibegin);
		assert(ibegin == optr);

		outputs.resize(num_outputs);
		memcpy(outputs.data(), scratch.data() + (scratch.size() - num_outputs), num_outputs * sizeof(float));
	}

	float MultiLayerBrain::AddGradient(unsigned int num_target_outputs, const vector<float>& inputs, vector<float>& outputs, vector<float>& derrdin, vector<float>& scratch, const vector<float>& derrdout, MultiLayerBrain& gradient) const
	{
		unsigned int num_outputs = layer_sizes[layer_sizes.size() - 1];

		assert(layer_sizes.size() == gradient.layer_sizes.size());
		assert(derrdout.size() == num_outputs);
		assert(num_target_outputs <= num_outputs);

		unsigned int needed_size = 0;
		for(unsigned int i = 0; i < layer_sizes.size(); ++i)
		{
			assert(layer_sizes[i] == gradient.layer_sizes[i]);
			needed_size += layer_sizes[i];
		}

		scratch.assign(needed_size * 2, 0);				// 2 values per node:	value; d(error^2)/d(value)

		memcpy(scratch.data(), inputs.data(), sizeof(float) * min(inputs.size(), layer_sizes[0]));

		float* values = scratch.data();
		float* derrorsq = values + needed_size;
		float* scratch_end = values + needed_size * 2;

		const float* ibegin = scratch.data();
		float* vptr = values + layer_sizes[0];

		// evaluate
		for(unsigned int i = 0; i < matrices.size(); ++i)
		{
			const float* iend = ibegin + layer_sizes[i];
			const float* mptr = matrices[i].data();
			for(float* vend = vptr + layer_sizes[i + 1]; vptr != vend; ++vptr)
			{
				float tot = 0.0f;
				for(const float* iptr = ibegin; iptr != iend; ++mptr, ++iptr)
					tot += *mptr * *iptr;
				*vptr = tanhf(tot);
			}

			assert(mptr == matrices[i].data() + matrices[i].size());

			ibegin = iend;
		}

		outputs.resize(num_outputs);
		memcpy(outputs.data(), values + (needed_size - num_outputs), num_outputs * sizeof(float));

		// backprop
		float errortot = 0.0f;

		for(unsigned int i = matrices.size() - 1; i < matrices.size(); --i)		// deliberately inverted for loop (relies on uint overflow)
		{
			unsigned int inputs_begin = 0;
			for(unsigned int j = 0; j < i; ++j)
				inputs_begin += layer_sizes[j];
			unsigned int outputs_begin = inputs_begin + layer_sizes[i];
			unsigned int outputs_end   = outputs_begin + layer_sizes[i + 1];

			// last matrix (first one processed in backprop) uses special computation for d(error^2)/d(value)
			if(i + 1 == matrices.size())
			{
				errortot += ComputeDerrorDoutputs(num_target_outputs, values + outputs_begin, derrdout.data(), derrorsq + outputs_begin, scratch_end);
				memcpy(derrorsq + outputs_begin + num_target_outputs, derrdout.data(), (num_outputs - num_target_outputs) * sizeof(float));
			}
			
			// compute gradient for this matrix
			float* dbegin = derrorsq + inputs_begin;
			const float* ibegin = values + inputs_begin;
			const float* iend   = values + outputs_begin;

			const float* mptr = matrices[i].data();
			float* gptr = gradient.matrices[i].data();

			const float* vptr = values + outputs_begin;
			const float* vend = values + outputs_end;

			for(const float* doptr = derrorsq + outputs_begin; vptr != vend; ++vptr, ++doptr)
			{
				float phi_prime = 1.0f - *vptr * *vptr;		// derivative of tanh(x) = 1 - tanh(x)^2
				float shared_mult = *doptr * phi_prime;

				assert(vptr < scratch_end);
				assert(doptr < scratch_end);

				float* diptr = dbegin;
				for(const float *iptr = ibegin; iptr != iend; ++iptr, ++gptr, ++diptr, ++mptr)
				{
					assert(iptr < scratch_end);
					assert(diptr < scratch_end);

					*gptr += shared_mult * *iptr;
					*diptr += shared_mult * *mptr;
				}
			}

			assert(mptr == matrices[i].data() + matrices[i].size());
			assert(gptr == gradient.matrices[i].data() + gradient.matrices[i].size());
		}

		derrdin.resize(layer_sizes[0]);
		memcpy(derrdin.data(), derrorsq, derrdin.size() * sizeof(float));

		return errortot;
	}

	void MultiLayerBrain::Train(const MultiLayerBrain& gradient, float learning_rate)
	{
		assert(layer_sizes.size() == gradient.layer_sizes.size());

		for(unsigned int i = 0; i < matrices.size(); ++i)
		{
			assert(layer_sizes[i] == gradient.layer_sizes[i]);
			assert(matrices[i].size() == gradient.matrices[i].size());

			const float* gptr = gradient.matrices[i].data();
			const float* gend = gptr + matrices[i].size();
			for(float *mptr = matrices[i].data(); gptr != gend; ++mptr, ++gptr)
				*mptr -= learning_rate * *gptr;
		}
	}

	void MultiLayerBrain::Randomize(float scale)
	{
		float range = scale * 2.0f;
		for(unsigned int i = 0; i < matrices.size(); ++i)
		{
			vector<float>& matrix = matrices[i];
			for(unsigned int j = 0; j < matrix.size(); ++j)
				matrix[j] += Random3D::Rand() * range - scale;
		}
	}

	void MultiLayerBrain::RandomizeOne(float scale)
	{
		unsigned int tot = 0;
		for(unsigned int i = 0; i < matrices.size(); ++i)
			tot += matrices[i].size();

		unsigned int r = (unsigned)Random3D::RandInt() % tot;
		for(unsigned int i = 0; i < matrices.size(); ++i)
			if(r < matrices[i].size())
			{
				matrices[i][r] += Random3D::Rand(-scale, scale);
				return;
			}
			else
				r -= matrices[i].size();
	}

	void MultiLayerBrain::GetCoeffTotals(float& tot, float& sqtot) const
	{
		tot = sqtot = 0.0f;
		for(unsigned int i = 0; i < matrices.size(); ++i)
		{
			const vector<float>& matrix = matrices[i];
			for(unsigned int j = 0; j < matrix.size(); ++j)
			{
				float v = matrix[j];
				tot += fabs(v);
				sqtot += v * v;
			}
		}
	}


	float MultiLayerBrain::ComputeDerrorDoutputs(unsigned int count, const float* outputs, const float* correct_outputs, float* derrdout, const float* scratch_end) const
	{
		float errortot = 0.0f;

		const float* optr = outputs;
		const float* oend = optr + count;

		const float* cptr = correct_outputs;
		for(float *dptr = derrdout; optr != oend; ++dptr, ++cptr, ++optr)
		{
			assert(dptr < scratch_end);

			float error = *optr - *cptr;
			*dptr = 2.0f * error;

			errortot += error * error;
		}

		return errortot;
	}
}
