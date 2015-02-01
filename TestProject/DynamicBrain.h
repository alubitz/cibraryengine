#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	struct DynamicBrain
	{
		struct Neuron
		{
			float value;
			float tot;

			// TODO: add stuff here

			Neuron() : value(0.0f), tot(0.0f) { }

			void Update(const float* goal_scores)
			{
				value = tanhf(tot) * 0.5f + 0.5f;
				
				tot = 0.0f;
			}
		};

		struct Synapse
		{
			Neuron *from, *to;

			float coeff;

			float oldscore;
			float recent_delta;

			unsigned int scorecat;

			// TODO: add stuff here

			Synapse() : from(NULL), to(NULL), coeff(0.0f), oldscore(0.0f), recent_delta(0.0f), scorecat(0) { }

			void IncrementNeuron() const { to->tot += from->value * coeff; }

			void Update(const float* scores)
			{
				static const float recent_delta_keep = 0.95f;
				static const float oldscore_keep     = 0.75f;
				static const float oldscore_update   = 1.0f - oldscore_keep;

				static const float random_scale      = 7.5f;
				static const float random_exponent   = 6.0f;
				static const float reinforce         = 1000.0f;
				static const float delta_scale       = 0.02f;

				float score = scores[scorecat];
				oldscore = oldscore * oldscore_keep + score * oldscore_update;

				float dscore = score - oldscore;
				float delta = delta_scale * tanhf(-reinforce * recent_delta * dscore + Random3D::Rand(-random_scale, random_scale) * pow(Random3D::Rand(), random_exponent));
				coeff += delta;

				recent_delta = recent_delta * recent_delta_keep + delta;
			}
		};

		unsigned int num_neurons, num_synapses, num_goals;

		Neuron* neurons;
		Synapse* synapses;

		DynamicBrain() : num_neurons(0), num_synapses(0), num_goals(0), neurons(NULL), synapses(NULL) { }

		DynamicBrain(unsigned int num_neurons, unsigned int num_synapses, unsigned int num_goals) :
			num_neurons (num_neurons),
			num_synapses(num_synapses),
			num_goals   (num_goals),
			neurons (new Neuron [num_neurons]),
			synapses(new Synapse[num_synapses])
		{
		}

		~DynamicBrain() { if(neurons) { delete[] neurons; neurons = NULL; } if(synapses) { delete[] synapses; synapses = NULL; } }

		DynamicBrain(const DynamicBrain& other) :
			num_neurons (other.num_neurons),
			num_synapses(other.num_synapses),
			num_goals   (other.num_goals),
			neurons (new Neuron [num_neurons]),
			synapses(new Synapse[num_synapses])
		{
			CopyArrays(other);
		}

		void operator =(const DynamicBrain& other)
		{
			if(&other != this)
			{
				if(neurons  != NULL && num_neurons  < other.num_neurons ) { delete[] neurons;  neurons  = NULL; }
				if(synapses != NULL && num_synapses < other.num_synapses) { delete[] synapses; synapses = NULL; }

				num_neurons  = other.num_neurons;
				num_synapses = other.num_synapses;
				num_goals    = other.num_goals;

				if(neurons  == NULL) { neurons  = new Neuron [num_neurons]; }
				if(synapses == NULL) { synapses = new Synapse[num_synapses]; }

				CopyArrays(other);
			}
		}

		void CopyArrays(const DynamicBrain& other)			// precondition: destination arrays are sufficient size
		{
			memcpy(neurons,  other.neurons,  num_neurons  * sizeof(Neuron));
			memcpy(synapses, other.synapses, num_synapses * sizeof(Synapse));

			int offset = neurons - other.neurons;
			for(Synapse* sptr = synapses, *send = sptr + num_synapses; sptr != send; ++sptr)
			{
				sptr->from += offset;
				sptr->to   += offset;
			}
		}

		void Update(const float* goal_scores)
		{
			for(Synapse *sptr = synapses, *send = sptr + num_synapses; sptr != send; ++sptr)
				sptr->IncrementNeuron();

			for(Neuron  *nptr = neurons,  *nend = nptr + num_neurons;  nptr != nend; ++nptr)
				nptr->Update(goal_scores);

			for(Synapse *sptr = synapses, *send = sptr + num_synapses; sptr != send; ++sptr)
				sptr->Update(goal_scores);
		}
	};
}
