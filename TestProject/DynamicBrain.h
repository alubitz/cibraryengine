#pragma once

#include "StdAfx.h"

namespace Test
{
	struct DynamicBrain
	{
		struct Neuron
		{
			float value;
			float tot;

			// TODO: add stuff here

			Neuron() : value(0.0f), tot(0.0f) { }

			void Update() { value = tanhf(tot) * 0.5f + 0.5f; tot = 0.0f; }
		};

		struct Synapse
		{
			Neuron *from, *to;

			float base_coeff;
			float fatigue;

			// TODO: add stuff here

			Synapse() : from(NULL), to(NULL), base_coeff(0), fatigue(0) { }

			void IncrementNeuron() const { to->tot += from->value * GetUseCoeff(); }
			float GetUseCoeff() const { return base_coeff * (1.0f - fatigue); }				// TODO: do this better

			void Update(const float* scores) { }											// TODO: implement this
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
				nptr->Update();

			for(Synapse *sptr = synapses, *send = sptr + num_synapses; sptr != send; ++sptr)
				sptr->Update(goal_scores);
		}
	};
}
