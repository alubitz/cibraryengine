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

			float fatigue, ftot;

			// TODO: add stuff here

			Neuron() : value(0.0f), tot(0.0f), fatigue(0.0f), ftot(0.0f) { }

			void Update(const float* goal_scores)
			{
				value = (tanhf(tot) * 0.5f + 0.5f) * (1.0f - fatigue);
				tot = 0.0f;

				// TODO: do this better
				fatigue = max(0.0f, min(1.0f, fatigue * 0.9f + ftot * 0.1f));
				ftot = 0.0f;
			}
		};

		struct Synapse
		{
			Neuron *from, *to;

			float coeff;

			// TODO: add stuff here

			Synapse() : from(NULL), to(NULL), coeff(0.0f) { }

			void IncrementNeuron() const
			{
				float delta = from->value * coeff;
				from->ftot += delta;
				to->tot    += delta;
			}

			void Update(const float* scores) { }							// TODO: implement this
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
