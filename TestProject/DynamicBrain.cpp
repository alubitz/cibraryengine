#include "StdAfx.h"
#include "DynamicBrain.h"

namespace Test
{
	/*
	 * DynamicBrain usage example
	 */

	struct DBDemoSimulation
	{
		float GetInputValue (unsigned int index)    { return 0.0f; }		// TODO: implement this
		float GetGoalScore  (unsigned int category) { return 0.0f; }		// TODO: implement this
		void  SetOutputValue(unsigned int index, float value) { }			// TODO: implement this
	};

	void UsageExample()
	{
		static const unsigned int num_input_neurons    = 3;
		static const unsigned int num_output_neurons   = 2;
		static const unsigned int num_misc_neurons     = 5;
		static const unsigned int total_neurons        = num_input_neurons + num_output_neurons + num_misc_neurons;

		static const unsigned int max_synapses         = total_neurons * (total_neurons - num_input_neurons);

		static const unsigned int num_score_categories = 3;

		DynamicBrain brain(total_neurons, max_synapses, num_score_categories);

		DBDemoSimulation sim;
		for(unsigned int i = 0; i < 10; ++i)
		{
			for(unsigned int j = 0; j < num_input_neurons; ++j)
				brain.neurons[j].value = sim.GetInputValue(j);

			float goal_scores[num_score_categories];
			for(unsigned int j = 0; j < num_score_categories; ++j)
				goal_scores[j] = sim.GetGoalScore(j);

			brain.Update(goal_scores);

			for(unsigned int j = 0; j < num_output_neurons; ++j)
				sim.SetOutputValue(j, brain.neurons[j + num_input_neurons].value);
		}
	}
}
