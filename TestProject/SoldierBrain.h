#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;

	class SoldierBrain
	{
		private:

			struct Genome
			{
				vector<float> brain;
				float score;

				Genome() : brain(), score(-1) { }
				Genome(unsigned int size) : brain(size), score(-1) { }
			};

			static vector<Genome> genomes;
			static unsigned int batch, active_genome, trial;
			static unsigned int num_inputs, num_outputs, num_memories, brain_size;

			static vector<float> memory;

			static bool finished;

			static string debug_text;

		public:

			static void Load();
			static void Save();

			static void NextBrain(unsigned int num_inputs, unsigned int num_outputs, unsigned int num_memories, float& max_lifetime);	// max_lifetime = -1 means no max
			static void Process(vector<float>& inputs, vector<float>& outputs);											// note: all params may be modified
			static void Finish(float score);
			static bool IsFinished();

			static string GetDebugText();
	};
}
