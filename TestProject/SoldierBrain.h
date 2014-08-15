#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class SoldierBrain
	{
		private:

			struct Imp;
			static Imp* imp;

		public:

			static void Load();
			static void Save();

			static void NextBrain(unsigned int num_inputs, unsigned int num_outputs, unsigned int num_memories, float& max_score);	// max_score = -1 means no max
			static void Process(vector<float>& inputs, vector<float>& outputs);											// note: all params may be modified
			static void Finish(float score);
			static bool IsFinished();

			static string GetDebugText();

			static Texture2D* GetDebugImage();
	};
}
