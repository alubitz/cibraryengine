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

			static const unsigned int NumScoringCategories = 6;

			static void Load();
			static void Save();

			static void NextBrain(unsigned int num_inputs, unsigned int num_outputs, unsigned int num_memories);
			static void Process(vector<float>& inputs, vector<float>& outputs);											// note: all params may be modified
			static void Finish(const float* scores);
			static bool IsFinished();

			static string GetDebugText();

			static Texture2D* GetDebugImage();
	};
}
