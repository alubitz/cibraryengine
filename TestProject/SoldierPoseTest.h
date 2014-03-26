#pragma once

#include "StdAfx.h"

namespace Test
{
	class Soldier;

	struct SoldierPoseTest
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			unsigned int num_inputs, num_outputs, num_coeffs;

			SoldierPoseTest();			// loads from file
			~SoldierPoseTest();			// saves to file

			void Begin(Soldier* soldier);
			void End(Soldier* soldier);
	};
}
