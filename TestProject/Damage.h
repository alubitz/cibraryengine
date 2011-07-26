#pragma once

#include "../CibraryEngine/CibraryEngine.h"

namespace Test
{
	using namespace CibraryEngine;

	struct Damage
	{
		void* causer;							// was IDamageBlame in the C# version
		float amount;

		Damage();
		Damage(void* causer, float amount);
	};
}