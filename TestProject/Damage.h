#pragma once

#include "StdAfx.h"

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