#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class Dood;

	class AIController : public Controller
	{
		public:

			AIController(GameState* gs);

			virtual void Update(TimingInfo time);
			virtual bool CanSee(Dood* target);
	};
}
