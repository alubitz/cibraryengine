#pragma once

#include "../CibraryEngine/CibraryEngine.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class TestGame;

	unsigned int LoadLevel(TestGame* game, string level_name);
}
