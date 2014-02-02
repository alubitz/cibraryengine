#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class TestGame;

	unsigned int LoadLevel(TestGame* game, const string& level_name);
}
