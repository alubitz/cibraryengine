#pragma once

#include "StdAfx.h"

namespace Test
{
	struct Team
	{
		int id;

		Team(int id);

		bool Attacks(Team& team);
	};

	bool operator ==(Team a, Team b);
}
