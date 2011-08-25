#include "StdAfx.h"

#include "Team.h"

namespace Test
{
	/*
	 * Team methods
	 */
	Team::Team(int id) : id(id) { }

	bool Team::Attacks(Team& team)
	{
		if(team.id == 0)
			return false;					// nobody attacks neutral players
		else if(id == -1)
			return true;					// free-for-all players attack everybody
		else if(id != team.id)
			return true;					// normal team players attack other teams
		else
			return false;					// normal team players don't attack their own teammates
	}

	bool operator ==(Team a, Team b) { return a.id == b.id; }
}
