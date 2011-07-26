#include "WeaponIntrinsic.h"
#include "TestGame.h"

namespace Test
{
	WeaponIntrinsic::WeaponIntrinsic(TestGame* game_state, Dood* owner) :
		Weapon(game_state),
		owner(owner)
	{
		sound_owner = owner;
	}
}
