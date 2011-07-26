#include "WeaponEquip.h"
#include "TestGame.h"
#include "Dood.h"
#include "Shot.h"

namespace Test
{
	/*
	 * WeaponEquip methods
	 */
	WeaponEquip::WeaponEquip(GameState* game_state, Dood* owner) :
		Weapon(game_state),
		owner(owner),
		gun_xform(Mat4::Identity())
	{
		sound_owner = owner;
	}

	bool WeaponEquip::NeedsReloading() { return false; }
	void WeaponEquip::BeginReload() { }
}
