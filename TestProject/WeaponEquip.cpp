#include "StdAfx.h"
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

	void WeaponEquip::Equip(Dood* new_owner)
	{
		sound_owner = owner = new_owner;
		owner->equipped_weapon = this;
	}

	void WeaponEquip::UnEquip(Dood* old_owner)
	{
		if(owner != old_owner)
			DEBUG();

		owner->equipped_weapon = NULL;
		sound_owner = owner = NULL;
	}
}
