#pragma once

#include "Weapon.h"
#include "GlowyModelMaterial.h"

namespace Test
{
	// a player-equippable weapon
	class WeaponEquip : public Weapon
	{
		public:

			Dood* owner;

			// these are set by the owner!
			Mat4 gun_xform;
			Vec3 pos;
			Vec3 vel;

			WeaponEquip(GameState* game_state, Dood* owner);

			virtual bool NeedsReloading();
			virtual void BeginReload();
	};
}
