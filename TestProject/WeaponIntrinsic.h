#pragma once

#include "Weapon.h"

namespace Test
{
	class WeaponIntrinsic : public Weapon
	{
		public:

			Dood* owner;

			WeaponIntrinsic(TestGame* game_state, Dood* owner);
	};
}
