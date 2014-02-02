#pragma once

#include "WeaponIntrinsic.h"

namespace Test
{
	class CrabWeapon : public WeaponIntrinsic
	{
		public:

			float attack_ready_time;
			float attack_interval;

			CrabWeapon(TestGame* game_state, Dood* owner);

			void OwnerUpdate(const TimingInfo& time);

			void ClawAttackNormal(float now);
	};
}
