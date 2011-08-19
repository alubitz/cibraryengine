#include "StdAfx.h"
#include "Weapon.h"

#include "DSNMaterial.h"
#include "GlowyModelMaterial.h"
#include "Dood.h"
#include "TestGame.h"
#include "Shot.h"

namespace Test
{
	/*
	 * Weapon methods
	 */
	Weapon::Weapon(GameState* game) :
		Entity(game),
		fire_sustained(),
		fire_once(),
		inaccuracy(),
		sound_owner(NULL)
	{
	}

	void Weapon::OwnerUpdate(TimingInfo time) { }

	bool Weapon::IsFiring(int mode) { return fire_once[mode] || fire_sustained[mode]; }
	bool Weapon::IsFiring(int mode, bool sustained) { return sustained ? fire_sustained[mode] : fire_once[mode]; }
	void Weapon::SetFiring(int mode, bool sustained, bool value) { (sustained ? fire_sustained[mode] : fire_once[mode]) = value; }
	void Weapon::FireOnce(int mode) { fire_once[mode] = false; }

	void Weapon::Vis(SceneRenderer* renderer) { }
	void Weapon::VisCleanup() { }

	void Weapon::PlayWeaponSound(SoundBuffer* buffer, float vol, bool looping)
	{
		if(buffer != NULL)
		{
			if(sound_owner != NULL && sound_owner == ((TestGame*)game_state)->player_pawn)
				game_state->sound_system->PlayEffect(buffer, vol, looping);
			else
				game_state->sound_system->PlayEffect(buffer, sound_pos, sound_vel, vol, looping);
		}
	}

	bool Weapon::GetAmmoFraction(float& result) { return false; }
	bool Weapon::GetAmmoCount(int& result) { return false; }




	/*
	 * Weapon control channels
	 */
	BoolControlChannel Weapon::PrimaryFire = BoolControlChannel("PFire", false);
	BoolControlChannel Weapon::AltFire = BoolControlChannel("AFire", false);
	BoolControlChannel Weapon::Reload = BoolControlChannel("Reload", false);
}
