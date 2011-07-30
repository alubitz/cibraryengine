#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class DSNMaterial;
	struct DSNMaterialNodeData;
	class GlowyModelMaterial;
	struct GlowyModelMaterialNodeData;
	class Dood;
	class TestGame;
	class Shot;

	class Weapon : public Entity
	{
		public: static const unsigned int MaxFireModes = 16;

		private:

			bool fire_sustained[MaxFireModes];
			bool fire_once[MaxFireModes];

		public:

			struct Inaccuracy
			{
				float amount;
				float time;

				Inaccuracy(float amount, float time) : amount(amount), time(time) { }
			};
			list<Inaccuracy> inaccuracy;

			Dood* sound_owner;
			Vec3 sound_pos;
			Vec3 sound_vel;

			Weapon(GameState* game_state);

			virtual void OwnerUpdate(TimingInfo time);

			bool IsFiring(int mode);
			bool IsFiring(int mode, bool sustained);
			void SetFiring(int mode, bool sustained, bool value);
			void FireOnce(int mode);				// call this when your weapon fires in response to a fire-once input

			virtual void Vis(SceneRenderer* renderer);
			virtual void VisCleanup();

			virtual void PlayWeaponSound(SoundBuffer* buffer, float vol, bool looping);

			virtual bool GetAmmoFraction(float& result);

			static BoolControlChannel PrimaryFire;
			static BoolControlChannel AltFire;
			static BoolControlChannel Reload;
	};
}
