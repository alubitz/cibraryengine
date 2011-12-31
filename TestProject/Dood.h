#pragma once

#include "StdAfx.h"

#include "DSNMaterial.h"
#include "Shootable.h"
#include "Damage.h"

#include "Team.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class PoseAimingGun;
	class WeaponEquip;
	class WeaponIntrinsic;

	struct Damage;

	class Dood : public Pawn, public Shootable
	{
		private:

			float character_pose_time;

		protected:
			void InnerDispose();

			void DoPitchAndYawControls(float timestep);

		public:

			Team team;

			vector<Material*> materials;

			Vec3 pos;
			Vec3 vel;
			float yaw, pitch;
			Vec3 angular_vel;

			float hp;

			Bone* eye_bone;
			Bone* gun_hand_bone;

			UberModel* model;
			SkinnedCharacter* character;

			PoseAimingGun* p_adp;
			IKPose* ik_pose;

			RigidBodyInfo* rigid_body;
			PhysicsWorld* physics;

			float mass;
			Mat3 inverse_moi;
			float standing;
			float jump_start_timer;
			float jump_fuel;

			float ai_update_interval;
			float ai_timer;

			WeaponEquip* equipped_weapon;
			WeaponIntrinsic* intrinsic_weapon;

			Dood(GameState* gs, UberModel* model_, Vec3 pos, Team& team);

			Vec3 GetPosition();
			void SetPosition(Vec3 pos);

			void Vis(SceneRenderer* renderer);
			void VisCleanup();
			Mat4 GetViewMatrix();

			void PoseCharacter();
			void PoseCharacter(TimingInfo time);

			void Update(TimingInfo time);

			void Spawned();
			void DeSpawned();

			bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum);
			void TakeDamage(Damage damage, Vec3 from_dir);
			void Splatter(Shot* shot, Vec3 poi, Vec3 momentum);
			void Die(Damage cause);

			bool GetAmmoFraction(float& result);
			bool GetAmmoCount(int& result);

			struct AmmoFailureEvent : public Event
			{
				Dood* dood;
				WeaponEquip* weapon;
				AmmoFailureEvent(Dood* dood, WeaponEquip* weapon) : dood(dood), weapon(weapon) { }
			};
			EventDispatcher OnAmmoFailure;

			struct DamageTakenEvent : public Event
			{
				Dood* dood;
				Vec3 from_dir;
				Damage damage;
				bool cancel;

				DamageTakenEvent(Dood* dood, Vec3 from_dir, Damage damage) : dood(dood), from_dir(from_dir), damage(damage), cancel(false) { }
			};
			EventDispatcher OnDamageTaken;

			struct JumpFailureEvent : public Event
			{
				Dood* dood;
				JumpFailureEvent(Dood* dood) : dood(dood) { }
			};
			EventDispatcher OnJumpFailure;

			struct DeathEvent : public Event
			{
				Dood* dood;
				Damage cause;
				DeathEvent(Dood* dood, Damage cause) : dood(dood), cause(cause) { }
			};
			EventDispatcher OnDeath;

			struct MyContactResultCallback : public btCollisionWorld::ContactResultCallback
			{
				Dood* dood;

				MyContactResultCallback(Dood* dood);
				btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObject* colObj0, int partId0, int index0, const btCollisionObject* colObj1, int partId1, int index1);
			};

			MyContactResultCallback* contact_callback;
	};

	void PushDoodHandle(lua_State* L, Dood* dood);
}
