#pragma once

#include "StdAfx.h"

#include "Shootable.h"
#include "Damage.h"

#include "Team.h"

#include "../CibraryEngine/IKChain.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class WeaponEquip;
	class WeaponIntrinsic;

	struct Damage;

	class DoodOrientationConstraint;

	class Dood : public Pawn
	{
		private:

			float character_pose_time;

		protected:

			struct BoneShootable : Entity, Shootable
			{
				Dood* dood;
				RigidBody* rbi;

				BillboardMaterial* blood_material;

				BoneShootable(GameState* gs, Dood* dood, RigidBody* rbi, BillboardMaterial* blood_material) : Entity(gs), dood(dood), rbi(rbi), blood_material(blood_material) { }
				~BoneShootable() { }

				bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum);
			};

			virtual void InnerDispose();

			void DoPitchAndYawControls(TimingInfo time);
			virtual void DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward);
			virtual void DoMovementControls(TimingInfo time, Vec3 forward, Vec3 rightward);
			virtual void DoWeaponControls(TimingInfo time);
			virtual void PreUpdatePoses(TimingInfo time);
			virtual void PostUpdatePoses(TimingInfo time);

			void UpdateIKChain(IKChain* chain);

		public:

			Team team;

			vector<Material*> materials;
			BillboardMaterial* blood_material;

			Vec3 pos;
			Vec3 vel;
			float yaw, pitch;

			float jump_start_timer;

			float hp;

			Bone* eye_bone;

			UberModel* model;
			SkinnedCharacter* character;
			PosedCharacter* posey;

			Mat4 whole_xform;
			Vec3 origin;

			RigidBody* root_rigid_body;
			vector<RigidBody*> rigid_bodies;
			vector<BoneShootable*> shootables;
			vector<Bone*> rbody_to_posey;
			vector<RigidBody*> bone_to_rbody;
			vector<PhysicsConstraint*> constraints;

			DoodOrientationConstraint* orientation_constraint;

			PhysicsWorld* physics;
			ModelPhysics* mphys;

			float standing;

			WeaponEquip* equipped_weapon;
			WeaponIntrinsic* intrinsic_weapon;

			Dood(GameState* gs, UberModel* model_, ModelPhysics* mphys, Vec3 pos, Team& team);

			Vec3 GetPosition();
			void SetPosition(Vec3 pos);

			void Vis(SceneRenderer* renderer);
			Mat4 GetViewMatrix();

			SoundSource* PlayDoodSound(SoundBuffer* buffer, float vol, bool looping);

			void PoseCharacter();
			void PoseCharacter(TimingInfo time);

			virtual void Update(TimingInfo time);

			virtual void Spawned();
			virtual void DeSpawned();

			void TakeDamage(Damage damage, Vec3 from_dir);
			void Splatter(Shot* shot, Vec3 poi, Vec3 momentum);
			void Die(Damage cause);

			bool GetAmmoFraction(float& result);
			bool GetAmmoCount(int& result);

			struct ContactCallback : public CollisionCallback
			{
				Dood* dood;
				ContactCallback(Dood* dood);
				bool OnCollision(const ContactPoint& collision);
			} collision_callback;

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
	};

	void PushDoodHandle(lua_State* L, Dood* dood);
}
