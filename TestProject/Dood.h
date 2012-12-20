#pragma once

#include "StdAfx.h"

#include "Shootable.h"
#include "Damage.h"

#include "Team.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class WeaponEquip;
	class WeaponIntrinsic;

	struct Damage;

	class Dood : public Pawn
	{
		private:

			float character_pose_time;

		protected:

			struct BoneShootable : Entity, Shootable
			{
				Dood* dood;
				RigidBody* body;

				BillboardMaterial* blood_material;

				BoneShootable(GameState* gs, Dood* dood, RigidBody* body, BillboardMaterial* blood_material) : Entity(gs), dood(dood), body(body), blood_material(blood_material) { }
				~BoneShootable() { }

				bool GetShot(Shot* shot, const Vec3& poi, const Vec3& vel, float mass);
			};

			float yaw_rate, pitch_rate;

			virtual void InnerDispose();

			void DoPitchAndYawControls(TimingInfo time);
			virtual void DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward);
			virtual void DoMovementControls(TimingInfo time, Vec3 forward, Vec3 rightward);
			virtual void DoWeaponControls(TimingInfo time);
			virtual void PreUpdatePoses(TimingInfo time);
			virtual void PostUpdatePoses(TimingInfo time);

			RigidBody* RigidBodyForNamedBone(const string& name);

		public:

			Team team;

			vector<Material*> materials;
			BillboardMaterial* blood_material;

			Vec3 pos;
			Vec3 vel;
			float yaw, pitch;

			float jump_start_timer;

			float hp;
			bool alive;

			float ragdoll_timer;

			Bone* eye_bone;

			UberModel* model;
			SkinnedCharacter* character;
			PosedCharacter* posey;

			bool use_cheaty_physics;

			float vis_bs_radius;		// radius of bounding sphere used for frustum culling

			Mat4 whole_xform;
			Vec3 origin;

			RigidBody* root_rigid_body;
			vector<RigidBody*> rigid_bodies;
			vector<BoneShootable*> shootables;
			vector<Bone*> rbody_to_posey;
			vector<RigidBody*> bone_to_rbody;
			vector<PhysicsConstraint*> constraints;

			CollisionGroup* collision_group;

			map<unsigned int, RigidBody*> foot_bones;				// names of bones which should count for "standing"; put pairs with NULL in the constructor and they will be populated in Spawned

			PhysicsWorld* physics;
			ModelPhysics* mphys;

			WeaponEquip* equipped_weapon;
			WeaponIntrinsic* intrinsic_weapon;

			Dood(GameState* gs, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team);

			Vec3 GetPosition();
			void SetPosition(Vec3 pos);

			void Vis(SceneRenderer* renderer);
			Mat4 GetViewMatrix();
			Vec3 GetEyePos();

			SoundSource* PlayDoodSound(SoundBuffer* buffer, float vol, bool looping);

			void PoseCharacter();
			void PoseCharacter(TimingInfo time);

			void PhysicsToCharacter();
			virtual void PoseToPhysics(float timestep);

			virtual void Update(TimingInfo time);

			virtual void Spawned();
			virtual void DeSpawned();

			void TakeDamage(Damage damage, Vec3 from_dir);
			void Splatter(Shot* shot, const Vec3& poi, const Vec3& vel);

			virtual void Die(Damage cause);

			bool GetAmmoFraction(float& result);
			bool GetAmmoCount(int& result);

			struct StandingCallback : public CollisionCallback
			{
				Dood* dood;
				vector<RigidBody*> standing_on;
				float standing;

				void OnCollision(const ContactPoint& collision);

				void Reset();
				void ApplyVelocityChange(const Vec3& dv);

			} standing_callback;

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
