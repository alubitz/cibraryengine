#pragma once

#include "StdAfx.h"

#include "Shootable.h"
#include "Damage.h"

#include "JetpackNozzle.h"

#include "Team.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	class WeaponEquip;
	class WeaponIntrinsic;

	struct Damage;

	struct CBone;
	struct CJoint;

	class Dood : public Pawn
	{
		private:

			float pose_timer;

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

			bool use_cheaty_ori;

			float yaw_rate, pitch_rate;
			Vec3 desired_vel_2d;				// for convenience, this is computed in DoMovementControls and made available for use in ComputeDesiredVelocity

			virtual void InnerDispose();

			void DoPitchAndYawControls(const TimingInfo& time);
			virtual void DoJumpControls(const TimingInfo& time, const Vec3& forward, const Vec3& rightward) { }
			virtual void DoMovementControls(const TimingInfo& time, const Vec3& forward, const Vec3& rightward);
			virtual void DoWeaponControls(const TimingInfo& time);

			// for PhysicsToCharacter
			virtual void PreUpdatePoses(const TimingInfo& time)  { }
			virtual void PostUpdatePoses(const TimingInfo& time) { }

			// for PoseToPhysics
			virtual void DoCheatyPose(float timestep, const Vec3& net_vel);
			virtual void MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi) { }

			virtual void DoInitialPose();			// called by ::Spawned; use this to set orientations of the posey bones

			virtual void InitBoneHelpers()    { }
			virtual void InitJointHelpers()   { }	// also responsible for setting joint torque limits!
			virtual void InitJetpackNozzles() { jetpack_nozzles.clear(); }

		public:

			Team team;

			BillboardMaterial* blood_material;

			Vec3 pos;
			Vec3 vel;
			float yaw, pitch;

			float jump_start_timer;						// TODO: implement improved "jump timer" behavior

			float third_person_frac;
			float third_person_view_dist;

			float hp;
			bool alive;

			float ragdoll_timer;

			Bone* eye_bone;

			UberModel* model;
			SkinnedCharacter* character;
			PosedCharacter* posey;

			float vis_bs_radius;						// radius of bounding sphere used for frustum culling

			RigidBody* root_rigid_body;
			vector<RigidBody*> rigid_bodies;
			vector<BoneShootable*> shootables;
			vector<Bone*> rbody_to_posey;
			vector<RigidBody*> bone_to_rbody;
			vector<PhysicsConstraint*> constraints;

			set<RigidBody*> velocity_change_bodies;		// rigid bodies which should be affected by StandingCallback::ApplyVelocityChange

			CollisionGroup* collision_group;

			PhysicsWorld* physics;
			ModelPhysics* mphys;

			WeaponEquip*     equipped_weapon;
			WeaponIntrinsic* intrinsic_weapon;

			vector<CBone*> all_bones;
			vector<CJoint*> all_joints;
			vector<JetpackNozzle> jetpack_nozzles;

			struct MyContactPoint 
			{
				ContactPoint cp;
				Dood* dood;
				MyContactPoint(const ContactPoint& cp, Dood* dood) : cp(cp), dood(dood) { }
			};
			vector<MyContactPoint> old_contact_points;
			vector<MyContactPoint> new_contact_points;

			Dood(GameState* gs, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team);

			Vec3 GetPosition();
			void SetPosition(const Vec3& pos);

			virtual void Vis(SceneRenderer* renderer);
			Mat4 GetViewMatrix();
			Vec3 GetEyePos();

			SoundSource* PlayDoodSound(SoundBuffer* buffer, float vol, bool looping);

			RigidBody* RigidBodyForNamedBone(const string& name) const;
			
			void PoseCharacter(const TimingInfo& time);

			virtual void PhysicsToCharacter();
			virtual void PoseToPhysics(float timestep);

			virtual void Update(const TimingInfo& time);

			virtual void Spawned();
			virtual void DeSpawned();

			void TakeDamage(const Damage& damage, const Vec3& from_dir);
			void Splatter(Shot* shot, const Vec3& poi, const Vec3& vel);

			virtual void Die(const Damage& cause);

			bool GetAmmoFraction(float& result);
			bool GetAmmoCount(int& result);

			virtual void RegisterFeet() { }					// use this to create (custom?) FootState instances and add them to feet
			CBone* GetBone(const string& name) const;
			CJoint* GetJoint(const string& child_name) const;
			CJoint* GetJointOverrideTorques(const string& child_name, float torque) const { return GetJointOverrideTorques(child_name, torque, torque, torque); }
			CJoint* GetJointOverrideTorques(const string& child_name, float tx, float ty, float tz) const;
			void RegisterSymmetricJetpackNozzles(CBone* lbone, CBone* rbone, const Vec3& lpos, const Vec3& lnorm, float angle, float force);

			virtual bool GetRBScriptingName(RigidBody* rb, string& name) { return false; }
			virtual Vec3 GetDesiredJetpackAccel() { return Vec3(); }
			virtual int GetTickAge() { return 0; }

			void DoScriptedMotorControl(const string& filename);

			virtual void PreCPHFT(float timestep);
			virtual void PostCPHFT(float timestep);

			class FootState : public Disposable
			{
				public:

					unsigned int posey_id;
					Vec3 ee_pos;							// local-coords position of end effector

					Vec3 net_force, net_torque;
					Vec3 temp_net_force, temp_net_torque;

					RigidBody* body;

					vector<ContactPoint> contact_points;
					float no_contact_timer;

					bool standing;

					FootState(unsigned int posey_id, const Vec3& ee_pos) : posey_id(posey_id), ee_pos(ee_pos), body(nullptr), contact_points(), no_contact_timer(0.0f), standing(false) { }
			};
			vector<FootState*> feet;

			struct StandingCallback : public ContactCallback
			{
				Dood* dood;

				float angular_coeff;

				StandingCallback();

				void OnContact(const ContactPoint& contact);				// from CibraryEngine::ContactCallback
				void AfterResolution(const ContactPoint& cp);

				void PreCPHFT(float timestep);								// updates no_contact_timer each tick
				void PostCPHFT(float timestep);								// resets contact points

				void ApplyVelocityChange(const Vec3& dv);

				bool IsStanding();

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

				DamageTakenEvent(Dood* dood, const Vec3& from_dir, const Damage& damage) : dood(dood), from_dir(from_dir), damage(damage), cancel(false) { }
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
				DeathEvent(Dood* dood, const Damage& cause) : dood(dood), cause(cause) { }
			};
			EventDispatcher OnDeath;

		protected:

			void AddBoneToTable(CBone* bone, lua_State* L);
			void AddJointToTable(CJoint* joint, int i, lua_State* L);
			void AddNozzleToTable(int i, lua_State* L);
			void AddContactPointToTable(MyContactPoint& cp, int i, lua_State* L);
	};

	void PushDoodHandle(lua_State* L, Dood* dood);
}
