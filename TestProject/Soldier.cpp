#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"
#include "PlacedFootConstraint.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#define DIE_AFTER_ONE_SECOND   0

#define ENABLE_OLDVEL_TRACKING 0
#define ENABLE_NEW_JETPACKING  0

namespace Test
{
	/*
	 * Soldier constants
	 */
	static const float jump_speed         = 4.0f;

	static const float fly_accel_up       = 15.0f;
	static const float fly_accel_lateral  = 8.0f;

	static const float fuel_spend_rate    = 0.5f;
	static const float fuel_refill_rate   = 0.4f;
	static const float jump_to_fly_delay  = 0.3f;

	static const float torso2_yaw_offset  = 0.5f;




	/*
	 * Soldier's custom FootState class
	 */
	class SoldierFoot : public Dood::FootState
	{
		public:

			SoldierFoot(unsigned int posey_id, const Vec3& ee_pos);

			Quaternion OrientBottomToSurface(const Vec3& normal) const;
	};




	/*
	 * Soldier private implementation struct
	 */
	struct Soldier::Imp
	{
		bool init;

		struct CBone
		{
			string name;
			RigidBody* rb;
			Bone* posey;

			Vec3 local_com;

			Vec3 desired_torque;
			Vec3 applied_torque;	

#if ENABLE_OLDVEL_TRACKING
			Vec3 old_vel, old_rot;
			Vec3 actual_force, actual_torque;
#endif


			// inverse kinematics stuff
			struct State
			{
				Vec3 pos;
				Mat3 ori;
				Vec3 vel;
				Vec3 rot;
			} rest, best, test;

			typedef float (Imp::*OriChangeHandler)(CBone::State CBone::*which_state);

			OriChangeHandler handler;


			CBone() : handler(NULL) { }
			CBone(const Soldier* dood, const string& name, OriChangeHandler handler = NULL) : name(name), rb(dood->RigidBodyForNamedBone(name)), posey(dood->posey->skeleton->GetNamedBone(name)), local_com(rb->GetMassInfo().com), handler(handler) { }

#if ENABLE_OLDVEL_TRACKING
			void Reset(float inv_timestep, float use_coeff)
			{
				Vec3 nu_vel = rb->GetLinearVelocity();
				Vec3 nu_rot = rb->GetAngularVelocity();

				actual_force   = (nu_vel - old_vel) * (use_coeff * rb->GetMass());
				actual_torque  = Mat3(rb->GetTransformedMassInfo().moi) * (nu_rot - old_rot) * use_coeff;
				actual_torque -= applied_torque;

				old_vel = nu_vel;
				old_rot = nu_rot;
#else
			void Reset()
			{
#endif
				desired_torque = applied_torque = Vec3();
			}

			void ComputeDesiredTorque(const Quaternion& desired_ori, const Mat3& use_moi, float inv_timestep)
			{
				Quaternion ori      = rb->GetOrientation();
				Vec3 rot            = rb->GetAngularVelocity();

				Vec3 desired_rot    = (desired_ori * Quaternion::Reverse(ori)).ToRVec() * -inv_timestep;
				Vec3 desired_aaccel = (desired_rot - rot) * inv_timestep;

				desired_torque = use_moi * desired_aaccel;
			}

			void ComputeDesiredTorqueWithDefaultMoI(const Quaternion& desired_ori, float inv_timestep) { ComputeDesiredTorque(desired_ori, Mat3(rb->GetTransformedMassInfo().moi), inv_timestep); }
			void ComputeDesiredTorqueWithPosey(const Mat3& use_moi, float inv_timestep)                { ComputeDesiredTorque(posey->GetTransformationMatrix().ExtractOrientation(), use_moi, inv_timestep); }
			void ComputeDesiredTorqueWithDefaultMoIAndPosey(float inv_timestep)                        { ComputeDesiredTorque(posey->GetTransformationMatrix().ExtractOrientation(), Mat3(rb->GetTransformedMassInfo().moi), inv_timestep); }

			void InitIKStates(float timestep, float inv_timestep)
			{
				rest.ori = rb->GetOrientation().ToMat3();
				rest.pos = rb->GetPosition() + rest.ori * local_com;
				rest.vel = rb->GetLinearVelocity();
				rest.rot = rb->GetAngularVelocity();

				// TODO: decide whether or not to move all bones according to current vel/rot

				test = best = rest;
			}
		};

		CBone pelvis,    torso1, torso2, head;
		CBone lshoulder, luarm,  llarm,  lhand;
		CBone rshoulder, ruarm,  rlarm,  rhand;
		CBone luleg,     llleg,  lfoot;
		CBone ruleg,     rlleg,  rfoot;

		RigidBody* gun_rb;

		struct CJoint
		{
			SkeletalJointConstraint* sjc;
			CBone *a, *b;

			Vec3 actual;				// world-coords torque to be applied by this joint

			Mat3 oriented_axes;			// gets recomputed every time Reset() is called

			Vec3 r1, r2;

			CJoint() { }
			CJoint(const Soldier* dood, CBone& bone_a, CBone& bone_b, float max_torque)
			{
				RigidBody *arb = bone_a.rb, *brb = bone_b.rb;
				for(unsigned int i = 0; i < dood->constraints.size(); ++i)
				{
					SkeletalJointConstraint* j = (SkeletalJointConstraint*)dood->constraints[i];
					if(j->obj_a == arb && j->obj_b == brb)
					{
						a   = &bone_a;
						b   = &bone_b;
						sjc = j;

						sjc->min_torque = Vec3(-max_torque, -max_torque, -max_torque);
						sjc->max_torque = Vec3( max_torque,  max_torque,  max_torque);

						r1 = sjc->pos - a->local_com;
						r2 = sjc->pos - b->local_com;

						return;
					}
				}

				// joint not found?
				a = b = NULL;
				sjc = NULL;
			}

			void Reset()
			{
				sjc->apply_torque = actual = Vec3();
				oriented_axes = sjc->axes * Quaternion::Reverse(sjc->obj_a->GetOrientation()).ToMat3();
			}

			bool SetWorldTorque(const Vec3& torque)
			{
				Vec3 local_torque = oriented_axes * torque;

				const Vec3 &mint = sjc->min_torque, &maxt = sjc->max_torque;

				Vec3 old = oriented_axes.TransposedMultiply(sjc->apply_torque);

				sjc->apply_torque.x = max(mint.x, min(maxt.x, local_torque.x));
				sjc->apply_torque.y = max(mint.y, min(maxt.y, local_torque.y));
				sjc->apply_torque.z = max(mint.z, min(maxt.z, local_torque.z));

				Vec3 dif = sjc->apply_torque - local_torque;
				bool result = (dif.x != 0 || dif.y != 0 || dif.z != 0);

				if(result)
					actual = oriented_axes.TransposedMultiply(sjc->apply_torque);
				else
					actual = torque;

				Vec3 delta = actual - old;

				b->applied_torque -= delta;
				a->applied_torque += delta;

				return result;
			}

			bool SetTorqueToSatisfyA() { return SetWorldTorque(a->desired_torque - (a->applied_torque - actual)); }
			bool SetTorqueToSatisfyB() { return SetWorldTorque((b->applied_torque + actual) - b->desired_torque); }

			void SetBPosFromA(CBone::State CBone::* member)
			{
				const CBone::State& from = a->*member;
				CBone::State& to = b->*member;

				to.pos = from.pos + from.ori * r1 - to.ori * r2;
			}
			void SetAPosFromB(CBone::State CBone::* member)
			{
				const CBone::State& from = b->*member;
				CBone::State& to = a->*member;

				to.pos = from.pos + from.ori * r2 - to.ori * r1;
			}
			float GetIKError(CBone::State CBone::* member) const
			{
				const CBone::State &as = a->*member, &bs = b->*member;
				return ((as.pos + as.ori * r1) - (bs.pos + bs.ori * r2)).ComputeMagnitudeSquared();
			}
		};

		CJoint spine1, spine2, neck;
		CJoint lsja,   lsjb,   lelbow, lwrist;
		CJoint rsja,   rsjb,   relbow, rwrist;
		CJoint lhip,   lknee,  lankle;
		CJoint rhip,   rknee,  rankle;

		vector<CBone*>  all_bones;
		vector<CJoint*> all_joints;

		float timestep, inv_timestep;

		unsigned int tick_age;

		struct JetpackNozzle
		{
			CBone* bone;

			Vec3 pos;
			Vec3 cone_center;
			float cone_cossq;
			float max_force, max_forcesq;

			Vec3 world_force, world_torque;
			Vec3 try_force, try_torque;

			Vec3 world_center;
			Vec3 apply_pos;
			Mat3 force_to_torque;

			JetpackNozzle(CBone& bone, const Vec3& pos, const Vec3& cone_center, float cone_angle, float max_force) : bone(&bone), pos(pos), cone_center(cone_center), cone_cossq(cosf(cone_angle)), max_force(max_force), max_forcesq(max_force * max_force) { cone_cossq *= cone_cossq; }

			void Reset() { world_force = Vec3(); }

			void SolverInit(const Vec3& dood_com, float prop_frac)
			{
				const RigidBody& rb = *bone->rb;
				Mat3 rm = rb.GetOrientation().ToMat3();
				world_center = rm * cone_center;
				apply_pos    = rm * pos + rb.GetPosition();

				// compute force-to-torque Mat3
				Vec3 bone_com = rb.GetPosition() + rb.GetOrientation() * rb.GetMassInfo().com;
				Vec3 r1 = apply_pos - bone_com;
				Mat3 xr1 = Mat3(        0,   r1.z,  -r1.y,
									-r1.z,      0,   r1.x,
									 r1.y,  -r1.x,      0	);
				Vec3 r2 = bone_com - dood_com;
				Mat3 xr2 = Mat3(        0,   r2.z,  -r2.y,
									-r2.z,      0,   r2.x,
									 r2.y,  -r2.x,      0	);
				force_to_torque = xr1 + xr2;			// is this right?


				world_force  = world_center * max_force * prop_frac;
				world_torque = force_to_torque * world_force;

				try_force  = world_force;
				try_torque = world_torque;
			}

			void GetNudgeEffects(const Vec3& nudge, Vec3& nu_force, Vec3& nu_torque)
			{
				nu_force = world_force + nudge;

				/*
				float dot = Vec3::Dot(nu_force, world_center);
				if(dot <= 0.0f)
					nu_force = nu_torque = Vec3();
				else
				{
					Vec3 axial = world_center * dot;
					Vec3 ortho = nu_force - axial;
					float axialsq = axial.ComputeMagnitudeSquared();
					float orthosq = ortho.ComputeMagnitudeSquared();
					if(orthosq > axialsq * cone_cossq)
					{
						ortho *= sqrtf(axialsq * cone_cossq / orthosq);
						nu_force = axial + ortho;
					}*/

					float magsq = nu_force.ComputeMagnitudeSquared();
					if(magsq > max_forcesq)
						nu_force *= sqrtf(max_forcesq / magsq);

					nu_torque = force_to_torque * nu_force;
				//}
			}

			void ApplySelectedForce(float timestep)
			{
				//bone->rb->ApplyWorldForce(world_force, apply_pos);				// TODO: make this work?
				bone->rb->ApplyWorldImpulse(world_force * timestep, apply_pos);
			}
		};

		vector<JetpackNozzle> jetpack_nozzles;
		bool jetpacking;
		Vec3 desired_jp_accel;

		Vec3 desired_aim;

		Imp() :
			init(false),
			timestep(0),
			inv_timestep(0),
			tick_age(0)
		{
		}

		~Imp() { }

		void RegisterBone (CBone& bone)   { all_bones.push_back(&bone); }
		void RegisterJoint(CJoint& joint) { all_joints.push_back(&joint); }

		void Init(Soldier* dood)
		{
			//dood->collision_group->SetInternalCollisionsEnabled(true);		// TODO: resolve problems arising from torso2-arm1 collisions

			all_bones.clear();
			all_joints.clear();
			jetpack_nozzles.clear();

			RegisterBone( pelvis    = CBone( dood, "pelvis",    &Imp::PelvisOCH ));
			RegisterBone( torso1    = CBone( dood, "torso 1"    ));
			RegisterBone( torso2    = CBone( dood, "torso 2"    ));
			RegisterBone( head      = CBone( dood, "head"       ));
			RegisterBone( lshoulder = CBone( dood, "l shoulder" ));
			RegisterBone( luarm     = CBone( dood, "l arm 1"    ));
			RegisterBone( llarm     = CBone( dood, "l arm 2"    ));
			RegisterBone( lhand     = CBone( dood, "l hand"     ));
			RegisterBone( rshoulder = CBone( dood, "r shoulder" ));
			RegisterBone( ruarm     = CBone( dood, "r arm 1"    ));
			RegisterBone( rlarm     = CBone( dood, "r arm 2"    ));
			RegisterBone( rhand     = CBone( dood, "r hand"     ));
			RegisterBone( luleg     = CBone( dood, "l leg 1"    ));
			RegisterBone( llleg     = CBone( dood, "l leg 2"    ));
			RegisterBone( lfoot     = CBone( dood, "l foot"     ));
			RegisterBone( ruleg     = CBone( dood, "r leg 1"    ));
			RegisterBone( rlleg     = CBone( dood, "r leg 2"    ));
			RegisterBone( rfoot     = CBone( dood, "r foot"     ));

			float SP = 1500, N = 150, W = 200, E = 350, SB = 600, SA = 700, H = 1400, K = 800, A = 500;
			RegisterJoint( spine1 = CJoint( dood, pelvis,    torso1,    SP ));
			RegisterJoint( spine2 = CJoint( dood, torso1,    torso2,    SP ));
			RegisterJoint( neck   = CJoint( dood, torso2,    head,      N  ));
			RegisterJoint( lsja   = CJoint( dood, torso2,    lshoulder, SA ));
			RegisterJoint( lsjb   = CJoint( dood, lshoulder, luarm,     SB ));
			RegisterJoint( lelbow = CJoint( dood, luarm,     llarm,     E  ));
			RegisterJoint( lwrist = CJoint( dood, llarm,     lhand,     W  ));
			RegisterJoint( rsja   = CJoint( dood, torso2,    rshoulder, SA ));
			RegisterJoint( rsjb   = CJoint( dood, rshoulder, ruarm,     SB ));
			RegisterJoint( relbow = CJoint( dood, ruarm,     rlarm,     E  ));
			RegisterJoint( rwrist = CJoint( dood, rlarm,     rhand,     W  ));
			RegisterJoint( lhip   = CJoint( dood, pelvis,    luleg,     H  ));
			RegisterJoint( lknee  = CJoint( dood, luleg,     llleg,     K  ));
			RegisterJoint( lankle = CJoint( dood, llleg,     lfoot,     A  ));
			RegisterJoint( rhip   = CJoint( dood, pelvis,    ruleg,     H  ));
			RegisterJoint( rknee  = CJoint( dood, ruleg,     rlleg,     K  ));
			RegisterJoint( rankle = CJoint( dood, rlleg,     rfoot,     A  ));

			lknee.sjc->min_torque.y = lknee.sjc->min_torque.z = lknee.sjc->max_torque.y = lknee.sjc->max_torque.z = 0.0f;
			rknee.sjc->min_torque.y = rknee.sjc->min_torque.z = rknee.sjc->max_torque.y = rknee.sjc->max_torque.z = 0.0f;

			Vec3 upward(0, 1, 0);
			float jpn_angle = 1.0f;
			float jpn_force = 150.0f;		// 98kg * 15m/s^2 accel / 10 nozzles ~= 150N per nozzle

			jetpack_nozzles.push_back(JetpackNozzle( lshoulder, Vec3( 0.442619f, 1.576419f, -0.349652f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( lshoulder, Vec3( 0.359399f, 1.523561f, -0.366495f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( lshoulder, Vec3( 0.277547f, 1.480827f, -0.385142f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( rshoulder, Vec3(-0.359399f, 1.523561f, -0.366495f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( rshoulder, Vec3(-0.442619f, 1.576419f, -0.349652f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( rshoulder, Vec3(-0.277547f, 1.480827f, -0.385142f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( lfoot,     Vec3( 0.237806f, 0.061778f,  0.038247f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( lfoot,     Vec3( 0.238084f, 0.063522f, -0.06296f  ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( rfoot,     Vec3(-0.237806f, 0.061778f,  0.038247f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( rfoot,     Vec3(-0.238084f, 0.063522f, -0.06296f  ), upward, jpn_angle, jpn_force ));
		}

		void GetDesiredTorsoOris(Soldier* dood, Quaternion& p, Quaternion& t1, Quaternion& t2)
		{
			float pfrac = dood->pitch * (2.0f / float(M_PI)), pfsq = pfrac * pfrac;

			float t1_yaw   = dood->yaw + torso2_yaw_offset;
			float t1_pitch = dood->pitch * 0.4f + pfsq * pfrac * 0.95f;
			float t1_yaw2  = pfsq * 0.7f;

			t2 = Quaternion::FromRVec(0, -t1_yaw, 0) * Quaternion::FromRVec(t1_pitch, 0, 0) * Quaternion::FromRVec(0, -t1_yaw2, 0);

			float t2_yaw   = dood->yaw + torso2_yaw_offset * 0.5f;
			float t2_pitch = pfrac * 0.05f + pfrac * pfsq * 0.3f;
			float t2_yaw2  = pfsq * 0.15f;

			p = Quaternion::FromRVec(0, -t2_yaw, 0) * Quaternion::FromRVec(t2_pitch, 0, 0) * Quaternion::FromRVec(0, -t2_yaw2, 0);

			Quaternion twist_ori = p * Quaternion::Reverse(t2);
			t1 = Quaternion::FromRVec(twist_ori.ToRVec() * -0.5f) * t2;
		}

		void DoHeadOri(Soldier* dood, const TimingInfo& time)
		{
			Quaternion desired_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);			

			head.ComputeDesiredTorqueWithDefaultMoI(desired_ori, inv_timestep);
			neck.SetTorqueToSatisfyB();
		}

		void DoArmsAimingGun(Soldier* dood, const TimingInfo& time, const Quaternion& t2ori)
		{
			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
			{
				// compute desired per-bone net torques
				MassInfo hng_mass_infos[] = { lhand.rb->GetTransformedMassInfo(), rhand.rb->GetTransformedMassInfo(), gun->rigid_body->GetTransformedMassInfo() };
				Mat3 hng_moi = Mat3(MassInfo::Sum(hng_mass_infos, 3).moi);

				dood->PreparePAG(time, t2ori);

				lhand    .ComputeDesiredTorqueWithPosey( hng_moi,     inv_timestep );
				llarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				luarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				lshoulder.ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );

				rhand    .ComputeDesiredTorqueWithPosey( hng_moi,     inv_timestep );
				rlarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				ruarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				rshoulder.ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );

				// compute applied joint torques to achieve the per-bone applied torques we just came up with
				lwrist.SetWorldTorque(-lhand.desired_torque * 0.75f);
				rwrist.SetWorldTorque(-lwrist.actual - rhand.desired_torque);
				
				lelbow.SetTorqueToSatisfyB();
				lsjb  .SetTorqueToSatisfyB();
				lsja  .SetTorqueToSatisfyB();

				relbow.SetTorqueToSatisfyB();
				rsjb  .SetTorqueToSatisfyB();
				rsja  .SetTorqueToSatisfyB();
			}
		}

		void ComputeMomentumStuff(Soldier* dood, float& dood_mass, Vec3& dood_com, Vec3& com_vel, Vec3& angular_momentum)
		{
			com_vel = Vec3();
			dood_mass = 0.0f;
			for(set<RigidBody*>::iterator iter = dood->velocity_change_bodies.begin(); iter != dood->velocity_change_bodies.end(); ++iter)
			{
				RigidBody* rb = *iter;
				float mass = rb->GetMass();
				dood_mass += mass;
				dood_com  += rb->GetCenterOfMass() * mass;
				com_vel   += rb->GetLinearVelocity() * mass;
			}
			dood_com /= dood_mass;
			com_vel  /= dood_mass;

			MassInfo mass_info;
			Mat3& moi = *((Mat3*)((void*)mass_info.moi));						// moi.values and mass_info.moi occupy the same space in memory

			for(set<RigidBody*>::iterator iter = dood->velocity_change_bodies.begin(); iter != dood->velocity_change_bodies.end(); ++iter)
			{
				RigidBody* body = *iter;
				mass_info = body->GetTransformedMassInfo();

				// linear component
				float mass = mass_info.mass;
				Vec3 vel = body->GetLinearVelocity() - com_vel;
				Vec3 radius = mass_info.com - dood_com;
				angular_momentum += Vec3::Cross(vel, radius) * mass;

				// angular component
				angular_momentum += moi * body->GetAngularVelocity();
			}
		}

		void ResolveJetpackOutput(Soldier* dood, const TimingInfo& time, float dood_mass, const Vec3& dood_com, const Vec3& desired_jp_accel, const Vec3& desired_jp_torque)
		{
			unsigned int num_nozzles = jetpack_nozzles.size();

			for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
				iter->SolverInit(dood_com, 0.0f);

#if ENABLE_NEW_JETPACKING
			Vec3 desired_jp_force = desired_jp_accel * dood_mass;

			float torque_coeff = 50.0f;

			// search for nozzle forces to match the requested accel & torque
			Vec3 force_error, torque_error;
			float errsq, error;
			for(unsigned int i = 0; i < 500; ++i)
			{
				if(i == 0)
				{
					force_error  = -desired_jp_force;
					torque_error = -desired_jp_torque;
					for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
					{
						force_error  += iter->world_force;
						torque_error += iter->world_torque;
					}
					
					errsq = force_error.ComputeMagnitudeSquared() + torque_error.ComputeMagnitudeSquared() * torque_coeff;
				}
				else
				{
					float mutation_scale = error * 0.25f;
					Vec3 mutant_force  = force_error;
					Vec3 mutant_torque = torque_error;
					for(unsigned char j = 0; j < 3; ++j)
					{
						JetpackNozzle& jpn = jetpack_nozzles[Random3D::RandInt() % num_nozzles];

						jpn.GetNudgeEffects(Random3D::RandomNormalizedVector(Random3D::Rand(mutation_scale)), jpn.try_force, jpn.try_torque);

						mutant_force  += jpn.try_force  - jpn.world_force;
						mutant_torque += jpn.try_torque - jpn.world_torque;
					}

					float mutant_errsq = mutant_force.ComputeMagnitudeSquared() + mutant_torque.ComputeMagnitudeSquared() * torque_coeff;
					if(mutant_errsq < errsq)
					{
						for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
						{
							iter->world_force  = iter->try_force;
							iter->world_torque = iter->try_torque;
						}
						force_error  = mutant_force;
						torque_error = mutant_torque;

						errsq = mutant_errsq;
					}
				}

				error = sqrtf(errsq);
				Debug(((stringstream&)(stringstream() << "i = " << i << "; error squared = " << errsq << "; error = " << error << endl)).str());
				if(error < 1.0f)
					break;
			}
#endif
			
			// apply the nozzle forces we computed
			for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
				iter->ApplySelectedForce(timestep);
		}

		float ScorePose(CBone::State CBone::* which_state)
		{
			// TODO: implement this

			float rhip_error = 0.0f;//rhip.GetIKError(which_state);
			float head_error = (desired_aim - (head.*which_state).ori * Vec3(0, 0, 1)).ComputeMagnitudeSquared();

			// TODO: implement these
			float gun_error         = 0.0f;
			float net_momenta_error = 0.0f;
			float balance_error     = 0.0f;
			float joint_ori_error   = 0.0f;

			float bone_torque_cost  = 0.0f;

			// TODO: additional considerations?

			return rhip_error + head_error + gun_error + net_momenta_error + balance_error + joint_ori_error + bone_torque_cost;
		}

		float ScoreChanges(CBone::State CBone::* which_state, unsigned int bone, unsigned int comp)
		{
			CBone& cbone = *all_bones[bone];
			CBone::State& state = cbone.*which_state;

			// compute updated orientation for this bone
			state.ori = Mat3::FromRVec(-state.rot * timestep) * cbone.rest.ori;

			// compute updated positions for this and other bones as needed
			if(CBone::OriChangeHandler& och = cbone.handler)
				return (this->*och)(which_state);
			else
				return GenericOCH(which_state);
		}

		float PelvisOCH(CBone::State CBone::* which_state) { return GenericOCH(which_state); }				// TODO: implement this for real

		float GenericOCH(CBone::State CBone::* which_state)
		{
			// do a complete recomputation of the positions of all the unconstrained bones

			// make sure constrained bones stay in their constrained pos/ori
			(lfoot.*which_state).pos = lfoot.rest.pos;
			(rfoot.*which_state).pos = rfoot.rest.pos;

			// TODO: constraint hands' relative ori, head ori, etc.?

#if 1
			lankle.SetAPosFromB(which_state);
			rankle.SetAPosFromB(which_state);
			lknee .SetAPosFromB(which_state);
			rknee .SetAPosFromB(which_state);
			lhip  .SetAPosFromB(which_state);
			// rhip is a possible error source
			spine1.SetBPosFromA(which_state);
			spine2.SetBPosFromA(which_state);
			neck  .SetBPosFromA(which_state);
			lsja  .SetBPosFromA(which_state);
			rsja  .SetBPosFromA(which_state);
			lsjb  .SetBPosFromA(which_state);
			rsjb  .SetBPosFromA(which_state);
			lelbow.SetBPosFromA(which_state);
			relbow.SetBPosFromA(which_state);
			lwrist.SetBPosFromA(which_state);
			rwrist.SetBPosFromA(which_state);
			// TODO: compute resultant gun xform?
#endif

			return ScorePose(which_state);
		}

		void Update(Soldier* dood, const TimingInfo& time)
		{
			if(!init)
			{
				Init(dood);
				init = true;
			}

			timestep     = time.elapsed;
			inv_timestep = 1.0f / timestep;


			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
				gun_rb = gun->rigid_body;
			else
				gun_rb = NULL;

			// reset all the joints and bones
			for(vector<CJoint*>::iterator iter = all_joints.begin(); iter != all_joints.end(); ++iter)
				(*iter)->Reset();

#if ENABLE_OLDVEL_TRACKING
			float use_coeff = inv_timestep / 60.0f;				// TODO: make this less arbitrary?
			for(vector<CBone*>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
				(*iter)->Reset(inv_timestep, use_coeff);
#else
			for(vector<CBone*>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
				(*iter)->Reset();
#endif



			// do actual C/PHFT stuff
			Quaternion p, t1, t2;
			GetDesiredTorsoOris(dood, p, t1, t2);

			float dood_mass;
			Vec3 dood_com, com_vel, angular_momentum;
			ComputeMomentumStuff(dood, dood_mass, dood_com, com_vel, angular_momentum);

#if ENABLE_NEW_JETPACKING
			if(jetpacking)
			{
				Vec3 desired_jp_torque = angular_momentum * (-60.0f);
				ResolveJetpackOutput(dood, time, dood_mass, dood_com, desired_jp_accel, desired_jp_torque);
			}
			else
			{
				// this will be necessary for when rendering for jetpack flames is eventually added
				for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
					iter->Reset();
			}
#endif

			
			// TODO: decide whether each foot should remain/become grounded/ungrounded
			// TODO: select desired net l&a momenta for the system, in order to satisfy movement controls (forward/sidestep/jump)

			Quaternion desired_head_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);
			desired_aim = desired_head_ori * Vec3(0, 0, 1);

			// inverse kinematics maybe?
			for(vector<CBone*>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
				(*iter)->InitIKStates(timestep, inv_timestep);
			
			// TODO: set constrained bones' pos/ori?

			float rest_score = ScorePose(&CBone::best);
			float last_score = rest_score;

			unsigned int num_vars = all_bones.size() * 3;
			for(unsigned int i = 0; i < 20; ++i)
			{
				for(unsigned int j = 0; j < num_vars; ++j)
				{
					static const float delta         = 0.01f;
					static const float rate          = 5.0f;
					static const float dydx_coeff    = rate / delta;

					unsigned int bone = j / 3;
					unsigned int comp = j % 3;

					float& x = ((float*)&all_bones[bone]->best.rot)[comp];

					float x1 = x;
					float y1 = last_score;

					x += delta;

					float y2 = ScoreChanges(&CBone::best, bone, comp);
					float dydx = (y2 - y1) / delta;
					x = x1 - dydx * dydx_coeff;

					float y3 = ScoreChanges(&CBone::best, bone, comp);

					if(y1 < y2 && y1 < y3)
					{
						x = x1;
						last_score = y1;
					}
					else if(y2 < y1 && y2 < y3)
					{
						x = x1 + delta;
						last_score = y2;
					}
					else
						last_score = y3;
				}
			}



			// TODO: translate selected pose into joint torques somehow
			Debug(((stringstream&)(stringstream() << "initial score = " << rest_score << "; final score = " << last_score << "; ratio = " << (rest_score / last_score) << endl)).str());



			DoHeadOri      ( dood, time            );
			DoArmsAimingGun( dood, time,        t2 );

			Quaternion yaw_ori = Quaternion::FromRVec(0, -dood->yaw, 0);

			lfoot .ComputeDesiredTorqueWithDefaultMoI(yaw_ori, inv_timestep);
			rfoot .ComputeDesiredTorqueWithDefaultMoI(yaw_ori, inv_timestep);
			llleg .ComputeDesiredTorqueWithDefaultMoI(yaw_ori, inv_timestep);
			rlleg .ComputeDesiredTorqueWithDefaultMoI(yaw_ori, inv_timestep);
			luleg .ComputeDesiredTorqueWithDefaultMoI(yaw_ori, inv_timestep);
			ruleg .ComputeDesiredTorqueWithDefaultMoI(yaw_ori, inv_timestep);
			pelvis.ComputeDesiredTorqueWithDefaultMoI(p,       inv_timestep);
			torso1.ComputeDesiredTorqueWithDefaultMoI(t1,      inv_timestep);
			torso2.ComputeDesiredTorqueWithDefaultMoI(t2,      inv_timestep);

			spine2.SetTorqueToSatisfyB();
			spine1.SetTorqueToSatisfyB();
			lhip  .SetTorqueToSatisfyA();
			rhip  .SetTorqueToSatisfyA();
			lknee .SetTorqueToSatisfyB();
			rknee .SetTorqueToSatisfyB();
			lankle.SetTorqueToSatisfyB();
			rankle.SetTorqueToSatisfyB();

			++tick_age;
		}
	};




	/*
	 * Soldier methods
	 */
	Soldier::Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		imp(NULL),
		gun_hand_bone(NULL),
		p_ag(NULL),
		walk_pose(NULL),
		jet_bones(),
		jet_fuel(1.0f),
		jet_start_sound(NULL),
		jet_loop_sound(NULL),
		jet_loop(NULL)
	{
		use_cheaty_ori = false;

		yaw = Random3D::Rand(float(M_PI) * 2.0f);

		gun_hand_bone = character->skeleton->GetNamedBone("r grip");

		Cache<SoundBuffer>* sound_cache = game_state->content->GetCache<SoundBuffer>();
		jet_start_sound = sound_cache->Load("jet_start");
		jet_loop_sound  = sound_cache->Load("jet_loop" );

		standing_callback.angular_coeff = 1.0f;

		ragdoll_timer = 3600.0f;

		p_ag = new PoseAimingGun();
		//posey->active_poses.push_back(p_ag);

		imp = new Imp();
	}

	void Soldier::InnerDispose()
	{
		Dood::InnerDispose();

		if(imp) { delete imp; imp = NULL; }
	}

	void Soldier::DoJumpControls(const TimingInfo& time, const Vec3& forward, const Vec3& rightward)
	{
		float timestep = time.elapsed;

		bool can_recharge = true;
		bool jetted = false;
		if(control_state->GetBoolControl("jump"))
		{
			if(standing_callback.IsStanding() && time.total > jump_start_timer)							// jump off the ground
			{
				standing_callback.ApplyVelocityChange(Vec3(0, jump_speed, 0));

				jump_start_timer = time.total + jump_to_fly_delay;
			}
			else
			{
				can_recharge = false;

				if(jet_fuel > 0)
				{
					// jetpacking
					if(time.total > jump_start_timer)
					{
						jetted = true;

						if(jet_loop == NULL)
						{
							PlayDoodSound(jet_start_sound, 5.0f, false);
							jet_loop = PlayDoodSound(jet_loop_sound, 1.0f, true);
						}
						else
						{
							jet_loop->pos = pos;
							jet_loop->vel = vel;
						}

						jet_fuel -= timestep * (fuel_spend_rate);

						Vec3 fly_accel_vec = Vec3(0, fly_accel_up, 0);
						Vec3 horizontal_accel = forward * max(-1.0f, min(1.0f, control_state->GetFloatControl("forward"))) + rightward * max(-1.0f, min(1.0f, control_state->GetFloatControl("sidestep")));
						fly_accel_vec += horizontal_accel * fly_accel_lateral;

						imp->desired_jp_accel = fly_accel_vec;

#if !ENABLE_NEW_JETPACKING
						// TODO: remove this once similar functionality is moved to Soldier::Imp::ResolveJetpackOutput
						float total_mass = 0.0f;
						for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
							total_mass += (*iter)->GetMassInfo().mass;
						Vec3 apply_force = fly_accel_vec * total_mass;

						for(vector<RigidBody*>::iterator iter = jet_bones.begin(); iter != jet_bones.end(); ++iter)
							(*iter)->ApplyCentralForce(apply_force / float(jet_bones.size()));
#endif
					}
				}
				else
				{
					// out of fuel! flash hud gauge if it's relevant
					JumpFailureEvent evt = JumpFailureEvent(this);
					OnJumpFailure(&evt);
				}
			}
		}

		imp->jetpacking = jetted;

		if(!jetted && jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}

		if(can_recharge)
			jet_fuel = min(jet_fuel + fuel_refill_rate * timestep, 1.0f);
	}

	void Soldier::PreUpdatePoses(const TimingInfo& time)
	{
		imp->Update(this, time);
	}

	void Soldier::PhysicsToCharacter()
	{
		Dood::PhysicsToCharacter();

		// position and orient the gun
		if(equipped_weapon != NULL)
		{
			Gun* gun = dynamic_cast<Gun*>(equipped_weapon);

			if(gun != NULL && gun->rigid_body != NULL)
			{
				RigidBody* gun_rb = gun->rigid_body;
				Mat4 gun_xform = gun_rb->GetTransformationMatrix();

				equipped_weapon->gun_xform = gun_xform;
				equipped_weapon->sound_pos = equipped_weapon->pos = gun_xform.TransformVec3_1(0, 0, 0);
				equipped_weapon->sound_vel = equipped_weapon->vel = vel;
			}
			else if(gun_hand_bone != NULL)
			{
				equipped_weapon->gun_xform = Mat4::Translation(pos) * gun_hand_bone->GetTransformationMatrix() * Mat4::Translation(gun_hand_bone->rest_pos);
				equipped_weapon->sound_pos = equipped_weapon->pos = equipped_weapon->gun_xform.TransformVec3_1(0, 0, 0);
				equipped_weapon->sound_vel = equipped_weapon->vel = vel;
			}
		}
	}

	void Soldier::RegisterFeet()
	{
		feet.push_back(new SoldierFoot(Bone::string_table["l foot"], Vec3( 0.238f, 0.000f, 0.065f)));
		feet.push_back(new SoldierFoot(Bone::string_table["r foot"], Vec3(-0.238f, 0.000f, 0.065f)));
	}

	void Soldier::Update(const TimingInfo& time)
	{
#if DIE_AFTER_ONE_SECOND
		if(time.total > 1.0f)
			Die(Damage());
#endif

		Dood::Update(time);
	}

	void Soldier::Die(const Damage& cause)
	{
		if(jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}

		Dood::Die(cause);
	}

	void Soldier::Spawned()
	{
		Dood::Spawned();

		if(!is_valid)
			return;		

		p_ag->torso2_ori = Quaternion::FromRVec(0, -(yaw + torso2_yaw_offset), 0);

		unsigned int lshoulder_name = Bone::string_table["l shoulder"], rshoulder_name = Bone::string_table["r shoulder"];
		for(unsigned int i = 0; i < character->skeleton->bones.size(); ++i)
			if(character->skeleton->bones[i]->name == lshoulder_name || character->skeleton->bones[i]->name == rshoulder_name)
				jet_bones.push_back(bone_to_rbody[i]);
	}

	void Soldier::DeSpawned()
	{
		Dood::DeSpawned();

		if(jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}
	}

	void Soldier::DoInitialPose()
	{
		Dood::DoInitialPose();

		PreparePAG(TimingInfo(0, 0), Quaternion::FromRVec(0, -(yaw + torso2_yaw_offset), 0));
	}

	void Soldier::PreparePAG(const TimingInfo& time, const Quaternion& t2ori)
	{
		p_ag->yaw   = yaw;
		p_ag->pitch = pitch;
		p_ag->torso2_ori = posey->skeleton->bones[0]->ori = t2ori;
		p_ag->UpdatePose(time);

		for(boost::unordered_map<unsigned int, BoneInfluence>::iterator iter = p_ag->bones.begin(); iter != p_ag->bones.end(); ++iter)
			posey->skeleton->GetNamedBone(iter->first)->ori = Quaternion::FromRVec(iter->second.ori);

		posey->skeleton->InvalidateCachedBoneXforms();
	}




	/*
	 * SoldierFoot methods
	 */
	SoldierFoot::SoldierFoot(unsigned int posey_id, const Vec3& ee_pos) : FootState(posey_id, ee_pos) { }

	Quaternion SoldierFoot::OrientBottomToSurface(const Vec3& normal) const
	{
		static const Vec3 dirs[2] = { Vec3(0, 0, 1), Vec3(1, 0, 0) };

		Quaternion foot_ori = body->GetOrientation();
		for(unsigned int i = 0; i < 2; ++i)
		{
			Vec3 dir   = foot_ori * dirs[i];
			Vec3 level = dir - normal * Vec3::Dot(dir, normal);
			Vec3 cross = Vec3::Cross(dir, level);

			float level_mag = level.ComputeMagnitude();
			float cross_mag = cross.ComputeMagnitude();

			if(level_mag != 0 && cross_mag != 0 && fabs(cross_mag) <= fabs(level_mag))
				foot_ori = Quaternion::FromRVec(cross * (asinf(cross_mag / level_mag) / cross_mag)) * foot_ori;
		}

		return foot_ori;
	}
}
