#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#define DIE_AFTER_ONE_SECOND   0

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

	struct CoeffMatrix
	{
		static unsigned int next_id;

		unsigned int id;
		vector<float> matrix;

		float errtot, wtot;

		CoeffMatrix() : id(0), matrix(37 * 38 / 2 * 18), errtot(0), wtot(0) { }

		float GetScore() const { return errtot / wtot; }

		static bool Compare(const CoeffMatrix& a, const CoeffMatrix& b) { return a.wtot != 0 && b.wtot != 0 && a.GetScore() < b.GetScore(); }

		CoeffMatrix CreateMutant(float mutation_scale, float mutation_exp)
		{
			CoeffMatrix result = *this;

			for(unsigned int i = 0; i < result.matrix.size(); ++i)
				result.matrix[i] += mutation_scale * pow(Random3D::Rand(-1.0f, 1.0f), mutation_exp);
			result.id = ++next_id;

			result.errtot = result.wtot = 0;

			return result;
		}

		struct SaveComparator
		{
			float min_wtot;
			SaveComparator(float min_wtot) : min_wtot(min_wtot) { }

			bool operator ()(const CoeffMatrix& a, const CoeffMatrix& b) const
			{
				bool a_ok = a.wtot >= min_wtot, b_ok = b.wtot >= min_wtot;
				return a_ok && !b_ok || a_ok == b_ok && a.wtot != 0 && b.wtot != 0 && a.GetScore() < b.GetScore();
			}
		};
	};

	unsigned int CoeffMatrix::next_id = 0;

	list<CoeffMatrix> saved_coeffs;

	static float original_score = 0;

	static bool matrix_test_running = false;
	static unsigned int trial = 0;

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

			Vec3 prev_desired_torque;

			Vec3 old_rot, old_vel;
			Vec3 measured_torque, measured_force;

			CBone() { }
			CBone(const Soldier* dood, const string& name) : name(name), rb(dood->RigidBodyForNamedBone(name)), posey(dood->posey->skeleton->GetNamedBone(name)), local_com(rb->GetMassInfo().com) { }

			void Reset(float inv_timestep)
			{
				prev_desired_torque = desired_torque;
				desired_torque = applied_torque = Vec3();

				Vec3 new_rot = rb->GetAngularVelocity();
				measured_torque = Mat3(rb->GetTransformedMassInfo().moi) * (new_rot - old_rot) * inv_timestep;
				old_rot = new_rot;

				Vec3 new_vel = rb->GetLinearVelocity();
				measured_force = (new_vel - old_vel) * (inv_timestep * rb->GetMass());
				old_vel = new_vel;
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

			bool SetOrientedTorque(const Vec3& local_torque)
			{
				Vec3 old = oriented_axes.TransposedMultiply(sjc->apply_torque);

				const Vec3 &mint = sjc->min_torque, &maxt = sjc->max_torque;

				sjc->apply_torque.x = max(mint.x, min(maxt.x, local_torque.x));
				sjc->apply_torque.y = max(mint.y, min(maxt.y, local_torque.y));
				sjc->apply_torque.z = max(mint.z, min(maxt.z, local_torque.z));

				Vec3 dif = sjc->apply_torque - local_torque;
				bool result = (dif.x != 0 || dif.y != 0 || dif.z != 0);

				actual = oriented_axes.TransposedMultiply(sjc->apply_torque);

				Vec3 delta = actual - old;

				b->applied_torque -= delta;
				a->applied_torque += delta;

				return result;
			}
		};

		struct PDValue
		{
			float second_last;
			float last_error;
			float error;
			float derror_dt;
			float integral;

			PDValue() : last_error(0.0f), error(0.0f), derror_dt(0.0f), integral(0.0f) { }

			void Update(float new_error, float timestep, float inv_timestep)
			{
				second_last = last_error;
				last_error  = error;
				error       = new_error;
				derror_dt   = (error - last_error) * inv_timestep;
				integral   += error * timestep;
			}
		};

		PDValue pd_values[18];

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

			matrix_test_running = true;

			all_bones.clear();
			all_joints.clear();
			jetpack_nozzles.clear();

			RegisterBone( pelvis    = CBone( dood, "pelvis"     ));
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

			float SP = 5500, N = 150, W = 200, E = 350, SB = 600, SA = 700, H = 5400, K = 5800, A = 5500;
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

			//lknee.sjc->min_torque.y = lknee.sjc->min_torque.z = lknee.sjc->max_torque.y = lknee.sjc->max_torque.z = 0.0f;
			//rknee.sjc->min_torque.y = rknee.sjc->min_torque.z = rknee.sjc->max_torque.y = rknee.sjc->max_torque.z = 0.0f;

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

			for(vector<CBone*>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
				(*iter)->Reset(inv_timestep);



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

			


			// inverse kinematics maybe
			Quaternion lfoot_ori = ((SoldierFoot*)dood->feet[0])->OrientBottomToSurface(Vec3(0, 1, 0));
			Quaternion rfoot_ori = ((SoldierFoot*)dood->feet[1])->OrientBottomToSurface(Vec3(0, 1, 0));

			Scorer scorer(this, p, lfoot_ori, rfoot_ori);

			scorer.Search(40);
			//Debug(((stringstream&)(stringstream() << "initial score = " << scorer.rest.score << "; final score = " << scorer.best.score << "; ratio = " << (scorer.rest.score / scorer.best.score) << endl)).str());



			// translate selected pose into joint torques somehow
			DoHeadOri      ( dood, time     );
			DoArmsAimingGun( dood, time, t2 );

			pelvis.ComputeDesiredTorqueWithDefaultMoI(p,  inv_timestep);
			torso1.ComputeDesiredTorqueWithDefaultMoI(t1, inv_timestep);
			torso2.ComputeDesiredTorqueWithDefaultMoI(t2, inv_timestep);

			spine1.SetTorqueToSatisfyB();
			spine2.SetTorqueToSatisfyB();


			// ... "somehow" ...

			static const unsigned int tracked_bones       = 3;
			static const unsigned int tracked_values      = tracked_bones * 6;
			static const unsigned int num_inputs          = 1 + tracked_values * 2;
			static const unsigned int num_output_vecs     = 6;
			static const unsigned int num_output_floats   = num_output_vecs * 3;

			CBone* cbones[tracked_bones] =
			{
				&pelvis,
				&lfoot,
				&rfoot
			};
			IKState::Bone* ik_bones[tracked_bones] =
			{
				&scorer.best.pelvis,
				&scorer.best.lfoot,
				&scorer.best.rfoot
			};

			Mat3 yaw_unrotate = Mat3::FromRVec(0, -dood->yaw, 0);

			float errtot = 0.0f;
			for(unsigned int i = 0; i < tracked_bones; ++i)
			{
				const IKState::Bone& ik = *ik_bones[i];
				const CBone&         cb = *cbones  [i];
				const RigidBody&     rb = *cb.rb;

				MassInfo xmassinfo  = rb.GetTransformedMassInfo();

				Quaternion ori         = rb.GetOrientation();
				Quaternion desired_ori = Quaternion::FromRotationMatrix(ik.ori);

				/*
				Vec3 rot            = rb.GetAngularVelocity();
				Vec3 desired_rot    = (desired_ori * Quaternion::Reverse(ori)).ToRVec() * -inv_timestep;
				Vec3 desired_aaccel = (desired_rot - rot) * inv_timestep;			

				Vec3 desired_torque = Mat3(xmassinfo.moi) * desired_aaccel;
				Vec3 desired_force  = (ik.pos - xmassinfo.com) * (inv_timestep * xmassinfo.mass);

				Vec3   errors[2]    = { yaw_unrotate * (desired_torque - cb.measured_torque), yaw_unrotate * (desired_force  - cb.measured_force) };
				*/
				Vec3   errors[2]    = { yaw_unrotate * -(desired_ori * Quaternion::Reverse(ori)).ToRVec(), yaw_unrotate * (ik.pos - xmassinfo.com) };
				float* error_floats = (float*)errors;

				for(unsigned int j = 0; j < 6; ++j)
				{
					float error = error_floats[j];
					pd_values[i * 6 + j].Update(error, timestep, inv_timestep);

					if(matrix_test_running)
						 errtot += error * error;
				}
			}

			CoeffMatrix& test_coeffs = *saved_coeffs.begin();
			test_coeffs.errtot += errtot * timestep;
			test_coeffs.wtot   += timestep;

			float inputs[num_inputs];
			float* in_ptr = inputs;
			*(in_ptr++) = 1.0f;
			for(unsigned int i = 0; i < tracked_values; ++i)
			{
				const PDValue& pd = pd_values[i];
				*(in_ptr++) = pd.error;
				*(in_ptr++) = pd.error - pd.last_error;//derror_dt * timestep;			// TODO: remove fudge?
				//*(in_ptr++) = pd.second_last;//integral;
			}
			
			Vec3 outputs[num_output_vecs];
			float* output_floats = (float*)outputs;
			float* of_end        = output_floats + num_output_floats;

			const float* coeff_ptr = test_coeffs.matrix.data();
			for(float *ti_iter = inputs, *ti_end = ti_iter + num_inputs; ti_iter != ti_end; ++ti_iter)
				for(float* ti_jter = ti_iter; ti_jter != ti_end; ++ti_jter)
				{
					float term = *ti_iter * *ti_jter;
					for(float* of_iter = output_floats; of_iter != of_end; ++of_iter, ++coeff_ptr)
						*of_iter += term * *coeff_ptr;
				}

			CBone* obones[num_output_vecs] = { &luleg, &ruleg, &llleg, &rlleg, &lfoot, &rfoot };
			for(unsigned int i = 0; i < num_output_vecs; ++i)
			{
				CBone& ob = *obones[i];
				ob.desired_torque = ob.prev_desired_torque + yaw_unrotate.TransposedMultiply(outputs[i]);
			}
			lhip  .SetTorqueToSatisfyB();
			rhip  .SetTorqueToSatisfyB();
			lknee .SetTorqueToSatisfyB();
			rknee .SetTorqueToSatisfyB();
			lankle.SetTorqueToSatisfyB();
			rankle.SetTorqueToSatisfyB();



			// score amalgamation and mutation stuff
			static const unsigned int max_tick_age      = 95;
			static const unsigned int num_trials        = 20;

			static const unsigned int max_saved_mats    = 10;
			static const float        selection_exp     = 8.0f;
			static const float        min_repro_wtot    = 40.0f;
			static const float        mutation_chance   = 0.8f;
			static const float        mutation_scale    = 0.5f;
			static const float        mutation_exp      = 7.0f;

			++tick_age;
			if(matrix_test_running && tick_age >= max_tick_age)
			{
				matrix_test_running = false;

				++trial;
				if(trial == num_trials)
				{
					Debug(((stringstream&)(stringstream() << "id = " << test_coeffs.id << "; score = " << test_coeffs.GetScore() << " (" << test_coeffs.errtot << " / " << test_coeffs.wtot << ")" << endl)).str());

					saved_coeffs.sort(CoeffMatrix::Compare);
					if(saved_coeffs.size() > max_saved_mats)
						saved_coeffs.pop_back();

					float        fpick = pow(Random3D::Rand(), selection_exp);
					unsigned int ipick = (unsigned int)(saved_coeffs.size() * fpick);

					list<CoeffMatrix>::iterator iter = saved_coeffs.begin();
					for(unsigned int i = 0; i < ipick; ++i)
						++iter;

					bool mutant = iter->wtot >= min_repro_wtot && Random3D::Rand() < mutation_chance;

					for(list<CoeffMatrix>::iterator jter = saved_coeffs.begin(); jter != saved_coeffs.end(); ++jter)
						if(jter->id == 0)
							original_score = jter->GetScore();

					string tabs = "\t\t\t\t\t\t\t";
					stringstream ss;
					ss << tabs << "number of matrices = " << saved_coeffs.size() << "; latest id = " << CoeffMatrix::next_id << "; original score = " << original_score << endl;
					for(list<CoeffMatrix>::iterator jter = saved_coeffs.begin(); jter != saved_coeffs.end(); ++jter)
					{
						ss << tabs << '\t' << "id = " << jter->id << "; score = " << jter->GetScore() << " (" << jter->errtot << " / " << jter->wtot << ")";
						if(jter == iter)
							ss << ";\t\t" << (mutant ? "mutating" : "retest");
						ss << endl;
					}
					((TestGame*)dood->game_state)->debug_text = ss.str();

					if(mutant)
					{
						CoeffMatrix temp = (*iter).CreateMutant(mutation_scale, mutation_exp);

						saved_coeffs.push_front(temp);
					}
					else
					{
						list<CoeffMatrix>::iterator begin = saved_coeffs.begin();
						CoeffMatrix temp = *begin;
						*begin = *iter;
						*iter  = temp;
					}
					
					trial = 0;
				}
			}
		}



		struct IKState
		{
			// joints
			Vec3 lankle, rankle;
			Vec3 lknee,  rknee;

			struct Bone
			{
				Vec3 pos, vel, rot;				// pos is of the com, not model origin
				Mat3 ori;

				void SetConstrained(const RigidBody& rb)
				{
					ori = rb.GetOrientation().ToMat3();
					pos = rb.GetPosition() + ori * rb.GetMassInfo().com;

					vel = rot = Vec3();
				}
			};

			Bone lfoot,  llleg,  luleg;
			Bone rfoot,  rlleg,  ruleg;
			Bone pelvis;

			Vec3 hip_local_avg;
			Vec3 ubody_com_from_havg;
			float ubody_mass;

			Vec3 com;
			Vec3 desired_com;

			float iv_ori_error[4];
			float dv_ori_error[2];
			float dv_pos_error[2];

			float score;



			void SetInitialRVec(const CJoint& j, Vec3& rvec)
			{
				Quaternion aori = j.a->rb->GetOrientation();
				Quaternion bori = j.b->rb->GetOrientation();
				const Mat3& axes = j.sjc->axes;

				rvec = axes * Quaternion::Reverse(aori).ToMat3() * (bori * Quaternion::Reverse(aori)).ToRVec();
			}

			// updating the data of bone A (parent) based on the data of the child bone
			void UpdateAFromB(const Vec3& rvec, const CJoint& j, const Bone& rest_a, const Bone& rest_b, float inv_timestep, Bone& a, Bone& b)
			{
				const SkeletalJointConstraint& sjc = *j.sjc;
				Mat3 axes   = sjc.axes;
				Mat3 axes_t = axes.Transpose();

				a.ori = b.ori * axes_t * Mat3::FromRVec(-rvec) * axes;

				a.pos = b.pos + b.ori * j.r2 - a.ori * j.r1;
				a.vel = (a.pos - rest_a.pos) * inv_timestep;

				// TODO: check this; also maybe look for a more efficient way to do this computation?
				a.rot = (Quaternion::FromRotationMatrix(a.ori.Transpose() * rest_a.ori)).ToRVec() * inv_timestep;

				ComputeOriError(rvec, sjc, iv_ori_error[&rvec - &lankle]);
			}

			void ComputeOriError(const Vec3& rvec, const SkeletalJointConstraint& sjc, float& error)
			{
				const Vec3& mins = sjc.min_extents;
				const Vec3& maxs = sjc.max_extents;

				float dx = rvec.x - max(mins.x, min(maxs.x, rvec.x));
				float dy = rvec.y - max(mins.y, min(maxs.y, rvec.y));
				float dz = rvec.z - max(mins.z, min(maxs.z, rvec.z));

				error = Vec3::MagnitudeSquared(dx, dy, dz);
			}

			void ComputeDVError(const CJoint& joint, const Bone& a, const Bone& b, float& ori_error, float& pos_error)
			{
				const SkeletalJointConstraint& sjc = *joint.sjc;

				Quaternion a_to_b = Quaternion::FromRotationMatrix(b.ori) * Quaternion::Reverse(Quaternion::FromRotationMatrix(a.ori));

				const Mat3& axes = sjc.axes;
				Mat3 rmat = axes * a.ori.Transpose() * b.ori * axes.Transpose();
				Vec3 rvec = Quaternion::FromRotationMatrix(rmat).ToRVec();

				ComputeOriError(rvec, sjc, ori_error);

				Vec3 pos_delta = b.pos - a.pos + b.ori * joint.r2 - a.ori * joint.r1;
				pos_error = pos_delta.ComputeMagnitudeSquared();
			}

			void JointModified(unsigned int index, Imp* imp, const IKState& rest, float inv_timestep)
			{
				switch(index)
				{
					case 0:
						UpdateAFromB(lankle, imp->lankle, rest.llleg,  rest.lfoot,  inv_timestep, llleg,  lfoot );
						UpdateAFromB(lknee,  imp->lknee,  rest.luleg,  rest.llleg,  inv_timestep, luleg,  llleg );
						break;
					case 1:
						UpdateAFromB(rankle, imp->rankle, rest.rlleg,  rest.rfoot,  inv_timestep, rlleg,  rfoot );
						UpdateAFromB(rknee,  imp->rknee,  rest.ruleg,  rest.rlleg,  inv_timestep, ruleg,  rlleg );
						break;
					case 2:
						UpdateAFromB(lknee,  imp->lknee,  rest.luleg,  rest.llleg,  inv_timestep, luleg,  llleg );
						break;
					case 3:
						UpdateAFromB(rknee,  imp->rknee,  rest.ruleg,  rest.rlleg,  inv_timestep, ruleg,  rlleg );
						break;
				}

				PlacePelvis(imp, rest, inv_timestep);
			}

			static void IncrementComAndMassTots(const RigidBody* rb, const Bone& bone, Vec3& comtot, float& masstot)
			{
				float mass = rb->GetMass();
				masstot += mass;
				comtot  += bone.pos * mass;
			}

			void PlacePelvis(Imp* imp, const IKState& rest, float inv_timestep)
			{
				Vec3 lhpos = luleg.pos + luleg.ori * imp->lhip.sjc->pos;
				Vec3 rhpos = ruleg.pos + ruleg.ori * imp->rhip.sjc->pos;
				Vec3 avg   = (lhpos + rhpos) * 0.5f;

				pelvis.pos = avg - pelvis.ori * hip_local_avg;

				ComputeDVError(imp->lhip, pelvis, luleg, dv_ori_error[0], dv_pos_error[0]);
				ComputeDVError(imp->rhip, pelvis, ruleg, dv_ori_error[1], dv_pos_error[1]);

				Vec3 ubody_com = avg + ubody_com_from_havg;
				float masstot = ubody_mass;
				Vec3 comtot = ubody_com * ubody_mass;

				IncrementComAndMassTots(imp->lfoot.rb, lfoot, comtot, masstot);
				IncrementComAndMassTots(imp->rfoot.rb, rfoot, comtot, masstot);
				IncrementComAndMassTots(imp->llleg.rb, llleg, comtot, masstot);
				IncrementComAndMassTots(imp->rlleg.rb, rlleg, comtot, masstot);
				IncrementComAndMassTots(imp->luleg.rb, luleg, comtot, masstot);
				IncrementComAndMassTots(imp->ruleg.rb, ruleg, comtot, masstot);

				com = comtot / masstot;
			}

			float ScoreAll(Imp* imp, const IKState& rest, float inv_timestep)
			{
				hip_local_avg = (imp->lhip.sjc->pos + imp->rhip.sjc->pos) * 0.5f;

				UpdateAFromB(lankle, imp->lankle, rest.llleg,  rest.lfoot,  inv_timestep, llleg,  lfoot );
				UpdateAFromB(lknee,  imp->lknee,  rest.luleg,  rest.llleg,  inv_timestep, luleg,  llleg );
				UpdateAFromB(rankle, imp->rankle, rest.rlleg,  rest.rfoot,  inv_timestep, rlleg,  rfoot );
				UpdateAFromB(rknee,  imp->rknee,  rest.ruleg,  rest.rlleg,  inv_timestep, ruleg,  rlleg );

				PlacePelvis(imp, rest, inv_timestep);

				return score = SharedScoring(imp, rest);
			}

			float ScoreChanges(Imp* imp, const IKState& rest, unsigned int joint_index, float inv_timestep)
			{
				JointModified(joint_index, imp, rest, inv_timestep);

				return score = SharedScoring(imp, rest);
			}

			float SharedScoring(Imp* imp, const IKState& rest)
			{
				float iv_ori_error_tot        = iv_ori_error[0] + iv_ori_error[1] + iv_ori_error[2] + iv_ori_error[3];
				float dv_ori_error_tot        = dv_ori_error[0] + dv_ori_error[1];
				float dv_pos_error_tot        = dv_pos_error[0] + dv_pos_error[1];


				// TODO: compute these
				float impossible_force_error  = 0.0f;
				float impossible_torque_error = 0.0f;

				float impossibility_penalty   = iv_ori_error_tot +
												dv_ori_error_tot +
												dv_pos_error_tot +
												impossible_force_error +
												impossible_torque_error;

				float undesirability_penalty  = (com - desired_com).ComputeMagnitudeSquared();

				// TODO: compute per-joint torques and assign undesirability/impossibility there? probably too expensive

				return impossibility_penalty + undesirability_penalty * 1.0f;
			}
		};

		struct Scorer
		{
			Imp* imp;
			float inv_timestep;

			IKState rest, best, test1, test2;

			Scorer(Imp* imp, const Quaternion& pelvis_ori, const Quaternion& lfoot_ori, const Quaternion& rfoot_ori) : imp(imp)
			{
				inv_timestep = imp->inv_timestep;

				// set properties of constrained bones				// TODO: do this better
				rest.lfoot .SetConstrained(*imp->lfoot .rb);
				rest.rfoot .SetConstrained(*imp->rfoot .rb);
				rest.pelvis.SetConstrained(*imp->pelvis.rb);

				rest.lfoot .ori = lfoot_ori .ToMat3();
				rest.rfoot .ori = rfoot_ori .ToMat3();
				rest.pelvis.ori = pelvis_ori.ToMat3();

				rest.lfoot .pos.y = imp->lfoot .local_com.y;
				rest.rfoot .pos.y = imp->rfoot .local_com.y;
				
				MassInfo upper_body_massinfos[] =
				{
					imp->pelvis   .rb->GetTransformedMassInfo(),
					imp->torso1   .rb->GetTransformedMassInfo(),
					imp->torso2   .rb->GetTransformedMassInfo(),
					imp->head     .rb->GetTransformedMassInfo(),
					imp->lshoulder.rb->GetTransformedMassInfo(),
					imp->rshoulder.rb->GetTransformedMassInfo(),
					imp->luarm    .rb->GetTransformedMassInfo(),
					imp->ruarm    .rb->GetTransformedMassInfo(),
					imp->llarm    .rb->GetTransformedMassInfo(),
					imp->rlarm    .rb->GetTransformedMassInfo(),
					imp->lhand    .rb->GetTransformedMassInfo(),
					imp->rhand    .rb->GetTransformedMassInfo(),
					imp->gun_rb      ->GetTransformedMassInfo(),
				};
				MassInfo sum = MassInfo::Sum(upper_body_massinfos, sizeof(upper_body_massinfos) / sizeof(MassInfo));

				rest.ubody_com_from_havg = sum.com - (rest.pelvis.pos + rest.pelvis.ori * rest.hip_local_avg);		// ish
				rest.ubody_mass = sum.mass;

				// initialize joints with "default" orientations
				rest.SetInitialRVec(imp->lankle, rest.lankle);
				rest.SetInitialRVec(imp->rankle, rest.rankle);
				rest.SetInitialRVec(imp->lknee,  rest.lknee );
				rest.SetInitialRVec(imp->rknee,  rest.rknee );

				// TODO: determine and store "support polygon"

				rest.desired_com.x = (rest.lfoot.pos.x + rest.rfoot.pos.x) * 0.5f;
				rest.desired_com.z = (rest.lfoot.pos.z + rest.rfoot.pos.z) * 0.5f;
				rest.desired_com.y = 1.8f;		// measured value was about 1.8

				ScoreAll(rest);

				best = rest;
			}
				
			float ScoreAll(IKState& state) { return state.ScoreAll(imp, rest, inv_timestep); }

			float ScoreChanges(IKState& state, unsigned int joint_index) { return state.ScoreChanges(imp, rest, joint_index, inv_timestep); }

			void Search(unsigned int num_iterations)
			{
				unsigned int num_vars = 4 * 3;								// four joints times three degrees of freedom
				for(unsigned int i = 0; i < num_iterations; ++i)
				{
					for(unsigned int j = 0; j < num_vars; ++j)
					{
						unsigned int k = j;
						unsigned int joint_index = k / 3;

						float y0 = best.score;
						float scale = sqrtf(y0) * 0.1f;

						test1 = best;
						float& x1 = ((float*)&test1.lankle)[k];
						x1 += scale;
						float y1 = ScoreChanges(test1, joint_index);

						test2 = best;
						float& x2 = ((float*)&test2.lankle)[k];
						x2 -= scale;
						float y2 = ScoreChanges(test2, joint_index);

						if(y1 < y0 && y1 < y2)
							best = test1;
						else if(y2 < y0 && y2 < y1)
							best = test2;
					}
				}
			}

		};
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

		//yaw = Random3D::Rand(float(M_PI) * 2.0f);

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




	bool Soldier::IsExperimentDone() const { return imp->init && !matrix_test_running; }

	void Soldier::LoadMatrix()
	{
		saved_coeffs.clear();
		CoeffMatrix best_coeffs = CoeffMatrix();

		ifstream file("Files/coeffs", ios::in | ios::binary);
		if(!file)
			Debug("Unable to load coefficient matrix!\n");
		else
		{
			unsigned int size = ReadUInt32(file);

			if(size != best_coeffs.matrix.size())
				Debug("Coefficient matrix size doesn't match!\n");
			else
			{
				for(unsigned int i = 0; i < size; ++i)
					best_coeffs.matrix[i] = ReadSingle(file);
			}

			file.close();
		}

		saved_coeffs.push_back(best_coeffs);
	}

	void Soldier::SaveMatrix()
	{
		if(saved_coeffs.empty())
			return;
		saved_coeffs.sort(CoeffMatrix::SaveComparator(3.0f));

		const CoeffMatrix& best_coeffs = *saved_coeffs.begin();

		ofstream file("Files/coeffs", ios::out | ios::binary);
		if(!file)
			Debug("Failed to save coefficient matrix!\n");
		else
		{
			unsigned int size = best_coeffs.matrix.size();
			WriteUInt32(size, file);
			for(unsigned int i = 0; i < size; ++i)
				WriteSingle(best_coeffs.matrix[i], file);

			file.close();
		}
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
