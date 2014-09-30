#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"
#include "PlacedFootConstraint.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#include "SoldierBrain.h"

#define DIE_AFTER_ONE_SECOND   0

#define ENABLE_PELVIS_ANCHOR   0

#define NUM_BRAIN_ITERATIONS   2

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

			Vec3 desired_torque;
			Vec3 applied_torque;

			Vec3 old_vel, old_rot;
			Vec3 actual_force, actual_torque;

			CBone() { }
			CBone(const Soldier* dood, const string& name) : name(name), rb(dood->RigidBodyForNamedBone(name)), posey(dood->posey->skeleton->GetNamedBone(name)) { }

			void Reset()
			{
				static const float inv_timestep   = 60.0f;							// TODO: don't hard-code this
				static const float use_coeff      = inv_timestep / 60.0f;			// TODO: make this less arbitrary?

				Vec3 nu_vel = rb->GetLinearVelocity();
				Vec3 nu_rot = rb->GetAngularVelocity();

				actual_force   = (nu_vel - old_vel) * (use_coeff * rb->GetMass());
				actual_torque  = Mat3(rb->GetTransformedMassInfo().moi) * (nu_rot - old_rot) * use_coeff;
				actual_torque -= applied_torque;

				old_vel = nu_vel;
				old_rot = nu_rot;

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
		};

		CBone pelvis,    torso1, torso2, head;
		CBone lshoulder, luarm,  llarm,  lhand;
		CBone rshoulder, ruarm,  rlarm,  rhand;
		CBone luleg,     llleg,  lfoot;
		CBone ruleg,     rlleg,  rfoot;

		struct CJoint
		{
			SkeletalJointConstraint* sjc;
			CBone *a, *b;

			Vec3 actual;				// world-coords torque to be applied by this joint

			Mat3 oriented_axes;			// gets recomputed every time Reset() is called

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

		float scores    [SoldierBrain::NumScoringCategories];
		float max_scores[SoldierBrain::NumScoringCategories];

		Vec3 head_goal_rot;

		Quaternion head_goal_ori;
		Vec3 head_error;

		float yaw_vel, pitch_vel;

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

			unsigned int num_memories = 10;
			unsigned int num_inputs   = 102;
			unsigned int num_outputs  = 3;

			SoldierBrain::NextBrain(num_inputs, num_outputs, num_memories);

			yaw_vel = pitch_vel = 0.0f;
			SelectRandomAimVels(dood, 1.0f);

			head_error = Vec3();

			for(unsigned int i = 0; i < SoldierBrain::NumScoringCategories; ++i)
				scores[i] = max_scores[i] = 0.0f;

			// randomize initial velocities of bones, just to further diversify the initial states
			for(vector<CBone*>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
				ShakeRB((**iter).rb);
			ShakeRB(((Gun*)dood->equipped_weapon)->rigid_body);
		}

		void ShakeRB(RigidBody* rb)
		{
			static const float shake_amount = 0.25f;

			rb->SetLinearVelocity (rb->GetLinearVelocity()  + Random3D::RandomNormalizedVector(Random3D::Rand(shake_amount)));
			rb->SetAngularVelocity(rb->GetAngularVelocity() + Random3D::RandomNormalizedVector(Random3D::Rand(shake_amount)));
		}

		void SelectRandomAimVels(Soldier* dood, float new_frac)
		{
			static const float half_pi         = float(M_PI) * 0.5f;

			static const float yaw_rate        = 10.0f;		// hard-coded because Dood::yaw_rate and ::pitch_rate are inaccesible
			static const float pitch_rate      = 10.0f;
			static const float use_rate_frac   = 0.15f;		// nonetheless we probably don't want to actually rotate at such a high speed

			static const unsigned int dest_eta = 30;

			float pru = pitch_rate * use_rate_frac;
			float yru = yaw_rate   * use_rate_frac;

			float goto_vel   = min(pru, max(-pru, (Random3D::Rand(-half_pi, half_pi) - dood->pitch) / (dest_eta / 60.0f)));
			float random_vel = Random3D::Rand(-pru, pru);
			float lerp_a = Random3D::Rand() * Random3D::Rand();
			float lerp_b = 1.0f - lerp_a;

			float new_pvel = Random3D::Rand() * (goto_vel * lerp_a + random_vel * lerp_b);
			float new_yvel = Random3D::Rand() * (Random3D::Rand(-yru, yru));

			float old_frac = 1.0f - new_frac;
			pitch_vel = old_frac * pitch_vel + new_frac * new_pvel;
			yaw_vel   = old_frac * yaw_vel   + new_frac * new_yvel;
		}

		void DoAimUpdate(Soldier* dood)
		{
			// make the aim vel/dir change randomly from time to time
			if(Random3D::RandInt() % 15 == 0)
				SelectRandomAimVels(dood, 0.5f);
			else
				SelectRandomAimVels(dood, 0.02f);

			dood->control_state->SetFloatControl( "yaw",   timestep * yaw_vel   );
			dood->control_state->SetFloatControl( "pitch", timestep * pitch_vel );
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

		void DoAnchorStuff(Soldier* dood, const TimingInfo& time, const Quaternion& p)
		{
			static const float pos_helper_frac = 0.95f, pos_rest_frac = 1.0f - pos_helper_frac;
			static const float ori_helper_frac = 0.95f, ori_rest_frac = 1.0f - ori_helper_frac;

			Quaternion current_ori = pelvis.rb->GetOrientation();
			Quaternion ctd         = current_ori * Quaternion::Reverse(p);
			Quaternion use_ori     = Quaternion::FromRVec(ctd.ToRVec() * ori_rest_frac) * p;

			Vec3 local_com = pelvis.rb->GetMassInfo().com;
			Vec3 goal_pos = (local_com - use_ori * local_com);
			//goal_pos += Vec3(0, 1, 0);
			Vec3 use_pos = goal_pos * pos_helper_frac + pelvis.rb->GetPosition() * pos_rest_frac;

			pelvis.rb->SetPosition(use_pos);
			pelvis.rb->SetOrientation(use_ori);
			pelvis.rb->SetLinearVelocity(Vec3());
			pelvis.rb->SetAngularVelocity(Vec3());
		}

		void DoHeadOri(Soldier* dood, const TimingInfo& time)
		{
			Quaternion desired_ori = head_goal_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);

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

		void DoTorsoOris(Soldier* dood, const TimingInfo& time, const Quaternion& t1, const Quaternion& t2)
		{
			// figure out what net torques we want to apply to each bone
			torso2.ComputeDesiredTorqueWithDefaultMoI(t2, inv_timestep);
			torso1.ComputeDesiredTorqueWithDefaultMoI(t1, inv_timestep);

			// come up with joint torques to accomplish those net torques
			spine2.SetTorqueToSatisfyB();
			spine1.SetTorqueToSatisfyB();
		}

		void DoLegStuff(Soldier* dood, const TimingInfo& time, const Quaternion& t1, const Quaternion& t2)
		{
			// preparation and utility stuff for putting all the inputs into a big array of floats
			Vec3 translation = -torso2.rb->GetCenterOfMass();
			Mat3 rotation = Mat3::FromAxisAngle(0, 1, 0, dood->yaw);

			struct PushVec3
			{
				PushVec3(vector<float>& v, const Vec3& xyz)
				{
					v.push_back(tanhf(xyz.x));
					v.push_back(tanhf(xyz.y));
					v.push_back(tanhf(xyz.z));
				}
			};

			struct PushRB
			{
				// one rb = 8 Vec3 values (was 6 before adding actual_force and actual_torque)
				PushRB(CBone& bone, const Vec3& translation, const Mat3& rotation, vector<float>& v, float timestep)
				{
					RigidBody* rb = bone.rb;

					Mat3 rb_rm = rotation * rb->GetOrientation().ToMat3();
					Vec3 use_rot = rb->GetAngularVelocity() + rb->GetInvMoI() * bone.applied_torque * timestep;

					PushVec3(v, rotation * (rb->GetPosition() + translation));		// pos
					PushVec3(v, Vec3(rb_rm[0], rb_rm[1], rb_rm[2]));				// ori (rm)
					PushVec3(v, Vec3(rb_rm[3], rb_rm[4], rb_rm[6]));
					PushVec3(v, Vec3(rb_rm[6], rb_rm[7], rb_rm[8]));
					PushVec3(v, rotation * rb->GetLinearVelocity());				// vel
					PushVec3(v, rotation * use_rot);								// rot
					PushVec3(v, rotation * bone.actual_force);
					PushVec3(v, rotation * bone.actual_torque);
				}
			};

			vector<float> inputs;

			// now to actually put those inputs into the array, starting with the state of some relevant bones
			//PushRB(lfoot,     translation, rotation, inputs, timestep);
			//PushRB(rfoot,     translation, rotation, inputs, timestep);
			//PushRB(llleg,     translation, rotation, inputs, timestep);
			//PushRB(rlleg,     translation, rotation, inputs, timestep);
			//PushRB(luleg,     translation, rotation, inputs, timestep);
			//PushRB(ruleg,     translation, rotation, inputs, timestep);
			//PushRB(pelvis,    translation, rotation, inputs, timestep);
			//PushRB(torso1,    translation, rotation, inputs, timestep);
			PushRB(torso2,    translation, rotation, inputs, timestep);
			PushRB(lshoulder, translation, rotation, inputs, timestep);
			PushRB(rshoulder, translation, rotation, inputs, timestep);
			PushRB(head,      translation, rotation, inputs, timestep);

			// additional inputs describing the goal state, specifying the desired force & torque of certain bones
			struct GetGoalWorldVels
			{
				GetGoalWorldVels(RigidBody* rb, float inv_timestep, const Vec3& desired_pos, const Quaternion& desired_ori, Vec3& desired_vel, Vec3& desired_rot, Vec3& desired_force, Vec3& desired_torque)
				{
					Vec3       current_pos = rb->GetCenterOfMass();
					Quaternion current_ori = rb->GetOrientation();

					desired_vel = (desired_pos - current_pos) * inv_timestep;
					desired_rot = (desired_ori * Quaternion::Reverse(current_ori)).ToRVec() * -inv_timestep;

					desired_force  = (desired_vel - rb->GetLinearVelocity()) * (inv_timestep * rb->GetMass());
					desired_torque = Mat3(rb->GetTransformedMassInfo().moi) * (desired_rot - rb->GetAngularVelocity()) * -inv_timestep;
				}
			};

			CBone& use_bone   = head;
			CJoint& use_joint = neck;

			if(tick_age != 0)
				head_error = rotation * (use_bone.rb->GetAngularVelocity() - head_goal_rot);

			Vec3 dummy_vel, dummy_force, head_goal_torque;
			GetGoalWorldVels
			(
				use_bone.rb, inv_timestep,
				Quaternion::FromRVec(0, -dood->yaw, 0) * use_bone.rb->GetMassInfo().com + Vec3(0, 2, 0),
				head_goal_ori,
				dummy_vel, head_goal_rot, dummy_force, head_goal_torque
			);

			//use_bone.desired_torque = head_goal_torque;
			//use_joint.SetTorqueToSatisfyB();

			SkeletalJointConstraint* s2sjc = use_joint.sjc;
			const float* s2ats  = (float*)(&s2sjc->apply_torque);
			const float* s2mins = (float*)(&s2sjc->min_torque);
			const float* s2maxs = (float*)(&s2sjc->max_torque);
			
			// unrolled loop
			float value;
			{ value = *s2ats; inputs.push_back(value >= 0 ? value / *s2maxs : -value / *s2mins); ++s2ats; ++ s2mins; ++s2maxs; }
			{ value = *s2ats; inputs.push_back(value >= 0 ? value / *s2maxs : -value / *s2mins); ++s2ats; ++ s2mins; ++s2maxs; }
			{ value = *s2ats; inputs.push_back(value >= 0 ? value / *s2maxs : -value / *s2mins); }

			//use_joint.SetWorldTorque(Vec3());

			static const float make_hertz_hurt_less = 0.02f;
			PushVec3(inputs, rotation * head_goal_rot * make_hertz_hurt_less);

			


			// brain processes inputs and comes up with outputs (also it updates its memory)
			vector<float> outputs(3);
			unsigned int num_inputs = inputs.size();
			for(unsigned int i = 0; i < NUM_BRAIN_ITERATIONS; ++i)
			{
				inputs.resize(num_inputs);
				SoldierBrain::Process(inputs, outputs);
			}


			// apply the values the brain came up with
			struct SetJointTorque
			{
				SetJointTorque(CJoint& j, const Vec3& xyz)
				{
					float* min = (float*)&j.sjc->min_torque;
					float* max = (float*)&j.sjc->max_torque;
					float* vec = (float*)&xyz;
					float* res = (float*)&j.sjc->apply_torque;

					float* vec_end = vec + 3;

					for(; vec != vec_end; ++vec, ++min, ++max, ++res)
					{
						if(*vec > 0)
							*res = *vec * *max;
						else
							*res = *vec * -*min;
					}

					j.actual = j.oriented_axes.TransposedMultiply(j.sjc->apply_torque);
				}
			};

			Vec3* output_vec = (Vec3*)(outputs.data());

			//SetJointTorque(spine2, *(output_vec++));
			//SetJointTorque(spine1, *(output_vec++));
			//SetJointTorque(lhip,   *(output_vec++));
			//SetJointTorque(rhip,   *(output_vec++));
			//SetJointTorque(lknee,  *(output_vec++));
			//SetJointTorque(rknee,  *(output_vec++));
			//SetJointTorque(lankle, *(output_vec++));
			//SetJointTorque(rankle, *(output_vec++));
			//SetJointTorque(neck, *(output_vec++));

			neck.SetTorqueToSatisfyB();

			torso2.ComputeDesiredTorqueWithDefaultMoI(t2, inv_timestep);
			spine2.SetTorqueToSatisfyB();

			torso1.ComputeDesiredTorqueWithDefaultMoI(t1, inv_timestep);
			spine1.SetTorqueToSatisfyB();
		}

		void DoScoringStuff(Soldier* dood)
		{
			static const unsigned int max_sim_ticks = 45;

			if(tick_age != 0)
			{				
				float error_floats[SoldierBrain::NumScoringCategories] =
				{
					head_error.ComputeMagnitudeSquared()
				};

				float component_coeffs[SoldierBrain::NumScoringCategories] =
				{
					0.1f
				};

				for(unsigned int i = 0; i < SoldierBrain::NumScoringCategories; ++i)
				{
					float weight = timestep;
					scores[i]     += weight * expf(-component_coeffs[i] * error_floats[i]);
					max_scores[i] += weight;
				}
			}

			if(tick_age >= max_sim_ticks)
			{
				for(unsigned int i = 0; i < SoldierBrain::NumScoringCategories; ++i)
				{
					float inv_weight = 1.0f / max_scores[i];

					scores[i]     *= inv_weight;
					max_scores[i] *= inv_weight;
				}

				SoldierBrain::Finish(scores);
			}
		}

		void Update(Soldier* dood, const TimingInfo& time)
		{
			if(!init)
			{
				Init(dood);
				init = true;
			}

			timestep = time.elapsed;
			inv_timestep = 1.0f / timestep;

			if(!SoldierBrain::IsFinished())
			{
				DoScoringStuff(dood);
				DoAimUpdate(dood);				
			}


			// reset all the joints and bones
			for(vector<CJoint*>::iterator iter = all_joints.begin(); iter != all_joints.end(); ++iter)
				(*iter)->Reset();
			for(vector<CBone*>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
				(*iter)->Reset();


			// do actual C/PHFT stuff
			Quaternion p, t1, t2;
			GetDesiredTorsoOris(dood, p, t1, t2);

#if ENABLE_PELVIS_ANCHOR
			DoAnchorStuff  ( dood, time, p         );
#endif

			DoHeadOri      ( dood, time            );
			DoArmsAimingGun( dood, time,        t2 );
		//	DoTorsoOris    ( dood, time,    t1, t2 );
			DoLegStuff     ( dood, time,    t1, t2 );

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

						float total_mass = 0.0f;

						for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
							total_mass += (*iter)->GetMassInfo().mass;

						Vec3 apply_force = fly_accel_vec * total_mass;
						for(vector<RigidBody*>::iterator iter = jet_bones.begin(); iter != jet_bones.end(); ++iter)
							(*iter)->ApplyCentralForce(apply_force / float(jet_bones.size()));
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
		static const float half_pi = float(M_PI) * 0.5f;
		pitch = Random3D::Rand() * Random3D::Rand(-half_pi, half_pi);

		Dood::DoInitialPose();

		static const float randomness = 0.3f;

		posey->skeleton->GetNamedBone("pelvis")->pos += Vec3(0, 2.0f + Random3D::Rand(randomness), 0);

		Quaternion& ori = posey->skeleton->GetNamedBone("pelvis")->ori;
		ori = Quaternion::FromRVec(Random3D::RandomNormalizedVector(Random3D::Rand(randomness))) * ori;

		float lyaw = Random3D::Rand(0.25f);
		posey->skeleton->GetNamedBone( "l leg 1" )->ori = Quaternion::FromRVec(Random3D::Rand(-randomness, randomness), lyaw, Random3D::Rand(-randomness, randomness));
		posey->skeleton->GetNamedBone( "l leg 2" )->ori = Quaternion::FromRVec(Random3D::Rand(randomness), 0, 0);
		posey->skeleton->GetNamedBone( "l foot"  )->ori = Quaternion::FromRVec(Random3D::Rand(-randomness, randomness), lyaw + Random3D::Rand(-randomness, randomness), Random3D::Rand(-randomness, randomness));

		float ryaw = Random3D::Rand(0.25f);
		posey->skeleton->GetNamedBone( "r leg 1" )->ori = Quaternion::FromRVec(Random3D::Rand(-randomness, randomness), ryaw, Random3D::Rand(-randomness, randomness));
		posey->skeleton->GetNamedBone( "r leg 2" )->ori = Quaternion::FromRVec(Random3D::Rand(randomness), 0, 0);
		posey->skeleton->GetNamedBone( "r foot"  )->ori = Quaternion::FromRVec(Random3D::Rand(-randomness, randomness), ryaw + Random3D::Rand(-randomness, randomness), Random3D::Rand(-randomness, randomness));

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
