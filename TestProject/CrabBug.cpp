#include "StdAfx.h"
#include "CrabBug.h"

#include "WalkPose.h"

#include "CBone.h"
#include "CJoint.h"

#include "GAExperiment.h"

#define DIE_AFTER_ONE_SECOND   0

#define ENABLE_WALK_ANIMATIONS 0

namespace Test
{
	using namespace std;

	/*
	 * CrabBug constants
	 */
	static const float bug_leap_duration  = 0.5f;
	static const float bug_leap_speed     = 8.0f;



	/*
	 * CrabBug animations and related functions
	 */
#if ENABLE_WALK_ANIMATIONS
	static void GenerateHardCodedWalkAnimation(KeyframeAnimation* ka)
	{
		{
			Keyframe kf(0.5f);
			kf.next = 1;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3( 1,  0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3( 0, -1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3( 1,  0.5f, 0 ), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3( 0,  0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3( 1, -1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3( 0,	 0.5f, 0 ), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 2;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3( 0,  0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3( 1,	-1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3( 0,  0.5f, 0 ), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3( 1,  0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3( 0, -1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3( 1,  0.5f, 0 ), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3( 0, -0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3( 1,  1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3( 0, -0.5f, 0 ), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3( 1, -0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3( 0,  1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3( 1, -0.5f, 0 ), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3( 1, -0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3( 0,  1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3( 1, -0.5f, 0 ), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3( 0, -0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3( 1,  1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3( 0, -0.5f, 0 ), Vec3());

			ka->frames.push_back(kf);
		}
	}

	static void GenerateRestPose(KeyframeAnimation* ka)
	{
		ka->frames.clear();
		ka->name = "crabby rest pose";

		Keyframe kf;
		kf.duration = 2.0f;
		kf.next = 0;
		kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence();
		kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence();
		kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence();
		kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence();
		kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence();
		kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence();

		ka->frames.push_back(kf);
	}
#endif

	struct CrabLeg
	{
		CBone bones[3];
		CJoint joints[3];
	};




	/*
	* CrabBug private implementation struct
	*/
	struct CrabBug::Imp
	{
		bool init;
		bool clean_init;
		bool experiment_done;

		float score;

		GATrialToken ga_token;
		GAExperiment* experiment;

		Vec3 initial_pos;

		CBone carapace, head, tail;
		CJoint neck, tailj;

		CrabLeg llegs[3];
		CrabLeg rlegs[3];

		float pd_target[36];
		float pd_previous[36];
		float pd_integral[36];

		float timestep, inv_timestep;

		unsigned int tick_age, max_tick_age;

		Imp() : init(false), experiment_done(false), experiment(nullptr), timestep(0), inv_timestep(0), tick_age(0), max_tick_age(30) { }
		~Imp() { }

		void Init(CrabBug* dood)
		{
			dood->collision_group->SetInternalCollisionsEnabled(true);

			initial_pos = dood->pos;

			SharedInit(dood);
		}

		void ReInit(CrabBug* dood)
		{
			dood->yaw = dood->pitch = 0.0f;
			dood->pos = initial_pos;

			dood->DoInitialPose();
			dood->posey->skeleton->InvalidateCachedBoneXforms();

			for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
			{
				CBone& cb = *dood->all_bones[i];
				RigidBody& rb = *cb.rb;

				Bone* posey = cb.posey;
				posey->GetTransformationMatrix().Decompose(cb.initial_pos, cb.initial_ori);		// re-computes bones' initial states

				rb.SetPosition   (cb.initial_pos);
				rb.SetOrientation(cb.initial_ori);
				rb.SetLinearVelocity (Vec3());
				rb.SetAngularVelocity(Vec3());
			}

			SharedInit(dood);
		}

		void SharedInit(CrabBug* dood)
		{
			clean_init = false;
			experiment_done = false;
			tick_age = 0;

			score = 0.0f;

			memset(pd_previous, 0, sizeof(pd_previous));

			if(experiment != nullptr && ga_token.candidate == nullptr)
				ga_token = experiment->GetNextTrial();
		}

		void Update(CrabBug* dood, const TimingInfo& time)
		{
			if(!init)
			{
				Init(dood);
				init = true;
				return;
			}
			else if(experiment_done || experiment != nullptr && ga_token.candidate == nullptr)
			{
				ReInit(dood);
				return;
			}

			if(!clean_init)
			{
				ReInit(dood);

				//if(Random3D::RandInt() % 2 == 0)
				//	clean_init = true;
				//else
				//	return;

				clean_init = true;
			}

			timestep     = time.elapsed;
			inv_timestep = 1.0f / timestep;

			// reset all the joints and bones
			int ji = 0;
			for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
			{
				CJoint& joint = *dood->all_joints[i];
				joint.Reset();
				if(tick_age == 0)
				{
					joint.sjc->apply_torque = Vec3();
					joint.actual = Vec3();
				}
				joint.SetOrientedTorque(Vec3());
			}
			for (vector<CBone*>::iterator iter = dood->all_bones.begin(); iter != dood->all_bones.end(); ++iter)
			{
				(*iter)->Reset(inv_timestep);
				(*iter)->rb->ComputeInvMoI();			// force recomputation of a few cached fields, including ori_rm 
			}

			float total_mass = 0.0f;
			for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
				total_mass += dood->all_bones[i]->rb->GetMass();



			//Quaternion yaw_ori = Quaternion::FromAxisAngle(0, 1, 0, -dood->yaw);
			//for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
			//	dood->all_bones[i]->ComputeDesiredTorqueWithDefaultMoI(yaw_ori, inv_timestep);

			//dood->DoScriptedMotorControl("Files/Scripts/crab_motor_control.lua");

			//stringstream ss;
			//ss << "age = " << tick_age << endl;
			//for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
			//{
			//	const CJoint& j = *dood->all_joints[i];
			//	const Vec3& f = j.sjc->net_impulse_linear;
			//	const Vec3& t = j.sjc->net_impulse_angular;
			//	ss << "\t" << j.b->name << ": F = (" << f.x << ", " << f.y << ", " << f.z << "); T = (" << t.x << ", " << t.y << ", " << t.z << ")" << endl;
			//}
			//Debug(ss.str());

			if(experiment != nullptr && ga_token.candidate != nullptr && !ga_token.candidate->aborting)
			{
				//float total_errsq = 0.0f;

				// apply controls
				unsigned int pd_index = 0;
				for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
				{
					Vec3 use_torque;

					CJoint& joint = *dood->all_joints[i];
					SkeletalJointConstraint& sjc = *joint.sjc;
					float* mint = (float*)&sjc.min_torque;
					float* maxt = (float*)&sjc.max_torque;

					Vec3 rvec = joint.GetRVec();
					float* rptr = (float*)&rvec;
					float* tptr = (float*)&use_torque;

					for(unsigned int j = 0; j < 3; ++j, ++mint, ++maxt, ++rptr, ++tptr)
					{
						if(*maxt > *mint)
						{
							unsigned int ref_index = pd_index < 21 ? pd_index : pd_index - 15;

							if(tick_age == 0)
							{
								pd_target[pd_index] = *rptr;
								pd_integral[pd_index] = ga_token.candidate->initial[ref_index];
							}

							float current_error = *rptr - pd_target[pd_index];
							float derror_dt = (current_error - pd_previous[pd_index]) * inv_timestep;

							pd_integral[pd_index] += current_error * timestep;

							float ppart = current_error * ga_token.candidate->kp[ref_index];
							float ipart = pd_integral[pd_index] * ga_token.candidate->ki[ref_index];
							float dpart = derror_dt * ga_token.candidate->kd[ref_index];
							*tptr = total_mass * (ppart + ipart + dpart);

							pd_previous[pd_index] = current_error;

							++pd_index;
						}
					}

					joint.SetOrientedTorque(use_torque);
				}

				// compute score
				for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
					score += (Quaternion::Reverse(dood->all_bones[i]->initial_ori) * dood->all_bones[i]->rb->GetOrientation()).ToRVec().ComputeMagnitudeSquared();

				//score += total_errsq;
				//score += (carapace.initial_pos - carapace.rb->GetPosition()).ComputeMagnitudeSquared() * 10.0f;
			}

			++tick_age;

			if(experiment != nullptr && ga_token.candidate != nullptr)
			{
				float fail_threshold = experiment->GetEarlyFailThreshold();
				if(fail_threshold >= 0.0f && score + ga_token.candidate->score >= fail_threshold)
					ga_token.candidate->aborting = true;

				if(tick_age >= max_tick_age || ga_token.candidate->aborting)
				{
					if(!experiment_done)
					{
						experiment->TrialFinished(ga_token, score);
						experiment_done = true;

						ga_token = experiment->GetNextTrial();
					}
				}
			}
		}
	};



	
	/*
	 * CrabBug methods
	 */
	CrabBug::CrabBug(GameState* game_state, GAExperiment* experiment, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		imp(NULL),
		crab_heading(new CrabHeading())
	{
		hp *= 0.5f;

		//yaw = Random3D::Rand(float(M_PI) * 2.0f);

		ragdoll_timer = 10.0f;

		// character animation stuff
		posey->active_poses.push_back(crab_heading);

#if ENABLE_WALK_ANIMATIONS
		KeyframeAnimation kw, kr;
		GenerateHardCodedWalkAnimation(&kw);
		GenerateRestPose(&kr);

		posey->active_poses.push_back(new WalkPose(this, &kr, &kw, &kw, &kw, &kw, NULL, NULL));
#endif

		standing_callback.angular_coeff = 0.0f;

		imp = new Imp();
		imp->experiment = experiment;
	}

	void CrabBug::InnerDispose()
	{
		Dood::InnerDispose();

		if(imp) { delete imp; imp = NULL; }
	}

	void CrabBug::DoJumpControls(const TimingInfo& time, const Vec3& forward, const Vec3& rightward)
	{
		if(standing_callback.IsStanding() && control_state->GetBoolControl("leap") && time.total > jump_start_timer)
		{
			// crab bug leaps forward
			float leap_angle = 0.4f;
			Vec3 leap_vector = (forward * (cosf(leap_angle)) + Vec3(0, sinf(leap_angle), 0));

			standing_callback.ApplyVelocityChange(leap_vector * bug_leap_speed);

			jump_start_timer = time.total + bug_leap_duration;
		}
	}

	void CrabBug::PreUpdatePoses(const TimingInfo& time)
	{
		crab_heading->yaw = yaw;

		imp->Update(this, time);
	}

	void CrabBug::Update(const TimingInfo& time)
	{
#if DIE_AFTER_ONE_SECOND
		if(time.total > 1.0f)
			Die(Damage());
#endif

		Dood::Update(time);
	}

	void CrabBug::RegisterFeet()
	{
		feet.push_back(new FootState(Bone::string_table["l leg a 3"], Vec3( 0.288f,  0.0f,  1.293f)));
		feet.push_back(new FootState(Bone::string_table["r leg a 3"], Vec3(-0.288f,  0.0f,  1.293f)));
		feet.push_back(new FootState(Bone::string_table["l leg b 3"], Vec3( 2.068f,  0.0f,  0.470f)));
		feet.push_back(new FootState(Bone::string_table["r leg b 3"], Vec3(-2.068f,  0.0f,  0.470f)));
		feet.push_back(new FootState(Bone::string_table["l leg c 3"], Vec3( 0.798f,  0.0f, -1.366f)));
		feet.push_back(new FootState(Bone::string_table["r leg c 3"], Vec3(-0.798f,  0.0f, -1.366f)));
	}

	void CrabBug::MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi)
	{
		cheaty_rot = Vec3();
		Dood::MaybeSinkCheatyVelocity(timestep, cheaty_vel, cheaty_rot, net_mass, net_moi);
	}

	void CrabBug::InitBoneHelpers()
	{
		Dood::InitBoneHelpers();

		RegisterBone( imp->carapace = CBone(this, "carapace") );
		RegisterBone( imp->head     = CBone(this, "head"    ) );
		RegisterBone( imp->tail     = CBone(this, "tail"    ) );

		static const string sides[2] = { "l", "r" };
		static const string legs [3] = { "a", "b", "c" };
		static const string bones[3] = { "1", "2", "3" };

		for(int i = 0; i < 2; ++i)
		{
			CrabLeg* legs_array = i == 0 ? imp->llegs : imp->rlegs;
			for(int j = 0; j < 3; ++j)
				for(int k = 0; k < 3; ++k)
					RegisterBone( legs_array[j].bones[k] = CBone(this, ((stringstream&)(stringstream() << sides[i] << " leg " << legs[j] << " " << bones[k])).str()) );
		}
	}

	void CrabBug::InitJointHelpers()
	{
		Dood::InitJointHelpers();

		float N = 200, T = 150;

		float joint_strengths[3] = { 1400, 1200, 900 };
		float leg_multipliers[3] = { 0.9f, 1.0f, 0.9f };

		RegisterJoint( imp->neck  = CJoint( this, imp->carapace, imp->head, N ));
		RegisterJoint( imp->tailj = CJoint( this, imp->carapace, imp->tail, T ));

		for(int i = 0; i < 2; ++i)
		{
			CrabLeg* legs_array = i == 0 ? imp->llegs : imp->rlegs;
			for(int j = 0; j < 3; ++j)
				for(int k = 0; k < 3; ++k)
				{
					float x = leg_multipliers[j] * joint_strengths[k];
					float yz = j == 0 ? x : 0.0f;							// the 'knees' and 'ankes' can only rotate and torque on one axis

					RegisterJoint( legs_array[j].joints[k] = CJoint( this, k == 0 ? imp->carapace : legs_array[j].bones[k - 1], legs_array[j].bones[k], x, yz, yz) ); 
				}
		}

		//for(unsigned int i = 0; i < all_joints.size(); ++i)
		//	all_joints[i]->sjc->enable_motor = true;
	}
}
