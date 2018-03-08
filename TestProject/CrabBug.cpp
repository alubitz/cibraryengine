#include "StdAfx.h"
#include "CrabBug.h"

#include "WalkPose.h"

#include "CBone.h"
#include "CJoint.h"

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




	/*
	* CrabBug private implementation struct
	*/
	struct CrabBug::Imp
	{
		bool init;

		CBone carapace, head, tail;
		CBone llega1, llega2, llega3;
		CBone llegb1, llegb2, llegb3;
		CBone llegc1, llegc2, llegc3;
		CBone rlega1, rlega2, rlega3;
		CBone rlegb1, rlegb2, rlegb3;
		CBone rlegc1, rlegc2, rlegc3;

		CJoint neck, tailj;
		CJoint llja1, llja2, llja3;
		CJoint lljb1, lljb2, lljb3;
		CJoint lljc1, lljc2, lljc3;
		CJoint rlja1, rlja2, rlja3;
		CJoint rljb1, rljb2, rljb3;
		CJoint rljc1, rljc2, rljc3;

		float timestep, inv_timestep;

		unsigned int tick_age, max_tick_age;

		Imp() : init(false), timestep(0), inv_timestep(0), tick_age(0), max_tick_age(300) { }
		~Imp() { }

		void Init(CrabBug* dood)
		{
			init = true;
		}

		void Update(CrabBug* dood, const TimingInfo& time)
		{
			if(!init)
			{
				Init(dood);
				return;
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



			dood->DoScriptedMotorControl("Files/Scripts/crab_motor_control.lua");



			++tick_age;
		}
	};



	
	/*
	 * CrabBug methods
	 */
	CrabBug::CrabBug(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		imp(NULL),
		crab_heading(new CrabHeading())
	{
		hp *= 0.5f;

		yaw = Random3D::Rand(float(M_PI) * 2.0f);

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

		RegisterBone( imp->carapace = CBone(this, "carapace"  ));
		RegisterBone( imp->head     = CBone(this, "head"      ));
		RegisterBone( imp->tail     = CBone(this, "tail"      ));
		RegisterBone( imp->llega1   = CBone(this, "l leg a 1" ));
		RegisterBone( imp->llega2   = CBone(this, "l leg a 2" ));
		RegisterBone( imp->llega3   = CBone(this, "l leg a 3" ));
		RegisterBone( imp->llegb1   = CBone(this, "l leg b 1" ));
		RegisterBone( imp->llegb2   = CBone(this, "l leg b 2" ));
		RegisterBone( imp->llegb3   = CBone(this, "l leg b 3" ));
		RegisterBone( imp->llegc1   = CBone(this, "l leg c 1" ));
		RegisterBone( imp->llegc2   = CBone(this, "l leg c 2" ));
		RegisterBone( imp->llegc3   = CBone(this, "l leg c 3" ));
		RegisterBone( imp->rlega1   = CBone(this, "r leg a 1" ));
		RegisterBone( imp->rlega2   = CBone(this, "r leg a 2" ));
		RegisterBone( imp->rlega3   = CBone(this, "r leg a 3" ));
		RegisterBone( imp->rlegb1   = CBone(this, "r leg b 1" ));
		RegisterBone( imp->rlegb2   = CBone(this, "r leg b 2" ));
		RegisterBone( imp->rlegb3   = CBone(this, "r leg b 3" ));
		RegisterBone( imp->rlegc1   = CBone(this, "r leg c 1" ));
		RegisterBone( imp->rlegc2   = CBone(this, "r leg c 2" ));
		RegisterBone( imp->rlegc3   = CBone(this, "r leg c 3" ));
	}

	void CrabBug::InitJointHelpers()
	{
		Dood::InitJointHelpers();

		float N = 200, T = 150, J1 = 900, J2 = 700, J3 = 600;
		float A = 0.9f, B = 1.0f, C = 0.9f;

		RegisterJoint( imp->neck  = CJoint( this, imp->carapace, imp->head,   N      ));
		RegisterJoint( imp->tailj = CJoint( this, imp->carapace, imp->tail,   T      ));
		RegisterJoint( imp->llja1 = CJoint( this, imp->carapace, imp->llega1, J1 * A ));
		RegisterJoint( imp->llja2 = CJoint( this, imp->llega1,   imp->llega2, J2 * A ));
		RegisterJoint( imp->llja3 = CJoint( this, imp->llega2,   imp->llega3, J3 * A ));
		RegisterJoint( imp->lljb1 = CJoint( this, imp->carapace, imp->llegb1, J1 * B ));
		RegisterJoint( imp->lljb2 = CJoint( this, imp->llegb1,   imp->llegb2, J2 * B ));
		RegisterJoint( imp->lljb3 = CJoint( this, imp->llegb2,   imp->llegb3, J3 * B ));
		RegisterJoint( imp->lljc1 = CJoint( this, imp->carapace, imp->llegc1, J1 * C ));
		RegisterJoint( imp->lljc2 = CJoint( this, imp->llegc1,   imp->llegc2, J2 * C ));
		RegisterJoint( imp->lljc3 = CJoint( this, imp->llegc2,   imp->llegc3, J3 * C ));
		RegisterJoint( imp->rlja1 = CJoint( this, imp->carapace, imp->rlega1, J1 * A ));
		RegisterJoint( imp->rlja2 = CJoint( this, imp->rlega1,   imp->rlega2, J2 * A ));
		RegisterJoint( imp->rlja3 = CJoint( this, imp->rlega2,   imp->rlega3, J3 * A ));
		RegisterJoint( imp->rljb1 = CJoint( this, imp->carapace, imp->rlegb1, J1 * B ));
		RegisterJoint( imp->rljb2 = CJoint( this, imp->rlegb1,   imp->rlegb2, J2 * B ));
		RegisterJoint( imp->rljb3 = CJoint( this, imp->rlegb2,   imp->rlegb3, J3 * B ));
		RegisterJoint( imp->rljc1 = CJoint( this, imp->carapace, imp->rlegc1, J1 * C ));
		RegisterJoint( imp->rljc2 = CJoint( this, imp->rlegc1,   imp->rlegc2, J2 * C ));
		RegisterJoint( imp->rljc3 = CJoint( this, imp->rlegc2,   imp->rlegc3, J3 * C ));

		// TODO: set secondary- and tertiary-axis torque limits to zero for most bones
	}
}
