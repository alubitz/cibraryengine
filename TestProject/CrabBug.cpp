#include "StdAfx.h"
#include "CrabBug.h"

#include "WalkPose.h"

#define DIE_AFTER_ONE_SECOND 0

#define ENABLE_WALK_ANIMATIONS 1

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

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	1,	0.3f,	0	), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	0,	-1,		0	), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	1,	0.5f,	0	), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	0,	0.3f,	0	), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	1,	-1,		0	), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	0,	0.5f,	0	), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 2;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	0,	0.3f,	0	), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	1,	-1,		0	), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	0,	0.5f,	0	), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	1,	0.3f,	0	), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	0,	-1,		0	), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	1,	0.5f,	0	), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	0,	-0.3f,	0	), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	1,	1,		0	), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	0,	-0.5f,	0	), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	1,	-0.3f,	0	), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	0,	1,		0	), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	1,	-0.5f,	0	), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	1,	-0.3f,	0	), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	0,	1,		0	), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	1,	-0.5f,	0	), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	0,	-0.3f,	0	), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	1,	1,		0	), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	0,	-0.5f,	0	), Vec3());

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
	 * CrabBug methods
	 */
	CrabBug::CrabBug(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
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

	void CrabBug::PreUpdatePoses(const TimingInfo& time) { crab_heading->yaw = yaw; }

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
		feet.push_back(new FootState(Bone::string_table["l leg a 3"]));
		feet.push_back(new FootState(Bone::string_table["r leg a 3"]));
		feet.push_back(new FootState(Bone::string_table["l leg b 3"]));
		feet.push_back(new FootState(Bone::string_table["r leg b 3"]));
		feet.push_back(new FootState(Bone::string_table["l leg c 3"]));
		feet.push_back(new FootState(Bone::string_table["r leg c 3"]));
	}
}
