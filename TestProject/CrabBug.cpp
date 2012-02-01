#include "StdAfx.h"
#include "CrabBug.h"

namespace Test
{
	using namespace std;

	/*
	 * CrabBug constants
	 */
	float bug_leap_duration = 0.5f;
	float bug_leap_speed = 8.0f;




	void GenerateHardCodedWalkAnimation(IKPose* ik_pose)
	{
		KeyframeAnimation* ka = ik_pose->keyframe_animation;

		{
			Keyframe kf(0.5f);
			kf.next = 1;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	1,	0.3f,	0	), Vec3(), 1);
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	0,	-1,		0	), Vec3(), 1);
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	1,	0.5f,	0	), Vec3(), 1);

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	0,	0.3f,	0	), Vec3(), 1);
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	1,	-1,		0	), Vec3(), 1);
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	0,	0.5f,	0	), Vec3(), 1);

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 2;
			
			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	0,	0.3f,	0	), Vec3(), 1);
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	1,	-1,		0	), Vec3(), 1);
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	0,	0.5f,	0	), Vec3(), 1);

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	1,	0.3f,	0	), Vec3(), 1);
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	0,	-1,		0	), Vec3(), 1);
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	1,	0.5f,	0	), Vec3(), 1);

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	0,	-0.3f,	0	), Vec3(), 1);
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	1,	1,		0	), Vec3(), 1);
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	0,	-0.5f,	0	), Vec3(), 1);

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	1,	-0.3f,	0	), Vec3(), 1);
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	0,	1,		0	), Vec3(), 1);
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	1,	-0.5f,	0	), Vec3(), 1);
			
			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	1,	-0.3f,	0	), Vec3(), 1);
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	0,	1,		0	), Vec3(), 1);
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	1,	-0.5f,	0	), Vec3(), 1);

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	0,	-0.3f,	0	), Vec3(), 1);
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	1,	1,		0	), Vec3(), 1);
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	0,	-0.5f,	0	), Vec3(), 1);
			
			ka->frames.push_back(kf);
		}
	}




	/*
	 * CrabBug methods
	 */
	CrabBug::CrabBug(GameState* game_state, UberModel* model, Vec3 pos, Team& team) :
		Dood(game_state, model, pos, team),
		walk_pose()
	{
		hp *= 0.5f;

		// character animation stuff
		character->active_poses.push_back(&walk_pose);

		ik_pose = new IKPose(game_state, character->skeleton, pos, pitch, yaw);
		GenerateHardCodedWalkAnimation(ik_pose);

		ik_pose->AddEndEffector("l leg a 3", Vec3(	0.27f,	0,	1.29f	), true);
		ik_pose->AddEndEffector("r leg a 3", Vec3(	-0.27f,	0,	1.29f	), false);
		ik_pose->AddEndEffector("l leg b 3", Vec3(	1.98f,	0,	0.44f	), false);
		ik_pose->AddEndEffector("r leg b 3", Vec3(	-1.98f,	0,	0.44f	), true);
		ik_pose->AddEndEffector("l leg c 3", Vec3(	0.80f,	0,	-1.36f	), true);
		ik_pose->AddEndEffector("r leg c 3", Vec3(	-0.80f,	0,	-1.36f	), false);

		character->active_poses.push_back(ik_pose);
	}

	void CrabBug::InnerDispose()
	{
		delete ik_pose;
		ik_pose = NULL;

		Dood::InnerDispose();
	}

	void CrabBug::DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward)
	{
		if (standing > 0 && control_state->GetBoolControl("leap") && time.total > jump_start_timer)
		{
			// crab bug leaps forward
			float leap_angle = 0.4f;
			Vec3 leap_vector = (forward * (cosf(leap_angle)) + Vec3(0, sinf(leap_angle), 0)) * (mass * bug_leap_speed);
			rigid_body->body->applyCentralImpulse(btVector3(leap_vector.x, leap_vector.y, leap_vector.z));

			jump_start_timer = time.total + bug_leap_duration;
		}
	}

	void CrabBug::PreUpdatePoses(TimingInfo time)
	{
		walk_pose.pos = pos;
		walk_pose.yaw = yaw;
	}

	void CrabBug::Update(TimingInfo time)
	{
		pos = ik_pose->pos;
		yaw = ik_pose->yaw;
		pitch = ik_pose->pitch;

		Dood::Update(time);

		ik_pose->SetDesiredState(game_state->ik_solver, pos, pitch, yaw);
	}
}
