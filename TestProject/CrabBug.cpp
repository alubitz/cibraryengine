#include "StdAfx.h"
#include "CrabBug.h"

#include "ConverterWhiz.h"

namespace Test
{
	using namespace std;

	/*
	 * CrabBug constants
	 */
	float bug_leap_duration = 0.5f;
	float bug_leap_speed = 8.0f;




	void GenerateHardCodedWalkAnimation(KeyframeAnimation* ka)
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




	/*
	 * CrabBug methods
	 */
	CrabBug::CrabBug(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		walk_pose(new WalkPose())
	{
		hp *= 0.5f;

		// character animation stuff
		pose_character->active_poses.push_back(walk_pose);

		// ik_pose->AddEndEffector("l leg a 3", Vec3(	0.27f,	0,	1.29f	), true);
		// ik_pose->AddEndEffector("r leg a 3", Vec3(	-0.27f,	0,	1.29f	), false);
		// ik_pose->AddEndEffector("l leg b 3", Vec3(	1.98f,	0,	0.44f	), false);
		// ik_pose->AddEndEffector("r leg b 3", Vec3(	-1.98f,	0,	0.44f	), true);
		// ik_pose->AddEndEffector("l leg c 3", Vec3(	0.80f,	0,	-1.36f	), true);
		// ik_pose->AddEndEffector("r leg c 3", Vec3(	-0.80f,	0,	-1.36f	), false);
	}

	void CrabBug::DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward)
	{
		if (standing > 0 && control_state->GetBoolControl("leap") && time.total > jump_start_timer)
		{
			// crab bug leaps forward
			float leap_angle = 0.4f;
			Vec3 leap_vector = (forward * (cosf(leap_angle)) + Vec3(0, sinf(leap_angle), 0));
			for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
				root_rigid_body->ApplyCentralImpulse(leap_vector * ((*iter)->GetMassInfo().mass) * bug_leap_speed);

			jump_start_timer = time.total + bug_leap_duration;
		}
	}

	void CrabBug::PreUpdatePoses(TimingInfo time)
	{
		walk_pose->pos = pos;
		walk_pose->yaw = yaw;
	}

	void CrabBug::Update(TimingInfo time)
	{
#if 0
		if(time.total > 5.0f)
			Die(Damage());
#endif

		Dood::Update(time);
	}




	void CrabBug::GetBoneEntries(vector<BoneEntry>& bone_entries)
	{
		//push bones here
		vector<Sphere> carapace_spheres;
		carapace_spheres.push_back(Sphere(	Vec3(	0.0f,	1.07f,	0.56f),		0.10f));
		carapace_spheres.push_back(Sphere(	Vec3(	0.0f,	1.18f,	-0.34f),	0.15f));
		carapace_spheres.push_back(Sphere(	Vec3(	0.0f,	1.04f,	-0.63f),	0.15f));
		carapace_spheres.push_back(Sphere(	Vec3(	0.38f,	1.00f,	-0.55f),	0.12f));
		carapace_spheres.push_back(Sphere(	Vec3(	0.59f,	1.02f,	-0.09f),	0.15f));
		carapace_spheres.push_back(Sphere(	Vec3(	-0.38f,	1.00f,	-0.55f),	0.12f));
		carapace_spheres.push_back(Sphere(	Vec3(	-0.59f,	1.02f,	-0.09f),	0.15f));

		bone_entries.push_back(BoneEntry("carapace",	"",				Vec3(	0.0f,	0.94f,	-0.03f),	carapace_spheres, 15));
		bone_entries.push_back(BoneEntry("crabhead",	"carapace",		Vec3(	0.0f,	1.02f,	0.48f),		0.10f,	Vec3(	0.0f,	0.75f,	0.16f	),	0.03f,	3));
		bone_entries.push_back(BoneEntry("tail",		"carapace",		Vec3(	0.0f,	0.94f,	-0.74f),	0.10f,	Vec3(	0.0f,	0.45f,	-0.41f	),	0.02f,	5));
		bone_entries.push_back(BoneEntry("l leg a 1",	"carapace",		Vec3(	0.31f,	0.84f,	0.43f),		0.10f,	Vec3(	0.40f,	0.55f,	0.58f	),	0.07f,	3));
		bone_entries.push_back(BoneEntry("l leg a 2",	"l leg a 1",	Vec3(	0.40f,	0.55f,	0.58f),		0.10f,	Vec3(	0.43f,	0.71f,	0.99f	),	0.07f,	4));
		bone_entries.push_back(BoneEntry("l leg a 3",	"l leg a 2",	Vec3(	0.43f,	0.71f,	0.99f),		0.10f,	Vec3(	0.30f,	0.05f,	1.28f	),	0.05f,	4));
		bone_entries.push_back(BoneEntry("l leg b 1",	"carapace",		Vec3(	0.73f,	0.90f,	0.06f),		0.10f,	Vec3(	1.06f,	0.72f,	0.19f	),	0.07f,	4));
		bone_entries.push_back(BoneEntry("l leg b 2",	"l leg b 1",	Vec3(	1.06f,	0.72f,	0.19f),		0.10f,	Vec3(	1.66f,	1.07f,	0.34f	),	0.07f,	7));
		bone_entries.push_back(BoneEntry("l leg b 3",	"l leg b 2",	Vec3(	1.66f,	1.07f,	0.34f),		0.10f,	Vec3(	1.94f,	0.05f,	0.43f	),	0.05f,	5));
		bone_entries.push_back(BoneEntry("l leg c 1",	"carapace",		Vec3(	0.47f,	0.82f,	-0.48f),	0.10f,	Vec3(	0.55f,	0.68f,	-0.69f	),	0.07f,	3));
		bone_entries.push_back(BoneEntry("l leg c 2",	"l leg c 1",	Vec3(	0.55f,	0.68f,	-0.69f),	0.10f,	Vec3(	0.69f,	0.77f,	-1.07f	),	0.07f,	3));
		bone_entries.push_back(BoneEntry("l leg c 3",	"l leg c 2",	Vec3(	0.69f,	0.77f,	-1.07f),	0.10f,	Vec3(	0.79f,	0.05f,	-1.32f	),	0.05f,	4));
		bone_entries.push_back(BoneEntry("r leg a 1",	"carapace",		Vec3(	-0.31f,	0.84f,	0.43f),		0.10f,	Vec3(	-0.40f,	0.55f,	0.58f	),	0.07f,	3));
		bone_entries.push_back(BoneEntry("r leg a 2",	"r leg a 1",	Vec3(	-0.40f,	0.55f,	0.58f),		0.10f,	Vec3(	-0.43f,	0.71f,	0.99f	),	0.07f,	4));
		bone_entries.push_back(BoneEntry("r leg a 3",	"r leg a 2",	Vec3(	-0.43f,	0.71f,	0.99f),		0.10f,	Vec3(	-0.30f,	0.05f,	1.28f	),	0.05f,	4));
		bone_entries.push_back(BoneEntry("r leg b 1",	"carapace",		Vec3(	-0.73f,	0.90f,	0.06f),		0.10f,	Vec3(	-1.06f,	0.72f,	0.19f	),	0.07f,	4));
		bone_entries.push_back(BoneEntry("r leg b 2",	"r leg b 1",	Vec3(	-1.06f,	0.72f,	0.19f),		0.10f,	Vec3(	-1.66f,	1.07f,	0.34f	),	0.07f,	7));
		bone_entries.push_back(BoneEntry("r leg b 3",	"r leg b 2",	Vec3(	-1.66f,	1.07f,	0.34f),		0.10f,	Vec3(	-1.94f,	0.05f,	0.43f	),	0.05f,	5));
		bone_entries.push_back(BoneEntry("r leg c 1",	"carapace",		Vec3(	-0.47f,	0.82f,	-0.48f),	0.10f,	Vec3(	-0.55f,	0.68f,	-0.69f	),	0.07f,	3));
		bone_entries.push_back(BoneEntry("r leg c 2",	"r leg c 1",	Vec3(	-0.55f,	0.68f,	-0.69f),	0.10f,	Vec3(	-0.69f,	0.77f,	-1.07f	),	0.07f,	4));
		bone_entries.push_back(BoneEntry("r leg c 3",	"r leg c 2",	Vec3(	-0.69f,	0.77f,	-1.07f),	0.10f,	Vec3(	-0.79f,	0.05f,	-1.32f	),	0.05f,	4));
	}
}
