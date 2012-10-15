#include "StdAfx.h"
#include "ArtilleryBug.h"

#include "ConverterWhiz.h"

namespace Test
{
	/*
	 * ArtilleryBug methods
	 */
	ArtilleryBug::ArtilleryBug(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		walk_pose(new WalkPose())
	{
		posey->active_poses.push_back(walk_pose);
		character->mat_tex_precision = 1024.0f;

		foot_bones[Bone::string_table["l leg a 3"]] = NULL;
		foot_bones[Bone::string_table["r leg a 3"]] = NULL;
		foot_bones[Bone::string_table["l leg b 3"]] = NULL;
		foot_bones[Bone::string_table["r leg b 3"]] = NULL;
		foot_bones[Bone::string_table["l leg c 3"]] = NULL;
		foot_bones[Bone::string_table["r leg c 3"]] = NULL;

		vis_bs_radius = 25.0f;
	}

	void ArtilleryBug::GetBoneEntries(vector<BoneEntry>& bone_entries)
	{
		vector<Sphere> carapace_spheres;
		carapace_spheres.push_back(Sphere(Vec3(0.0f,	6.15f,	5.68f	),	2.2f));
		carapace_spheres.push_back(Sphere(Vec3(0.0f,	8.42f,	2.29f	),	4.0f));
		carapace_spheres.push_back(Sphere(Vec3(0.0f,	8.50f,	-2.51f	),	4.5f));
		carapace_spheres.push_back(Sphere(Vec3(0.0f,	9.08f,	-8.17f	),	3.2f));

		bone_entries.push_back(BoneEntry(	"carapace",		"",				Vec3(	0.0f,		0.0f,	0.0f	),															carapace_spheres,	200.0f	));		// body
		bone_entries.push_back(BoneEntry(	"llega1",		"carapace",		Vec3(	2.60f,		7.25f,	3.41f	),	1.2f,	Vec3(	3.85f,		5.59f,	5.32f	),	0.5f,						100.0f	));		// left legs
		bone_entries.push_back(BoneEntry(	"llega2",		"llega1",		Vec3(	3.85f,		5.59f,	5.32f	),	0.5f,	Vec3(	5.09f,		6.77f,	8.38f	),	0.5f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"llega3",		"llega2",		Vec3(	5.09f,		6.77f,	8.38f	),	0.5f,	Vec3(	6.68f,		0.2f,	12.27f	),	0.2f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"llegb1",		"carapace",		Vec3(	3.45f,		7.26f,	1.09f	),	1.2f,	Vec3(	6.69f,		5.76f,	2.33f	),	0.5f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"llegb2",		"llegb1",		Vec3(	6.69f,		5.76f,	2.33f	),	0.5f,	Vec3(	10.62f,		8.63f,	4.71f	),	0.5f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"llegb3",		"llegb2",		Vec3(	10.62f,		8.63f,	4.71f	),	0.5f,	Vec3(	16.38f,		0.2f,	8.56f	),	0.2f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"llegc1",		"carapace",		Vec3(	3.39f,		7.23f,	-1.09f	),	1.2f,	Vec3(	5.12f,		5.71f,	-3.53f	),	0.5f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"llegc2",		"llegc1",		Vec3(	5.12f,		5.71f,	-3.53f	),	0.5f,	Vec3(	5.88f,		10.23f,	-7.98f	),	0.5f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"llegc3",		"llegc2",		Vec3(	5.88f,		10.23f,	-7.98f	),	0.5f,	Vec3(	7.05f,		0.2f,	-15.31f	),	0.2f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"rlega1",		"carapace",		Vec3(	-2.60f,		7.25f,	3.41f	),	1.2f,	Vec3(	-3.85f,		5.59f,	5.32f	),	0.5f,						100.0f	));		// right legs
		bone_entries.push_back(BoneEntry(	"rlega2",		"rlega1",		Vec3(	-3.85f,		5.59f,	5.32f	),	0.5f,	Vec3(	-5.09f,		6.77f,	8.38f	),	0.5f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"rlega3",		"rlega2",		Vec3(	-5.09f,		6.77f,	8.38f	),	0.5f,	Vec3(	-6.68f,		0.2f,	12.27f	),	0.2f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"rlegb1",		"carapace",		Vec3(	-3.45f,		7.26f,	1.09f	),	1.2f,	Vec3(	-6.69f,		5.76f,	2.33f	),	0.5f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"rlegb2",		"rlegb1",		Vec3(	-6.69f,		5.76f,	2.33f	),	0.5f,	Vec3(	-10.62f,	8.63f,	4.71f	),	0.5f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"rlegb3",		"rlegb2",		Vec3(	-10.62f,	8.63f,	4.71f	),	0.5f,	Vec3(	-16.38f,	0.2f,	8.56f	),	0.2f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"rlegc1",		"carapace",		Vec3(	-3.39f,		7.23f,	-1.09f	),	1.2f,	Vec3(	-5.12f,		5.71f,	-3.53f	),	0.5f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"rlegc2",		"rlegc1",		Vec3(	-5.12f,		5.71f,	-3.53f	),	0.5f,	Vec3(	-5.88f,		10.23f,	-7.98f	),	0.5f,						100.0f	));
		bone_entries.push_back(BoneEntry(	"rlegc3",		"rlegc2",		Vec3(	-5.88f,		10.23f,	-7.98f	),	0.5f,	Vec3(	-7.05f,		0.2f,	-15.31f	),	0.2f,						100.0f	));
	}

	void ArtilleryBug::PreUpdatePoses(TimingInfo time)
	{
		walk_pose->pos = pos;
		walk_pose->yaw = yaw;
	}
}
