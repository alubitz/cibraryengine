#include "StdAfx.h"
#include "SoldierConverter.h"

#include "ConverterWhiz.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	void ConvertSoldier(ContentMan* content)
	{
		vector<BoneEntry> bones = vector<BoneEntry>();
		bones.push_back(BoneEntry("pelvis",		"",				Vec3(	0,		1,		0		)));
		bones.push_back(BoneEntry("torso 1",	"pelvis",		Vec3(	0,		1.25f,	0.05f	)));
		bones.push_back(BoneEntry("torso 2",	"torso 1",		Vec3(	0,		1.32f,	0.05f	)));
		bones.push_back(BoneEntry("torso 3",	"torso 2",		Vec3(	0,		1.65f,	0		)));
		bones.push_back(BoneEntry("head",		"torso 3",		Vec3(	0,		1.7f,	0.05f	)));
		bones.push_back(BoneEntry("l shoulder",	"torso 3",		Vec3(	0.27f,	1.6f,	0		)));
		bones.push_back(BoneEntry("l arm 1",	"l shoulder",	Vec3(	0.33f,	1.43f,	0		)));
		bones.push_back(BoneEntry("l arm 2",	"l arm 1",		Vec3(	0.47f,	1.22f,	0		)));
		bones.push_back(BoneEntry("l hand",		"l arm 2",		Vec3(	0.6f,	1.03f,	0		)));
		bones.push_back(BoneEntry("l leg 1",	"pelvis",		Vec3(	0.14f,	0.88f,	0		)));
		bones.push_back(BoneEntry("l leg 2",	"l leg 1",		Vec3(	0.22f,	0.45f,	0		)));
		bones.push_back(BoneEntry("l foot",		"l leg 2",		Vec3(	0.28f,	0.08f,	0.06f	)));
		bones.push_back(BoneEntry("r shoulder",	"torso 3",		Vec3(	-0.27f,	1.6f,	0		)));
		bones.push_back(BoneEntry("r arm 1",	"r shoulder",	Vec3(	-0.33f,	1.43f,	0		)));
		bones.push_back(BoneEntry("r arm 2",	"r arm 1",		Vec3(	-0.47f,	1.22f,	0		)));
		bones.push_back(BoneEntry("r hand",		"r arm 2",		Vec3(	-0.6f,	1.03f,	0		)));
		bones.push_back(BoneEntry("r leg 1",	"pelvis",		Vec3(	-0.14f,	0.88f,	0		)));
		bones.push_back(BoneEntry("r leg 2",	"r leg 1",		Vec3(	-0.22f,	0.45f,	0		)));
		bones.push_back(BoneEntry("r foot",		"r leg 2",		Vec3(	-0.28f,	0.08f,	0.06f	)));

		Cache<VertexBuffer>* vtn_cache = content->GetCache<VertexBuffer>();
		for(unsigned int i = 0; i < bones.size(); i++)
		{
			BoneEntry& bone = bones[i];
			bone.model = vtn_cache->Load(bone.name);
		}

		SkinnedModel* model = SkinnedModel::WrapVertexBuffer(vtn_cache->Load("soldier"), "soldier");

		Skeleton* skeleton = model->skeleton = new Skeleton();

		for(unsigned int i = 0; i < bones.size(); i++)
		{
			BoneEntry& bone = bones[i];
			skeleton->AddBone(bone.name, Quaternion::Identity(), bone.pos);
		}

		for(unsigned int i = 0; i < bones.size(); i++)
		{
			BoneEntry& bone = bones[i];

			bone.model = vtn_cache->Load(bone.name);

			string& parent_name = bone.parent;
			if(parent_name != "")
				for(unsigned int j = 0; j < skeleton->bones.size(); j++)
				{
					Bone* parent = skeleton->bones[j];
					if(parent->name == parent_name)
					{
						skeleton->bones[i]->parent = parent;
						break;
					}
				}
		}

		for(vector<MaterialModelPair>::iterator iter = model->material_model_pairs.begin(); iter != model->material_model_pairs.end(); iter++)
		{
			VertexBuffer* vbo = (VertexBuffer*)iter->vbo;
			for(unsigned int jter = 0; jter < vbo->GetNumVerts(); jter++)
			{
				SkinVInfo vertex_info = GetSkinVInfo(vbo, jter);
				Vec3 pos = vertex_info.x;

				VertexBoneWeightInfo vbwi = VertexBoneWeightInfo();

				for(unsigned int bone_index = 0; bone_index < bones.size(); bone_index++)
				{
					BoneEntry& bone_entry = bones[bone_index];
					VertexBuffer* model_vbo = bone_entry.model;

					for(unsigned int lter = 0; lter < model_vbo->GetNumVerts(); lter++)
					{
						float dist_sq = (GetVTNTT(model_vbo, lter).x - pos).ComputeMagnitudeSquared();
						if(dist_sq < 0.01)
							vbwi.AddValue(bone_index, 1.0f / (dist_sq + 0.000001f));
					}
				}

				unsigned char indices[4];
				unsigned char weights[4];
				vbwi.GetByteValues(indices, weights);
				for(int k = 0; k < 4; k++)
				{
					vertex_info.indices[k] = indices[k];
					vertex_info.weights[k] = weights[k];
				}
			}
		}

		FixSoldierBones(skeleton);

		SaveAAK("Files/Models/soldier.aak", model, true);
	}

	void FixSoldierBones(Skeleton* skel)
	{
		map<string, Vec3> joint_positions = map<string, Vec3>();

		joint_positions["pelvis"]		= Vec3(	0.0f,	1.12f,	0.03f	);
		joint_positions["torso 1"]		= Vec3(	0.0f,	1.21f,	0.02f	);
		joint_positions["torso 2"]		= Vec3(	0.0f,	1.28f,	0.05f	);
		joint_positions["torso 3"]		= Vec3(	0.0f,	1.40f,	0.05f	);
		joint_positions["head"]			= Vec3(	0.0f,	1.75f,	0.03f	);

		joint_positions["l shoulder"]	= Vec3(	0.19f,	1.54f,	0.01f	);
		joint_positions["l arm 1"]		= Vec3(	0.26f,	1.51f,	0.01f	);
		joint_positions["l arm 2"]		= Vec3(	0.39f,	1.35f,	0.01f	);
		joint_positions["l hand"]		= Vec3(	0.52f,	1.12f,	0.02f	);
		joint_positions["l leg 1"]		= Vec3(	0.10f,	1.09f,	0.02f	);
		joint_positions["l leg 2"]		= Vec3(	0.19f,	0.63f,	-0.01f	);
		joint_positions["l foot"]		= Vec3(	0.25f,	0.23f,	-0.03f	);

		joint_positions["r shoulder"]	= Vec3(	-0.19f,	1.54f,	0.01f	);
		joint_positions["r arm 1"]		= Vec3(	-0.26f,	1.51f,	0.01f	);
		joint_positions["r arm 2"]		= Vec3(	-0.39f,	1.35f,	0.01f	);
		joint_positions["r hand"]		= Vec3(	-0.52f,	1.12f,	0.02f	);
		joint_positions["r leg 1"]		= Vec3(	-0.10f,	1.09f,	0.02f	);
		joint_positions["r leg 2"]		= Vec3(	-0.19f,	0.63f,	-0.01f	);
		joint_positions["r foot"]		= Vec3(	-0.25f,	0.23f,	-0.03f	);

		for(vector<Bone*>::iterator iter = skel->bones.begin(); iter != skel->bones.end(); iter++)
		{
			Bone* bone = *iter;

			map<string, Vec3>::iterator found = joint_positions.find(bone->name);
			if(found != joint_positions.end())
				bone->rest_pos = found->second;
		}
	}
}
