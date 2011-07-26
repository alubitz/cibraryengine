#include "SoldierConverter.h"

#include "../CibraryEngine/CibraryEngine.h"
#include "ConverterWhiz.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	void ConvertSoldier(ContentMan* content)
	{
		vector<BoneEntry> bones = vector<BoneEntry>();
		bones.push_back(BoneEntry("pelvis",		"",				Vec3(0,		1,		0)));
		bones.push_back(BoneEntry("torso 1",	"pelvis",		Vec3(0,		1.25,	0.05)));
		bones.push_back(BoneEntry("torso 2",	"torso 1",		Vec3(0,		1.32,	0.05)));
		bones.push_back(BoneEntry("torso 3",	"torso 2",		Vec3(0,		1.65,	0)));
		bones.push_back(BoneEntry("head",		"torso 3",		Vec3(0,		1.7,	0.05)));
		bones.push_back(BoneEntry("l shoulder",	"torso 3",		Vec3(0.27,	1.6,	0)));
		bones.push_back(BoneEntry("l arm 1",	"l shoulder",	Vec3(0.33,	1.43,	0)));
		bones.push_back(BoneEntry("l arm 2",	"l arm 1",		Vec3(0.47,	1.22,	0)));
		bones.push_back(BoneEntry("l hand",		"l arm 2",		Vec3(0.6,	1.03,	0)));
		bones.push_back(BoneEntry("l leg 1",	"pelvis",		Vec3(0.14,	0.88,	0)));
		bones.push_back(BoneEntry("l leg 2",	"l leg 1",		Vec3(0.22,	0.45,	0)));
		bones.push_back(BoneEntry("l foot",		"l leg 2",		Vec3(0.28,	0.08,	0.06)));
		bones.push_back(BoneEntry("r shoulder",	"torso 3",		Vec3(-0.27,	1.6,	0)));
		bones.push_back(BoneEntry("r arm 1",	"r shoulder",	Vec3(-0.33,	1.43,	0)));
		bones.push_back(BoneEntry("r arm 2",	"r arm 1",		Vec3(-0.47,	1.22,	0)));
		bones.push_back(BoneEntry("r hand",		"r arm 2",		Vec3(-0.6,	1.03,	0)));
		bones.push_back(BoneEntry("r leg 1",	"pelvis",		Vec3(-0.14,	0.88,	0)));
		bones.push_back(BoneEntry("r leg 2",	"r leg 1",		Vec3(-0.22,	0.45,	0)));
		bones.push_back(BoneEntry("r foot",		"r leg 2",		Vec3(-0.28,	0.08,	0.06)));

		for(unsigned int i = 0; i < bones.size(); i++)
		{
			BoneEntry& bone = bones[i];
			bone.model = content->Load<VTNModel>(bone.name);
		}

		SkinnedModel* model = SkinnedModel::CopyVTNModel(content->Load<VTNModel>("soldier"), "soldier");

		Skeleton* skeleton = model->skeleton = new Skeleton();

		for(unsigned int i = 0; i < bones.size(); i++)
		{
			BoneEntry& bone = bones[i];
			skeleton->AddBone(bone.name, Quaternion::Identity(), bone.pos);
		}

		for(unsigned int i = 0; i < bones.size(); i++)
		{
			BoneEntry& bone = bones[i];

			bone.model = content->Load<VTNModel>(bone.name);

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
			SkinVInfoVertexBuffer* vbo = (SkinVInfoVertexBuffer*)iter->vbo;
			for(vector<SkinVInfo>::iterator jter = vbo->vertex_infos.begin(); jter != vbo->vertex_infos.end(); jter++)
			{
				SkinVInfo& vertex_info = *jter;
				Vec3 pos = vertex_info.x;

				VertexBoneWeightInfo vbwi = VertexBoneWeightInfo();

				for(unsigned int bone_index = 0; bone_index < bones.size(); bone_index++)
				{
					BoneEntry& bone_entry = bones[bone_index];
					VUVNTTCVertexBuffer* model_vbo = bone_entry.model->GetVBO();

					for(vector<VUVNTTC>::iterator lter = model_vbo->vertex_infos.begin(); lter != model_vbo->vertex_infos.end(); lter++)
					{
						float dist_sq = (lter->x - pos).ComputeMagnitudeSquared();
						if(dist_sq < 0.01)
							vbwi.AddValue(bone_index, 1.0 / (dist_sq + 0.000001));
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

		joint_positions["pelvis"]		= Vec3(	0.00,	1.12,	0.03);
		joint_positions["torso 1"]		= Vec3(	0.00,	1.21,	0.02);
		joint_positions["torso 2"]		= Vec3(	0.00,	1.28,	0.05);
		joint_positions["torso 3"]		= Vec3(	0.00,	1.40,	0.05);
		joint_positions["head"]			= Vec3(	0.00,	1.75,	0.03);

		joint_positions["l shoulder"]	= Vec3(	0.19,	1.54,	0.01);
		joint_positions["l arm 1"]		= Vec3(	0.26,	1.51,	0.01);
		joint_positions["l arm 2"]		= Vec3(	0.39,	1.35,	0.01);
		joint_positions["l hand"]		= Vec3(	0.52,	1.12,	0.02);
		joint_positions["l leg 1"]		= Vec3(	0.10,	1.09,	0.02);
		joint_positions["l leg 2"]		= Vec3(	0.19,	0.63,	-0.01);
		joint_positions["l foot"]		= Vec3(	0.25,	0.23,	-0.03);

		joint_positions["r shoulder"]	= Vec3(	-0.19,	1.54,	0.01);
		joint_positions["r arm 1"]		= Vec3(	-0.26,	1.51,	0.01);
		joint_positions["r arm 2"]		= Vec3(	-0.39,	1.35,	0.01);
		joint_positions["r hand"]		= Vec3(	-0.52,	1.12,	0.02);
		joint_positions["r leg 1"]		= Vec3(	-0.10,	1.09,	0.02);
		joint_positions["r leg 2"]		= Vec3(	-0.19,	0.63,	-0.01);
		joint_positions["r foot"]		= Vec3(	-0.25,	0.23,	-0.03);

		for(vector<Bone*>::iterator iter = skel->bones.begin(); iter != skel->bones.end(); iter++)
		{
			Bone* bone = *iter;

			map<string, Vec3>::iterator found = joint_positions.find(bone->name);
			if(found != joint_positions.end())
				bone->rest_pos = found->second;
		}
	}
}
