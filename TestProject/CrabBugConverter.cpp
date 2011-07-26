#include "CrabBugConverter.h"

#include "../CibraryEngine/CibraryEngine.h"
#include "ConverterWhiz.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	void ConvertCrabBug(ContentMan* content)
	{
		vector<BoneEntry> bones = vector<BoneEntry>();

		//push bones here
		vector<Sphere> carapace_spheres;
		carapace_spheres.push_back(Sphere(	Vec3(0.0,	1.07,	0.56),	0.10));
		carapace_spheres.push_back(Sphere(	Vec3(0.0,	1.18,	-0.34),	0.15));
		carapace_spheres.push_back(Sphere(	Vec3(0.0,	1.04,	-0.63),	0.15));
		carapace_spheres.push_back(Sphere(	Vec3(0.38,	1.00,	-0.55),	0.12));
		carapace_spheres.push_back(Sphere(	Vec3(0.59,	1.02,	-0.09),	0.15));
		carapace_spheres.push_back(Sphere(	Vec3(-0.38,	1.00,	-0.55),	0.12));
		carapace_spheres.push_back(Sphere(	Vec3(-0.59,	1.02,	-0.09),	0.15));

		bones.push_back(BoneEntry("carapace",		"",				Vec3(0.0,	0.94,	-0.03),	carapace_spheres, 15));
		bones.push_back(BoneEntry("crabhead",		"carapace",		Vec3(0.0,	1.02,	0.48),		0.10,			Vec3(0.0,	0.75,	0.16),		0.03,	3));
		bones.push_back(BoneEntry("tail",			"carapace",		Vec3(0.0,	0.94,	-0.74),		0.10,			Vec3(0.0,	0.45,	-0.41),		0.02,	5));
		bones.push_back(BoneEntry("l leg a 1",		"carapace",		Vec3(0.31,	0.84,	0.43),		0.10,			Vec3(0.40,	0.55,	0.58),		0.07,	3));
		bones.push_back(BoneEntry("l leg a 2",		"l leg a 1",	Vec3(0.40,	0.55,	0.58),		0.10,			Vec3(0.43,	0.71,	0.99),		0.07,	4));
		bones.push_back(BoneEntry("l leg a 3",		"l leg a 2",	Vec3(0.43,	0.71,	0.99),		0.10,			Vec3(0.30,	0.05,	1.28),		0.05,	4));
		bones.push_back(BoneEntry("l leg b 1",		"carapace",		Vec3(0.73,	0.90,	0.06),		0.10,			Vec3(1.06,	0.72,	0.19),		0.07,	4));
		bones.push_back(BoneEntry("l leg b 2",		"l leg b 1",	Vec3(1.06,	0.72,	0.19),		0.10,			Vec3(1.66,	1.07,	0.34),		0.07,	7));
		bones.push_back(BoneEntry("l leg b 3",		"l leg b 2",	Vec3(1.66,	1.07,	0.34),		0.10,			Vec3(1.94,	0.05,	0.43),		0.05,	5));
		bones.push_back(BoneEntry("l leg c 1",		"carapace",		Vec3(0.47,	0.82,	-0.48),		0.10,			Vec3(0.55,	0.68,	-0.69),		0.07,	3));
		bones.push_back(BoneEntry("l leg c 2",		"l leg c 1",	Vec3(0.55,	0.68,	-0.69),		0.10,			Vec3(0.69,	0.77,	-1.07),		0.07,	3));
		bones.push_back(BoneEntry("l leg c 3",		"l leg c 2",	Vec3(0.69,	0.77,	-1.07),		0.10,			Vec3(0.79,	0.05,	-1.32),		0.05,	4));
		bones.push_back(BoneEntry("r leg a 1",		"carapace",		Vec3(-0.31,	0.84,	0.43),		0.10,			Vec3(-0.40,	0.55,	0.58),		0.07,	3));
		bones.push_back(BoneEntry("r leg a 2",		"r leg a 1",	Vec3(-0.40,	0.55,	0.58),		0.10,			Vec3(-0.43,	0.71,	0.99),		0.07,	4));
		bones.push_back(BoneEntry("r leg a 3",		"r leg a 2",	Vec3(-0.43,	0.71,	0.99),		0.10,			Vec3(-0.30,	0.05,	1.28),		0.05,	4));
		bones.push_back(BoneEntry("r leg b 1",		"carapace",		Vec3(-0.73,	0.90,	0.06),		0.10,			Vec3(-1.06,	0.72,	0.19),		0.07,	4));
		bones.push_back(BoneEntry("r leg b 2",		"r leg b 1",	Vec3(-1.06,	0.72,	0.19),		0.10,			Vec3(-1.66,	1.07,	0.34),		0.07,	7));
		bones.push_back(BoneEntry("r leg b 3",		"r leg b 2",	Vec3(-1.66,	1.07,	0.34),		0.10,			Vec3(-1.94,	0.05,	0.43),		0.05,	5));
		bones.push_back(BoneEntry("r leg c 1",		"carapace",		Vec3(-0.47,	0.82,	-0.48),		0.10,			Vec3(-0.55,	0.68,	-0.69),		0.07,	3));
		bones.push_back(BoneEntry("r leg c 2",		"r leg c 1",	Vec3(-0.55,	0.68,	-0.69),		0.10,			Vec3(-0.69,	0.77,	-1.07),		0.07,	4));
		bones.push_back(BoneEntry("r leg c 3",		"r leg c 2",	Vec3(-0.69,	0.77,	-1.07),		0.10,			Vec3(-0.79,	0.05,	-1.32),		0.05,	4));

		for(unsigned int i = 0; i < bones.size(); i++)
		{
			BoneEntry& bone = bones[i];
			bone.model = content->Load<VTNModel>(bone.name);
		}

		SkinnedModel* model = SkinnedModel::CopyVTNModel(content->Load<VTNModel>("crab_bug"), "crab_bug");

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

		UberModel* uber = UberModelLoader::CopySkinnedModel(model);

		for(unsigned int i = 0; i < bones.size(); i++)
		{
			BoneEntry& entry = bones[i];

			UberModel::BonePhysics phys;

			phys.bone_name = entry.name;
			phys.mass = entry.mass;

			// generating shape from spheres list
			unsigned int num_spheres = entry.spheres.size();
			btVector3 sphere_positions[num_spheres];
			btScalar sphere_radii[num_spheres];
			for(unsigned int j = 0; j < num_spheres; j++)
			{
				Vec3 center = entry.spheres[j].center - entry.pos;
				sphere_positions[j] = btVector3(center.x, center.y, center.z);
				sphere_radii[j] = entry.spheres[j].radius;
			}
			phys.shape = new btMultiSphereShape(sphere_positions, sphere_radii, num_spheres);

			phys.pos = entry.pos;
			for(unsigned int j = 0; j < bones.size(); j++)
				if(bones[j].name == entry.parent)
				{
					phys.pos -= bones[j].pos;
					break;
				}

			uber->bone_physics.push_back(phys);
		}

		UberModelLoader::SaveZZZ(uber, "Files/Models/crab_bug.zzz");
	}
}
