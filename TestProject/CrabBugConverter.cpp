#include "StdAfx.h"
#include "CrabBugConverter.h"

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
		carapace_spheres.push_back(Sphere(	Vec3(	0.0f,	1.07f,	0.56f),		0.10f));
		carapace_spheres.push_back(Sphere(	Vec3(	0.0f,	1.18f,	-0.34f),	0.15f));
		carapace_spheres.push_back(Sphere(	Vec3(	0.0f,	1.04f,	-0.63f),	0.15f));
		carapace_spheres.push_back(Sphere(	Vec3(	0.38f,	1.00f,	-0.55f),	0.12f));
		carapace_spheres.push_back(Sphere(	Vec3(	0.59f,	1.02f,	-0.09f),	0.15f));
		carapace_spheres.push_back(Sphere(	Vec3(	-0.38f,	1.00f,	-0.55f),	0.12f));
		carapace_spheres.push_back(Sphere(	Vec3(	-0.59f,	1.02f,	-0.09f),	0.15f));

		bones.push_back(BoneEntry("carapace",	"",				Vec3(	0.0f,	0.94f,	-0.03f),	carapace_spheres, 15));
		bones.push_back(BoneEntry("crabhead",	"carapace",		Vec3(	0.0f,	1.02f,	0.48f),		0.10f,	Vec3(	0.0f,	0.75f,	0.16f	),	0.03f,	3));
		bones.push_back(BoneEntry("tail",		"carapace",		Vec3(	0.0f,	0.94f,	-0.74f),	0.10f,	Vec3(	0.0f,	0.45f,	-0.41f	),	0.02f,	5));
		bones.push_back(BoneEntry("l leg a 1",	"carapace",		Vec3(	0.31f,	0.84f,	0.43f),		0.10f,	Vec3(	0.40f,	0.55f,	0.58f	),	0.07f,	3));
		bones.push_back(BoneEntry("l leg a 2",	"l leg a 1",	Vec3(	0.40f,	0.55f,	0.58f),		0.10f,	Vec3(	0.43f,	0.71f,	0.99f	),	0.07f,	4));
		bones.push_back(BoneEntry("l leg a 3",	"l leg a 2",	Vec3(	0.43f,	0.71f,	0.99f),		0.10f,	Vec3(	0.30f,	0.05f,	1.28f	),	0.05f,	4));
		bones.push_back(BoneEntry("l leg b 1",	"carapace",		Vec3(	0.73f,	0.90f,	0.06f),		0.10f,	Vec3(	1.06f,	0.72f,	0.19f	),	0.07f,	4));
		bones.push_back(BoneEntry("l leg b 2",	"l leg b 1",	Vec3(	1.06f,	0.72f,	0.19f),		0.10f,	Vec3(	1.66f,	1.07f,	0.34f	),	0.07f,	7));
		bones.push_back(BoneEntry("l leg b 3",	"l leg b 2",	Vec3(	1.66f,	1.07f,	0.34f),		0.10f,	Vec3(	1.94f,	0.05f,	0.43f	),	0.05f,	5));
		bones.push_back(BoneEntry("l leg c 1",	"carapace",		Vec3(	0.47f,	0.82f,	-0.48f),	0.10f,	Vec3(	0.55f,	0.68f,	-0.69f	),	0.07f,	3));
		bones.push_back(BoneEntry("l leg c 2",	"l leg c 1",	Vec3(	0.55f,	0.68f,	-0.69f),	0.10f,	Vec3(	0.69f,	0.77f,	-1.07f	),	0.07f,	3));
		bones.push_back(BoneEntry("l leg c 3",	"l leg c 2",	Vec3(	0.69f,	0.77f,	-1.07f),	0.10f,	Vec3(	0.79f,	0.05f,	-1.32f	),	0.05f,	4));
		bones.push_back(BoneEntry("r leg a 1",	"carapace",		Vec3(	-0.31f,	0.84f,	0.43f),		0.10f,	Vec3(	-0.40f,	0.55f,	0.58f	),	0.07f,	3));
		bones.push_back(BoneEntry("r leg a 2",	"r leg a 1",	Vec3(	-0.40f,	0.55f,	0.58f),		0.10f,	Vec3(	-0.43f,	0.71f,	0.99f	),	0.07f,	4));
		bones.push_back(BoneEntry("r leg a 3",	"r leg a 2",	Vec3(	-0.43f,	0.71f,	0.99f),		0.10f,	Vec3(	-0.30f,	0.05f,	1.28f	),	0.05f,	4));
		bones.push_back(BoneEntry("r leg b 1",	"carapace",		Vec3(	-0.73f,	0.90f,	0.06f),		0.10f,	Vec3(	-1.06f,	0.72f,	0.19f	),	0.07f,	4));
		bones.push_back(BoneEntry("r leg b 2",	"r leg b 1",	Vec3(	-1.06f,	0.72f,	0.19f),		0.10f,	Vec3(	-1.66f,	1.07f,	0.34f	),	0.07f,	7));
		bones.push_back(BoneEntry("r leg b 3",	"r leg b 2",	Vec3(	-1.66f,	1.07f,	0.34f),		0.10f,	Vec3(	-1.94f,	0.05f,	0.43f	),	0.05f,	5));
		bones.push_back(BoneEntry("r leg c 1",	"carapace",		Vec3(	-0.47f,	0.82f,	-0.48f),	0.10f,	Vec3(	-0.55f,	0.68f,	-0.69f	),	0.07f,	3));
		bones.push_back(BoneEntry("r leg c 2",	"r leg c 1",	Vec3(	-0.55f,	0.68f,	-0.69f),	0.10f,	Vec3(	-0.69f,	0.77f,	-1.07f	),	0.07f,	4));
		bones.push_back(BoneEntry("r leg c 3",	"r leg c 2",	Vec3(	-0.69f,	0.77f,	-1.07f),	0.10f,	Vec3(	-0.79f,	0.05f,	-1.32f	),	0.05f,	4));

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

		UberModel* uber = UberModelLoader::CopySkinnedModel(model);

		for(unsigned int i = 0; i < bones.size(); i++)
		{
			BoneEntry& entry = bones[i];

			UberModel::BonePhysics phys;

			phys.bone_name = entry.name;
			phys.mass = entry.mass;

			// generating shape from spheres list
			unsigned int num_spheres = entry.spheres.size();
			btVector3* sphere_positions = new btVector3[num_spheres];
			btScalar* sphere_radii = new btScalar[num_spheres];
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

			delete[] sphere_positions;
			delete[] sphere_radii;
		}

		UberModelLoader::SaveZZZ(uber, "Files/Models/crab_bug.zzz");
	}
}
