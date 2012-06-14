#include "StdAfx.h"
#include "ConverterWhiz.h"

namespace Test
{
	/*
	 * Functions for creating UberModels
	 */
	UberModel* AutoSkinUberModel(ContentMan* content, string vtn_name, string material, vector<BoneEntry>& bone_entries)
	{
		Cache<VertexBuffer>* vtn_cache = content->GetCache<VertexBuffer>();

		// could use a list of mmp-names?
		VertexBuffer* vtn = vtn_cache->Load(vtn_name);
		SkinnedModel* skinny = SkinnedModel::WrapVertexBuffer(vtn, material);

		vector<VertexBuffer*> submodel_vbos;
		for(vector<BoneEntry>::iterator iter = bone_entries.begin(); iter != bone_entries.end(); ++iter)
			submodel_vbos.push_back(vtn_cache->Load(iter->name));

		SkinnedModel::AutoSkinModel(skinny, submodel_vbos);

		return UberModelLoader::CopySkinnedModel(skinny);
	}

	void SetUberModelSkeleton(UberModel* uber, vector<BoneEntry>& bone_entries)
	{
		unsigned int num_bones = bone_entries.size();

		// create a lookup table, name of bone --> index in array
		unordered_map<string, unsigned int> bone_indices;
		for(unsigned int i = 0; i < num_bones; ++i)
			bone_indices[bone_entries[i].name] = i;

		// now make the changes to the UberModel
		uber->bones.clear();
		for(vector<BoneEntry>::iterator iter = bone_entries.begin(); iter != bone_entries.end(); ++iter)
		{
			UberModel::Bone bone;

			unordered_map<string, unsigned int>::iterator found = bone_indices.find(iter->parent);
			if(found != bone_indices.end())
				bone.parent = found->second + 1;
			else
				bone.parent = 0;

			bone.name = iter->name;
			bone.pos = iter->pos;
			bone.ori = Quaternion::Identity();

			uber->bones.push_back(bone);
		}
	}

	ModelPhysics* ModelPhysicsFromBoneEntries(vector<BoneEntry>& bone_entries)
	{
		unsigned int num_bones = bone_entries.size();

		// create a lookup table, name of bone --> index in array
		unordered_map<string, unsigned int> bone_indices;
		for(unsigned int i = 0; i < num_bones; ++i)
			bone_indices[bone_entries[i].name] = i;

		ModelPhysics result;

		for(vector<BoneEntry>::iterator iter = bone_entries.begin(); iter != bone_entries.end(); ++iter)
		{
			// create the bone
			ModelPhysics::BonePhysics bone;

			bone.bone_name = iter->name;

			if(unsigned int n_spheres = iter->spheres.size())
			{
				Sphere* spheres = new Sphere[n_spheres];

				for(unsigned int i = 0; i < n_spheres; ++i)
				{
					const Sphere& sphere = iter->spheres[i];
					spheres[i] = Sphere(sphere.center - iter->pos, sphere.radius);			// collision shape coordinates are relative to bone's point of attachment
				}

				bone.collision_shape = new MultiSphereShape(spheres, n_spheres);

				MassInfo mass_info = bone.collision_shape->ComputeMassInfo();

				bone.mass_info = mass_info * (iter->mass / mass_info.mass);

				delete[] spheres;
			}

			bone_indices[iter->name] = result.bones.size();
			result.bones.push_back(bone);

			// now create the joint that connects it to its parent bone, if there is one
			if(!iter->parent.empty())
			{
				ModelPhysics::JointPhysics joint;

				joint.joint_name = "";
				joint.bone_a = bone_indices[iter->name] + 1;
				joint.bone_b = bone_indices[iter->parent] + 1;

				joint.pos = iter->pos;

				// some rather arbitrary default values here
				joint.axes = Mat3::Identity();
				joint.max_extents = Vec3(1.0f, 1.0f, 1.0f);
				joint.angular_damp = Vec3(1.0, 1.0f, 1.0f);

				result.joints.push_back(joint);
			}
		}

		return new ModelPhysics(result);
	}
}
