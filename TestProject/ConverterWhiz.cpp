#include "StdAfx.h"
#include "ConverterWhiz.h"

namespace Test
{
	/*
	 * Functions for creating collision shapes
	 */
	btCollisionShape* ShapeFromVertexBuffer(VertexBuffer* vbo)
	{
		btTriangleMesh* mesh = new btTriangleMesh();
		mesh->m_weldingThreshold = 0.05f;
		for(unsigned int i = 0; i < vbo->GetNumVerts();)
		{
			VTNTT vinfo_a = GetVTNTT(vbo, i++);
			VTNTT vinfo_b = GetVTNTT(vbo, i++);
			VTNTT vinfo_c = GetVTNTT(vbo, i++);

			btVector3 a = btVector3(vinfo_a.x.x, vinfo_a.x.y, vinfo_a.x.z);
			btVector3 b = btVector3(vinfo_b.x.x, vinfo_b.x.y, vinfo_b.x.z);
			btVector3 c = btVector3(vinfo_c.x.x, vinfo_c.x.y, vinfo_c.x.z);

			mesh->addTriangle(a, b, c, true);
		}
		return new btBvhTriangleMeshShape(mesh, false);
	}

	btCollisionShape* ShapeFromSkinnedModel(SkinnedModel* model)
	{
		btTriangleMesh* mesh = new btTriangleMesh();
		mesh->m_weldingThreshold = 0.05f;
		for(vector<MaterialModelPair>::iterator iter = model->material_model_pairs.begin(); iter != model->material_model_pairs.end(); ++iter)
		{
			VertexBuffer* vbo = iter->vbo;

			for(unsigned int i = 0; i < vbo->GetNumVerts();)
			{
				SkinVInfo vinfo_a = GetSkinVInfo(vbo, i++);
				SkinVInfo vinfo_b = GetSkinVInfo(vbo, i++);
				SkinVInfo vinfo_c = GetSkinVInfo(vbo, i++);

				btVector3 a = btVector3(vinfo_a.x.x, vinfo_a.x.y, vinfo_a.x.z);
				btVector3 b = btVector3(vinfo_b.x.x, vinfo_b.x.y, vinfo_b.x.z);
				btVector3 c = btVector3(vinfo_c.x.x, vinfo_c.x.y, vinfo_c.x.z);

				mesh->addTriangle(a, b, c, true);
			}
		}
		return new btBvhTriangleMeshShape(mesh, false);
	}

	btCollisionShape* HullFromVertexBuffer(VertexBuffer* vbo)
	{
		
		btConvexHullShape* shape = new btConvexHullShape();

		for(unsigned int i = 0; i < vbo->GetNumVerts();)
		{
			VTNTT vinfo_a = GetVTNTT(vbo, i++);
			VTNTT vinfo_b = GetVTNTT(vbo, i++);
			VTNTT vinfo_c = GetVTNTT(vbo, i++);

			shape->addPoint(btVector3(vinfo_a.x.x, vinfo_a.x.y, vinfo_a.x.z));
			shape->addPoint(btVector3(vinfo_b.x.x, vinfo_b.x.y, vinfo_b.x.z));
			shape->addPoint(btVector3(vinfo_c.x.x, vinfo_c.x.y, vinfo_c.x.z));
		}
		return shape;
	}




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

	void SetUberModelBonePhysics(UberModel* uber, vector<BoneEntry>& bone_entries)
	{
		unsigned int num_bones = bone_entries.size();

		// create a lookup table, name of bone --> index in array
		unordered_map<string, unsigned int> bone_indices;
		for(unsigned int i = 0; i < num_bones; ++i)
			bone_indices[bone_entries[i].name] = i;

		// now make the changes to the UberModel
		uber->bone_physics.clear();

		for(vector<BoneEntry>::iterator iter = bone_entries.begin(); iter != bone_entries.end(); ++iter)
		{
			UberModel::BonePhysics phys;

			phys.bone_name = iter->name;
			phys.mass = iter->mass;
			phys.pos = iter->pos;

			if(!iter->parent.empty())
			{
				unordered_map<string, unsigned int>::iterator found = bone_indices.find(iter->parent);
				if(found != bone_indices.end())
					phys.pos -= bone_entries[found->second].pos;
			}

			if(unsigned int n_spheres = iter->spheres.size())
			{
				btVector3* centers = new btVector3[n_spheres];
				float* radii = new float[n_spheres];

				for(unsigned int i = 0; i < n_spheres; ++i)
				{
					Vec3 center_vec = iter->spheres[i].center;
					centers[i] = btVector3(center_vec.x, center_vec.y, center_vec.z);

					radii[i] = iter->spheres[i].radius;
				}

				phys.shape = new btMultiSphereShape(centers, radii, n_spheres);

				delete[] centers;
				delete[] radii;
			}

			uber->bone_physics.push_back(phys);
		}
	}
}
