#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	struct BoneEntry
	{
		string name;
		string parent;
		Vec3 pos;

		VertexBuffer* model;

		vector<Sphere> spheres;

		float mass;

		BoneEntry(string name, string parent, Vec3 pos) :
			name(name),
			parent(parent),
			pos(pos),
			model(NULL),
			spheres(),
			mass(0.2f)
		{
			spheres.push_back(Sphere(Vec3(pos), 0.15f));
		}

		BoneEntry(string name, string parent, Vec3 top, float top_radius, Vec3 bottom, float bottom_radius, float mass) :
			name(name),
			parent(parent),
			pos(top),
			model(NULL),
			spheres(),
			mass(mass)
		{
			spheres.push_back(Sphere(top, top_radius));
			spheres.push_back(Sphere(bottom, bottom_radius));
		}

		BoneEntry(string name, string parent, Vec3 pos, vector<Sphere> spheres, float mass) :
			name(name),
			parent(parent),
			pos(pos),
			model(NULL),
			spheres(spheres),
			mass(mass)
		{
		}
	};

	// For creating UberModels
	UberModel* AutoSkinUberModel(ContentMan* content, string vtn_name, string material, vector<BoneEntry>& bone_entries);
	void SetUberModelSkeleton(UberModel* uber, vector<BoneEntry>& bone_entries);
	void SetUberModelBonePhysics(UberModel* uber, vector<BoneEntry>& bone_entries);
}
