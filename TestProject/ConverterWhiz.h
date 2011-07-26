#pragma once

#include "../CibraryEngine/CibraryEngine.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	struct BoneEntry
	{
		string name;
		string parent;
		Vec3 pos;

		VTNModel* model;

		vector<Sphere> spheres;

		float mass;

		BoneEntry(string name, string parent, Vec3 pos) :
			name(name),
			parent(parent),
			pos(pos),
			model(NULL),
			spheres(),
			mass(0.2)
		{
			spheres.push_back(Sphere(Vec3(pos), 0.15));
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

	btCollisionShape* ShapeFromVTNModel(VTNModel* model);
	btCollisionShape* ShapeFromSkinnedModel(SkinnedModel* model);

	btCollisionShape* HullFromVTNModel(VTNModel* model);
}
