#include "StdAfx.h"
#include "DATBone.h"

namespace DoodAnimTool
{
	/*
	 * DATBone methods
	 */
	DATBone::DATBone(unsigned int bone_index, unsigned int name, CollisionShape* shape) :
		bone_index(bone_index),
		name(name),
		shape(shape),
		center(shape->ComputeMassInfo().com),
		selected(false),
		helper(NULL)
	{
	}

	DATBone::DATBone(unsigned int bone_index, unsigned int name, CollisionShape* shape, UberModel* helper) :
		bone_index(bone_index),
		name(name),
		shape(shape),
		center(shape->ComputeMassInfo().com),
		selected(false),
		helper(helper)
	{
	}

	void DATBone::DrawHelperObject(SceneRenderer* renderer, Skeleton* skeleton)
	{
		if(helper != NULL)
			helper->Vis(renderer, 0, skeleton->bones[bone_index]->GetTransformationMatrix(), NULL);
	}
}
