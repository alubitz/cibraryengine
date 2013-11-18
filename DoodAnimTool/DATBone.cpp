#include "StdAfx.h"
#include "DATBone.h"

namespace DoodAnimTool
{
	/*
	 * DATBone methods
	 */
	DATBone::DATBone(Bone* bone, CollisionShape* shape, unsigned int bone_index, int parent_index) :
		bone(bone),
		shape(shape),
		bone_index(bone_index),
		parent_index(parent_index),
		selected(false),
		center(shape->ComputeMassInfo().com)
	{
	}
}
