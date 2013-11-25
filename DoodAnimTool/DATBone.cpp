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
		selected(false)
	{
	}
}
