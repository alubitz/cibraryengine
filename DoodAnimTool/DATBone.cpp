#include "StdAfx.h"
#include "DATBone.h"

namespace DoodAnimTool
{
	/*
	 * DATBone methods
	 */
	DATBone::DATBone(Bone* bone, CollisionShape* shape, int parent_index) :
		bone(bone),
		shape(shape),
		parent_index(parent_index),
		selected(false)
	{
	}
}
