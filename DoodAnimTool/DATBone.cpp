#include "StdAfx.h"
#include "DATBone.h"

namespace DoodAnimTool
{
	/*
	 * DATBone methods
	 */
	DATBone::DATBone(Bone* bone, CollisionShape* shape) : bone(bone), shape(shape), selected(false) { }
}
