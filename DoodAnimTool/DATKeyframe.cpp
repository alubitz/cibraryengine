#include "StdAfx.h"
#include "DATKeyframe.h"

namespace DoodAnimTool
{
	/*
	 * DATKeyframe methods
	 */
	DATKeyframe::DATKeyframe(unsigned int num_bones, unsigned int num_constraints) :
		num_bones(num_bones),
		data(new KBone[num_bones]),
		num_constraints(num_constraints),
		enabled_constraints(new bool[num_constraints])
	{
		memset(enabled_constraints, true, num_constraints * sizeof(bool));		// all constraints are enabled by default
	}

	DATKeyframe::DATKeyframe(const DATKeyframe& other) :
		num_bones(other.num_bones),
		data(new KBone[num_bones]),
		num_constraints(other.num_constraints),
		enabled_constraints(new bool[num_constraints])
	{
		memcpy(data,				other.data,					num_bones		* sizeof(KBone));
		memcpy(enabled_constraints,	other.enabled_constraints,	num_constraints	* sizeof(bool) );
	}

	void DATKeyframe::operator =(const DATKeyframe& other)
	{
		if(data && num_bones == other.num_bones)
			memcpy(data, other.data, num_bones * sizeof(KBone));
		else
		{
			if(data) { delete[] data; }

			num_bones = other.num_bones;
			data = new KBone[num_bones];
			memcpy(data, other.data, num_bones * sizeof(KBone));
		}

		if(enabled_constraints && num_constraints == other.num_constraints)
			memcpy(enabled_constraints, other.enabled_constraints, num_constraints * sizeof(bool));
		else
		{
			if(enabled_constraints) { delete[] enabled_constraints; }

			num_constraints = other.num_constraints;
			enabled_constraints = new bool[num_constraints];
			memcpy(enabled_constraints, other.enabled_constraints, num_constraints * sizeof(bool));
		}
	}

	DATKeyframe::~DATKeyframe()
	{
		if(data)				{ delete[] data;				data = NULL; }
		if(enabled_constraints)	{ delete[] enabled_constraints;	enabled_constraints = NULL; }
	}
}
