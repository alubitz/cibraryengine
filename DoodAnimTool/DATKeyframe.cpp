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

	unsigned int DATKeyframe::Read(istream& stream)
	{
		BinaryChunk whole;
		whole.Read(stream);

		if(!stream)
			return 1;

		if(whole.GetName() != "POSE____")
			return 2;
		else
		{
			istringstream ss(whole.data);
					
			unsigned int num_bones = ReadUInt32(ss);
			unsigned int num_constraints = ReadUInt32(ss);

			DATKeyframe k(num_bones, num_constraints);

			for(unsigned int i = 0; i < num_bones; ++i)
			{
				DATKeyframe::KBone& bone = k.data[i];
				bone.pos = ReadVec3(ss);
				bone.ori = ReadQuaternion(ss);
			}

			for(unsigned int i = 0; i < num_constraints; ++i)
				k.enabled_constraints[i] = ReadBool(ss);					

			if(!ss)
				return 3;
			else
			{
				*this = k;
				return 0;
			}
		}
	}

	void DATKeyframe::Write(ostream& stream) const
	{
		stringstream ss;
		WriteUInt32(num_bones, ss);
		WriteUInt32(num_constraints, ss);

		for(unsigned int i = 0; i < num_bones; ++i)
		{
			const DATKeyframe::KBone& bone = data[i];
			WriteVec3(bone.pos, ss);
			WriteQuaternion(bone.ori, ss);
		}

		for(unsigned int i = 0; i < num_constraints; ++i)
			WriteBool(enabled_constraints[i], ss);

		BinaryChunk bc("POSE____");
		bc.data = ss.str();
		bc.Write(stream);
	}
}
