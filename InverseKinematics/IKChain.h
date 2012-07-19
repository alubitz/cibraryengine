#pragma once
#include "StdAfx.h"

namespace InverseKinematics
{
	using namespace CibraryEngine;

	struct IKChain
	{
		Bone* begin;
		Bone* end;

		struct ChainNode
		{
			Bone* from;
			Bone* to;
			Bone* child;						// parent can be determined using child->parent

			Quaternion ori;
			Quaternion target_ori;				// the orientation it was trying to achieve at the end of the last frame
			Vec3 rot;

			Mat3 axes;
			Vec3 min_extents, max_extents;

			ChainNode() : from(NULL), to(NULL), child(NULL) { }
			ChainNode(Bone* from, Bone* to, Bone* child) :
				from(from),
				to(to),	
				child(child),
				ori(Quaternion::Identity()),
				target_ori(Quaternion::Identity()),
				rot(),
				axes(Mat3::Identity()),
				min_extents(-2, -2, -2),
				max_extents(2, 2, 2)
			{
			}
		};
		vector<ChainNode> bones;				// chain of bones from base to end (including both)

		struct ChainValues
		{
			int size;
			float* begin;
			float* end;

			ChainValues(const ChainValues& other) : size(other.size), begin(new float[size]), end(&begin[size])
			{
				for(float *p = other.begin, *my_ptr = begin; p != other.end; ++p, ++my_ptr)
					*my_ptr = *p;
			}
			void operator =(const ChainValues& other)
			{
				if(&other == this)
					return;

				delete[] begin;

				size = other.size;
				begin = new float[size];
				end = &begin[size];

				for(float *p = other.begin, *my_ptr = begin; p != other.end; ++p, ++my_ptr)
					*my_ptr = *p;
			}

			float& operator[](int index) { return begin[index]; }

			ChainValues(int size) : size(size), begin(new float[size]), end(&begin[size])
			{
				for(float* ptr = begin; ptr != end; ++ptr)
					*ptr = 0.0f;
			}
			~ChainValues() { delete[] begin; begin = end = NULL; }
		};

		IKChain(Bone* end, Bone* base, ModelPhysics* mphys);

		/** Creates a ChainValues instance with size appropriate for this IKChain */
		ChainValues CreateChainValues();

		/** Figure out where this arrangement puts our end effector */
		Mat4 GetEndTransform(const ChainValues& values) const;
	};
}
