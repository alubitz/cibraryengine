#pragma once
#include "StdAfx.h"

namespace InverseKinematics
{
	using namespace CibraryEngine;

	class StepPose : public Pose
	{
		public:

			Bone* end;
			Bone* base;

			struct ChainNode
			{
				Bone* from;
				Bone* to;
				Bone* child;						// parent can be determined using child->parent

				Quaternion ori;
				Vec3 rot;

				Mat3 axes;
				Vec3 min_extents, max_extents;

				ChainNode() : from(NULL), to(NULL), child(NULL) { }
				ChainNode(Bone* from, Bone* to, Bone* child) :
					from(from),
					to(to),
					child(child),
					ori(Quaternion::Identity()),
					rot(),
					axes(Mat3::Identity()),
					min_extents(-2, -2, -2),
					max_extents(2, 2, 2)
				{
				}
			};
			vector<ChainNode> chain;				// chain of bones from base to end (including both)

			Quaternion desired_end_ori;
			Vec3 desired_end_pos;
			float arrive_time;

			bool arrived;

			StepPose(Bone* end, Bone* base, ModelPhysics* mphys);

			void UpdatePose(TimingInfo time);

			void MaintainPosition(TimingInfo& time);
			void SeekPosition(TimingInfo& time);

			void SetDestination(const Vec3& pos, const Quaternion& ori, float time);
	};
}
