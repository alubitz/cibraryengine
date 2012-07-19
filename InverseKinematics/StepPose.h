#pragma once
#include "StdAfx.h"

namespace InverseKinematics
{
	using namespace CibraryEngine;

	struct IKChain;

	class StepPose : public Pose
	{
		public:

			IKChain* chain;

			Quaternion desired_end_ori;
			Vec3 desired_end_pos;
			float arrive_time;

			Vec3 step_pos;							// may or may not be equal to desired ori/pos
			Quaternion step_ori;
			float step_arrive;

			bool arrived;
			bool lifting;

			Vec3 dood_vel;

			StepPose(Bone* end, Bone* base, ModelPhysics* mphys);
			~StepPose();

			void UpdatePose(TimingInfo time);

			void SeekPosition(TimingInfo& time);

			void SetDestination(const Vec3& pos, const Quaternion& ori, float time);
			void Step(const Vec3& pos, const Quaternion& ori, float now, float time);
	};
}
