#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	class DATKeyframe;
	struct PoseChainNode;

	class JointOrientations
	{
		private:

			void CopyDataDirect(const JointOrientations& other);

		public:

			unsigned int num_joints;
			Vec3* data;

			JointOrientations() : num_joints(0), data(NULL) { }
			JointOrientations(unsigned int num_joints);
			JointOrientations(const JointOrientations& other);

			~JointOrientations();

			void operator =(const JointOrientations& other);

			vector<PoseChainNode> GetPoseChain(const ModelPhysics* mphys, bool* starters) const;
			void UsePoseChain(const PoseChainNode* chain_begin, const PoseChainNode* chain_end, DATKeyframe& pose) const;
	};
}
