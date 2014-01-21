#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	class DATKeyframe;

	class JointOrientations
	{
		private:

			void CopyDataDirect(const JointOrientations& other);

		public:

			unsigned int num_joints;
			Vec3* data;

			JointOrientations(unsigned int num_joints);
			JointOrientations(const JointOrientations& other);

			~JointOrientations();

			void operator =(const JointOrientations& other);

			struct PoseChainNode
			{
				unsigned int index;
				unsigned int a, b;
				bool a_known;
				Vec3 pos;
				Mat3 axest;

				PoseChainNode() { }
				PoseChainNode(unsigned int index, unsigned int a, unsigned int b, bool a_known, const Vec3& pos, const Mat3& axest) : index(index), a(a), b(b), a_known(a_known), pos(pos), axest(axest) { }
			};

			vector<PoseChainNode> GetPoseChain(const ModelPhysics* mphys, unsigned int start_where = 0) const;
			void UsePoseChain(const vector<PoseChainNode>& chain, DATKeyframe& pose) const;
	};
}
