#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace std;

	class PoseyDood;
	class DATKeyframe;
	struct PoseChainNode;
	class JointOrientations;

	class SolverInstance
	{
		public:

			PoseyDood* dood;

			string debug_text;

			DATKeyframe* pose;
			bool* locked_bones;

			bool cache_valid;
			float cached_score;
			JointOrientations* cached_jos;
			vector<PoseChainNode> cached_chain;

			bool stopped;
			unsigned int noprogress_count;

			SolverInstance(PoseyDood* dood, DATKeyframe* pose);
			~SolverInstance();

			void InvalidateCache();
			void OnStop(float value);
	};
}
