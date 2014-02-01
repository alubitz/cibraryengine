#include "StdAfx.h"
#include "SolverInstance.h"

#include "DATKeyframe.h"
#include "PoseChainNode.h"
#include "PoseyDood.h"
#include "JointOrientations.h"

namespace DoodAnimTool
{
	/*
	 * SolverInstance methods
	 */
	SolverInstance::SolverInstance(PoseyDood* dood, DATKeyframe* pose) :
		dood(dood),
		debug_text(),
		pose(pose),
		locked_bones(new bool[pose->num_bones]),
		cache_valid(false),
		cached_jos(new JointOrientations(0)),
		stopped(false),
		noprogress_count(0)
	{
		memset(locked_bones, 0, pose->num_bones * sizeof(bool));
	}

	SolverInstance::~SolverInstance()
	{
		if(locked_bones) { delete[] locked_bones; locked_bones = NULL; }
		if(cached_jos)   { delete   cached_jos;   cached_jos   = NULL; }
	}



	void SolverInstance::InvalidateCache()
	{
		cache_valid = stopped = false;
		noprogress_count = 0;

		debug_text = string();
	}
	
	void SolverInstance::OnStop(float value)
	{
		cached_score = value;
		stopped = true;

		debug_text = ((stringstream&)(stringstream() << "(STOPPED) " << cached_score)).str();
	}



	void SolverInstance::DebugJos()
	{
		for(unsigned int i = 0; i < cached_jos->num_joints; ++i)
		{
			Vec3 vec = dood->mphys->joints[i].axes.Transpose() * cached_jos->data[i];
			Debug(((stringstream&)(stringstream() << "joints[" << i << "] ori = (" << vec.x << ", " << vec.y << ", " << vec.z << ")" << endl)).str());
		}
	}
}
