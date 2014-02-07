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



	void SolverInstance::DebugJos() const
	{
		for(unsigned int i = 0, chain_end = cached_chain.size() - 1; i <= chain_end; ++i)
		{
			const PoseChainNode& node = cached_chain[i];
			unsigned int node_index = node.index;
			const ModelPhysics::JointPhysics& joint = dood->mphys->joints[node_index];
			unsigned int bone = joint.bone_a;
			const string& name = bone > 0 ? dood->mphys->bones[bone - 1].bone_name : string();

			Vec3 vec = joint.axes.Transpose() * cached_jos->data[node_index];
			Debug(((stringstream&)(stringstream() << (i == 0 ? "{ { \"" : "{ \"" )<< name << "\",  " << vec.x << ", " << vec.y << ", " << vec.z << (i == chain_end ? " } }" : " },") << endl)).str());
		}
	}
}
