#include "StdAfx.h"
#include "PoseSolverState.h"

namespace DoodAnimTool
{
	/*
	 * PoseSolverState methods
	 */
	PoseSolverState::PoseSolverState(const DATKeyframe& initial, const JointOrientations& joints) :
		joints(joints),
		initial(initial),
		current(initial),
		next(initial.num_bones, initial.num_constraints),
		contrib_count(new unsigned int[initial.num_bones])
	{
	}

	PoseSolverState::~PoseSolverState() { if(contrib_count) { delete contrib_count; contrib_count = NULL; } }

	void PoseSolverState::PreIteration()
	{
		for(unsigned int i = 0; i < initial.num_bones; ++i)
		{
#if 1
			next.data[i].pos = Vec3();
			next.data[i].ori = Quaternion();		// zero quaternion on purpose
			contrib_count[i] = 0;
#else
			next.data[i] = current.data[i];
			contrib_count[i] = 1;
#endif
		}

		for(unsigned int i = 0; i < ERROR_TYPES; ++i)
			errors[i] = 0.0f;
	}

	void PoseSolverState::PostIteration()
	{
		for(unsigned int i = 0; i < initial.num_bones; ++i)
		{
			DATKeyframe::KBone& cur_datum = current.data[i];
			DATKeyframe::KBone& next_datum = next.data[i];

			if(unsigned int& count = contrib_count[i])					// only update bones whose state has actually changed
			{
				cur_datum.pos = next_datum.pos / float(count);
				cur_datum.ori = Quaternion::Normalize(next_datum.ori);
			}
		}
	}

	DATKeyframe PoseSolverState::GetFinalPose() { return current; }
}
