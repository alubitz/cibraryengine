#include "StdAfx.h"
#include "IKWalkPose.h"

#include "Random3D.h"

namespace CibraryEngine
{
	/*
	 * IKWalkPose methods
	 */
	IKWalkPose::IKWalkPose(Skeleton* pose_skel, Skeleton* physics_skel, unsigned int root_bone, ModelPhysics* mphys) :
		Pose(),
		mphys(mphys),
		pose_skel(pose_skel),
		physics_skel(physics_skel),
		root_bone(root_bone),
		end_effectors(),
		arrive_time(-1)
	{
	}

	IKWalkPose::~IKWalkPose()
	{
		for(vector<EndEffector*>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
			delete *iter;
		end_effectors.clear();
	}

	void IKWalkPose::UpdatePose(TimingInfo time)
	{
		float timestep = time.elapsed;
		if(timestep > 0)
		{
			float now = time.total;
			float next_arrival = -1;

			// find the soonest upcoming arrival time
			if(now < arrive_time)
				next_arrival = arrive_time;
			for(vector<EndEffector*>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
			{
				EndEffector* ee = *iter;
				if(now < ee->arrive_time)
					if(next_arrival == -1 || ee->arrive_time < next_arrival)
						next_arrival = ee->arrive_time;
			}

			Seek(timestep, max(timestep, next_arrival - now));
		}
	}

	void IKWalkPose::Seek(float timestep, float foresight)
	{		
		unsigned int num_variables = 0;
		for(vector<EndEffector*>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
			num_variables += (*iter)->chain->bones.size();
		num_variables *= 3;

		vector<float> best =		vector<float>(num_variables);
		vector<float> guess =		vector<float>(num_variables);
		vector<float> rot =			vector<float>(num_variables);
		vector<float> min_extents =	vector<float>(num_variables);
		vector<float> max_extents =	vector<float>(num_variables);

		vector<float>::iterator best_iter =		best.begin();
		vector<float>::iterator guess_iter =	guess.begin();
		vector<float>::iterator rot_iter =		rot.begin();
		vector<float>::iterator min_iter =		min_extents.begin();
		vector<float>::iterator max_iter =		max_extents.begin();

		// rot = the solution we came up with last time
		for(vector<EndEffector*>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
		{
			const vector<IKChain::ChainNode>& bones = (*iter)->chain->bones;
			
			for(vector<IKChain::ChainNode>::const_iterator jter = bones.begin(); jter != bones.end(); ++jter)
			{
				const IKChain::ChainNode& node = *jter;

				*(min_iter++) = jter->min_extents.x;
				*(min_iter++) = jter->min_extents.y;
				*(min_iter++) = jter->min_extents.z;

				*(max_iter++) = jter->max_extents.x;
				*(max_iter++) = jter->max_extents.y;
				*(max_iter++) = jter->max_extents.z;

				Vec3 vec = Mat3::Invert(node.axes) * node.target_ori.ToPYR();
				*(rot_iter++) = max(node.min_extents.x, min(node.max_extents.x, vec.x));
				*(rot_iter++) = max(node.min_extents.y, min(node.max_extents.y, vec.y));
				*(rot_iter++) = max(node.min_extents.z, min(node.max_extents.z, vec.z));
			}
		}

		float best_score = -1;

		// look for incremenetally better solutions
		for(int i = 0; i < 100; ++i)
		{
			if(i == 0)
				guess = vector<float>(num_variables);			// initialize all to 0.0f
			else if(i == 1)
				guess = rot;									// try the solution we came up with last time
			else
			{
				guess = best;									// mutations off of the best solution so far

				int num_mutations = Random3D::RandInt(1, 3);
				for(int i = 0; i < num_mutations; ++i)
				{
					int index = Random3D::RandInt(num_variables);

					float min_val = min_extents[index], max_val = max_extents[index];
					float range = max_val - min_val;

					guess[index] = max(min_val, min(max_val, guess[index] + Random3D::Rand(-0.1f * range, 0.1f * range)));
				}
			}

			float score = EvaluateSolution(guess);

			if(best_score == -1 || score < best_score)
			{
				best = guess;
				best_score = score;

				// stop iterating once score is "good enough"
				if(score < 0.0001f)
					break;
			}
		}

		// apply the best solution we came up with
		best_iter = best.begin();
		for(vector<EndEffector*>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
		{
			for(vector<IKChain::ChainNode>::iterator jter = (*iter)->chain->bones.begin(); jter != (*iter)->chain->bones.end(); ++jter)
			{
				IKChain::ChainNode& node = *jter;

				float x = *(best_iter++), y = *(best_iter++), z = *(best_iter++);
				node.target_ori = Quaternion::FromPYR(node.axes * Vec3(x, y, z));

				Quaternion& ori = node.ori;

				Vec3 angle = (Quaternion::Reverse(ori) * node.target_ori).ToPYR();
				node.rot = angle / foresight;

				ori *= Quaternion::FromPYR(node.rot * timestep);

				SetBonePose(node.to->name, ori.ToPYR(), Vec3());
			}
		}
	}

	void IKWalkPose::AddEndEffector(unsigned int from, unsigned int to) { end_effectors.push_back(new EndEffector(pose_skel->GetNamedBone(from), pose_skel->GetNamedBone(to), mphys)); }

	float IKWalkPose::EvaluateSolution(const vector<float>& values)
	{
		// TODO: implement this
		map<unsigned int, Mat4> bone_matrices;

		for(vector<EndEffector*>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
		{
			EndEffector* ee = *iter;
			for(vector<IKChain::ChainNode>::iterator jter = ee->chain->bones.begin(); jter != ee->chain->bones.end(); ++jter)
			{
				// TODO: idk :(
			}
		}

		// TODO: return an actual score instead of just 1 all the time
		return 1.0f;
	}
}
