#include "StdAfx.h"
#include "IKWalkPose.h"

#include "Random3D.h"

#include "Physics.h"

#include "RigidBody.h"
#include "JointConstraint.h"
#include "PlacedFootConstraint.h"

namespace CibraryEngine
{
	/*
	 * IKWalkPose::EndEffector methods
	 */
	IKWalkPose::EndEffector::EndEffector(PhysicsWorld* physics, RigidBody* foot) : physics(physics), foot(foot), placed(NULL), desired_pos(), desired_ori(Quaternion::Identity()), arrive_time(-1), arrived(true) { }
	IKWalkPose::EndEffector::~EndEffector() { UnlockPlacedFoot(); }

	void IKWalkPose::EndEffector::LockPlacedFoot(RigidBody* base)
	{
		if(placed)
		{
			if(placed->obj_a == foot && placed->obj_b == base)
				return;
			else
				UnlockPlacedFoot();
		}

		placed = new PlacedFootConstraint(foot, base, Vec3());			// TODO: use a real position
		physics->AddConstraint(placed);
	}

	void IKWalkPose::EndEffector::UnlockPlacedFoot()
	{
		if(placed)
		{
			physics->RemoveConstraint(placed);

			placed->Dispose();
			delete placed;
			placed = NULL;
		}
	}




	/*
	 * IKWalkPose methods
	 */
	IKWalkPose::IKWalkPose(PhysicsWorld* physics, const vector<RigidBody*>& rigid_bodies, const vector<JointConstraint*>& constraints, const vector<Bone*>& rbody_to_posey) :
		Pose(),
		physics(physics),
		rigid_bodies(rigid_bodies),
		joints(),
		end_effectors(),
		arrive_time(-1)
	{
		for(vector<JointConstraint*>::const_iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
		{
			JointConstraint* constraint = *iter;

			Bone *bone_a = NULL, *bone_b = NULL;

			unsigned int i = 0;
			for(vector<RigidBody*>::const_iterator jter = rigid_bodies.begin(); jter != rigid_bodies.end(); ++jter, ++i)
			{
				if(*jter == constraint->obj_a)
					bone_a = rbody_to_posey[i];
				else if(*jter == constraint->obj_b)
					bone_b = rbody_to_posey[i];
			}

			if(bone_a && bone_b)
			{
				if(bone_a == bone_b->parent)
					joints.push_back(Joint(constraint, constraint->desired_ori, bone_b->name, false));
				else if(bone_b == bone_a->parent)
					joints.push_back(Joint(constraint, constraint->desired_ori, bone_a->name, true));
			}
		}
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

	void IKWalkPose::AddEndEffector(RigidBody* foot) { end_effectors.push_back(new EndEffector(physics, foot)); }

	void IKWalkPose::Seek(float timestep, float foresight)
	{
		unsigned int num_variables = joints.size() * 3;

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
		for(vector<Joint>::iterator iter = joints.begin(); iter != joints.end(); ++iter)
		{
			const Joint& joint = *iter;
			const JointConstraint& constraint = *joint.constraint;

			*(min_iter++) = constraint.min_extents.x;
			*(min_iter++) = constraint.min_extents.y;
			*(min_iter++) = constraint.min_extents.z;

			*(max_iter++) = constraint.max_extents.x;
			*(max_iter++) = constraint.max_extents.y;
			*(max_iter++) = constraint.max_extents.z;

			Quaternion target_ori = joint.target_ori;
			Vec3 vec = Mat3::Invert(constraint.axes) * target_ori.ToPYR();
			*(rot_iter++) = max(constraint.min_extents.x, min(constraint.max_extents.x, vec.x));
			*(rot_iter++) = max(constraint.min_extents.y, min(constraint.max_extents.y, vec.y));
			*(rot_iter++) = max(constraint.min_extents.z, min(constraint.max_extents.z, vec.z));
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
		for(vector<Joint>::iterator iter = joints.begin(); iter != joints.end(); ++iter)
		{
			Joint& joint = *iter;
			JointConstraint& constraint = *joint.constraint;

			float x = *(best_iter++), y = *(best_iter++), z = *(best_iter++);
			joint.target_ori = Quaternion::FromPYR(constraint.axes * Vec3(x, y, z));

			Quaternion ori = Quaternion::Reverse(constraint.obj_a->GetOrientation()) * constraint.obj_b->GetOrientation();

			Quaternion target_relative = Quaternion::Reverse(ori) * joint.target_ori;
			float a_coeff = timestep / foresight, b_coeff = 1.0f - a_coeff;

			ori *= target_relative * a_coeff + Quaternion::Identity() * b_coeff;

			SetBonePose(joint.set_pose_id, ori.ToPYR(), Vec3());
		}
	}

	float IKWalkPose::EvaluateSolution(const vector<float>& values)
	{
		return 1.0f;					// TODO: implement this for real
	}
}
