#include "StdAfx.h"
#include "StepPose.h"

#include "IKChain.h"
#include "ModelPhysics.h"

#include "Random3D.h"

namespace CibraryEngine
{
	/*
	 * StepPose methods
	 */
	StepPose::StepPose(Bone* base, Bone* end, ModelPhysics* mphys) :
		chain(new IKChain(base, end, mphys)),
		arrive_time(-1),
		arrived(true),
		lifting(false),
		dood_vel()
	{
		// desired state defaults to initial state
		Mat4 end_xform = end->GetTransformationMatrix();
		end_xform.Decompose(desired_end_pos, desired_end_ori);
	}

	StepPose::~StepPose() { delete chain; chain = NULL; }

	void StepPose::UpdatePose(TimingInfo time) { if(time.elapsed > 0) { SeekPosition(time); } }

	void StepPose::SeekPosition(TimingInfo& time)
	{
		if(time.total >= arrive_time)			// TODO: actually check for arrival instead
		{
			if(lifting)
			{
				lifting = false;

				desired_end_pos = step_pos;
				desired_end_ori = step_ori;
				arrive_time = step_arrive;
				arrived = false;
			}
			else
				arrived = true;
		}

		float timestep = time.elapsed;
		float foresight = max(timestep, arrive_time - time.total);

		// find a set of joint orientations which will best satisfy our goal...
		int num_floats = chain->bones.size() * 3;
		IKChain::ChainValues rot =			chain->CreateChainValues();
		IKChain::ChainValues best =			chain->CreateChainValues();
		IKChain::ChainValues guess =		chain->CreateChainValues();
		IKChain::ChainValues min_extents =	chain->CreateChainValues();
		IKChain::ChainValues max_extents =	chain->CreateChainValues();

		float* rot_ptr = rot.begin;
		float* best_ptr = best.begin;
		float* min_ptr = min_extents.begin;
		float* max_ptr = max_extents.begin;

		for(vector<IKChain::ChainNode>::iterator iter = chain->bones.begin(); iter != chain->bones.end(); ++iter)
		{
			const IKChain::ChainNode& node = *iter;

			*(best_ptr++) = 0.0f;
			*(best_ptr++) = 0.0f;
			*(best_ptr++) = 0.0f;

			*(min_ptr++) = node.min_extents.x;
			*(min_ptr++) = node.min_extents.y;
			*(min_ptr++) = node.min_extents.z;
			
			*(max_ptr++) = node.max_extents.x;
			*(max_ptr++) = node.max_extents.y;
			*(max_ptr++) = node.max_extents.z;

			Vec3& vec = Mat3::Invert(node.axes) * node.target_ori.ToPYR();
			*(rot_ptr++) = max(node.min_extents.x, min(node.max_extents.x, vec.x));
			*(rot_ptr++) = max(node.min_extents.y, min(node.max_extents.y, vec.y));
			*(rot_ptr++) = max(node.min_extents.z, min(node.max_extents.z, vec.z));
		}

		float best_score = -1;
		for(int i = 0; i < 50; ++i)
		{
			if(i == 0)
				guess = chain->CreateChainValues();				// all values = 0.0f
			else if(i == 1)
				guess = rot;
			else
			{
				guess = best;

				int mutations = Random3D::RandInt(1, 3);		// situations may arise where a single mutation may hurt, but two or three may help
				for(int j = 0; j < mutations; ++j)
				{
					int mutate = Random3D::RandInt(num_floats);
					float minimum = min_extents[mutate], maximum = max_extents[mutate], range = maximum - minimum;

					guess[mutate] = min(maximum, max(minimum, guess[mutate] + Random3D::Rand(-0.01f * range, 0.01f * range)));
				}
			}

			Mat4 xform = chain->GetEndTransform(guess);

			// score based on how closely it matches what we're after
			Quaternion end_ori;
			Vec3 end_pos;
			xform.Decompose(end_pos, end_ori);

			end_pos += dood_vel * foresight;			// account for linear velocity of the base
			end_pos -= desired_end_pos;
			end_ori -= desired_end_ori;

			float score = end_pos.ComputeMagnitudeSquared() + end_ori.w * end_ori.w + end_ori.x * end_ori.x + end_ori.y * end_ori.y + end_ori.z * end_ori.z;
			if(best_score == -1 || score < best_score)
			{
				best_score = score;
				best = guess;

				// stop iterating once score is "good enough"
				if(score < 0.0001f)
					break;
			}
		}

		// now compute velocities to achieve that set of orientations, and set the bones' poses accordingly
		best_ptr = best.begin;
		for(vector<IKChain::ChainNode>::iterator iter = chain->bones.begin(); iter != chain->bones.end(); ++iter)
		{
			IKChain::ChainNode& node = *iter;

			float x = *(best_ptr++), y = *(best_ptr++), z = *(best_ptr++);
			iter->target_ori = Quaternion::FromPYR(node.axes * Vec3(x, y, z));

			Vec3 angle = (Quaternion::Invert(node.ori) * node.target_ori).ToPYR();
			node.rot = angle / foresight;

			Quaternion& ori = node.ori;
			ori *= Quaternion::FromPYR(node.rot * timestep);

			SetBonePose(node.to->name, ori.ToPYR(), Vec3());
		}
	}

	void StepPose::SetDestination(const Vec3& pos, const Quaternion& ori, float time)
	{
		desired_end_pos = pos;
		desired_end_ori = ori;
		arrive_time = time;

		arrived = false;
	}
	
	void StepPose::Step(const Vec3& pos, const Quaternion& ori, float now, float time)
	{
		step_pos = pos;
		step_ori = ori;
		step_arrive = time;

		lifting = true;

		desired_end_pos = desired_end_pos * 0.15f + step_pos * 0.85f + Vec3(0, 0.2f, 0);
		desired_end_ori = ((desired_end_ori + step_ori) * 0.5f) * Quaternion::FromPYR(0, 0, 0);

		arrive_time = (step_arrive + now) * 0.5f;
		arrived = false;
	}
}
