#include "StdAfx.h"
#include "StepPose.h"

namespace InverseKinematics
{
	/*
	 * StepPose methods
	 */
	StepPose::StepPose(Bone* end, Bone* base, ModelPhysics* mphys) : end(end), base(base), chain(), arrive_time(-1), arrived(true)
	{
		// desired state defaults to initial state
		Mat4 end_xform = end->GetTransformationMatrix();
		end_xform.Decompose(desired_end_pos, desired_end_ori);

		// enumerate the chain of bones to go from "base" to "end"
		vector<Bone*> end_chain, base_chain;
		Bone* cur = end;
		while(cur)
		{
			end_chain.push_back(cur);
			cur = cur->parent;
		}

		cur = base;
		while(cur)
		{
			bool any = false;
			for(vector<Bone*>::reverse_iterator iter = end_chain.rbegin(); iter != end_chain.rend(); ++iter)
				if(*iter == cur)
				{
					base_chain.insert(base_chain.end(), iter, end_chain.rend());

					any = true;
					break;
				}

			if(any)
				break;
			else
			{
				base_chain.push_back(cur);
				cur = cur->parent;
			}
		}

		// we've figured out what bones are in the chain... now to actually create it
		Bone* prev = NULL;
		for(vector<Bone*>::iterator iter = base_chain.begin(); iter != base_chain.end(); ++iter)
		{
			Bone* cur = *iter;
			if(prev)
			{
				ChainNode node;
				if(prev == cur->parent)
					node = ChainNode(prev, cur, cur);
				else if(cur == prev->parent)
					node = ChainNode(prev, cur, prev);
				else
					throw exception("ERROR! Sequential bones don't have a parent-child relationship!\n");

				// find the joint which governs these two bones...
				for(vector<ModelPhysics::JointPhysics>::iterator jter = mphys->joints.begin(); jter != mphys->joints.end(); ++jter)
				{
					unsigned int bone_a = Bone::string_table[mphys->bones[jter->bone_a - 1].bone_name];
					unsigned int bone_b = Bone::string_table[mphys->bones[jter->bone_b - 1].bone_name];
					
					if(bone_a == cur->name && bone_b == prev->name || bone_a == prev->name && bone_b == cur->name)
					{
						node.axes = jter->axes;
						node.min_extents = jter->min_extents;
						node.max_extents = jter->max_extents;

						break;
					}
				}

				chain.push_back(node);
			}

			prev = *iter;
		}
	}

	void StepPose::UpdatePose(TimingInfo time)
	{
		if(arrived)
			MaintainPosition(time);
		else
			SeekPosition(time);
	}

	void StepPose::MaintainPosition(TimingInfo& time)
	{
		// TODO: implement this
	}

	void StepPose::SeekPosition(TimingInfo& time)
	{
		if(time.total >= arrive_time)			// TODO: actually check for arrival instead
		{
			arrived = true;
			return;
		}

		float timestep = time.elapsed;
		float foresight = max(timestep, arrive_time - time.total);

		struct RotValues
		{
			int size;
			float* begin;
			float* end;

			RotValues(const RotValues& other) : size(other.size), begin(new float[size]), end(&begin[size])
			{
				for(float *p = other.begin, *my_ptr = begin; p != other.end; ++p, ++my_ptr)
					*my_ptr = *p;
			}
			void operator =(const RotValues& other)
			{
				if(&other == this)
					return;

				delete[] begin;

				size = other.size;
				begin = new float[size];
				end = &begin[size];

				for(float *p = other.begin, *my_ptr = begin; p != other.end; ++p, ++my_ptr)
					*my_ptr = *p;
			}

			float& operator[](int index) { return begin[index]; }

			RotValues(int size) : size(size), begin(new float[size]), end(&begin[size])
			{
				for(float* ptr = begin; ptr != end; ++ptr)
					*ptr = 0.0f;
			}
			~RotValues() { delete[] begin; begin = end = NULL; }
		};

		// find a set of joint orientations which will best satisfy our goal...
		int num_floats = chain.size() * 3;
		RotValues rot = RotValues(num_floats);
		RotValues best = RotValues(num_floats);
		RotValues guess = RotValues(num_floats);
		RotValues min_extents = RotValues(num_floats);
		RotValues max_extents = RotValues(num_floats);

		float* rot_ptr = rot.begin;
		float* best_ptr = best.begin;
		float* guess_ptr = guess.begin;
		float* min_ptr = min_extents.begin;
		float* max_ptr = max_extents.begin;

		for(vector<ChainNode>::iterator iter = chain.begin(); iter != chain.end(); ++iter)
		{
			*(best_ptr++) = 0.0f;
			*(best_ptr++) = 0.0f;
			*(best_ptr++) = 0.0f;

			*(min_ptr++) = iter->min_extents.x;
			*(min_ptr++) = iter->min_extents.y;
			*(min_ptr++) = iter->min_extents.z;
			
			*(max_ptr++) = iter->max_extents.x;
			*(max_ptr++) = iter->max_extents.y;
			*(max_ptr++) = iter->max_extents.z;

			Vec3& vec = Mat3::Invert(iter->axes) * iter->target_ori.ToPYR();
			*(rot_ptr++) = max(iter->min_extents.x, min(iter->max_extents.x, vec.x));
			*(rot_ptr++) = max(iter->min_extents.y, min(iter->max_extents.y, vec.y));
			*(rot_ptr++) = max(iter->min_extents.z, min(iter->max_extents.z, vec.z));
		}

		float best_score = -1;
		for(int i = 0; i < 20; ++i)
		{
			if(i == 0)
				guess = RotValues(num_floats);					// all values = 0.0f
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

			// figure out where this arrangement puts our end effector
			Mat4 xform = base->GetTransformationMatrix();

			bool guess_valid = true;

			guess_ptr = guess.begin;
			for(vector<ChainNode>::iterator iter = chain.begin(); iter != chain.end(); ++iter)
			{
				Bone* child = iter->child;

				float x = *(guess_ptr++), y = *(guess_ptr++), z = *(guess_ptr++);
				Quaternion ori = Quaternion::FromPYR(iter->axes * Vec3(x, y, z));

				Mat4 to_rest_pos = Mat4::Translation(child->rest_pos);
				Mat4 from_rest_pos = Mat4::Translation(-child->rest_pos);
				Mat4 rotation_mat = Mat4::FromQuaternion(ori * child->rest_ori);
				Mat4 offset = Mat4::Translation(child->pos);

				Mat4 net = to_rest_pos * rotation_mat * offset * from_rest_pos;
				if(iter->to != iter->child)
					net = Mat4::Invert(net);

				xform *= net;
			}

			// score based on how closely it matches what we're after
			Quaternion end_ori;
			Vec3 end_pos;
			xform.Decompose(end_pos, end_ori);
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

		Debug(((stringstream&)(stringstream() << "best sqrtf(score) = " << sqrtf(best_score) << endl)).str());

		// now compute velocities to achieve that set of orientations, and set the bones' poses accordingly
		best_ptr = best.begin;
		for(vector<ChainNode>::iterator iter = chain.begin(); iter != chain.end(); ++iter)
		{
			float x = *(best_ptr++), y = *(best_ptr++), z = *(best_ptr++);
			iter->target_ori = Quaternion::FromPYR(iter->axes * Vec3(x, y, z));

			Vec3 angle = (Quaternion::Invert(iter->ori) * iter->target_ori).ToPYR();
			iter->rot = angle / (arrive_time - time.total);

			Quaternion& ori = iter->ori;
			ori *= Quaternion::FromPYR(iter->rot * timestep);

			SetBonePose(iter->to->name, ori.ToPYR(), Vec3());
		}
	}

	void StepPose::SetDestination(const Vec3& pos, const Quaternion& ori, float time)
	{
		desired_end_pos = pos;
		desired_end_ori = ori;
		arrive_time = time;

		arrived = false;
	}
}
