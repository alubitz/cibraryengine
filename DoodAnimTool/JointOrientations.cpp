#include "StdAfx.h"
#include "JointOrientations.h"

#include "DATKeyframe.h"

namespace DoodAnimTool
{
	/*
	 * JointOrientations methods
	 */
	void JointOrientations::CopyDataDirect(const JointOrientations& other) {memcpy( data, other.data, num_joints * sizeof(Vec3)); }

	JointOrientations::JointOrientations(unsigned int num_joints) : num_joints(num_joints), data(new Vec3[num_joints]) { }
	JointOrientations::JointOrientations(const JointOrientations& other) : num_joints(other.num_joints), data(new Vec3[num_joints]) { CopyDataDirect(other); }

	JointOrientations::~JointOrientations() { if(data) { delete[] data; data = NULL; } }

	void JointOrientations::operator =(const JointOrientations& other)
	{
		if(data && num_joints == other.num_joints)
			CopyDataDirect(other);
		else
		{
			if(data) { delete[] data; }

			num_joints = other.num_joints;
			data = new Vec3[num_joints];
			CopyDataDirect(other);
		}
	}



	vector<JointOrientations::PoseChainNode> JointOrientations::GetPoseChain(const ModelPhysics* mphys, unsigned int start_where) const
	{
		vector<PoseChainNode> result;

		unsigned int num_bones = mphys->bones.size();
		bool* bones_known = new bool[num_bones];
		memset(bones_known, 0, num_bones * sizeof(bool));

		bones_known[start_where] = true;

		bool* joints_done = new bool[num_joints];
		memset(joints_done, 0, num_joints * sizeof(bool));

		unsigned int remaining = num_joints;
		while(remaining != 0)
		{
			bool any = false;
			for(unsigned int i = 0; i < num_joints; ++i)
				if(!joints_done[i])
				{
					const ModelPhysics::JointPhysics& joint = mphys->joints[i];
					int bone_a = joint.bone_a - 1;
					int bone_b = joint.bone_b - 1;
					assert(bone_a >= 0 && bone_b >= 0);

					bool a_known = bones_known[bone_a];
					bool b_known = bones_known[bone_b];

					if(a_known || b_known)
					{
						if(!a_known || !b_known)
						{
							result.push_back(PoseChainNode(i, bone_a, bone_b, a_known, joint.pos, joint.axes.Transpose()));
							bones_known[bone_a] = bones_known[bone_b] = true;
						}

						joints_done[i] = true;
						--remaining;

						any = true;
					}
				}

			if(!any)
			{
				Debug(((stringstream&)(stringstream() << "JointOrientations::GetPoseChain was unable to resolve the transforms of all bones; " << remaining << " bones were left unresolved" << endl)).str());
				break;
			}
		}

		delete[] bones_known;
		delete[] joints_done;

		return result;
	}

	void JointOrientations::UsePoseChain(const vector<PoseChainNode>& chain, DATKeyframe& pose) const
	{
		for(vector<PoseChainNode>::const_iterator iter = chain.begin(); iter != chain.end(); ++iter)
		{
			if(iter->a_known)
			{
				DATKeyframe::KBone&       b_state = pose.data[iter->b];
				const DATKeyframe::KBone& a_state = pose.data[iter->a];

				Quaternion actual_ori = Quaternion::FromRVec(iter->axest * -data[iter->index]);
				b_state.ori = a_state.ori * actual_ori;

				Vec3 apos = a_state.ori * iter->pos + a_state.pos;
				Vec3 bpos = b_state.ori * iter->pos + b_state.pos;

				b_state.pos += apos - bpos;
			}
			else
			{
				DATKeyframe::KBone&       a_state = pose.data[iter->a];
				const DATKeyframe::KBone& b_state = pose.data[iter->b];

				Quaternion actual_ori = Quaternion::FromRVec(iter->axest * -data[iter->index]);
				a_state.ori = b_state.ori * Quaternion::Reverse(actual_ori);

				Vec3 apos = a_state.ori * iter->pos + a_state.pos;
				Vec3 bpos = b_state.ori * iter->pos + b_state.pos;

				a_state.pos += bpos - apos;
			}
		}
	}
}
