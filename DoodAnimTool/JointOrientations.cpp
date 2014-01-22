#include "StdAfx.h"
#include "JointOrientations.h"

#include "DATKeyframe.h"
#include "PoseChainNode.h"

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



	vector<PoseChainNode> JointOrientations::GetPoseChain(const ModelPhysics* mphys, bool* starters) const
	{
		unsigned int num_bones = mphys->bones.size();

		unsigned int arraysize = num_bones + num_joints;
		bool* boolarray   = new bool[arraysize];
		bool* bones_known = boolarray;
		bool* joints_done = boolarray + num_bones;

		memcpy(bones_known, starters, num_bones * sizeof(bool));
		memset(joints_done, 0, num_joints * sizeof(bool));
		
		vector<PoseChainNode> results;
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
							if(a_known)
								results.push_back(PoseChainNode(i, joint.pos, bone_a, bone_b, -joint.axes.Transpose()));
							else
								results.push_back(PoseChainNode(i, joint.pos, bone_b, bone_a, joint.axes.Transpose()));

							bones_known[bone_a] = bones_known[bone_b] = true;
						}

						joints_done[i] = true;
						--remaining;

						any = true;
					}
				}

			if(!any)
			{
				Debug(((stringstream&)(stringstream() << "JointOrientations::GetPoseChain was unable to resolve all of the bones' transforms; " << remaining << " bones were left unresolved" << endl)).str());
				break;
			}
		}

		delete[] boolarray;

		return results;
	}

	void JointOrientations::UsePoseChain(const PoseChainNode* chain_begin, const PoseChainNode* chain_end, DATKeyframe& pose) const
	{
		for(const PoseChainNode* iter = chain_begin; iter != chain_end; ++iter)
		{
			const DATKeyframe::KBone& from_state = pose.data[iter->from];
			DATKeyframe::KBone&       to_state   = pose.data[iter->to];

			const Vec3& ipos = iter->pos;

			to_state.ori = from_state.ori * Quaternion::FromRVec(iter->mat * data[iter->index]);
			to_state.pos = from_state.ori * ipos - to_state.ori * ipos + from_state.pos;				// apparently my Quaternion*Vec3 is not distributive
		}
	}
}
