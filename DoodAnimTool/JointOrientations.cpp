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

	void JointOrientations::PoseBones(DATKeyframe& pose, const ModelPhysics* mphys, unsigned int start_where) const
	{
		unsigned int num_bones = pose.num_bones;
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
							if(a_known)
							{
								DATKeyframe::KBone&       b_state = pose.data[bone_b];
								const DATKeyframe::KBone& a_state = pose.data[bone_a];

								Quaternion actual_ori = Quaternion::FromRVec(joint.axes.Transpose() * -data[i]);
								b_state.ori = a_state.ori * actual_ori;

								Vec3 apos = Mat4::FromPositionAndOrientation(a_state.pos, a_state.ori).TransformVec3_1(joint.pos);
								Vec3 bpos = Mat4::FromPositionAndOrientation(b_state.pos, b_state.ori).TransformVec3_1(joint.pos);

								b_state.pos += apos - bpos;

								bones_known[bone_b] = true;
							}
							else
							{
								DATKeyframe::KBone&       a_state = pose.data[bone_a];
								const DATKeyframe::KBone& b_state = pose.data[bone_b];

								Quaternion actual_ori = Quaternion::FromRVec(joint.axes.Transpose() * -data[i]);
								a_state.ori = b_state.ori * Quaternion::Reverse(actual_ori);

								Vec3 apos = Mat4::FromPositionAndOrientation(a_state.pos, a_state.ori).TransformVec3_1(joint.pos);
								Vec3 bpos = Mat4::FromPositionAndOrientation(b_state.pos, b_state.ori).TransformVec3_1(joint.pos);

								a_state.pos += bpos - apos;

								bones_known[bone_a] = true;
							}
						}

						joints_done[i] = true;
						--remaining;

						any = true;
					}
				}

			if(!any)
			{
				Debug(((stringstream&)(stringstream() << "JointOrientations::PoseBones was unable to resolve the transforms of all bones; " << remaining << " bones were left unresolved" << endl)).str());
				break;
			}
		}

		delete[] joints_done;
		delete[] bones_known;
	}
}
