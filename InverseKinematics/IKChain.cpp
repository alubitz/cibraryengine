#include "StdAfx.h"
#include "IKChain.h"

namespace InverseKinematics
{
	/*
	 * IKChain methods
	 */
	IKChain::IKChain(Bone* begin, Bone* end, ModelPhysics* mphys) : begin(begin), end(end), bones()
	{
		// enumerate the chain of bones to go from "base" to "end"
		vector<Bone*> end_chain, base_chain;
		Bone* cur = end;
		while(cur)
		{
			end_chain.push_back(cur);
			cur = cur->parent;
		}

		cur = begin;
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

				bones.push_back(node);
			}

			prev = *iter;
		}
	}

	IKChain::ChainValues IKChain::CreateChainValues() { return ChainValues(bones.size() * 3); }

	Mat4 IKChain::GetEndTransform(const IKChain::ChainValues& values) const
	{
		Mat4 xform = begin->GetTransformationMatrix();

		const float* value_ptr = values.begin;
		for(vector<ChainNode>::const_iterator iter = bones.begin(); iter != bones.end(); ++iter)
		{
			const ChainNode& node = *iter;
			const Bone& child = *node.child;

			float x = *(value_ptr++), y = *(value_ptr++), z = *(value_ptr++);
			Quaternion ori = Quaternion::FromPYR(node.axes * Vec3(x, y, z));

			Mat4 to_rest_pos = Mat4::Translation(child.rest_pos);
			Mat4 from_rest_pos = Mat4::Translation(-child.rest_pos);
			Mat4 rotation_mat = Mat4::FromQuaternion(ori * child.rest_ori);
			Mat4 offset = Mat4::Translation(child.pos);

			Mat4 net = to_rest_pos * rotation_mat * offset * from_rest_pos;
			if(node.to != node.child)
				net = Mat4::Invert(net);

			xform *= net;
		}

		return xform;
	}
}
