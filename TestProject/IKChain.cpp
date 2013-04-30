#include "StdAfx.h"
#include "IKChain.h"

namespace Test
{
	/*
	 * IKChain::Node methods
	 */
	IKChain::Node::Node(Bone* cur, Bone* next, bool parent_next) :
		jc(NULL),
		cur_rb(NULL),
		next_rb(NULL),
		cur_bone(cur),
		next_bone(next),
		parent_next(parent_next),
		a_next(false)
	{
	}




	/*
	 * IKChain methods
	 */
	IKChain::IKChain(Bone* root, Bone* end) : chain()
	{
		// find the first ancestor these two bones have in common
		Bone* common_ancestor = NULL;
		for(Bone* root_ancestor = root; root_ancestor != NULL; root_ancestor = root_ancestor->parent)
			for(Bone* end_ancestor = end; end_ancestor != NULL; end_ancestor = end_ancestor->parent)
				if(root_ancestor == end_ancestor)
				{
					common_ancestor = root_ancestor;
					break;
				}

		if(common_ancestor != NULL)
		{
			// TODO: add nodes from root to that ancestor

			// TODO: add nodes from that ancestor to end
		}
	}
}
