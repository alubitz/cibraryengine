#include "StdAfx.h"

#include "IKSolver.h"
#include "SkeletalAnimation.h"

namespace CibraryEngine
{
	/*
	 * IKSolver::IKObject methods
	 */
	IKSolver::IKObject::IKObject(Skeleton* skeleton) : result(NULL), skeleton(skeleton) { }

	void IKSolver::IKObject::ComputeNextState(PhysicsWorld* physics, TimingInfo time)
	{
		// TODO: make this do stuff; this will be considerably trickier than implementing ApplyComputedState
	}

	void IKSolver::IKObject::ApplyComputedState()
	{
		// TODO: make this do stuff
	}




	/*
	 * IKSolver methods
	 */
	IKSolver::IKSolver(PhysicsWorld* physics) : ik_objects(), physics(physics) { }

	void IKSolver::ClearObjects()
	{
		for(map<void*, IKObject*>::iterator iter = ik_objects.begin(); iter != ik_objects.end(); iter++)
		{
			IKObject* obj = iter->second;
			if(obj->skeleton != NULL)
				delete obj->skeleton;
			delete obj;
		}
		ik_objects.clear();
	}

	void IKSolver::AddObject(void* user_ptr, Skeleton* skeleton)
	{
		ik_objects[user_ptr] = new IKObject(skeleton);
	}

	void IKSolver::DeleteObject(void* user_ptr)
	{
		map<void*, IKObject*>::iterator found = ik_objects.find(user_ptr);
		if(found != ik_objects.end())
		{
			IKObject* obj = found->second;
			if(obj->skeleton != NULL)
				delete obj->skeleton;
			delete obj;

			ik_objects.erase(found);
		}
	}

	Skeleton* IKSolver::GetObjectSkeleton(void* user_ptr)
	{
		map<void*, IKObject*>::iterator found = ik_objects.find(user_ptr);
		if(found != ik_objects.end())
			return found->second->skeleton;
		else
			return NULL;
	}

	void IKSolver::Update(TimingInfo time)
	{
		map<void*, IKObject*>::iterator iter;

		for(iter = ik_objects.begin(); iter != ik_objects.end(); iter++)
			iter->second->ComputeNextState(physics, time);

		for(iter = ik_objects.begin(); iter != ik_objects.end(); iter++)
			iter->second->ApplyComputedState();
	}
}
