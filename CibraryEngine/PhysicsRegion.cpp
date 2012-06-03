#include "StdAfx.h"
#include "PhysicsRegion.h"

#include "Physics.h"
#include "RigidBody.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	/*
	 * PhysicsRegion methods
	 */
	PhysicsRegion::PhysicsRegion() : rigid_bodies() { }

	void PhysicsRegion::InnerDispose() { }




	void PhysicsRegion::AddRigidBody(RigidBody* body)
	{
		if(body == NULL)
			return;
		
		if(rigid_bodies.find(body) != rigid_bodies.end())
			return;

		ShapeType shape_type = body->GetCollisionShape()->GetShapeType();
		shape_bodies[shape_type].insert(body);

		// NOTE: gravity still needs to be assigned by PhysicsWorld
	}

	bool PhysicsRegion::RemoveRigidBody(RigidBody* body)
	{
		if(body == NULL)
			return false;

		unordered_set<RigidBody*>::iterator found = rigid_bodies.find(body);
		if(found != rigid_bodies.end())
		{
			rigid_bodies.erase(found);

			ShapeType shape_type = body->GetCollisionShape()->GetShapeType();
			shape_bodies[shape_type].erase(body);

			return true;
		}
		else
			return false;
	}

	void PhysicsRegion::TakeOwnership(RigidBody* body)
	{
		rigid_bodies.insert(body);
		shape_bodies[body->GetCollisionShape()->GetShapeType()].insert(body);
	}

	void PhysicsRegion::Disown(RigidBody* body)
	{
		rigid_bodies.erase(body);
		shape_bodies[body->GetCollisionShape()->GetShapeType()].erase(body);
	}




	void PhysicsRegion::UpdateVel(float timestep)
	{
		for(unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->UpdateVel(timestep);
	}
	void PhysicsRegion::UpdatePos(float timestep)
	{
		for(unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
		{
			(*iter)->UpdatePos(timestep);
			// TODO: transfer ownership of this RigidBody to another PhysicsRegion as appropriate
		}
	}

	void PhysicsRegion::DoRayUpdates(float timestep)
	{
		// TODO: implement this
	}

	void PhysicsRegion::AddCollisions(CollisionGraph& collision_graph)
	{
		for(unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
		{
			for(unordered_set<RigidBody*>::iterator jter = iter; jter != rigid_bodies.end(); ++jter)
				if(jter != iter)
				{
					// TODO: detect collisions between this pair of objects
				}

			// TODO: detect collisions with objects in nearby chunks
		}
	}
}
