#include "StdAfx.h"
#include "PhysicsRegion.h"

#include "Physics.h"
#include "RigidBody.h"
#include "CollisionGraph.h"

#include "CollisionShape.h"
#include "RayShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Util.h"

namespace CibraryEngine
{
	/*
	 * PhysicsRegion methods
	 */
	PhysicsRegion::PhysicsRegion() : Disposable(), all_objects(), active_objects(), inactive_objects(), static_objects() { }
	void PhysicsRegion::InnerDispose()
	{
		for(unsigned int i = 0; i < ST_ShapeTypeMax; ++i)
		{
			for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(); iter != all_objects[i].end(); ++iter)
			{
				RigidBody* body = *iter;
				
				body->regions.erase(this);
				if(body->regions.empty())
				{
					// TODO: handle orphaned object here
				}
			}

			all_objects[i].clear();
			active_objects[i].clear();
			inactive_objects[i].clear();
			static_objects[i].clear();
		}
	}

	void PhysicsRegion::AddRigidBody(RigidBody* body)
	{
		ShapeType type = body->GetCollisionShape()->GetShapeType();

		all_objects[type].insert(body);

		if(!body->can_move)
			static_objects[type].insert(body);
		else if(body->active)
			active_objects[type].insert(body);
		else
			inactive_objects[type].insert(body);
	}

	void PhysicsRegion::RemoveRigidBody(RigidBody* body)
	{
		ShapeType type = body->GetCollisionShape()->GetShapeType();

		all_objects[type].erase(body);

		if(!body->can_move)
			static_objects[type].erase(body);
		else if(body->active)
			active_objects[type].erase(body);
		else
			inactive_objects[type].erase(body);		
	}

	void PhysicsRegion::TakeOwnership(RigidBody* body)
	{
		AddRigidBody(body);
		body->regions.insert(this);
	}

	void PhysicsRegion::Disown(RigidBody* body)
	{
		RemoveRigidBody(body);

		body->regions.erase(this);
		if(body->regions.empty())
		{
			// TODO: deal with orphaned object here
		}
	}



	void PhysicsRegion::DebugDrawRegion(SceneRenderer* renderer)
	{
		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(); iter != all_objects[i].end(); ++iter)
				(*iter)->DebugDraw(renderer);
	}

	// forward declare some stuff
	static void DoRaySphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);
	static void DoRayMesh(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);
	static void DoRayPlane(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);
	static void DoRayMultisphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);

	


	void PhysicsRegion::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time, RigidBody* ibody)
	{
		Ray ray;
		ray.origin = from;
		ray.direction = to - from;

		list<RayResult> hits;

		// query neighboring regions for potentially relevant objects
		AABB ray_aabb(from);
		ray_aabb.Expand(to);

		unordered_set<RigidBody*> relevant_objects[ST_ShapeTypeMax];
		GetRelevantObjects(ray_aabb, relevant_objects);

		// now do the actual collision testing
		for(unordered_set<RigidBody*>::iterator jter = relevant_objects[ST_Sphere].begin(); jter != relevant_objects[ST_Sphere].end(); ++jter)
			DoRaySphere(ibody, *jter, ray, max_time, hits);

		for(unordered_set<RigidBody*>::iterator jter = relevant_objects[ST_TriangleMesh].begin(); jter != relevant_objects[ST_TriangleMesh].end(); ++jter)
			DoRayMesh(ibody, *jter, ray, max_time, hits);

		for(unordered_set<RigidBody*>::iterator jter = relevant_objects[ST_InfinitePlane].begin(); jter != relevant_objects[ST_InfinitePlane].end(); ++jter)
			DoRayPlane(ibody, *jter, ray, max_time, hits);

		for(unordered_set<RigidBody*>::iterator jter = relevant_objects[ST_MultiSphere].begin(); jter != relevant_objects[ST_MultiSphere].end(); ++jter)
			DoRayMultisphere(ibody, *jter, ray, max_time, hits);

		// run the collision callback on whatever we found
		if(!hits.empty())
		{
			hits.sort();

			for(list<RayResult>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
				if(callback.OnCollision(jter->p))
					break;
		}
	}

	void PhysicsRegion::GetRelevantObjects(const AABB& aabb, unordered_set<RigidBody*>* results)
	{
		for(unsigned int i = ST_Sphere; i < ST_ShapeTypeMax; ++i)						// skip past ST_Ray
		{
			results[i].insert(active_objects[i].begin(), active_objects[i].end());
			results[i].insert(inactive_objects[i].begin(), inactive_objects[i].end());
			results[i].insert(static_objects[i].begin(), static_objects[i].end());
		}
	}




	/*
	 * Ray intersect functions
	 */
	static void DoRaySphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		float first, second;

		if(Util::RaySphereIntersect(ray, Sphere(jbody->GetPosition(), ((SphereShape*)jbody->GetCollisionShape())->radius), first, second))
			if(first >= 0 && first < max_time)
			{
				ContactPoint p;

				p.a.obj = ibody;
				p.b.obj = jbody;
				p.a.pos = ray.origin + first * ray.direction;
				p.b.norm = Vec3::Normalize(p.a.pos - jbody->GetPosition(), 1.0f);

				hits.push_back(RayResult(first, p));
			}
	}

	static void DoRayMesh(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		TriangleMeshShape* mesh = (TriangleMeshShape*)jbody->GetCollisionShape();

		Mat4 inv_mat = jbody->GetInvTransform();

		Ray ray_cut;
		ray_cut.origin = inv_mat.TransformVec3(ray.origin, 1.0f);
		ray_cut.direction = inv_mat.TransformVec3(ray.direction * max_time, 0.0f);

		vector<Intersection> mesh_hits = mesh->RayTest(ray_cut);

		for(vector<Intersection>::iterator kter = mesh_hits.begin(); kter != mesh_hits.end(); ++kter)
		{
			float t = kter->time * max_time;
			if(t >= 0 && t < max_time)
			{
				ContactPoint p;

				p.a.obj = ibody;
				p.b.obj = jbody;
				p.a.pos = ray.origin + t * ray.direction;
				p.b.norm = kter->normal;

				hits.push_back(RayResult(t, p));
			}
		}
	}

	static void DoRayPlane(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		InfinitePlaneShape* plane = (InfinitePlaneShape*)jbody->GetCollisionShape();

		float t = Util::RayPlaneIntersect(ray, plane->plane);
		if(t >= 0 && t < max_time)
		{
			ContactPoint p;

			p.a.obj = ibody;
			p.b.obj = jbody;
			p.a.pos = ray.origin + t * ray.direction;
			p.b.norm = plane->plane.normal;

			hits.push_back(RayResult(t, p));
		}
	}

	static void DoRayMultisphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		MultiSphereShape* shape = (MultiSphereShape*)jbody->GetCollisionShape();

		Mat4 inv_mat = jbody->GetInvTransform();

		Ray nu_ray;
		nu_ray.origin = inv_mat.TransformVec3(ray.origin, 1.0f);
		nu_ray.direction = inv_mat.TransformVec3(ray.direction * max_time, 0.0f);

		ContactPoint p;
		float t;
		if(shape->CollisionCheck(nu_ray, p, t, ibody, jbody))
		{
			p.a.pos = jbody->GetTransformationMatrix().TransformVec3(p.b.pos, 1.0f);
			hits.push_back(RayResult(t * max_time, p));
		}
	}

}
