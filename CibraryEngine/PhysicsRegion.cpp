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
	PhysicsRegion::PhysicsRegion() : Disposable(), rigid_bodies(), shape_bodies(), static_bodies(), overlapping_bodies() { }
	void PhysicsRegion::InnerDispose()
	{
		// dispose rigid bodies
		for(unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
		{
			(*iter)->Dispose();
			delete *iter;
		}

		rigid_bodies.clear();

		for(unsigned int i = 0; i < ST_ShapeTypeMax; ++i)
			shape_bodies[i].clear();
	}

	void PhysicsRegion::GetRelevantNeighbors(const AABB& aabb, vector<PhysicsRegion*>& results) { results.clear(); }

	void PhysicsRegion::GetRelevantObjects(const AABB& aabb, vector<RigidBody*>& results)
	{
		results.clear();

		results.insert(results.end(), rigid_bodies.begin(), rigid_bodies.end());

		for(unsigned int i = ST_Sphere; i < ST_ShapeTypeMax; ++i)						// skip past ST_Ray
			results.insert(results.end(), overlapping_bodies[i].begin(), overlapping_bodies[i].end());
	}




	void PhysicsRegion::AddRigidBody(RigidBody* body)
	{
		if(body == NULL)
			return;
		
		if(rigid_bodies.find(body) != rigid_bodies.end())
			return;

		rigid_bodies.insert(body);
		body->region = this;

		ShapeType type = body->GetCollisionShape()->GetShapeType();

		shape_bodies[type].insert(body);
		if(!body->can_move)
		{
			static_bodies[type].insert(body);

			// TODO: add this rigid body to the static bodies list of neighboring regions
		}
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

			// TODO: if this rigid body is static, remove it from the static bodies list of neighboring regions

			return true;
		}
		else
			return false;
	}

	void PhysicsRegion::TakeOwnership(RigidBody* body)
	{
		rigid_bodies.insert(body);
		shape_bodies[body->GetCollisionShape()->GetShapeType()].insert(body);

		body->region = this;
	}

	void PhysicsRegion::Disown(RigidBody* body)
	{
		rigid_bodies.erase(body);
		shape_bodies[body->GetCollisionShape()->GetShapeType()].erase(body);
	}



	void PhysicsRegion::DebugDrawRegion(SceneRenderer* renderer)
	{
		for(unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->DebugDraw(renderer);
	}

	// forward declare some stuff
	struct Hit
	{
		float t;
		ContactPoint p;

		Hit(float t, ContactPoint p) : t(t), p(p) { }
		bool operator <(Hit& h) { return t < h.t; }
	};
	static void DoRaySphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<Hit>& hits);
	static void DoRayMesh(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<Hit>& hits);
	static void DoRayPlane(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<Hit>& hits);
	static void DoRayMultisphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<Hit>& hits);

	


	void PhysicsRegion::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time, RigidBody* ibody)
	{
		Ray ray;
		ray.origin = from;
		ray.direction = to - from;

		list<Hit> hits;

		// query neighboring regions for potentially relevant objects
		AABB ray_aabb(from);
		ray_aabb.Expand(to);

		vector<PhysicsRegion*> neighbors;
		GetRelevantNeighbors(ray_aabb, neighbors);

		unordered_set<RigidBody*> neighbor_objects[ST_ShapeTypeMax];
		vector<RigidBody*> objects;
		for(vector<PhysicsRegion*>::iterator iter = neighbors.begin(); iter != neighbors.end(); ++iter)
		{
			(*iter)->GetRelevantObjects(ray_aabb, objects);

			for(vector<RigidBody*>::iterator iter = objects.begin(); iter != objects.end(); ++iter)
				neighbor_objects[(*iter)->GetCollisionShape()->GetShapeType()].insert(*iter);
		}

		// ray-sphere collisions
		for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_Sphere].begin(); jter != shape_bodies[ST_Sphere].end(); ++jter)
			DoRaySphere(ibody, *jter, ray, max_time, hits);
		for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_Sphere].begin(); jter != overlapping_bodies[ST_Sphere].end(); ++jter)
			DoRaySphere(ibody, *jter, ray, max_time, hits);
		for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_Sphere].begin(); jter != neighbor_objects[ST_Sphere].end(); ++jter)
			DoRaySphere(ibody, *jter, ray, max_time, hits);
				
		// ray-mesh collisions
		for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_TriangleMesh].begin(); jter != shape_bodies[ST_TriangleMesh].end(); ++jter)
			DoRayMesh(ibody, *jter, ray, max_time, hits);
		for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_TriangleMesh].begin(); jter != overlapping_bodies[ST_TriangleMesh].end(); ++jter)
			DoRayMesh(ibody, *jter, ray, max_time, hits);
		for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_TriangleMesh].begin(); jter != neighbor_objects[ST_TriangleMesh].end(); ++jter)
			DoRayMesh(ibody, *jter, ray, max_time, hits);

		// ray-plane collisions
		for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_InfinitePlane].begin(); jter != shape_bodies[ST_InfinitePlane].end(); ++jter)
			DoRayPlane(ibody, *jter, ray, max_time, hits);
		for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_InfinitePlane].begin(); jter != overlapping_bodies[ST_InfinitePlane].end(); ++jter)
			DoRayPlane(ibody, *jter, ray, max_time, hits);
		for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_InfinitePlane].begin(); jter != neighbor_objects[ST_InfinitePlane].end(); ++jter)
			DoRayPlane(ibody, *jter, ray, max_time, hits);

		// ray-multisphere collisions
		for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_MultiSphere].begin(); jter != shape_bodies[ST_MultiSphere].end(); ++jter)
			DoRayMultisphere(ibody, *jter, ray, max_time, hits);
		for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_MultiSphere].begin(); jter != overlapping_bodies[ST_MultiSphere].end(); ++jter)
			DoRayMultisphere(ibody, *jter, ray, max_time, hits);
		for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_MultiSphere].begin(); jter != neighbor_objects[ST_MultiSphere].end(); ++jter)
			DoRayMultisphere(ibody, *jter, ray, max_time, hits);

		// run the collision callback on whatever we found
		if(!hits.empty())
		{
			hits.sort();

			for(list<Hit>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
				if(callback.OnCollision(jter->p))
					break;
		}
	}

	void PhysicsRegion::UpdateVel(float timestep)
	{
		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			if(CollisionShape::CanShapeTypeMove((ShapeType)i))
				for(unordered_set<RigidBody*>::iterator iter = shape_bodies[i].begin(); iter != shape_bodies[i].end(); ++iter)
					(*iter)->UpdateVel(timestep);
		
	}
	void PhysicsRegion::UpdatePos(float timestep)
	{
		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			if(CollisionShape::CanShapeTypeMove((ShapeType)i))
				for(unordered_set<RigidBody*>::iterator iter = shape_bodies[i].begin(); iter != shape_bodies[i].end(); ++iter)
				{
					(*iter)->UpdatePos(timestep);
					// TODO: transfer ownership of this RigidBody to another PhysicsRegion as appropriate
				}
	}

	void PhysicsRegion::ResetForces()
	{
		for(unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->ResetForces();
	}

	void PhysicsRegion::SetGravity(const Vec3& gravity)
	{
		// set gravity of all rigid bodies within the world
		for(unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->gravity = gravity;
	}

	void PhysicsRegion::DoRayUpdates(float timestep, CollisionCallback& callback)
	{
		for(unordered_set<RigidBody*>::iterator iter = shape_bodies[ST_Ray].begin(); iter != shape_bodies[ST_Ray].end(); ++iter)
		{
			RigidBody* ibody = *iter;
			RayTest(ibody->GetPosition(), ibody->GetPosition() + ibody->GetLinearVelocity(), callback, timestep, ibody);
		}
	}

	void PhysicsRegion::ResetOverlappingObjects()
	{
		for(int i = 1; i < ST_ShapeTypeMax; ++i)
		{
			overlapping_bodies[i].clear();
			overlapping_bodies[i].insert(static_bodies[i].begin(), static_bodies[i].end());
		}
	}







	void PhysicsRegion::AddNearPairs(float timestep, NearPairs& results)
	{
		for(unordered_set<RigidBody*>::iterator iter = shape_bodies[ST_Sphere].begin(); iter != shape_bodies[ST_Sphere].end(); ++iter)
		{
			RigidBody* ibody = *iter;
			float radius = ((SphereShape*)ibody->GetCollisionShape())->radius;

			Vec3 pos = ibody->GetPosition();
			Vec3 vel = ibody->GetLinearVelocity();

			list<ContactPoint> hits;

			// query neighboring regions for potentially relevant objects
			AABB sphere_aabb(pos, radius);
			sphere_aabb.Expand(AABB(pos + vel * timestep, radius));

			vector<PhysicsRegion*> neighbors;
			GetRelevantNeighbors(sphere_aabb, neighbors);

			unordered_set<RigidBody*> neighbor_objects[ST_ShapeTypeMax];
			vector<RigidBody*> objects;
			for(vector<PhysicsRegion*>::iterator jter = neighbors.begin(); jter != neighbors.end(); ++jter)
			{
				(*jter)->GetRelevantObjects(sphere_aabb, objects);

				for(vector<RigidBody*>::iterator kter = objects.begin(); kter != objects.end(); ++kter)
					neighbor_objects[(*kter)->GetCollisionShape()->GetShapeType()].insert(*kter);

				// add this body to the other region so that subsequent collision tests can find it
				(*jter)->overlapping_bodies[ST_Sphere].insert(ibody);
			}

			NearPairs::Data::iterator sphere_sphere = results.GetSet(ST_Sphere, ST_Sphere);
			for(unordered_set<RigidBody*>::iterator jter = iter; jter != shape_bodies[ST_Sphere].end(); ++jter)
				if(iter != jter)
					results.AddPair(sphere_sphere, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_Sphere].begin(); jter != overlapping_bodies[ST_Sphere].end(); ++jter)
				results.AddPair(sphere_sphere, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_Sphere].begin(); jter != neighbor_objects[ST_Sphere].end(); ++jter)
				results.AddPair(sphere_sphere, ibody, *jter);

			NearPairs::Data::iterator sphere_mesh = results.GetSet(ST_Sphere, ST_TriangleMesh);
			for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_TriangleMesh].begin(); jter != shape_bodies[ST_TriangleMesh].end(); ++jter)
				results.AddPair(sphere_mesh, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_TriangleMesh].begin(); jter != overlapping_bodies[ST_TriangleMesh].end(); ++jter)
				results.AddPair(sphere_mesh, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_TriangleMesh].begin(); jter != neighbor_objects[ST_TriangleMesh].end(); ++jter)
				results.AddPair(sphere_mesh, ibody, *jter);

			NearPairs::Data::iterator sphere_plane = results.GetSet(ST_Sphere, ST_InfinitePlane);
			for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_InfinitePlane].begin(); jter != shape_bodies[ST_InfinitePlane].end(); ++jter)
				results.AddPair(sphere_plane, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_InfinitePlane].begin(); jter != overlapping_bodies[ST_InfinitePlane].end(); ++jter)
				results.AddPair(sphere_plane, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_InfinitePlane].begin(); jter != neighbor_objects[ST_InfinitePlane].end(); ++jter)
				results.AddPair(sphere_plane, ibody, *jter);

			NearPairs::Data::iterator sphere_msphere = results.GetSet(ST_Sphere, ST_MultiSphere);
			for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_MultiSphere].begin(); jter != shape_bodies[ST_MultiSphere].end(); ++jter)
				results.AddPair(sphere_msphere, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_MultiSphere].begin(); jter != overlapping_bodies[ST_MultiSphere].end(); ++jter)
				results.AddPair(sphere_msphere, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_MultiSphere].begin(); jter != neighbor_objects[ST_MultiSphere].end(); ++jter)
				results.AddPair(sphere_msphere, ibody, *jter);
		}

		for(unordered_set<RigidBody*>::iterator iter = shape_bodies[ST_MultiSphere].begin(); iter != shape_bodies[ST_MultiSphere].end(); ++iter)
		{
			RigidBody* ibody = *iter;
			MultiSphereShape* ishape = (MultiSphereShape*)ibody->GetCollisionShape();

			Vec3 vel = ibody->GetLinearVelocity();
			Vec3 pos = ibody->GetPosition();
			Mat4 xform = ibody->GetTransformationMatrix();

			AABB i_aabb = ishape->GetAABB();
			list<ContactPoint> hits;

			// query neighboring regions for potentially relevant objects
			AABB msphere_aabb(i_aabb.GetTransformedAABB(xform));

			vector<PhysicsRegion*> neighbors;
			GetRelevantNeighbors(msphere_aabb, neighbors);

			unordered_set<RigidBody*> neighbor_objects[ST_ShapeTypeMax];
			vector<RigidBody*> objects;
			for(vector<PhysicsRegion*>::iterator jter = neighbors.begin(); jter != neighbors.end(); ++jter)
			{
				(*jter)->GetRelevantObjects(msphere_aabb, objects);

				for(vector<RigidBody*>::iterator kter = objects.begin(); kter != objects.end(); ++kter)
					neighbor_objects[(*kter)->GetCollisionShape()->GetShapeType()].insert(*kter);

				// add this body to the other region so that subsequent collision tests can find it
				(*jter)->overlapping_bodies[ST_MultiSphere].insert(ibody);
			}

			NearPairs::Data::iterator msphere_mesh = results.GetSet(ST_MultiSphere, ST_TriangleMesh);
			for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_TriangleMesh].begin(); jter != shape_bodies[ST_TriangleMesh].end(); ++jter)
				results.AddPair(msphere_mesh, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_TriangleMesh].begin(); jter != overlapping_bodies[ST_TriangleMesh].end(); ++jter)
				results.AddPair(msphere_mesh, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_TriangleMesh].begin(); jter != neighbor_objects[ST_TriangleMesh].end(); ++jter)
				results.AddPair(msphere_mesh, ibody, *jter);

			NearPairs::Data::iterator msphere_plane = results.GetSet(ST_MultiSphere, ST_InfinitePlane);
			for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_InfinitePlane].begin(); jter != shape_bodies[ST_InfinitePlane].end(); ++jter)
				results.AddPair(msphere_plane, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_InfinitePlane].begin(); jter != overlapping_bodies[ST_InfinitePlane].end(); ++jter)
				results.AddPair(msphere_plane, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_InfinitePlane].begin(); jter != neighbor_objects[ST_InfinitePlane].end(); ++jter)
				results.AddPair(msphere_plane, ibody, *jter);

			/*
			NearPairs::Data::iterator msphere_msphere = results.GetSet(ST_MultiSphere, ST_MultiSphere);
			for(unordered_set<RigidBody*>::iterator jter = iter; jter != shape_bodies[ST_MultiSphere].end(); ++jter)
				if(jter != iter)
					results.AddPair(msphere_msphere, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = overlapping_bodies[ST_MultiSphere].begin(); jter != overlapping_bodies[ST_MultiSphere].end(); ++jter)
				results.AddPair(msphere_msphere, ibody, *jter);
			for(unordered_set<RigidBody*>::iterator jter = neighbor_objects[ST_MultiSphere].begin(); jter != neighbor_objects[ST_MultiSphere].end(); ++jter)
				results.AddPair(msphere_msphere, ibody, *jter);
			*/
		}
	}




	/*
	 * Ray intersect functions
	 */
	static void DoRaySphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<Hit>& hits)
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

				hits.push_back(Hit(first, p));
			}
	}

	static void DoRayMesh(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<Hit>& hits)
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

				hits.push_back(Hit(t, p));
			}
		}
	}

	static void DoRayPlane(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<Hit>& hits)
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

			hits.push_back(Hit(t, p));
		}
	}

	static void DoRayMultisphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<Hit>& hits)
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
			hits.push_back(Hit(t * max_time, p));
		}
	}



	
}
