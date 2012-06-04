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
	PhysicsRegion::PhysicsRegion() : Disposable(), rigid_bodies(), shape_bodies() { }
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



	void PhysicsRegion::AddRigidBody(RigidBody* body)
	{
		if(body == NULL)
			return;
		
		if(rigid_bodies.find(body) != rigid_bodies.end())
			return;

		rigid_bodies.insert(body);

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




	void PhysicsRegion::DebugDrawRegion(SceneRenderer* renderer)
	{
		for(unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->DebugDraw(renderer);
	}

	void PhysicsRegion::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time, RigidBody* ibody)
	{
		// TODO: check against other PhysicsRegions

		Ray ray;
		ray.origin = from;
		ray.direction = to - from;

		struct Hit
		{
			float t;
			ContactPoint p;

			Hit(float t, ContactPoint p) : t(t), p(p) { }
			bool operator <(Hit& h) { return t < h.t; }
		};

		list<Hit> hits;

		// ray-sphere collisions
		for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_Sphere].begin(); jter != shape_bodies[ST_Sphere].end(); ++jter)
		{
			RigidBody* jbody = *jter;

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
				
		// ray-mesh collisions
		for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_TriangleMesh].begin(); jter != shape_bodies[ST_TriangleMesh].end(); ++jter)
		{
			RigidBody* jbody = *jter;
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

		// ray-plane collisions
		for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_InfinitePlane].begin(); jter != shape_bodies[ST_InfinitePlane].end(); ++jter)
		{
			RigidBody* jbody = *jter;

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

		// ray-multisphere collisions
		for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_MultiSphere].begin(); jter != shape_bodies[ST_MultiSphere].end(); ++jter)
		{
			RigidBody* jbody = *jter;
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

	void PhysicsRegion::AddCollisions(float timestep, CollisionGraph& collision_graph)
	{
		// TODO: check against other PhysicsRegions

		// handle all the collisions involving spheres
		for(unordered_set<RigidBody*>::iterator iter = shape_bodies[ST_Sphere].begin(); iter != shape_bodies[ST_Sphere].end(); ++iter)
		{
			RigidBody* ibody = *iter;
			float radius = ((SphereShape*)ibody->GetCollisionShape())->radius;

			Vec3 pos = ibody->GetPosition();
			Vec3 vel = ibody->GetLinearVelocity();

			list<ContactPoint> hits;

			// sphere-sphere collisions
			for(unordered_set<RigidBody*>::iterator jter = iter; jter != shape_bodies[ST_Sphere].end(); ++jter)
			{
				if(iter != jter)
				{
					RigidBody* jbody = *jter;
					float sr = radius + ((SphereShape*)jbody->GetCollisionShape())->radius;

					Vec3 other_pos = jbody->GetPosition();
					Vec3 other_vel = jbody->GetLinearVelocity();

					Ray ray;
					ray.origin = pos - other_pos;
					ray.direction = vel - other_vel;

					float first = 0, second = 0;
					if(ray.origin.ComputeMagnitudeSquared() < sr * sr || Util::RaySphereIntersect(ray, Sphere(Vec3(), sr), first, second))
						if(first >= 0 && first <= timestep)
						{
							ContactPoint p;

							p.a.obj = ibody;
							p.b.obj = jbody;
							p.a.pos = pos + first * vel;
							p.b.pos = other_pos + first * other_vel;
							p.b.norm = Vec3::Normalize(p.a.pos - other_pos, 1.0f);
							p.a.norm = -p.b.norm;

							hits.push_back(p);
						}
				}
			}

			// sphere-mesh collisions
			for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_TriangleMesh].begin(); jter != shape_bodies[ST_TriangleMesh].end(); ++jter)
			{
				RigidBody* jbody = *jter;
				TriangleMeshShape* shape = (TriangleMeshShape*)jbody->GetCollisionShape();

				Ray ray;
				ray.origin = pos;
				ray.direction = vel;

				vector<unsigned int> relevant_triangles = shape->GetRelevantTriangles(AABB(pos, radius));
				for(vector<unsigned int>::iterator kter = relevant_triangles.begin(); kter != relevant_triangles.end(); ++kter)
				{
					TriangleMeshShape::TriCache tri = shape->GetTriangleData(*kter);
							
					float dist = tri.DistanceToPoint(pos);
					if(dist < radius)
					{
						ContactPoint p;

						p.a.obj = ibody;
						p.b.obj = jbody;
						p.a.pos = pos - tri.plane.normal * radius;
						p.b.pos = p.a.pos;
						p.a.norm = -tri.plane.normal;
						p.b.norm = tri.plane.normal;

						hits.push_back(p);
					}
				}
			}

			// sphere-plane collisions
			for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_InfinitePlane].begin(); jter != shape_bodies[ST_InfinitePlane].end(); ++jter)
			{
				RigidBody* jbody = *jter;
				InfinitePlaneShape* shape = (InfinitePlaneShape*)jbody->GetCollisionShape();

				Vec3 plane_norm = shape->plane.normal;
				float plane_offset = shape->plane.offset;

				float center_offset = Vec3::Dot(plane_norm, pos) - plane_offset;
				float vel_dot = Vec3::Dot(plane_norm, vel);

				// if the sphere is moving away from the plane, don't bounce!
				if(vel_dot <= 0.0f)
				{
					float dist = fabs(center_offset) - radius;
					float t = center_offset - radius < 0.0f ? 0.0f : dist / fabs(vel_dot);
					if(t >= 0 && t <= timestep)
					{
						ContactPoint p;

						p.a.obj = ibody;
						p.b.obj = jbody;
						p.a.pos = pos - plane_norm * radius;
						p.b.pos = p.a.pos - plane_norm * (Vec3::Dot(p.a.pos, plane_norm) - plane_offset);
						p.b.norm = plane_norm;
						p.a.norm = -plane_norm;

						hits.push_back(p);
					}
				}
			}

			// sphere-multisphere collisions
			for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_MultiSphere].begin(); jter != shape_bodies[ST_MultiSphere].end(); ++jter)
			{
				RigidBody* jbody = *jter;
				MultiSphereShape* shape = (MultiSphereShape*)jbody->GetCollisionShape();

				Mat4 inv_xform = Mat4::Invert(jbody->GetTransformationMatrix());
				Vec3 nu_pos = inv_xform.TransformVec3(pos, 1.0f);

				ContactPoint cp;
				if(shape->CollisionCheck(Sphere(nu_pos, radius), cp, ibody, jbody))
					hits.push_back(cp);
			}

			if(!hits.empty())
				for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
					collision_graph.AddContactPoint(*jter);
		}

		// handle all the collisions involving multispheres
		for(unordered_set<RigidBody*>::iterator iter = shape_bodies[ST_MultiSphere].begin(); iter != shape_bodies[ST_MultiSphere].end(); ++iter)
		{
			RigidBody* ibody = *iter;
			MultiSphereShape* ishape = (MultiSphereShape*)ibody->GetCollisionShape();

			Vec3 vel = ibody->GetLinearVelocity();
			Vec3 pos = ibody->GetPosition();
			Mat4 xform = ibody->GetTransformationMatrix();

			AABB i_aabb = ishape->GetAABB();
			list<ContactPoint> hits;
			
			// multisphere-mesh collisions
			for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_TriangleMesh].begin(); jter != shape_bodies[ST_TriangleMesh].end(); ++jter)
			{
				RigidBody* jbody = *jter;
				TriangleMeshShape* jshape = (TriangleMeshShape*)jbody->GetCollisionShape();

				Mat4 inv_net_xform = jbody->GetInvTransform() * ibody->GetTransformationMatrix();
				AABB xformed_aabb = i_aabb.GetTransformedAABB(inv_net_xform);						// the AABB of the multisphere in the coordinate system of the mesh

				vector<unsigned int> relevant_triangles = jshape->GetRelevantTriangles(xformed_aabb);
				if(relevant_triangles.empty())
					continue;

				ContactPoint p;
				for(vector<unsigned int>::iterator kter = relevant_triangles.begin(); kter != relevant_triangles.end(); ++kter)
				{
					TriangleMeshShape::TriCache tri = jshape->GetTriangleData(*kter);

					ContactPoint p;
					if(ishape->CollisionCheck(inv_net_xform, tri, p, ibody, jbody))
					{
						Mat4 j_xform = jbody->GetTransformationMatrix();

						p.a.pos = p.b.pos = j_xform.TransformVec3(p.a.pos, 1.0f);
						p.a.norm = j_xform.TransformVec3(p.a.norm, 0.0f);
						p.b.norm = -p.a.norm;

						hits.push_back(p);
					}
				}
			}

			// multisphere-plane collisions
			for(unordered_set<RigidBody*>::iterator jter = shape_bodies[ST_InfinitePlane].begin(); jter != shape_bodies[ST_InfinitePlane].end(); ++jter)
			{
				RigidBody* jbody = *jter;
				InfinitePlaneShape* jshape = (InfinitePlaneShape*)jbody->GetCollisionShape();

				ContactPoint p;
				if(ishape->CollisionCheck(xform, jshape->plane, p, ibody, jbody))
					hits.push_back(p);
			}

			// multisphere-multisphere collisions
			for(unordered_set<RigidBody*>::iterator jter = iter; jter != shape_bodies[ST_MultiSphere].end(); ++jter)
			{
				if(iter != jter)
				{
					RigidBody* jbody = *jter;
					MultiSphereShape* jshape = (MultiSphereShape*)jbody->GetCollisionShape();

					Mat4 net_xform = ibody->GetInvTransform() * jbody->GetTransformationMatrix();

					ContactPoint p;
					if(ishape->CollisionCheck(net_xform, jshape, p, ibody, jbody))
					{
						p.a.pos = p.b.pos = xform.TransformVec3(p.a.pos, 1.0f);
						p.a.norm = xform.TransformVec3(p.a.norm, 0.0f);
						p.b.norm = -p.a.norm;

						hits.push_back(p);
					}
				}
			}

			if(!hits.empty())
				for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
					collision_graph.AddContactPoint(*jter);
		}
	}
}
