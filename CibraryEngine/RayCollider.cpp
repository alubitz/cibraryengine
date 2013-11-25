#include "StdAfx.h"
#include "RayCollider.h"

#include "Util.h"

#include "Physics.h"
#include "RigidBody.h"
#include "CollisionGroup.h"

#include "Sphere.h"

#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"
#include "ConvexMeshShape.h"

#include "DebugDrawMaterial.h"
#include "SceneRenderer.h"
#include "RenderNode.h"


namespace CibraryEngine
{
	/*
	 * RayResult methods
	 */
	RayResult::RayResult() : collider(NULL), body(NULL) { }

	RayResult::RayResult(RayCollider* collider, RigidBody* body, const Vec3& pos, const Vec3& norm, float t) :
		collider(collider),
		body(body),
		pos(pos),
		norm(norm),
		t(t)
	{
	}

	bool RayResult::operator <(const RayResult& h) { return t < h.t; }




	/*
	 * RayCollider methods
	 */
	RayCollider::RayCollider(Entity* user_entity, const Vec3& pos, const Vec3& vel_, float mass) :
		DynamicsObject(user_entity, COT_RayCollider, MassInfo(mass), pos),
		ray_callback(NULL)
	{
		vel = vel_;

		restitution = 0.8f;
		friction = 0.0f;
		linear_damp = 0.1f;
	}

	void RayCollider::CollideRigidBody(RigidBody* body, const Ray& ray, float max_time, list<RayResult>& hits, RayCollider* collider)
	{
		switch(body->GetShapeType())
		{
			case ST_Sphere:			RayCollider::CollideSphere(		body, ray, max_time, hits, collider); return;
			case ST_TriangleMesh:	RayCollider::CollideMesh(		body, ray, max_time, hits, collider); return;
			case ST_InfinitePlane:	RayCollider::CollidePlane(		body, ray, max_time, hits, collider); return; 
			case ST_MultiSphere:	RayCollider::CollideMultisphere(body, ray, max_time, hits, collider); return;
			case ST_ConvexMesh:		RayCollider::CollideConvexMesh(	body, ray, max_time, hits, collider); return;
		}
	}

	void RayCollider::CollideCollisionGroup(CollisionGroup* group, const Ray& ray, float max_time, list<RayResult>& hits, RayCollider* collider)
	{
		for(boost::unordered_set<RigidBody*>::iterator iter = group->children.begin(); iter != group->children.end(); ++iter)
			CollideRigidBody(*iter, ray, max_time, hits, collider);
	}

	void RayCollider::CollideSphere(RigidBody* body, const Ray& ray, float max_time, list<RayResult>& hits, RayCollider* collider)
	{
		float first, second;

		if(Util::RaySphereIntersect(ray, Sphere(body->GetPosition(), ((SphereShape*)body->GetCollisionShape())->radius), first, second))
			if(first >= 0 && first < max_time)
			{
				Vec3 pos = ray.origin + first * ray.direction;
				hits.push_back(RayResult(collider, body, pos, Vec3::Normalize(pos - body->GetPosition(), 1.0f), first));
			}
	}

	void RayCollider::CollideMesh(RigidBody* body, const Ray& ray, float max_time, list<RayResult>& hits, RayCollider* collider)
	{
		TriangleMeshShape* mesh = (TriangleMeshShape*)body->GetCollisionShape();

		Mat4 inv_mat = body->GetInvTransform();
		Ray scaled_ray(inv_mat.TransformVec3_1(ray.origin), inv_mat.TransformVec3_0(ray.direction * max_time));

		vector<Intersection> mesh_hits = mesh->RayTest(scaled_ray);
		for(vector<Intersection>::iterator kter = mesh_hits.begin(), hits_end = mesh_hits.end(); kter != hits_end; ++kter)
		{
			float t = kter->time * max_time;
			if(t >= 0 && t < max_time)
			{
				Vec3 pos = ray.origin + t * ray.direction;
				hits.push_back(RayResult(collider, body, pos, kter->normal, kter->time));
			}
		}
	}

	void RayCollider::CollidePlane(RigidBody* body, const Ray& ray, float max_time, list<RayResult>& hits, RayCollider* collider)
	{
		InfinitePlaneShape* plane = (InfinitePlaneShape*)body->GetCollisionShape();

		Ray use_ray;
		use_ray.origin = ray.origin;
		use_ray.direction = ray.direction * max_time;
		float t = Util::RayPlaneIntersect(use_ray, plane->plane);
		if(t >= 0 && t < 1.0f)
			hits.push_back(RayResult(collider, body, ray.origin + t * use_ray.direction, plane->plane.normal, t));
	}

	void RayCollider::CollideMultisphere(RigidBody* body, const Ray& ray, float max_time, list<RayResult>& hits, RayCollider* collider)
	{
		MultiSphereShape* shape = (MultiSphereShape*)body->GetCollisionShape();

		Mat4 inv_mat = body->GetInvTransform();

		Ray nu_ray(inv_mat.TransformVec3_1(ray.origin), inv_mat.TransformVec3_0(ray.direction * max_time));

		RayResult rr;
		if(shape->CollideRay(nu_ray, rr, collider, body))
		{
			rr.pos = body->GetTransformationMatrix().TransformVec3_1(rr.pos);
			hits.push_back(rr);
		}
	}

	void RayCollider::CollideConvexMesh(RigidBody* body, const Ray& ray, float max_time, list<RayResult>& hits, RayCollider* collider)
	{
		ConvexMeshShape* shape = (ConvexMeshShape*)body->GetCollisionShape();

		Mat4 inv_mat = body->GetInvTransform();

		Ray nu_ray(inv_mat.TransformVec3_1(ray.origin), inv_mat.TransformVec3_0(ray.direction * max_time));

		RayResult rr;
		if(shape->CollideRay(nu_ray, rr, collider, body))
		{
			rr.pos = body->GetTransformationMatrix().TransformVec3_1(rr.pos);
			hits.push_back(rr);
		}
	}



	void RayCollider::DoCollisionResponse(RayResult& collision)
	{
		RigidBody* body = collision.body;
		bool body_can_move = body->can_move;

		const Vec3& use_pos = collision.pos;
		Vec3 normal = -collision.norm;

		Vec3 dv = body->GetLocalVelocity(use_pos) - vel;
		float nvdot = Vec3::Dot(normal, dv);
		if(nvdot < 0.0f)
		{
			float B;
			float use_mass = PhysicsWorld::GetUseMass(this, body, use_pos, normal, B);
			float restitution = this->restitution * body->restitution;
			float use_friction = friction * body->friction;
			float impulse_mag = -(1.0f + restitution) * B * use_mass;

			if(impulse_mag < 0)
			{
				Vec3 impulse = normal * impulse_mag;

				// normal force
				if(impulse.ComputeMagnitudeSquared() != 0)
				{
					ApplyCentralImpulse(impulse);
					if(body_can_move)
						body->ApplyWorldImpulse(-impulse, use_pos);

					// applying this impulse means we need to recompute dv and nvdot!
					dv = body->GetLocalVelocity(use_pos) - vel;
					nvdot = Vec3::Dot(normal, dv);
				}

				Vec3 t_dv = dv - normal * nvdot;
				float t_dv_magsq = t_dv.ComputeMagnitudeSquared();

				// linear friction
				if(t_dv_magsq > 0)								// object is moving; apply kinetic friction
				{
					float t_dv_mag = sqrtf(t_dv_magsq), inv_tdmag = 1.0f / t_dv_mag;
					Vec3 u_tdv = t_dv * inv_tdmag;

					use_mass = PhysicsWorld::GetUseMass(this, body, use_pos, u_tdv);

					Vec3 fric_impulse = t_dv * min(use_mass, fabs(impulse_mag * use_friction * inv_tdmag));

					ApplyCentralImpulse(fric_impulse);
					if(body_can_move)
						body->ApplyWorldImpulse(-fric_impulse, use_pos);
				}

				if(body_can_move)
					body->active = true;
			}
		}
	}



	RayCallback* RayCollider::GetRayCallback() const		{ return ray_callback; }
	void RayCollider::SetRayCallback(RayCallback* callback)	{ ray_callback = callback; }


	void RayCollider::DebugDraw(SceneRenderer* renderer)
	{
		static const float r         = 0.1f;			// tickmark draw radius
		static const float vel_coeff = 0.1f;			// scaling factor with which to draw the velocity vector

		DebugDrawMaterial* ddm = DebugDrawMaterial::GetDebugDrawMaterial();

		float px = pos.x, py = pos.y, pz = pos.z;

		renderer->objects.push_back(RenderNode(ddm, ddm->New(Vec3(px - r, py,     pz    ), Vec3(px + r, py,     pz    )), 1.0f));
		renderer->objects.push_back(RenderNode(ddm, ddm->New(Vec3(px,     py - r, pz    ), Vec3(px,     py + r, pz    )), 1.0f));
		renderer->objects.push_back(RenderNode(ddm, ddm->New(Vec3(px,     py,     pz - r), Vec3(px,     py,     pz + r)), 1.0f));

		renderer->objects.push_back(RenderNode(ddm, ddm->New(pos, pos + vel * vel_coeff, Vec3(1, 0, 0)), 1.0f));
	}
}
