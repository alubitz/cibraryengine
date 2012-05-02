#include "StdAfx.h"
#include "Physics.h"

#include "CollisionShape.h"
#include "RayShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

#include "CollisionGraph.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Util.h"

#include "DebugDrawMaterial.h"

#define MAX_SEQUENTIAL_SOLVER_ITERATIONS 50

namespace CibraryEngine
{
	/*
	 * PhysicsWorld private implementation struct
	 */
	struct PhysicsWorld::Imp
	{
		boost::unordered_set<RigidBody*> rigid_bodies;											// List of all of the rigid bodies in the physical simulation

		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> > shape_bodies;		// lists of rigid bodies by shape type

		Vec3 gravity;

		float internal_timer, timer_interval;

		Imp();
		~Imp();

		void AddRigidBody(RigidBody* body);
		bool RemoveRigidBody(RigidBody* body);

		void GetUseMass(const Vec3& direction, const ContactPoint& cp, float& A, float& B);

		// returns whether or not a collision response was actually needed (i.e. whether an impulse was applied to prevent interpenetration)
		bool DoCollisionResponse(const ContactPoint& cp);

		void DoFixedStep();

		void RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time = 1.0f, RigidBody* ibody = NULL);
	};




	/*
	 * RigidBody private implementation struct
	 */
	struct RigidBody::Imp
	{
		Vec3 pos;
		Vec3 vel;
		Quaternion ori;
		Vec3 rot;

		Vec3 force, torque;
		Vec3 applied_force, applied_torque;

		bool apply_force_directly;

		Vec3 gravity;

		MassInfo mass_info;
		CollisionShape* shape;

		// cached values for inverses of stuff
		float inv_mass;
		Mat3 inv_moi;

		bool xform_valid;
		Mat4 xform, inv_xform;

		float bounciness;
		float friction;

		bool can_move, can_rotate;
		bool active;								// TODO: support deactivation and related stuffs

		Entity* user_entity;

		CollisionCallback* collision_callback;

		Imp() : gravity(), mass_info(), shape(NULL), user_entity(NULL), collision_callback(NULL) { }
		Imp(CollisionShape* shape, MassInfo mass_info, Vec3 pos = Vec3(), Quaternion ori = Quaternion::Identity()) :
			pos(pos),
			vel(),
			ori(ori),
			rot(),
			force(),
			torque(),
			applied_force(),
			applied_torque(),
			apply_force_directly(false),
			gravity(),
			mass_info(mass_info),
			shape(shape),
			xform_valid(false),
			bounciness(!shape->CanMove() ? 1.0f : shape->GetShapeType() == ST_Ray ? 0.8f : 0.2f),
			friction(shape->GetShapeType() == ST_InfinitePlane ? 1.0f : shape->GetShapeType() == ST_Ray ? 0.0f : 1.0f),
			can_move(shape->CanMove() && mass_info.mass > 0),
			can_rotate(false),
			active(can_move),
			user_entity(NULL),
			collision_callback(NULL)
		{
			Mat3 moi_rm(mass_info.moi);

			if(moi_rm.Determinant() != 0.0f)
				can_rotate = true;

			inv_mass = mass_info.mass = 0.0f ? 0.0f : 1.0f / mass_info.mass;
			inv_moi = ComputeInvMoi();
		}

		~Imp()
		{
			if(shape != NULL)
			{
				shape->Dispose();
				delete shape;

				shape = NULL;
			}
		}

		Mat3 ComputeInvMoi() { Mat3 rm(ori.ToMat3()); return rm.Transpose() * Mat3::Invert(Mat3(mass_info.moi)) * rm; }

		void UpdateVel(float timestep)
		{
			if(active)
			{
				vel += (force * timestep) / mass_info.mass;

				if(can_rotate)
				{
					inv_moi = ComputeInvMoi();
					rot += inv_moi * (torque * timestep);
				}
			}

			ResetToApplied();
		}

		void UpdatePos(float timestep)
		{
			if(active)
			{
				pos += ori.ToMat3().Transpose() * mass_info.com;
				pos += vel * timestep;

				ori *= Quaternion::FromPYR(rot * timestep);

				pos -= ori.ToMat3().Transpose() * mass_info.com;

				xform_valid = false;
			}
		}

		void ComputeXform()
		{
			xform = Mat4::FromPositionAndOrientation(pos, ori);
			inv_xform = Mat4::Invert(xform);

			xform_valid = true;
		}

		void ComputeXformAsNeeded() { if(!xform_valid) { ComputeXform(); } }

		void ResetForces() 
		{
			applied_force = can_move ? gravity * mass_info.mass : Vec3();
			applied_torque = Vec3();
		}

		void ResetToApplied()
		{
			force = applied_force;
			torque = applied_torque;
		}

		// force is a world-space direction
		// local_poi is in the coordinate system of the object
		// returns a world-space direction
		Vec3 LocalForceToTorque(const Vec3& force, const Vec3& local_poi) { return Vec3::Cross(force, ori.ToMat3().Transpose() * (local_poi - mass_info.com)); }

		// point is in world-space
		// returns a world-space velocity
		Vec3 GetLocalVelocity(const Vec3& point) { return vel + Vec3::Cross(point - (pos + ori.ToMat3().Transpose() * mass_info.com), rot); }

		// impulse is a world-space direction
		// local_poi is in the coordinate system of the object
		void ApplyImpulse(const Vec3& impulse, const Vec3& local_poi)
		{
			if(active)
			{
				if(can_rotate)
					rot += inv_moi * LocalForceToTorque(impulse, local_poi);
				vel += impulse * inv_mass;
			}
		}

		void ApplyCentralImpulse(const Vec3& impulse) { if(active) { vel += impulse * inv_mass; } }
	};




	/*
	 * PhysicsWorld and PhysicsWorld::Imp methods
	 */
	PhysicsWorld::Imp::Imp() : rigid_bodies(), gravity(0, -9.8f, 0), internal_timer(), timer_interval(1.0f / 60.0f) { }

	PhysicsWorld::Imp::~Imp()
	{
		// dispose rigid bodies
		for(boost::unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->Dispose();
		rigid_bodies.clear();
	}

	void PhysicsWorld::Imp::AddRigidBody(RigidBody* r)
	{
		if(r == NULL)
			return;

		if(rigid_bodies.find(r) != rigid_bodies.end())
			return;

		rigid_bodies.insert(r);
		if(r->GetCollisionShape() == NULL)
			Debug("RigidBody has a NULL collision shape!\n");

		ShapeType shape_type = r->GetCollisionShape()->GetShapeType();
		if(shape_bodies.find(shape_type) == shape_bodies.end())
			shape_bodies[shape_type] = boost::unordered_set<RigidBody*>();
		shape_bodies[shape_type].insert(r);

		// set gravity upon adding to world
		r->imp->gravity = gravity;
	}
			
	bool PhysicsWorld::Imp::RemoveRigidBody(RigidBody* r)
	{
		if(r == NULL)
			return false;

		boost::unordered_set<RigidBody*>::iterator found = rigid_bodies.find(r);
		if(found != rigid_bodies.end())
		{
			rigid_bodies.erase(found);

			ShapeType shape_type = r->GetCollisionShape()->GetShapeType();
			if(shape_bodies.find(shape_type) != shape_bodies.end())
				shape_bodies[shape_type].erase(r);

			return true;
		}
		else
			return false;
	}

	// much cleaner than that quadratic formula nonsense
	void PhysicsWorld::Imp::GetUseMass(const Vec3& direction, const ContactPoint& cp, float& A, float& B)
	{
		RigidBody* ibody = cp.a.obj;
		RigidBody* jbody = cp.b.obj;
		RigidBody::Imp* iimp = ibody->imp;
		RigidBody::Imp* jimp = jbody->imp;

		float m1 = iimp->mass_info.mass, m2 = jimp->mass_info.mass;

		A = B = 0;

		if(jimp->can_move)
		{
			A = iimp->inv_mass + jimp->inv_mass;

			Vec3 i_lvel = iimp->vel, j_lvel = jimp->vel;
			B = Vec3::Dot(i_lvel, direction) - Vec3::Dot(j_lvel, direction);
		}
		else
		{
			A = iimp->inv_mass;
			B = Vec3::Dot(iimp->vel, direction);
		}

		if(iimp->can_rotate)
		{
			Vec3 nr1 = Vec3::Cross(direction, cp.a.pos - ibody->GetTransformationMatrix().TransformVec3(iimp->mass_info.com, 1.0f));
			A += Vec3::Dot(iimp->inv_moi * nr1, nr1);
			B += Vec3::Dot(iimp->rot, nr1);
		}

		if(jimp->can_rotate)
		{
			Vec3 nr2 = Vec3::Cross(direction, cp.b.pos - jbody->GetTransformationMatrix().TransformVec3(jimp->mass_info.com, 1.0f));
			A += Vec3::Dot(jimp->inv_moi * nr2, nr2);
			B -= Vec3::Dot(jimp->rot, nr2);
		}
	}

	bool PhysicsWorld::Imp::DoCollisionResponse(const ContactPoint& cp)
	{
		// TODO: make it so if A and B were swapped, the outcome would be the same

		RigidBody* ibody = cp.a.obj;
		RigidBody* jbody = cp.b.obj;

		RigidBody::Imp* iimp = ibody->imp;
		RigidBody::Imp* jimp = jbody->imp;

		bool j_can_move = jimp->can_move;

		float m1 = iimp->mass_info.mass;
		float m2 = jimp->mass_info.mass;

		if(m1 + m2 > 0)
		{
			Vec3 i_poi = ibody->GetInvTransform().TransformVec3(cp.a.pos, 1.0f);
			Vec3 j_poi = jbody->GetInvTransform().TransformVec3(cp.b.pos, 1.0f);

			Vec3 i_v = iimp->GetLocalVelocity(cp.a.pos);
			Vec3 j_v = jimp->GetLocalVelocity(cp.b.pos);
					
			Vec3 dv = j_v - i_v;
			const Vec3& normal = Vec3::Normalize(cp.a.norm - cp.b.norm);

			float nvdot = Vec3::Dot(normal, dv);
			if(nvdot < 0.0f)
			{
				float A, B;
				GetUseMass(normal, cp, A, B);

				float use_mass = 1.0f / A;
				float bounciness = iimp->bounciness * jimp->bounciness;
				float impulse_mag = -(1.0f + bounciness) * B * use_mass;
				
				if(impulse_mag < 0)
				{
					Vec3 impulse = normal * impulse_mag;

					if(impulse.ComputeMagnitudeSquared() != 0)
					{
						ibody->ApplyImpulse(impulse, i_poi);
						if(j_can_move)
							jbody->ApplyImpulse(-impulse, j_poi);
					}

					float sfric_coeff = iimp->friction * jimp->friction;
					float kfric_coeff = 0.9f * sfric_coeff;

					Vec3 t_dv = dv - normal * nvdot;
					float t_dv_magsq = t_dv.ComputeMagnitudeSquared();

					if(t_dv_magsq > 0.001f)							// object is moving; apply kinetic friction
					{
						float t_dv_mag = sqrtf(t_dv_magsq);

						GetUseMass(t_dv / t_dv_mag, cp, A, B);
						use_mass = 1.0f / A;

						Vec3 fric_impulse = t_dv * min(use_mass, fabs(impulse_mag * kfric_coeff / t_dv_mag));

						ibody->ApplyImpulse(fric_impulse, i_poi);
						if(j_can_move)
							jbody->ApplyImpulse(-fric_impulse, j_poi);
					}
					else											// object isn't moving; apply static friction
					{
						Vec3 df = jimp->applied_force - iimp->applied_force;
						float nfdot = Vec3::Dot(normal, df);

						Vec3 t_df = df - normal * nfdot;
						float t_df_mag = t_df.ComputeMagnitude();

						float fric_i_mag = min(impulse_mag * sfric_coeff, t_df_mag);
						if(fric_i_mag > 0)
						{
							Vec3 fric_impulse = t_df * (-fric_i_mag / t_df_mag);

							ibody->ApplyImpulse(fric_impulse, i_poi);
							if(j_can_move)
								jbody->ApplyImpulse(-fric_impulse, j_poi);
						}
					}

					return true;
				}
			}
		}

		return false;
	}

	void PhysicsWorld::Imp::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time, RigidBody* ibody)
	{
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator spheres = shape_bodies.find(ST_Sphere);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator meshes = shape_bodies.find(ST_TriangleMesh);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator planes = shape_bodies.find(ST_InfinitePlane);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator multispheres = shape_bodies.find(ST_MultiSphere);

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
		if(spheres != shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator jter = spheres->second.begin(); jter != spheres->second.end(); ++jter)
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
		}
				
		// ray-mesh collisions
		if(meshes != shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator jter = meshes->second.begin(); jter != meshes->second.end(); ++jter)
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
		}

		// ray-plane collisions
		if(planes != shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator jter = planes->second.begin(); jter != planes->second.end(); ++jter)
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
		}

		// ray-multisphere collisions
		if(multispheres != shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator jter = multispheres->second.begin(); jter != multispheres->second.end(); ++jter)
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
		}

		if(!hits.empty())
		{
			hits.sort();

			for(list<Hit>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
				if(callback.OnCollision(jter->p))
					break;
		}
	}

	void PhysicsWorld::Imp::DoFixedStep()
	{
		float timestep = timer_interval;

		// set forces to what was applied by gravity / user forces
		for(boost::unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->imp->UpdateVel(timestep);

		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator rays = shape_bodies.find(ST_Ray);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator spheres = shape_bodies.find(ST_Sphere);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator meshes = shape_bodies.find(ST_TriangleMesh);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator planes = shape_bodies.find(ST_InfinitePlane);	
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator multispheres = shape_bodies.find(ST_MultiSphere);

		// handle all the collisions involving rays
		if(rays != shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator iter = rays->second.begin(); iter != rays->second.end(); ++iter)
			{
				RigidBody* ibody = *iter;

				struct RayCallback : public CollisionCallback
				{
					Imp* imp;
					CollisionCallback* callback;
					RayCallback(Imp* imp, CollisionCallback* callback) : imp(imp), callback(callback) { }

					bool OnCollision(const ContactPoint& cp)			// return value controls whether to continue iterating through ray collisions
					{
						if(callback && !callback->OnCollision(cp))
							return true;
						else
						{
							imp->DoCollisionResponse(cp);
							
							return true;
						}
					}
				} my_callback(this, ibody->GetCollisionCallback());

				RayTest(ibody->GetPosition(), ibody->GetPosition() + ibody->GetLinearVelocity(), my_callback, timestep, ibody);
			}
		}


		CollisionGraph collision_graph;


		// handle all the collisions involving spheres
		if(spheres != shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator iter = spheres->second.begin(); iter != spheres->second.end(); ++iter)
			{
				RigidBody* ibody = *iter;
				float radius = ((SphereShape*)ibody->GetCollisionShape())->radius;

				Vec3 pos = ibody->GetPosition();
				Vec3 vel = ibody->GetLinearVelocity();

				list<ContactPoint> hits;

				// sphere-sphere collisions
				for(boost::unordered_set<RigidBody*>::iterator jter = iter; jter != spheres->second.end(); ++jter)
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
				if(meshes != shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = meshes->second.begin(); jter != meshes->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						TriangleMeshShape* shape = (TriangleMeshShape*)jbody->GetCollisionShape();

						Ray ray;
						ray.origin = pos;
						ray.direction = vel;

						vector<unsigned int> relevant_triangles = shape->GetRelevantTriangles(AABB(Vec3(pos.x - radius, pos.y - radius, pos.z - radius), Vec3(pos.x + radius, pos.y + radius, pos.z + radius)));
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
				}

				// sphere-plane collisions
				if(planes != shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = planes->second.begin(); jter != planes->second.end(); ++jter)
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
				}

				// sphere-multisphere collisions
				if(multispheres != shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = multispheres->second.begin(); jter != multispheres->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						MultiSphereShape* shape = (MultiSphereShape*)jbody->GetCollisionShape();

						Mat4 inv_xform = Mat4::Invert(jbody->GetTransformationMatrix());
						Vec3 nu_pos = inv_xform.TransformVec3(pos, 1.0f);

						ContactPoint cp;
						if(shape->CollisionCheck(Sphere(nu_pos, radius), cp, ibody, jbody))
							hits.push_back(cp);
					}
				}

				if(!hits.empty())
				{
					CollisionCallback* callback = ibody->GetCollisionCallback();

					for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
					{
						if(callback)
							callback->OnCollision(*jter);

						collision_graph.AddContactPoint(*jter);
					}
				}
			}
		}



		// handle all the collisions involving multispheres
		if(multispheres != shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator iter = multispheres->second.begin(); iter != multispheres->second.end(); ++iter)
			{
				RigidBody* ibody = *iter;
				MultiSphereShape* ishape = (MultiSphereShape*)ibody->GetCollisionShape();

				Vec3 vel = ibody->GetLinearVelocity();
				Vec3 pos = ibody->GetPosition();
				Mat4 xform = ibody->GetTransformationMatrix();

				list<ContactPoint> hits;

				// multisphere-mesh collisions
				if(meshes != shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = meshes->second.begin(); jter != meshes->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						TriangleMeshShape* jshape = (TriangleMeshShape*)jbody->GetCollisionShape();

						// TODO: implement this
					}
				}

				// multisphere-plane collisions
				if(planes != shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = planes->second.begin(); jter != planes->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						InfinitePlaneShape* jshape = (InfinitePlaneShape*)jbody->GetCollisionShape();

						ContactPoint p;
						if(ishape->CollisionCheck(xform, jshape->plane, p, ibody, jbody))
							hits.push_back(p);
					}
				}

				// multisphere-multisphere collisions
				for(boost::unordered_set<RigidBody*>::iterator jter = iter; jter != multispheres->second.end(); ++jter)
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
				{
					CollisionCallback* callback = ibody->GetCollisionCallback();

					for(list<ContactPoint>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
					{
						if(callback)
							callback->OnCollision(*jter);

						collision_graph.AddContactPoint(*jter);
					}
				}
			}
		}

		// solve collision graph
		for(int i = 0; i < MAX_SEQUENTIAL_SOLVER_ITERATIONS; ++i)
		{
			bool any = false;
			for(vector<ContactPoint*>::iterator iter = collision_graph.contact_points.begin(); iter != collision_graph.contact_points.end(); ++iter)
				if(DoCollisionResponse(**iter))
					any = true;

			if(!any)
				break;
		}

		// update positions
		for(boost::unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->imp->UpdatePos(timestep);
	}

	PhysicsWorld::PhysicsWorld() : imp(new Imp()) { }

	void PhysicsWorld::InnerDispose() { delete imp; imp = NULL; }

	void PhysicsWorld::AddRigidBody(RigidBody* r) { imp->AddRigidBody(r); }
	bool PhysicsWorld::RemoveRigidBody(RigidBody* r) { return imp->RemoveRigidBody(r); }

	void PhysicsWorld::Update(TimingInfo time)
	{
		imp->internal_timer += time.elapsed;
		while(imp->internal_timer >= imp->timer_interval)
		{
			imp->DoFixedStep();
			imp->internal_timer -= imp->timer_interval;
		}

		for(boost::unordered_set<RigidBody*>::iterator iter = imp->rigid_bodies.begin(); iter != imp->rigid_bodies.end(); ++iter)
		{
			(*iter)->ResetForces();
			(*iter)->imp->apply_force_directly = false;
		}
	}

	void PhysicsWorld::DebugDrawWorld(SceneRenderer* renderer)
	{
		for(boost::unordered_set<RigidBody*>::iterator iter = imp->rigid_bodies.begin(); iter != imp->rigid_bodies.end(); ++iter)
			(*iter)->DebugDraw(renderer);

		renderer->Render();
		renderer->Cleanup();
	}

	Vec3 PhysicsWorld::GetGravity() { return imp->gravity; }
	void PhysicsWorld::SetGravity(const Vec3& gravity)
	{
		imp->gravity = gravity;

		// set gravity of all rigid bodies within the world
		for(boost::unordered_set<RigidBody*>::iterator iter = imp->rigid_bodies.begin(); iter != imp->rigid_bodies.end(); ++iter)
			(*iter)->imp->gravity = gravity;
	}

	void PhysicsWorld::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback) { imp->RayTest(from, to, callback); }



	
	/*
	 * RigidBody methods
	 */
	RigidBody::RigidBody() : imp(new Imp()) { }
	RigidBody::RigidBody(CollisionShape* shape, MassInfo mass_info, Vec3 pos, Quaternion ori) : imp(new Imp(shape, mass_info, pos, ori)) { }
	
	void RigidBody::InnerDispose()
	{
		if(imp != NULL)
		{
			delete imp; 
			imp = NULL; 
		}
	}
	void RigidBody::DisposePreservingCollisionShape() { imp->shape = NULL; Dispose(); }

	Vec3 RigidBody::GetPosition() { return imp->pos; }
	void RigidBody::SetPosition(Vec3 pos) { imp->pos = pos; imp->xform_valid = false; }

	Quaternion RigidBody::GetOrientation() { return imp->ori; }
	void RigidBody::SetOrientation(Quaternion ori) { imp->ori = ori; imp->xform_valid = false; }

	Mat4 RigidBody::GetTransformationMatrix() { imp->ComputeXformAsNeeded(); return imp->xform; }
	Mat4 RigidBody::GetInvTransform() { imp->ComputeXformAsNeeded(); return imp->inv_xform; }

	void RigidBody::SetBounciness(float bounciness) { imp->bounciness = bounciness; }
	void RigidBody::SetFriction(float friction) { imp->friction = friction; }
	float RigidBody::GetBounciness() { return imp->bounciness; }
	float RigidBody::GetFriction() { return imp->friction; }

	bool RigidBody::MergesSubgraphs() { return imp->shape->CanMove() && imp->shape->GetShapeType() != ST_Ray; }

	void RigidBody::ApplyForce(const Vec3& force, const Vec3& local_poi)
	{
		if(imp->apply_force_directly)
		{
			imp->torque += imp->LocalForceToTorque(force, local_poi);
			imp->force += force;
		}
		else
		{
			imp->applied_torque += imp->LocalForceToTorque(force, local_poi);
			imp->applied_force += force;
		}
	}

	void RigidBody::ApplyImpulse(const Vec3& impulse, const Vec3& local_poi) { imp->ApplyImpulse(impulse, local_poi); }

	void RigidBody::ApplyCentralForce(const Vec3& force)
	{
		if(imp->apply_force_directly)
			imp->force += force;
		else
			imp->applied_force += force; 
	}

	void RigidBody::ApplyCentralImpulse(const Vec3& impulse) { imp->ApplyCentralImpulse(impulse); }

	void RigidBody::ResetForces() { imp->ResetForces(); }

	Vec3 RigidBody::GetLinearVelocity() { return imp->vel; }
	void RigidBody::SetLinearVelocity(const Vec3& vel) { imp->vel = vel; }

	Vec3 RigidBody::GetAngularVelocity() { return imp->rot; }
	void RigidBody::SetAngularVelocity(const Vec3& vel) { imp->rot = vel; }

	Vec3 RigidBody::GetLocalVelocity(const Vec3& point) { return imp->GetLocalVelocity(point); }

	MassInfo RigidBody::GetMassInfo() { return imp->mass_info; }

	void RigidBody::DebugDraw(SceneRenderer* renderer) { imp->shape->DebugDraw(renderer, imp->pos, imp->ori); }

	void RigidBody::SetCollisionCallback(CollisionCallback* callback) { imp->collision_callback = callback; }
	CollisionCallback* RigidBody::GetCollisionCallback() { return imp->collision_callback; }

	CollisionShape* RigidBody::GetCollisionShape() { return imp->shape; }

	Entity* RigidBody::GetUserEntity() { return imp->user_entity; }
	void RigidBody::SetUserEntity(Entity* entity) { imp->user_entity = entity; }




	/*
	 * MassInfo methods
	 */
	MassInfo::MassInfo() : mass(0), com(), moi() { }
	MassInfo::MassInfo(Vec3 point, float mass) : mass(mass), com(point), moi() { }

	void MassInfo::operator +=(MassInfo right)
	{
		float totalmass = mass + right.mass;
		Vec3 new_com = (com * mass + right.com * right.mass) / totalmass;

		float left_moi[9];
		float right_moi[9];
		GetAlternatePivotMoI(new_com - com, moi, mass, left_moi);
		GetAlternatePivotMoI(new_com - right.com, right.moi, right.mass, right_moi);

		mass = totalmass;
		com = new_com;
		for(int i = 0; i < 9; ++i)
			moi[i] = left_moi[i] + right_moi[i];
	}
	MassInfo MassInfo::operator +(MassInfo other) { MassInfo temp = *this; temp += other; return temp; }

	void MassInfo::operator *=(float coeff)
	{
		mass *= coeff;

		for(int i = 0; i < 9; ++i)
			moi[i] *= coeff;
	}
	MassInfo MassInfo::operator *(float coeff) { MassInfo temp = *this; temp *= coeff; return temp; }

	void MassInfo::GetAlternatePivotMoI(Vec3 a, float* I, float m, float* result)
	{
		float a_squared = a.ComputeMagnitudeSquared();

		result[0] = I[0] + m * (a_squared * 1 - a.x * a.x);
		result[1] = I[1] + m * (a_squared * 0 - a.x * a.y);
		result[2] = I[2] + m * (a_squared * 0 - a.x * a.z);

		result[3] = I[3] + m * (a_squared * 0 - a.y * a.x);
		result[4] = I[4] + m * (a_squared * 1 - a.y * a.y);
		result[5] = I[5] + m * (a_squared * 0 - a.y * a.z);

		result[6] = I[6] + m * (a_squared * 0 - a.z * a.x);
		result[7] = I[7] + m * (a_squared * 0 - a.z * a.y);
		result[8] = I[8] + m * (a_squared * 1 - a.z * a.z);
	}

	MassInfo MassInfo::FromCollisionShape(CollisionShape* shape, float mass) { return shape->ComputeMassInfo() * mass; }

	MassInfo MassInfo::ReadMassInfo(istream& stream)
	{
		MassInfo result;

		result.mass = ReadSingle(stream);
		result.com = ReadVec3(stream);

		for(char i = 0; i < 9; ++i)
			result.moi[i] = ReadSingle(stream);

		return result;
	}

	void MassInfo::Write(ostream& stream)
	{
		WriteSingle(mass, stream);
		WriteVec3(com, stream);

		for(char i = 0; i < 9; ++i)
			WriteSingle(moi[i], stream);
	}
}
