#include "StdAfx.h"
#include "Physics.h"

#include "CollisionShape.h"
#include "RayShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Util.h"

namespace CibraryEngine
{
	static vector<ContactPoint*> cp_recycle_bin;




	/*
	 * PhysicsWorld private implementation struct
	 */
	struct PhysicsWorld::Imp
	{
		boost::unordered_set<RigidBody*> rigid_bodies;											// List of all of the rigid bodies in the physical simulation

		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> > shape_bodies;		// lists of rigid bodies by shape type

		boost::unordered_set<ContactPoint*> contact_points;										// list of continuous contact points between objects

		Vec3 gravity;

		Imp();
		~Imp();

		void AddRigidBody(RigidBody* body);
		bool RemoveRigidBody(RigidBody* body);

		void DoCollisionResponse(ContactPoint& cp);
		bool DoContinuousContactUpdate(ContactPoint* cp);										// returns true if the contact point should persist

		ContactPoint* AddContinuousContact(const ContactPoint& cp);
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

		Vec3 force, force_impulse;
		Vec3 torque, torque_impulse;

		Vec3 gravity;

		MassInfo mass_info;
		CollisionShape* shape;

		bool can_move, can_rotate;
		bool active;								// TODO: support deactivation and related stuffs

		Entity* user_entity;

		CollisionCallback* collision_callback;

		boost::unordered_set<ContactPoint*> contact_points;

		Imp() : gravity(), mass_info(), shape(NULL), user_entity(NULL), collision_callback(NULL) { }
		Imp(CollisionShape* shape, MassInfo mass_info, Vec3 pos = Vec3(), Quaternion ori = Quaternion::Identity()) :
			pos(pos),
			vel(),
			ori(ori),
			rot(),
			gravity(),
			mass_info(mass_info),
			shape(shape),
			can_move(shape->CanMove()),
			can_rotate(false),
			active(can_move),
			user_entity(NULL),
			collision_callback(NULL),
			contact_points()
		{
			Mat3 moi_rm(mass_info.moi);

			if(moi_rm.Determinant() != 0.0f)
				can_rotate = true;
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

		void Update(const TimingInfo& time)
		{
			if(active)
			{
				float timestep = time.elapsed;

				pos += vel * timestep;
				ori *= Quaternion::FromPYR(rot);

				vel += (force * timestep + force_impulse) / mass_info.mass;

				if(can_rotate)
					rot += Mat3::Invert(ori.ToMat3() * Mat3(mass_info.moi)) * (torque * timestep + torque_impulse);
			}

			ResetForces();	
		}

		void ResetForces() 
		{
			force = can_move ? gravity * mass_info.mass : Vec3();
			torque = force_impulse = torque_impulse = Vec3();
		}

		bool NeedsCollision(RigidBody* other)
		{
			for(boost::unordered_set<ContactPoint*>::iterator iter = contact_points.begin(); iter != contact_points.end(); ++iter)
			{
				ContactPoint* cp = *iter;
				if(cp->a.obj == other || cp->b.obj == other)
					return false;
			}

			return true;
		}
	};




	/*
	 * PhysicsWorld and PhysicsWorld::Imp methods
	 */
	PhysicsWorld::Imp::Imp() : rigid_bodies(), gravity(0, -9.8f, 0) { }

	PhysicsWorld::Imp::~Imp()
	{
		// dispose rigid bodies
		for(boost::unordered_set<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->Dispose();
		rigid_bodies.clear();
		
		// put contact points into recycle bin
		for(boost::unordered_set<ContactPoint*>::iterator iter = contact_points.begin(); iter != contact_points.end(); ++iter)
			cp_recycle_bin.push_back(*iter);
		contact_points.clear();
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

			for(boost::unordered_set<ContactPoint*>::iterator iter = r->imp->contact_points.begin(); iter != r->imp->contact_points.end(); ++iter)
			{
				ContactPoint* cp = *iter;

				contact_points.erase(cp);
				cp_recycle_bin.push_back(cp);
				
				if(cp->a.obj == r)
					cp->b.obj->imp->contact_points.erase(cp);
				else
					cp->a.obj->imp->contact_points.erase(cp);
			}

			ShapeType shape_type = r->GetCollisionShape()->GetShapeType();
			if(shape_bodies.find(shape_type) != shape_bodies.end())
				shape_bodies[shape_type].erase(r);

			return true;
		}
		else
			return false;
	}

	void PhysicsWorld::Imp::DoCollisionResponse(ContactPoint& cp)
	{
		// TODO: make it so if A and B were swapped, the outcome would be the same

		RigidBody* ibody = cp.a.obj;
		RigidBody* jbody = cp.b.obj;

		bool j_can_move = jbody->imp->can_move;

		float m1 = ibody->imp->mass_info.mass;
		float m2 = jbody->imp->mass_info.mass;
		if(m1 + m2 > 0)
		{
			Vec3 i_poi = Mat4::Invert(ibody->GetTransformationMatrix()).TransformVec3(cp.a.pos, 1.0f);
			Vec3 j_poi = Mat4::Invert(jbody->GetTransformationMatrix()).TransformVec3(cp.b.pos, 1.0f);
					
			Vec3 dv = jbody->GetLinearVelocity() - ibody->GetLinearVelocity();
			const Vec3& normal = cp.a.norm;

			float nvdot = Vec3::Dot(normal, dv);

			if(fabs(nvdot) > 0.01f)
			{
				Vec3 impulse = normal * (nvdot * (j_can_move ? (m1 * m2 / (m1 + m2)) : m1));				// TODO: get this formula right

				if(impulse.ComputeMagnitudeSquared() != 0)
				{
					ibody->ApplyImpulse(impulse, i_poi);
					if(j_can_move)
						jbody->ApplyImpulse(-impulse, j_poi);
				}
			}
			
			//AddContinuousContact(cp);
			DoContinuousContactUpdate(&cp);
		}
	}

	bool PhysicsWorld::Imp::DoContinuousContactUpdate(ContactPoint* cp)
	{
		// TODO: update the contact point, and determine if it needs to be severed; this will probably depend on the types of collision shapes involved

		RigidBody* ibody = cp->a.obj;
		RigidBody* jbody = cp->b.obj;

		bool j_can_move = jbody->imp->can_move;

		float m1 = ibody->imp->mass_info.mass;
		float m2 = jbody->imp->mass_info.mass;
		if(m1 + m2 > 0)
		{
			Vec3 i_poi = Mat4::Invert(ibody->GetTransformationMatrix()).TransformVec3(cp->a.pos, 1.0f);
			Vec3 j_poi = Mat4::Invert(jbody->GetTransformationMatrix()).TransformVec3(cp->b.pos, 1.0f);

			Vec3 df = jbody->imp->force - ibody->imp->force;
			const Vec3& normal = Vec3::Normalize(cp->a.norm - cp->b.norm);

			Vec3 dv = jbody->GetLinearVelocity() - ibody->GetLinearVelocity();

			// see if there neeeds to be a normal force applied to keep these objects from interpenetrating any further
			float nfdot = Vec3::Dot(normal, df);
			if(nfdot < 0)
			{
				float f_mag = nfdot * (j_can_move ? (m1 * m2 / (m1 + m2)) : 1.0f);			// TODO: get the other case right
				Vec3 normal_force = normal * f_mag;

				ibody->ApplyForce(normal_force, i_poi);
				if(j_can_move)
					jbody->ApplyForce(-normal_force, j_poi);

				// apply friction
				float kfric_coeff = 0.9f;					// TODO: compute this based on properties the objects, maybe?
				float sfric_coeff = 1.0f;

				Vec3 t_dv = dv - normal * Vec3::Dot(dv, normal);
				float t_dv_magsq = t_dv.ComputeMagnitudeSquared();

				if(t_dv_magsq > 0.000001f)					// object is moving; apply kinetic friction
				{
					Vec3 fric_force = t_dv * (-f_mag * kfric_coeff / sqrtf(t_dv_magsq));

					ibody->ApplyForce(fric_force, i_poi);
					if(j_can_move)
						jbody->ApplyForce(-fric_force, j_poi);
				}
				else										// object isn't moving; apply static friction
				{
					Vec3 t_df = df - normal * nfdot;
					float t_df_mag = t_df.ComputeMagnitude();

					float fric_f_mag = min(f_mag * sfric_coeff, t_df_mag);
					if(fric_f_mag > 0)
					{
						Vec3 fric_force = t_df * (-fric_f_mag / t_df_mag);

						ibody->ApplyForce(fric_force, i_poi);
						if(j_can_move)
							jbody->ApplyForce(-fric_force, j_poi);
					}
				}
			}
			
			float nvdot = Vec3::Dot(dv, normal);
			if(nvdot > 0.0f)
 				return false;
		}

		return true;		// TODO: figure out under what conditions a continuous contact point should be removed
	}

	ContactPoint* PhysicsWorld::Imp::AddContinuousContact(const ContactPoint& cp)
	{
		ContactPoint* result;
		if(cp_recycle_bin.empty())
			result = new ContactPoint();
		else
		{
			result = cp_recycle_bin[cp_recycle_bin.size() - 1];
			cp_recycle_bin.pop_back();
		}

		*result = cp;

		cp.a.obj->imp->contact_points.insert(result);
		cp.b.obj->imp->contact_points.insert(result);
		
		contact_points.insert(result);
		return result;
	}

	PhysicsWorld::PhysicsWorld() : imp(new Imp()) { }

	void PhysicsWorld::InnerDispose() { delete imp; imp = NULL; }

	void PhysicsWorld::AddRigidBody(RigidBody* r) { imp->AddRigidBody(r); }
	bool PhysicsWorld::RemoveRigidBody(RigidBody* r) { return imp->RemoveRigidBody(r); }

	void PhysicsWorld::Update(TimingInfo time)
	{
		// update continuous contact points
		for(boost::unordered_set<ContactPoint*>::iterator iter = imp->contact_points.begin(); iter != imp->contact_points.end();)
		{
			ContactPoint* cp = *iter;
			if(!imp->DoContinuousContactUpdate(cp))
			{
				cp_recycle_bin.push_back(cp);

				// erase contact point
				cp->a.obj->imp->contact_points.erase(cp);
				cp->b.obj->imp->contact_points.erase(cp);
				iter = imp->contact_points.erase(iter);
			}
			else
				++iter;
		}



		// TODO: collision detection:
		//
		//		start storing broadphase info
		//		rigid bodies: tell the broadphase what it needs to know about you
		//
		//		start collection of batched operations
		//		rigid bodies: based on broadphase info, add appropriate batch ops (somehow make sure not to do any pairs doubly)
		//
		//		do the batched operations
		//

		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator rays = imp->shape_bodies.find(ST_Ray);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator spheres = imp->shape_bodies.find(ST_Sphere);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator meshes = imp->shape_bodies.find(ST_TriangleMesh);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator planes = imp->shape_bodies.find(ST_InfinitePlane);

		struct Hit
		{
			float t;
			ContactPoint p;

			Hit(float t, ContactPoint p) : t(t), p(p) { }
			bool operator <(Hit& h) { return t < h.t; }
		};

		// handle all the collisions involving rays
		if(rays != imp->shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator iter = rays->second.begin(); iter != rays->second.end(); ++iter)
			{
				RigidBody* ibody = *iter;

				Ray ray;
				ray.origin = ibody->GetPosition();
				ray.direction = ibody->GetLinearVelocity();

				list<Hit> hits;

				// ray-sphere collisions
				if(spheres != imp->shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = spheres->second.begin(); jter != spheres->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						if(!ibody->imp->NeedsCollision(jbody))
							continue;

						float first, second;

						if(Util::RaySphereIntersect(ray, Sphere(jbody->GetPosition(), ((SphereShape*)jbody->GetCollisionShape())->radius), first, second))
							if(first > 0 && first <= time.elapsed)
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
				if(meshes != imp->shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = meshes->second.begin(); jter != meshes->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						if(!ibody->imp->NeedsCollision(jbody))
							continue;

						TriangleMeshShape* mesh = (TriangleMeshShape*)jbody->GetCollisionShape();
						
						vector<unsigned int> index_a, index_b, index_c;
						for(vector<TriangleMeshShape::Tri>::iterator kter = mesh->triangles.begin(); kter != mesh->triangles.end(); ++kter)
						{
							index_a.push_back(kter->indices[0]);
							index_b.push_back(kter->indices[1]);
							index_c.push_back(kter->indices[2]);
						}

						vector<Ray> ray_list;
						ray_list.push_back(ray);			// TODO: transform this to account for the rigid body's orientation and position

						vector<Intersection> mesh_hits = Util::RayTriangleListIntersect(mesh->vertices, index_a, index_b, index_c, ray_list)[0];
						
						for(vector<Intersection>::iterator kter = mesh_hits.begin(); kter != mesh_hits.end(); ++kter)
						{
							float t = kter->time;
							if(t > 0 && t < time.elapsed)
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
				if(planes != imp->shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = planes->second.begin(); jter != planes->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						if(!ibody->imp->NeedsCollision(jbody))
							continue;

						InfinitePlaneShape* plane = (InfinitePlaneShape*)jbody->GetCollisionShape();

						float t = Util::RayPlaneIntersect(ray, plane->plane);
						if(t > 0 && t <= time.elapsed)
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

				if(!hits.empty())
				{
					hits.sort();

					if(CollisionCallback* callback = ibody->GetCollisionCallback())
					{
						for(list<Hit>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
							if(callback->OnCollision(jter->p))
								break;
					}
					else
					{
						// TODO: bounce off the first object?
					}
				}
			}
		}

		// handle all the collisions involving spheres
		if(spheres != imp->shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator iter = spheres->second.begin(); iter != spheres->second.end(); ++iter)
			{
				RigidBody* ibody = *iter;

				float radius = ((SphereShape*)ibody->GetCollisionShape())->radius;

				Vec3 pos = ibody->GetPosition();
				Vec3 vel = ibody->GetLinearVelocity();

				list<Hit> hits;

				// sphere-sphere collisions
				for(boost::unordered_set<RigidBody*>::iterator jter = iter; jter != spheres->second.end(); ++jter)
				{
					if(iter != jter)
					{
						RigidBody* jbody = *jter;
						if(!ibody->imp->NeedsCollision(jbody))
							continue;

						float sr = radius + ((SphereShape*)jbody->GetCollisionShape())->radius;

						Vec3 other_pos = jbody->GetPosition();
						Vec3 other_vel = jbody->GetLinearVelocity();

						Ray ray;
						ray.origin = pos - other_pos;
						ray.direction = vel - other_vel;

						float first = 0, second = 0;
						if(ray.origin.ComputeMagnitudeSquared() < sr * sr || Util::RaySphereIntersect(ray, Sphere(Vec3(), sr), first, second))
							if(first >= 0 && first <= time.elapsed)
							{
								ContactPoint p;

								p.a.obj = ibody;
								p.b.obj = jbody;
								p.a.pos = pos + first * vel;
								p.b.pos = other_pos + first * other_vel;
								p.b.norm = Vec3::Normalize(p.a.pos- other_pos, 1.0f);
								p.a.norm = -p.b.norm;

								hits.push_back(Hit(first, p));
							}
					}
				}

				if(meshes != imp->shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = meshes->second.begin(); jter != meshes->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						if(!ibody->imp->NeedsCollision(jbody))
							continue;

						Ray ray;
						ray.origin = pos;
						ray.direction = vel;

						TriangleMeshShape* shape = (TriangleMeshShape*)jbody->GetCollisionShape();
						for(vector<TriangleMeshShape::Tri>::iterator kter = shape->triangles.begin(); kter != shape->triangles.end(); ++kter)
						{
							unsigned int* indices = kter->indices;
							Vec3& a = shape->vertices[indices[0]];
							Vec3& b = shape->vertices[indices[1]];
							Vec3& c = shape->vertices[indices[2]];

							Plane plane(Plane::FromTriangleVertices(a, b, c));

							float vndot = Vec3::Dot(vel, plane.normal);
							if(vndot <= 0.0f)
							{
								float dist = Util::TriangleMinimumDistance(a, b, c, pos);

								if(dist < radius)
								{

									ContactPoint p;

									p.a.obj = ibody;
									p.b.obj = jbody;
									p.a.pos = pos;
									p.b.pos = pos;
									p.b.norm = -plane.normal;
									p.a.norm = plane.normal;

									hits.push_back(Hit(0, p));
								}
							}
						}
					}
				}

				// sphere-plane collisions
				if(planes != imp->shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = planes->second.begin(); jter != planes->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						if(!ibody->imp->NeedsCollision(jbody))
							continue;

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
							if(t >= 0 && t <= time.elapsed)
							{
								ContactPoint p;

								p.a.obj = ibody;
								p.b.obj = jbody;
								p.a.pos = pos + t * vel;
								p.b.pos = p.a.pos - plane_norm * (Vec3::Dot(p.a.pos, plane_norm) - plane_offset);
								p.b.norm = plane_norm;
								p.a.norm = -plane_norm;

								hits.push_back(Hit(t, p));
							}
						}
					}
				}

				if(!hits.empty())
				{
					hits.sort();

					if(CollisionCallback* callback = ibody->GetCollisionCallback())
					{
						for(list<Hit>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
							if(callback->OnCollision(jter->p))
							{
								imp->DoCollisionResponse(jter->p);
								break;
							}
					}
					else 
						imp->DoCollisionResponse(hits.begin()->p);
				}
			}
		}

		// update positions and apply forces
		for(boost::unordered_set<RigidBody*>::iterator iter = imp->rigid_bodies.begin(); iter != imp->rigid_bodies.end(); ++iter)
			(*iter)->Update(time);
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
	void RigidBody::SetPosition(Vec3 pos) { imp->pos = pos; }

	Quaternion RigidBody::GetOrientation() { return imp->ori; }
	void RigidBody::SetOrientation(Quaternion ori) { imp->ori = ori; }

	Mat4 RigidBody::GetTransformationMatrix() { return Mat4::FromPositionAndOrientation(imp->pos, imp->ori); }

	void RigidBody::ApplyForce(const Vec3& force, const Vec3& local_poi)
	{
		imp->torque += Vec3::Cross(force, local_poi);
		imp->force += force;
	}
	void RigidBody::ApplyImpulse(const Vec3& impulse, const Vec3& local_poi)
	{
		imp->torque_impulse += Vec3::Cross(impulse, local_poi);;
		imp->force_impulse += impulse;
	}

	void RigidBody::ApplyCentralForce(const Vec3& force) { imp->force += force; }
	void RigidBody::ApplyCentralImpulse(const Vec3& impulse) { imp->force_impulse += impulse; }

	void RigidBody::ResetForces() { imp->ResetForces(); }

	Vec3 RigidBody::GetLinearVelocity() { return imp->vel; }
	void RigidBody::SetLinearVelocity(const Vec3& vel) { imp->vel = vel; }

	void RigidBody::Update(TimingInfo time) { imp->Update(time); }
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
