#include "StdAfx.h"
#include "Physics.h"

#include "CollisionShape.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Util.h"

namespace CibraryEngine
{
	/*
	 * PhysicsWorld private implementation struct
	 */
	struct PhysicsWorld::Imp
	{
		boost::unordered_set<RigidBody*> rigid_bodies;			// List of all of the rigid bodies in the physical simulation

		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> > shape_bodies;		// lists of rigid bodies by shape type

		Vec3 gravity;

		Imp();
		~Imp();

		void AddRigidBody(RigidBody* body);
		bool RemoveRigidBody(RigidBody* body);
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
		bool active;						// TODO: support deactivation and related stuffs

		Entity* user_entity;

		CollisionCallback* collision_callback;

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
			collision_callback(NULL)
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

		void Update(TimingInfo time)
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
			force = gravity * mass_info.mass;
			torque = force_impulse = torque_impulse = Vec3();
		}
	};




	/*
	 * PhysicsWorld and PhysicsWorld::Imp methods
	 */
	PhysicsWorld::Imp::Imp() : rigid_bodies(), gravity(0, -9.8f, 0) { }

	PhysicsWorld::Imp::~Imp()
	{
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

	PhysicsWorld::PhysicsWorld() : imp(new Imp()) { }

	void PhysicsWorld::InnerDispose() { delete imp; imp = NULL; }

	void PhysicsWorld::AddRigidBody(RigidBody* r) { imp->AddRigidBody(r); }
	bool PhysicsWorld::RemoveRigidBody(RigidBody* r) { return imp->RemoveRigidBody(r); }

	void PhysicsWorld::Update(TimingInfo time)
	{
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
		// friction goes here

		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator rays = imp->shape_bodies.find(ST_Ray);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator spheres = imp->shape_bodies.find(ST_Sphere);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator meshes = imp->shape_bodies.find(ST_TriangleMesh);
		boost::unordered_map<ShapeType, boost::unordered_set<RigidBody*> >::iterator planes = imp->shape_bodies.find(ST_InfinitePlane);

		// handle all the collisions involving rays
		if(rays != imp->shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator iter = rays->second.begin(); iter != rays->second.end(); ++iter)
			{
				RigidBody* ibody = *iter;

				Ray ray;
				ray.origin = ibody->GetPosition();
				ray.direction = ibody->GetLinearVelocity();

				struct Hit
				{
					float t;
					ContactPoint p;

					Hit(float t, ContactPoint p) : t(t), p(p) { }
					bool operator <(Hit& h) { return t < h.t; }
				};
				list<Hit> hits;

				if(spheres != imp->shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = spheres->second.begin(); jter != spheres->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						float first, second;

						if(Util::RaySphereIntersect(ray, Sphere(jbody->GetPosition(), ((SphereShape*)jbody->GetCollisionShape())->radius), first, second))
							if(first > 0 && first <= time.elapsed)
							{
								ContactPoint p;

								p.obj_a = ibody;
								p.obj_b = jbody;
								p.pos_a = ray.origin + first * ray.direction;
								p.norm_b = Vec3::Normalize(p.pos_a - jbody->GetPosition(), 1.0f);

								hits.push_back(Hit(first, p));
							}
					}
				}
				
				if(meshes != imp->shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = meshes->second.begin(); jter != meshes->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;

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

								p.obj_a = ibody;
								p.obj_b = jbody;
								p.pos_a = ray.origin + t * ray.direction;
								p.norm_b = kter->normal;

								hits.push_back(Hit(t, p));
							}
						}
					}
				}

				if(planes != imp->shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = planes->second.begin(); jter != planes->second.end(); ++jter)
					{
						RigidBody* jbody = *jter;
						InfinitePlaneShape* plane = (InfinitePlaneShape*)jbody->GetCollisionShape();

						float t = Util::RayPlaneIntersect(ray, plane->plane);
						if(t > 0 && t <= time.elapsed)
						{
							ContactPoint p;

							p.obj_a = ibody;
							p.obj_b = jbody;
							p.pos_a = ray.origin + t * ray.direction;
							p.norm_b = plane->plane.normal;

							hits.push_back(Hit(t, p));
						}
					}
				}

				hits.sort();

				CollisionCallback* callback = ibody->GetCollisionCallback();
				if(callback != NULL)
				{
					for(list<Hit>::iterator jter = hits.begin(); jter != hits.end(); ++jter)
						if(callback->OnCollision(jter->p))
							break;
				}
				else if(!hits.empty())
				{
					// TODO: bounce?
				}
			}
		}

		// handle all the collisions involving spheres
		if(spheres != imp->shape_bodies.end())
		{
			for(boost::unordered_set<RigidBody*>::iterator iter = spheres->second.begin(); iter != spheres->second.end(); ++iter)
			{
				for(boost::unordered_set<RigidBody*>::iterator jter = iter; jter != spheres->second.end(); ++jter)
				{
					if(*iter != *jter)
					{
						// TODO: sphere-sphere
					}
				}

				if(meshes != imp->shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = meshes->second.begin(); jter != meshes->second.end(); ++jter)
					{
						// TODO: sphere-mesh
					}
				}

				if(planes != imp->shape_bodies.end())
				{
					for(boost::unordered_set<RigidBody*>::iterator jter = planes->second.begin(); jter != planes->second.end(); ++jter)
					{
						// TODO: sphere-plane
					}
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
