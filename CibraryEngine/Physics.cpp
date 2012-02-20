#include "StdAfx.h"
#include "Physics.h"

#include "CollisionShape.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"

namespace CibraryEngine
{
	/*
	 * PhysicsWorld private implementation struct
	 */
	struct PhysicsWorld::Imp
	{
		boost::unordered_set<RigidBody*> rigid_bodies;			// List of all of the rigid bodies in the physical simulation

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

		CollisionCallback* collision_callback;

		Imp() : gravity(), mass_info(), shape() { }
		Imp(CollisionShape* shape, MassInfo mass_info, Vec3 pos = Vec3(), Quaternion ori = Quaternion::Identity()) :
			pos(pos),
			vel(),
			ori(ori),
			rot(),
			gravity(),
			mass_info(mass_info),
			shape(shape),
			collision_callback(NULL)
		{
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
			float timestep = time.elapsed;

			pos += vel * timestep;
			ori *= Quaternion::FromPYR(rot);

			vel += (force * timestep + force_impulse) / mass_info.mass;
			rot += Mat3::Invert(Mat3(mass_info.moi)) * (torque * timestep + torque_impulse);

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
		// find collisions, compute forces

		for(boost::unordered_set<RigidBody*>::iterator iter = imp->rigid_bodies.begin(); iter != imp->rigid_bodies.end(); ++iter)
			(*iter)->Update(time);
	}

	void PhysicsWorld::DebugDrawWorld() { }

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

	void RigidBody::SetCollisionCallback(CollisionCallback* callback) { imp->collision_callback = callback; }
	CollisionCallback* RigidBody::GetCollisionCallback() { return imp->collision_callback; }




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

	MassInfo MassInfo::FromCollisionShape(CollisionShape* shape, float mass)
	{
		// TODO: implement this for real
		return MassInfo(Vec3(), mass);
	}
}
