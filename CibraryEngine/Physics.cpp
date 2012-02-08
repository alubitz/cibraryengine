#include "StdAfx.h"
#include "Physics.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"

#include "btBulletWorldImporter.h"
#include "btBulletFile.h"

namespace CibraryEngine
{
	/*
	 * PhysicsWorld private implementation struct
	 */
	struct PhysicsWorld::Imp
	{
		btBroadphaseInterface* broadphase;
		btDefaultCollisionConfiguration* collision_configuration;
		btCollisionDispatcher* dispatcher;
		btSequentialImpulseConstraintSolver* solver;
		btDynamicsWorld* dynamics_world;

		list<RigidBodyInfo*> rigid_bodies;			// List of all of the rigid bodies in the physical simulation

		Imp();
		~Imp();

		void AddRigidBody(RigidBodyInfo* body);
		bool RemoveRigidBody(RigidBodyInfo* body);
	};




	/*
	 * RigidBodyInfo private implementation struct
	 */
	struct RigidBodyInfo::Imp
	{
		/** The shape of the object */
		btCollisionShape* shape;
		/** Bullet Physics Library's rigid body */
		btRigidBody* body;

		/** The position and orientation of this object */
		btMotionState* motion_state;

		Imp() : shape(NULL), body(NULL), motion_state(NULL) { }
		Imp(btCollisionShape* shape, MassInfo mass_info, Vec3 pos = Vec3(), Quaternion ori = Quaternion::Identity()) : shape(shape), body(NULL), motion_state(NULL)
		{
			float mass = mass_info.mass;
			Vec3 inertia = mass_info.GetDiagonalMoI();

			motion_state = new btDefaultMotionState(btTransform(btQuaternion(ori.x, ori.y, ori.z, ori.w), btVector3(pos.x, pos.y, pos.z)));

			body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, NULL, shape, btVector3(inertia.x, inertia.y, inertia.z)));
			body->setMotionState(motion_state);
		}

		~Imp()
		{
			delete body;
			delete shape;
			delete motion_state;
		}

		Vec3 GetPosition()
		{
			btTransform transform;
			motion_state->getWorldTransform(transform);
			btVector3 origin = transform.getOrigin();

			return Vec3((float)origin.getX(), (float)origin.getY(), (float)origin.getZ());
		}

		void SetPosition(Vec3 pos)
		{
			btTransform transform;
			motion_state->getWorldTransform(transform);
			transform.setOrigin(btVector3(pos.x, pos.y, pos.z));
			motion_state->setWorldTransform(transform);

			body->setMotionState(motion_state);
		}

		void SetOrientation(Quaternion ori)
		{
			btTransform transform;
			motion_state->getWorldTransform(transform);
			transform.setRotation(btQuaternion(ori.x, ori.y, ori.z, ori.w));
			motion_state->setWorldTransform(transform);

			body->setMotionState(motion_state);
		}

		Mat4 GetTransformationMatrix()
		{
			btTransform transform;
			motion_state->getWorldTransform(transform);
			btVector3 offset = transform.getOrigin();
			btQuaternion rot = transform.getRotation();

			return Mat4::FromPositionAndOrientation(Vec3(offset.getX(), offset.getY(), offset.getZ()), Quaternion(rot.getW(), rot.getX(), rot.getY(), rot.getZ()));
		}

		void SetCustomCollisionEnabled(void* user_object)
		{
			body->setUserPointer(user_object);

			if(user_object)
				body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
			else
				body->setCollisionFlags((body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK) ^ btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
		}
	};




	/*
	 * ConeTwistConstraint private implementation struct
	 */
	struct ConeTwistConstraint::Imp
	{
		btConeTwistConstraint* constraint;

		Imp(btConeTwistConstraint* constraint) : constraint(constraint) { }
	};




	/*
	 * PhysicsWorld and PhysicsWorld::Imp methods
	 */
	PhysicsWorld::Imp::Imp() :
		broadphase(new btDbvtBroadphase()),
		collision_configuration(new btDefaultCollisionConfiguration()),
		dispatcher(new btCollisionDispatcher(collision_configuration)),
		solver(new btSequentialImpulseConstraintSolver()),
		dynamics_world(new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collision_configuration)),
		rigid_bodies()
	{
		dynamics_world->setGravity(btVector3(0, -9.8f, 0));
	}

	PhysicsWorld::Imp::~Imp()
	{
		for(list<RigidBodyInfo*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
		{
			dynamics_world->removeRigidBody((*iter)->imp->body);
			(*iter)->Dispose();
		}
		rigid_bodies.clear();

		delete dynamics_world;
		delete solver;
		delete dispatcher;
		delete collision_configuration;
		delete broadphase;
	}

	void PhysicsWorld::Imp::AddRigidBody(RigidBodyInfo* r)
	{
		if(r == NULL)
			return;

		for(list<RigidBodyInfo*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			if(*iter == r)
				return;

		dynamics_world->addRigidBody(r->imp->body);
		rigid_bodies.push_back(r);
	}
			
	bool PhysicsWorld::Imp::RemoveRigidBody(RigidBodyInfo* r)
	{
		if(r == NULL)
			return false;

		for(list<RigidBodyInfo*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			if(*iter == r)
			{
				rigid_bodies.erase(iter);
				dynamics_world->removeRigidBody(r->imp->body);
				return true;
			}

		return false;
	}




	PhysicsWorld::PhysicsWorld() : imp(new Imp()) { }

	void PhysicsWorld::InnerDispose() { delete imp; imp = NULL; }

	void PhysicsWorld::AddRigidBody(RigidBodyInfo* r) { imp->AddRigidBody(r); }
	bool PhysicsWorld::RemoveRigidBody(RigidBodyInfo* r) { return imp->RemoveRigidBody(r); }

	void PhysicsWorld::AddConstraint(ConeTwistConstraint* constraint, bool disable_collision) { imp->dynamics_world->addConstraint(constraint->imp->constraint, disable_collision); }
	void PhysicsWorld::RemoveConstraint(ConeTwistConstraint* constraint) { imp->dynamics_world->removeConstraint(constraint->imp->constraint); }

	void PhysicsWorld::Update(TimingInfo time) { imp->dynamics_world->stepSimulation(time.elapsed, 20); }

	void PhysicsWorld::SetDebugDrawer(btIDebugDraw* d) { imp->dynamics_world->setDebugDrawer(d); }
	void PhysicsWorld::DebugDrawWorld() { imp->dynamics_world->debugDrawWorld(); }

	void PhysicsWorld::RayTest(Vec3 from, Vec3 to, btCollisionWorld::RayResultCallback& callback) { imp->dynamics_world->rayTest(btVector3(from.x, from.y, from.z), btVector3(to.x, to.y, to.z), callback); }
	void PhysicsWorld::ContactTest(RigidBodyInfo* object, btCollisionWorld::ContactResultCallback& callback) { imp->dynamics_world->contactTest(object->imp->body, callback); }

	Vec3 PhysicsWorld::GetGravity()
	{	
		btVector3 gravity_vector = imp->dynamics_world->getGravity();
		return Vec3(gravity_vector.getX(), gravity_vector.getY(), gravity_vector.getZ());
	}
	void PhysicsWorld::SetGravity(const Vec3& gravity) { imp->dynamics_world->setGravity(btVector3(gravity.x, gravity.y, gravity.z)); }



	
	/*
	 * RigidBodyInfo methods
	 */
	RigidBodyInfo::RigidBodyInfo() : imp(new Imp()) { }
	RigidBodyInfo::RigidBodyInfo(btCollisionShape* shape, MassInfo mass_info, Vec3 pos, Quaternion ori) : imp(new Imp(shape, mass_info, pos, ori)) { }
	
	void RigidBodyInfo::InnerDispose()
	{
		if(imp != NULL)
		{
			delete imp; 
			imp = NULL; 
		}
	}
	void RigidBodyInfo::DisposePreservingCollisionShape() { imp->shape = NULL; Dispose(); }

	Vec3 RigidBodyInfo::GetPosition() { return imp->GetPosition(); }
	void RigidBodyInfo::SetPosition(Vec3 pos) { imp->SetPosition(pos); }

	void RigidBodyInfo::SetOrientation(Quaternion ori) { imp->SetOrientation(ori); }

	Mat4 RigidBodyInfo::GetTransformationMatrix() { return imp->GetTransformationMatrix(); }

	void RigidBodyInfo::Activate() { imp->body->activate(); }
	void RigidBodyInfo::ApplyImpulse(const Vec3& impulse, const Vec3& local_poi) { imp->body->applyImpulse(btVector3(impulse.x, impulse.y, impulse.z), btVector3(local_poi.x, local_poi.y, local_poi.z)); }
	void RigidBodyInfo::ApplyCentralImpulse(const Vec3& impulse) { imp->body->applyCentralImpulse(btVector3(impulse.x, impulse.y, impulse.z)); }
	void RigidBodyInfo::ApplyCentralForce(const Vec3& force) { imp->body->applyCentralForce(btVector3(force.x, force.y, force.z)); }

	void RigidBodyInfo::ClearForces() { imp->body->clearForces(); }

	Vec3 RigidBodyInfo::GetLinearVelocity()
	{ 
		btVector3 vel = imp->body->getLinearVelocity(); 
		return Vec3(vel.getX(), vel.getY(), vel.getZ());
	}
	void RigidBodyInfo::SetLinearVelocity(const Vec3& vel) { imp->body->setLinearVelocity(btVector3(vel.x, vel.y, vel.z)); }

	void RigidBodyInfo::SetFriction(float friction) { imp->body->setFriction(friction); }
	void RigidBodyInfo::SetDamping(float linear, float angular) { imp->body->setDamping(linear, angular); }
	void RigidBodyInfo::SetRestitution(float restitution) { imp->body->setRestitution(restitution); }

	void RigidBodyInfo::SetDeactivationTime(float time) { imp->body->setDeactivationTime(time); }
	void RigidBodyInfo::SetSleepingThresholds(float linear, float angular) { imp->body->setSleepingThresholds(linear, angular); }

	void RigidBodyInfo::SetCustomCollisionEnabled(void* user_object) { imp->SetCustomCollisionEnabled(user_object); }




	/*
	 * MassInfo methods
	 */
	MassInfo::MassInfo() : mass(0), com(), moi() { }
	MassInfo::MassInfo(Vec3 point, float mass) : mass(mass), com(point), moi() { }

	Vec3 MassInfo::GetDiagonalMoI() { return Vec3(moi[0], moi[4], moi[8]); }

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




	/*
	 * ConeTwistConstraint methods
	 */
	ConeTwistConstraint::ConeTwistConstraint(RigidBodyInfo* a_body, RigidBodyInfo* b_body, Quaternion a_ori, Vec3 a_pos, Quaternion b_ori, Vec3 b_pos) : imp(NULL)
	{ 
		const btTransform a_frame = btTransform(btQuaternion(a_ori.x, a_ori.y, a_ori.z, a_ori.w), btVector3(a_pos.x, a_pos.y, a_pos.z));
		const btTransform b_frame = btTransform(btQuaternion(b_ori.x, b_ori.y, b_ori.z, b_ori.w), btVector3(b_pos.x, b_pos.y, b_pos.z));
		
		imp = new Imp(new btConeTwistConstraint(*a_body->imp->body, *b_body->imp->body, a_frame, b_frame));
	}

	void ConeTwistConstraint::InnerDispose() { delete imp; imp = NULL; }
	void ConeTwistConstraint::SetLimit(Vec3 limits) { imp->constraint->setLimit(limits.x, limits.y, limits.z); }
	void ConeTwistConstraint::SetDamping(float damp) { imp->constraint->setDamping(damp); }




	/*
	 * CollisionShape I/O methods
	 */
	void WriteCollisionShape(btCollisionShape* shape, ostream& stream)
	{
		// serializing the collision shape
		int maxSerializeBufferSize = 1024*1024*5;
		btDefaultSerializer* serializer = new btDefaultSerializer(maxSerializeBufferSize);

		serializer->startSerialization();
		shape->serializeSingleShape(serializer);
		serializer->finishSerialization();

		unsigned int buffer_size = serializer->getCurrentBufferSize();

		WriteUInt32(buffer_size, stream);
		stream.write((const char*)serializer->getBufferPointer(), buffer_size);

		delete serializer;
	}

	btCollisionShape* ReadCollisionShape(istream& stream)
	{
		unsigned int buffer_size = ReadUInt32(stream);
		char* shape_data = new char[buffer_size];
		stream.read(shape_data, buffer_size);

		btBulletWorldImporter importer;
		importer.loadFileFromMemory(shape_data, buffer_size);

		delete shape_data;

		if(importer.getNumCollisionShapes() == 0)
			return NULL;
		else
			return importer.getCollisionShapeByIndex(0);
	}
}
