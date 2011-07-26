#include "Physics.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"

#include "btBulletWorldImporter.h"
#include "btBulletFile.h"

namespace CibraryEngine
{
	/*
	 * PhysicsWorld methods
	 */
	PhysicsWorld::PhysicsWorld() :
		broadphase(new btDbvtBroadphase()),
		collision_configuration(new btDefaultCollisionConfiguration()),
		dispatcher(new btCollisionDispatcher(collision_configuration)),
		solver(new btSequentialImpulseConstraintSolver()),
		//dynamics_world(new btContinuousDynamicsWorld(dispatcher, broadphase, solver, collision_configuration)),
		dynamics_world(new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collision_configuration)),
		rigid_bodies()
	{
		dynamics_world->setGravity(btVector3(0, -9.8f, 0));
	}

	void PhysicsWorld::InnerDispose()
	{
		for(list<RigidBodyInfo*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); iter++)
		{
			dynamics_world->removeRigidBody((*iter)->body);
			(*iter)->Dispose();
		}
		rigid_bodies.clear();

		delete dynamics_world;
		delete solver;
		delete dispatcher;
		delete collision_configuration;
		delete broadphase;
	}

	void PhysicsWorld::AddRigidBody(RigidBodyInfo* r)
	{
		if(r == NULL)
			return;

		for(list<RigidBodyInfo*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); iter++)
			if(*iter == r)
				return;

		dynamics_world->addRigidBody(r->body);
		rigid_bodies.push_back(r);
	}

	bool PhysicsWorld::RemoveRigidBody(RigidBodyInfo* r)
	{
		if(r == NULL)
			return false;

		for(list<RigidBodyInfo*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); iter++)
			if(*iter == r)
			{
				rigid_bodies.erase(iter);
				dynamics_world->removeRigidBody(r->body);
				return true;
			}

		return false;
	}

	void PhysicsWorld::Update(TimingInfo time)
	{
		dynamics_world->stepSimulation(time.elapsed, 20);
	}




	/*
	 * RigidBodyInfo methods
	 */
	RigidBodyInfo::RigidBodyInfo(btCollisionShape* shape, MassInfo mass_info, Vec3 pos, Quaternion ori) : shape(shape), body(NULL), motion_state(NULL)
	{
		float mass = mass_info.mass;
		Vec3 inertia = mass_info.GetDiagonalMoI();

		motion_state = new btDefaultMotionState(btTransform(btQuaternion(ori.x, ori.y, ori.z, ori.w), btVector3(pos.x, pos.y, pos.z)));

		body = new btRigidBody(btRigidBody::btRigidBodyConstructionInfo(mass, NULL, shape, btVector3(inertia.x, inertia.y, inertia.z)));
		body->setMotionState(motion_state);
	}

	void RigidBodyInfo::InnerDispose()
	{
		delete body;
		delete shape;
		delete motion_state;
	}

	void RigidBodyInfo::DisposePreservingCollisionShape()
	{
		shape = NULL;
		Dispose();
	}

	Vec3 RigidBodyInfo::GetPosition()
	{
		btTransform transform;
		motion_state->getWorldTransform(transform);
		btVector3 origin = transform.getOrigin();

		return Vec3((float)origin.getX(), (float)origin.getY(), (float)origin.getZ());
	}

	void RigidBodyInfo::SetPosition(Vec3 pos)
	{
		btTransform transform;
		motion_state->getWorldTransform(transform);
		transform.setOrigin(btVector3(pos.x, pos.y, pos.z));
		motion_state->setWorldTransform(transform);

		body->setMotionState(motion_state);
	}

	void RigidBodyInfo::SetOrientation(Quaternion ori)
	{
		btTransform transform;
		motion_state->getWorldTransform(transform);
		transform.setRotation(btQuaternion(ori.x, ori.y, ori.z, ori.w));
		motion_state->setWorldTransform(transform);

		body->setMotionState(motion_state);
	}

	Mat4 RigidBodyInfo::GetTransformationMatrix()
	{
		btTransform transform;
		motion_state->getWorldTransform(transform);
		btVector3 offset = transform.getOrigin();
		btQuaternion rot = transform.getRotation();

		return Mat4::FromPositionAndOrientation(Vec3(offset.getX(), offset.getY(), offset.getZ()), Quaternion(rot.getW(), rot.getX(), rot.getY(), rot.getZ()));
	}




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
		for(int i = 0; i < 9; i++)
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
