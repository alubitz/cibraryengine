#pragma once

#include "StdAfx.h"
#include "Disposable.h"
#include "Vector.h"
#include "Quaternion.h"

#include "SkeletalAnimation.h"

namespace CibraryEngine
{
	using namespace std;

	class Skeleton;
	class PhysicsWorld;

	struct TimingInfo;

	/** System for batch-processing inverse kinematics for multiple entities */
	class IKSolver : public Disposable
	{
		protected:

			/** Struct used internally by the IKSolver system */
			struct IKObject
			{
				/** Temporary variable used to store the result between calls to ComputeNextState and ApplyComputedState */
				Skeleton result;

				bool result_valid;

				/**
				 * Skeleton used for both input and output
				 * Stores bone positions, including the position and orientation of the root bone
				 */
				Skeleton* skeleton;

				/** The desired position of the character controller */
				Vec3 desired_pos;
				/** The desired orientation of the character controller */
				Quaternion desired_ori;

				/** The resultant position of the character controller */
				Vec3 result_pos;
				/** The resultant orientation of the character controller */
				Quaternion result_ori;

				/** Constructs an IK object with the given skeleton for I/O */
				IKObject(Skeleton* skeleton, Vec3 pos, Quaternion ori);

				/** Computes the state the skeleton should assume at the end of the update, and stores that temporarily */
				void ComputeNextState(PhysicsWorld* physics, TimingInfo time);

				/** Modifies the i/o skeleton to match the temporary result skeleton, then deletes the temporary one */
				void ApplyComputedState();
			};

			/** Collection of all the objects using inverse kinematics, indexed by a user pointer */
			map<void*, IKObject*> ik_objects;

			void InnerDispose();

		public:

			/** The physics world in which these inverse kinematics objects exist, and upon which their solutions depend */
			PhysicsWorld* physics;

			/** Creates an IKSolver for the given physics world */
			IKSolver(PhysicsWorld* physics);

			/** Removes all IK objects from the solver */
			void ClearObjects();
			/** Creates an IK object mapped to the given user pointer, assigning to it the given Skeleton */
			void AddObject(void* user_ptr, Skeleton* skeleton, Vec3 pos, Quaternion ori);
			/** Deletes the IK object mapped to the given user pointer */
			void DeleteObject(void* user_ptr);
			/** Gets the skeleton of the IK object mapped to the given user pointer */
			Skeleton* GetObjectSkeleton(void* user_ptr);
			
			void SetDesiredState(void* user_ptr, Vec3 pos, Quaternion ori);
			void GetResultState(void* user_ptr, Vec3& pos, Quaternion& ori);

			/** Updates the IK objects */
			void Update(TimingInfo time);
	};
}