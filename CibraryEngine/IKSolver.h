#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	class Skeleton;
	class PhysicsWorld;

	struct TimingInfo;

	/** System for batch-processing inverse kinematics for multiple entities */
	class IKSolver
	{
		protected:

			/** Struct used internally by the IKSolver system */
			struct IKObject
			{
				/** Temporary variable used to store the result between calls to ComputeNextState and ApplyComputedState */
				Skeleton* result;

				/**
				 * Skeleton used for both input and output
				 * Stores bone positions, including the position and orientation of the root bone
				 */
				Skeleton* skeleton;

				// TODO: add desired-state info here

				/** Constructs an IK object with the given skeleton for I/O */
				IKObject(Skeleton* skeleton);

				/** Computes the state the skeleton should assume at the end of the update, and stores that temporarily */
				void ComputeNextState(PhysicsWorld* physics, TimingInfo time);

				/** Modifies the i/o skeleton to match the temporary result skeleton, then deletes the temporary one */
				void ApplyComputedState();
			};

			/** Collection of all the objects using inverse kinematics, indexed by a user pointer */
			map<void*, IKObject*> ik_objects;

		public:

			/** The physics world in which these inverse kinematics objects exist, and upon which their solutions depend */
			PhysicsWorld* physics;

			/** Creates an IKSolver for the given physics world */
			IKSolver(PhysicsWorld* physics);

			/** Removes all IK objects from the solver */
			void ClearObjects();
			/** Creates an IK object mapped to the given user pointer, assigning to it the given Skeleton */
			void AddObject(void* user_ptr, Skeleton* skeleton);
			/** Deletes the IK object mapped to the given user pointer */
			void DeleteObject(void* user_ptr);
			/** Gets the skeleton of the IK object mapped to the given user pointer */
			Skeleton* GetObjectSkeleton(void* user_ptr);
			// TODO: add desired-state setter function here

			/** Updates the IK objects */
			void Update(TimingInfo time);
	};
}