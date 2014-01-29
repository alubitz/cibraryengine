#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	struct DATBone;
	class DATKeyframe;
	class Constraint;
	class CSkeletalJoint;
	class JointOrientations;
	struct PoseDelta;
	struct FixedJointPoseOp;
	class SolverInstance;

	class PoseyDood
	{
		private:

		public:

			ModelPhysics* mphys;

			vector<DATBone> bones;
			vector<unsigned int> id_to_bones;		// given a bone's "name", returns index into the above +1

			struct SpecialConstraint
			{
				string name;
				unsigned int index;		// index into constraints
				bool default_value;

				SpecialConstraint() { }
				SpecialConstraint(const string& name, unsigned int index, bool default_value) : name(name), index(index), default_value(default_value) { }
			};
			vector<SpecialConstraint> special_constraints;
			vector<CSkeletalJoint*>   skeletal_joints;
			vector<Constraint*>       constraints;

			Skeleton* skeleton;
			vector<Mat4> bone_matrices;
			SkinnedCharacterRenderInfo sk_rinfo;
			UberModel *dood_uber;

			PoseyDood();
			~PoseyDood();

			void LoadDood(const string& dood_name, ContentMan* content);	// need to separate this from constructor in order to have inheritance work properly

			virtual void CreateCustomHelperBones(Cache<ModelPhysics>* mphys_cache, Cache<UberModel>* uber_cache, Cache<Material>* mat_cache) { }
			virtual void CreateCustomConstraints() { }
			virtual void DoCustomKeyframeStuff(DATKeyframe& initial_pose) { }

			int GetBoneIndex(const string& bone_name);

			DATKeyframe GetDefaultPose();

			unsigned int AddSpecialConstraint(const string& name, bool default_val, Constraint* c);			// returns index of created SpecialConstraint
			void AddHelperBone(const string& bone_name, CollisionShape* shape, UberModel* uber);		

			JointOrientations JointOrientationsFromPose(const DATKeyframe& pose);
			void ApplyConstraints(SolverInstance& solver, const vector<PoseDelta>* deltas = NULL);
			float ScoreJOs(const DATKeyframe& test_pose, Constraint** constraints_begin, Constraint** constraints_end, const PoseDelta* deltas_begin, const PoseDelta* deltas_end);
			void GetFixedJointPoseOps(DATKeyframe& pose, bool* locked_bones, vector<FixedJointPoseOp>& results);

			void PoseBones(Skeleton* skeleton, const DATKeyframe& pose);
			void PoseBones(Skeleton* skeleton, const DATKeyframe& frame_a, const DATKeyframe& frame_b, float b_frac);
			void Vis(SceneRenderer* renderer, const DATKeyframe& pose, float timer, bool draw_uber);
	};
}
