#include "StdAfx.h"
#include "PoseyDood.h"

#include "DATBone.h"
#include "DATKeyframe.h"

#include "Constraint.h"
#include "CSkeletalJoint.h"
#include "CFixedJoint.h"

#include "PoseDelta.h"
#include "FixedJointPoseOp.h"
#include "PoseChainNode.h"

#define DEBUG_OUTPUT_SOLVER_PROGRESS   0

#define CONSTRAINT_SOLVER_ITERATIONS   1000
#define MUTATIONS_PER_ITERATION        3
#define MUTATION_RATE_PER_ERROR        0.05f
#define NO_PROGRESS_STOP_THRESHOLD     60

namespace DoodAnimTool
{
	/*
	 * PoseyDood methods
	 */
	PoseyDood::PoseyDood() : mphys(NULL), skeleton(NULL), dood_uber(NULL) { }

	void PoseyDood::LoadDood(const string& dood_name, ContentMan* content)
	{
		Cache<ModelPhysics>* mphys_cache = content->GetCache<ModelPhysics>();
		Cache<UberModel>*    uber_cache  = content->GetCache<UberModel>();
		Cache<Material>*     mat_cache   = content->GetCache<Material>();

		mphys     = mphys_cache->Load(dood_name);
		dood_uber = uber_cache->Load (dood_name);
		dood_uber->LoadCachedMaterials(mat_cache);

		skeleton = dood_uber->CreateSkeleton();

		bones.clear();
		
		ClearSolverStop();

		// match up skeleton bones with ModelPhysics bones
		unsigned int num_bones = skeleton->bones.size();
		for(unsigned int i = 0; i < num_bones; ++i)
		{
			Bone* bone = skeleton->bones[i];
			string bone_name = Bone::string_table[bone->name];

			for(vector<ModelPhysics::BonePhysics>::iterator jter = mphys->bones.begin(); jter != mphys->bones.end(); ++jter)
			{
				ModelPhysics::BonePhysics* pbone = &*jter;
				if(pbone->bone_name == bone_name)
				{
					bones.push_back(DATBone(i, bone->name, pbone->collision_shape));
					break;
				}
			}
		}

		// emancipate all the bones
		for(unsigned int i = 0; i < skeleton->bones.size(); ++i)
			skeleton->bones[i]->parent = NULL;

		CreateCustomHelperBones(mphys_cache, uber_cache, mat_cache);

		// setup to make GetBoneIndex work
		{
			unsigned int biggest_bone = 0;
			for(unsigned int i = 0; i < bones.size(); ++i)
				biggest_bone = max(biggest_bone, bones[i].name);

			id_to_bones.clear();
			id_to_bones.resize(biggest_bone + 1);

			for(unsigned int i = 0; i < bones.size(); ++i)
				id_to_bones[bones[i].name] = i + 1;
		}

		

		// create constraints (starting with the soldier-specific ones)
		CreateCustomConstraints();

		// add default constraints to the constraints list
		for(unsigned int i = 0; i < mphys->joints.size(); ++i)
		{
			CSkeletalJoint* sj = new CSkeletalJoint(mphys, i, bones);
			skeletal_joints.push_back(sj);
			constraints.push_back(sj);
		}
	}

	PoseyDood::~PoseyDood()
	{
		if(skeleton)				{ skeleton->Dispose();					delete skeleton;				skeleton = NULL; }
		if(sk_rinfo.bone_matrices)	{ sk_rinfo.bone_matrices->Dispose();	delete sk_rinfo.bone_matrices;	sk_rinfo.bone_matrices = NULL; }

		skeletal_joints.clear();
		for(vector<Constraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
			delete *iter;
		constraints.clear();
	}



	int PoseyDood::GetBoneIndex(const string& bone_name)
	{
		unsigned int id = Bone::string_table[bone_name];
		return id < id_to_bones.size() ? (signed)id_to_bones[id] - 1 : -1;
	}


	DATKeyframe PoseyDood::GetDefaultPose()
	{
		DATKeyframe pose(bones.size(), constraints.size());

		DoCustomKeyframeStuff(pose);

		for(vector<SpecialConstraint>::iterator iter = special_constraints.begin(); iter != special_constraints.end(); ++iter)
			pose.enabled_constraints[iter->index] = iter->default_value;

		return pose;
	}



	unsigned int PoseyDood::AddSpecialConstraint(const string& name, bool default_val, Constraint* c)
	{
		unsigned int pos = special_constraints.size();
				
		constraints.push_back(c);

		SpecialConstraint sc(name, pos, default_val);
		special_constraints.push_back(sc);

		return pos;
	}

	void PoseyDood::AddHelperBone(const string& bone_name, CollisionShape* shape, UberModel* uber)
	{
		unsigned int id = Bone::string_table[bone_name];
		skeleton->AddBone(id, Quaternion::Identity(), Vec3());
		bones.push_back(DATBone(skeleton->bones.size() - 1, id, shape, uber));
	}

	void PoseyDood::ClearSolverStop()
	{
		cached_score = -1;

#if DEBUG_OUTPUT_SOLVER_PROGRESS
		if(!stopped && noprogress_count != 0)
			Debug(((stringstream&)(stringstream() << "aborted after " << noprogress_count << " calls resulting in no progress" << endl)).str());
#endif

		stopped = false;

		noprogress_count = 0;
		debug_text = string();
	}

	void PoseyDood::OnSolverStop(float value)
	{
		cached_score = value;
		stopped = true;

		debug_text = ((stringstream&)(stringstream() << "(STOPPED) " << cached_score)).str();
#if DEBUG_OUTPUT_SOLVER_PROGRESS
		Debug(((stringstream&)(stringstream() << "STOPPED: " << cached_score << endl)).str());
#endif
	}

	JointOrientations PoseyDood::JointOrientationsFromPose(const DATKeyframe& pose)
	{
		unsigned int num_joints = skeletal_joints.size();

		JointOrientations result(num_joints);
		for(unsigned int i = 0; i < num_joints; ++i)
		{
			const ModelPhysics::JointPhysics& joint = *skeletal_joints[i]->joint;
			unsigned int bone_a = joint.bone_a - 1;
			unsigned int bone_b = joint.bone_b - 1;

			Quaternion a_to_b = Quaternion::Reverse(pose.data[bones[bone_a].bone_index].ori) * pose.data[bones[bone_b].bone_index].ori;
			result.data[i] = joint.GetClampedAngles(joint.axes * -a_to_b.ToRVec());
		}

		return result;
	}

	void PoseyDood::ApplyConstraints(DATKeyframe& pose, const vector<PoseDelta>* deltas)
	{
		// apply user edits
		if(deltas != NULL)
		{
			for(vector<PoseDelta>::const_iterator iter = deltas->begin(); iter != deltas->end(); ++iter)
				pose.data[iter->bone] = iter->new_state;

			ClearSolverStop();
		}
		else if(stopped)
			return;

		// get a condensed list of the active constraints
		vector<Constraint*> active_constraints;
		active_constraints.reserve(constraints.size());
		for(unsigned int i = 0; i < constraints.size(); ++i)
			if(pose.enabled_constraints[i])
				active_constraints.push_back(constraints[i]);
		Constraint** constraints_begin = active_constraints.data();
		Constraint** constraints_end   = constraints_begin + active_constraints.size();

		// figure out what bones to start from when going from JointOrientations to DATKeyframe
		unsigned int num_bones = bones.size();
		bool* locked_bones = new bool[num_bones];
		memset(locked_bones, 0, num_bones * sizeof(bool));
		for(unsigned int i = 0; i < num_bones; ++i)												// locked bones' xforms are initially known
			if(bones[i].locked)
				locked_bones[i] = true;
		for(Constraint** iter = constraints_begin; iter != constraints_end; ++iter)				// some constraints may set and lock a bones' initial xform as well
			(*iter)->SetLockedBones(pose, locked_bones);

		vector<FixedJointPoseOp> fjpops;														// CFixedJoint constraints may necessitate a special action later
		GetFixedJointPoseOps(pose, locked_bones, fjpops);

		bool any = false;																		// we need at least one bone to start from... see if we have one
		for(unsigned int i = 0; i < num_bones; ++i)
			if(locked_bones[i])
			{
				any = true;
				break;
			}
		if(!any)																				// if all else fails, start from bone 0
			locked_bones[0] = true;

		// get a list of deltas relevant to bones that aren't locked (implicitly or explicitly)
		PoseDelta *deltas_begin, *deltas_end;
		if(deltas != NULL)
		{
			deltas_begin = new PoseDelta[deltas->size()];
			deltas_end = deltas_begin;

			for(vector<PoseDelta>::const_iterator iter = deltas->begin(); iter != deltas->end(); ++iter)
				if(!locked_bones[iter->bone])
					*(deltas_end++) = *iter;
		}
		else
			deltas_begin = deltas_end = NULL;

		// figure out in what order the bone posing operations should be done (when going from JointOrientations to DATKeyframe)
		JointOrientations target_jos = cached_score >= 0 ? cached_jos : JointOrientationsFromPose(pose);
		vector<PoseChainNode> pose_chain = target_jos.GetPoseChain(mphys, locked_bones);
		PoseChainNode* chain_begin = pose_chain.data();
		PoseChainNode* chain_end   = chain_begin + pose_chain.size();

		delete[] locked_bones;

		// search for optimal solution
		DATKeyframe       best_pose(pose),      test_pose(pose);
		JointOrientations best_jos(target_jos), test_jos(target_jos);
		float             best_score,           test_score;

		unsigned int found_count = 0;
		float first_score;

		for(unsigned int i = 0; i < CONSTRAINT_SOLVER_ITERATIONS; ++i)
		{
			test_jos = best_jos;
			if(i != 0)
			{
				// mutate jos
				float mutation_rate = MUTATION_RATE_PER_ERROR * sqrtf(best_score);
				float coeff = mutation_rate * 2.0f, sub = mutation_rate;
				for(unsigned int j = 0; j < MUTATIONS_PER_ITERATION; ++j)
				{
					unsigned int index = Random3D::RandInt(test_jos.num_joints);

					Vec3& mutant = test_jos.data[index];
					mutant.x += Random3D::Rand() * coeff - sub;
					mutant.y += Random3D::Rand() * coeff - sub;
					mutant.z += Random3D::Rand() * coeff - sub;

					skeletal_joints[index]->joint->ClampAngles(mutant);
				}
			}

			// see what these jos do, score the results, and keep track of which jos have done the best so far
			test_pose = best_pose;
			test_jos.UsePoseChain(chain_begin, chain_end, test_pose);

			for(vector<FixedJointPoseOp>::iterator iter = fjpops.begin(); iter != fjpops.end(); ++iter)
			{
				const DATKeyframe::KBone& from = test_pose.data[iter->from];
				DATKeyframe::KBone& to         = test_pose.data[iter->to];

				Mat4 newxform = Mat4::FromPositionAndOrientation(from.pos, from.ori) * iter->xform;
				newxform.Decompose(to.pos, to.ori);
				to.ori = Quaternion::Reverse(to.ori);			// something fishy going on here...
			}


			test_score = ScoreJOs(test_pose, constraints_begin, constraints_end, deltas_begin, deltas_end);

			if(i == 0 || test_score < best_score)
			{
				best_score = test_score;
				best_pose  = test_pose;
				best_jos   = test_jos;

				++found_count;

				if(i == 0)
					first_score = best_score;
			}
		}

		pose = best_pose;
		cached_score = best_score;
		cached_jos   = best_jos;
		stopped = false;

		if(deltas_begin != NULL) { delete[] deltas_begin; }

		if(found_count == 1)
		{
			++noprogress_count;
			if(noprogress_count >= NO_PROGRESS_STOP_THRESHOLD)
			{
#if DEBUG_OUTPUT_SOLVER_PROGRESS
				Debug(((stringstream&)(stringstream() << noprogress_count << " calls resulting in no progress" << endl)).str());
#endif
				OnSolverStop(best_score);

				return;
			}
			else
			{
				if(noprogress_count == 1)
					debug_text = ((stringstream&)(stringstream() << best_score)).str();
			}
		}
		else
		{
			if(noprogress_count > 0)
			{
#if DEBUG_OUTPUT_SOLVER_PROGRESS
				Debug(((stringstream&)(stringstream() << noprogress_count << " " << (noprogress_count == 1 ? "call" : "calls") << " resulting in no progress" << endl)).str());
#endif
				noprogress_count = 0;
			}

			debug_text = ((stringstream&)(stringstream() << best_score)).str();
#if DEBUG_OUTPUT_SOLVER_PROGRESS
			Debug(((stringstream&)(stringstream() << "found = " << found_count << "; ratio = " << (first_score / best_score) << "; score = " << best_score << endl)).str());
#endif
		}
	}

	float PoseyDood::ScoreJOs(const DATKeyframe& test_pose, Constraint** constraints_begin, Constraint** constraints_end, const PoseDelta* deltas_begin, const PoseDelta* deltas_end)
	{
		float score = 0.0f;
		for(Constraint** iter = constraints_begin; iter != constraints_end; ++iter)
			score += (*iter)->GetErrorAmount(test_pose);

		for(const PoseDelta* iter = deltas_begin; iter != deltas_end; ++iter)
		{
			const DATKeyframe::KBone& bone = test_pose.data[iter->bone];
			if(iter->score_pos)
				score += (iter->new_state.pos - bone.pos).ComputeMagnitudeSquared();
			if(iter->score_ori)
			{
				float angle = (Quaternion::Reverse(iter->new_state.ori) * bone.ori).GetRotationAngle();
				score += angle * angle;
			}
		}

		return score;
	}

	void PoseyDood::GetFixedJointPoseOps(DATKeyframe& pose, bool* locked_bones, vector<FixedJointPoseOp>& results)
	{
		unsigned int num_bones = bones.size();
		unsigned int num_specials = special_constraints.size();
		unsigned int arraysize = num_bones + num_specials;

		bool* boolarray = new bool[arraysize];
		bool* known_bones = boolarray;
		memcpy(known_bones, locked_bones, num_bones * sizeof(bool));
		for(unsigned int i = 0; i < mphys->bones.size(); ++i)				// skeletal bones can be used as a starting point for fixed joint op chains
			known_bones[i] = true;

		bool* constraints_left = boolarray + num_bones;
		memset(constraints_left, 0, num_specials * sizeof(bool));
		unsigned int remaining = 0;

		for(unsigned int i = 0; i < num_specials; ++i)
		{
			const SpecialConstraint& sc = special_constraints[i];
			if(pose.enabled_constraints[sc.index])
				if(CFixedJoint* fj = dynamic_cast<CFixedJoint*>(constraints[sc.index]))
				{
					constraints_left[i] = true;
					++remaining;
				}
		}

		while(remaining != 0)
		{
			bool progress = false;

			for(vector<SpecialConstraint>::iterator iter = special_constraints.begin(); iter != special_constraints.end(); ++iter)
			{
				const SpecialConstraint& sc = *iter;
				if(pose.enabled_constraints[sc.index])
					if(CFixedJoint* fj = dynamic_cast<CFixedJoint*>(constraints[sc.index]))
					{
						bool &aknow = known_bones[fj->bone_a],  &bknow = known_bones[fj->bone_b];
						if(aknow || bknow)
						{
							bool &alock = locked_bones[fj->bone_a], &block = locked_bones[fj->bone_b];
							DATKeyframe::KBone& adata = pose.data[fj->bone_a];
							DATKeyframe::KBone& bdata = pose.data[fj->bone_b];
							Mat4 xform = Mat4::FromPositionAndOrientation(fj->socket_a - fj->relative_ori * fj->socket_b, fj->relative_ori);

							if(aknow && !bknow)
							{
								if(alock)
								{
									Mat4 bxform = Mat4::FromPositionAndOrientation(adata.pos, adata.ori) * xform;
									bxform.Decompose(bdata.pos, bdata.ori);
									bdata.ori = Quaternion::Reverse(bdata.ori);			// something fishy going on here...

									block = true;
								}
								else
									results.push_back(FixedJointPoseOp(fj->bone_a, fj->bone_b, xform));
								progress = bknow = true;
							}
							else if(bknow && !aknow)
							{
								Mat4 invxform = Mat4::Invert(xform);
								if(block)
								{
									Mat4 axform = Mat4::FromPositionAndOrientation(bdata.pos, bdata.ori) * invxform;
									axform.Decompose(adata.pos, adata.ori);
									alock = true;

									adata.ori = Quaternion::Reverse(adata.ori);			// something fishy going on here...
								}
								else
									results.push_back(FixedJointPoseOp(fj->bone_b, fj->bone_a, invxform));
								progress = aknow = true;
							}
						}
					}
			}

			if(!progress)
				break;
		}

		delete[] boolarray;
	}





	void PoseyDood::PoseBones(Skeleton* skeleton, const DATKeyframe& pose)
	{
		skeleton->InvalidateCachedBoneXforms();

		unsigned int num_bones = bones.size();
		for(unsigned int i = 0; i < num_bones; ++i)
		{
			const DATKeyframe::KBone& datum = pose.data[i];
			Bone* bone = skeleton->bones[bones[i].bone_index];

			bone->ori = datum.ori;
			bone->pos = datum.pos;
		}
	}

	void PoseyDood::PoseBones(Skeleton* skeleton, const DATKeyframe& frame_a, const DATKeyframe& frame_b, float b_frac)
	{
		skeleton->InvalidateCachedBoneXforms();

		float a_frac = 1.0f - b_frac;

		unsigned int num_bones = bones.size();
		for(unsigned int i = 0; i < num_bones; ++i)
		{
			const DATKeyframe::KBone &adat = frame_a.data[i], &bdat = frame_b.data[i];
			Bone* bone = skeleton->bones[bones[i].bone_index];

			bone->ori = adat.ori * a_frac + bdat.ori * b_frac;
			bone->pos = adat.pos * a_frac + bdat.pos * b_frac;
		}
	}

	void PoseyDood::Vis(SceneRenderer* renderer, const DATKeyframe& pose, float timer, bool draw_uber)
	{
		sk_rinfo.Invalidate();

		PoseBones(skeleton, pose);
		skeleton->InvalidateCachedBoneXforms();

		if(draw_uber)
		{
			// draw the skinned character
			skeleton->GetBoneMatrices(bone_matrices);
			sk_rinfo.num_bones = bone_matrices.size();
			sk_rinfo.bone_matrices = SkinnedCharacter::MatricesToTexture1D(bone_matrices, sk_rinfo.bone_matrices);
			dood_uber->Vis(renderer, 0, Mat4::Identity(), &sk_rinfo);

			// draw helper bones (e.g. gun, ground placeholder for placed foot constraints)
			for(unsigned int i = 0; i < bones.size(); ++i)
				bones[i].DrawHelperObject(renderer, skeleton);
		}

		// draw outlines of bones' collision shapes
		{
			Vec3 unselected_color = Vec3(0.5f, 0.5f, 0.5f);
			Vec3 selected_color   = Vec3(1.0f, 1.0f, 0.5f) * (0.5f + 0.5f * sinf(timer * float(M_PI) * 2.0f * 4.0f));

			for(unsigned int i = 0; i < bones.size(); ++i)
			{
				const DATBone& bone = bones[i];
				const Bone* skel_bone = skeleton->bones[bone.bone_index];
				bone.shape->DebugDraw(renderer, skel_bone->pos, skel_bone->ori, bone.selected ? selected_color : unselected_color);
			}
		}
	}
}
