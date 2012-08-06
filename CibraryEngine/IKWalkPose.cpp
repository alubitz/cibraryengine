#include "StdAfx.h"
#include "IKWalkPose.h"

#include "Random3D.h"

#include "Physics.h"

#include "RigidBody.h"
#include "JointConstraint.h"
#include "PlacedFootConstraint.h"

namespace CibraryEngine
{
	/*
	 * IKWalkPose::EndEffector methods
	 */
	IKWalkPose::EndEffector::EndEffector(PhysicsWorld* physics, ModelPhysics* mphys, Bone* from, Bone* to, RigidBody* foot) :
		physics(physics),
		foot(foot),
		placed(NULL),
		chain(new IKChain(from, to, mphys)),
		desired_pos(),
		desired_ori(Quaternion::Identity()),
		arrive_time(-1),
		arrived(true)
	{
	}

	IKWalkPose::EndEffector::~EndEffector()
	{
		UnlockPlacedFoot();
		if(chain) { delete chain; chain = NULL; }
	}

	void IKWalkPose::EndEffector::LockPlacedFoot(RigidBody* base)
	{
		if(placed)
		{
			if(placed->obj_a == foot && placed->obj_b == base)
				return;
			else
				UnlockPlacedFoot();
		}

		placed = new PlacedFootConstraint(foot, base, foot->GetTransformationMatrix().TransformVec3_1(foot->GetMassInfo().com));
		physics->AddConstraint(placed);
	}

	void IKWalkPose::EndEffector::UnlockPlacedFoot()
	{
		if(placed)
		{
			physics->RemoveConstraint(placed);

			placed->Dispose();
			delete placed;
			placed = NULL;
		}
	}




	/*
	 * IKWalkPose methods
	 */
	IKWalkPose::IKWalkPose(PhysicsWorld* physics, ModelPhysics* mphys, const vector<RigidBody*>& rigid_bodies, const vector<JointConstraint*>& all_joints, const vector<JointConstraint*>& constraints, const vector<Bone*>& rbody_to_posey) :
		Pose(),
		physics(physics),
		mphys(mphys),
		rigid_bodies(rigid_bodies),
		all_joints(all_joints),
		joints(),
		end_effectors(),
		rbody_to_posey(rbody_to_posey),
		arrive_time(-1)
	{
		for(vector<JointConstraint*>::const_iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
		{
			JointConstraint* constraint = *iter;

			Bone *bone_a = NULL, *bone_b = NULL;

			unsigned int i = 0;
			for(vector<RigidBody*>::const_iterator jter = rigid_bodies.begin(); jter != rigid_bodies.end(); ++jter, ++i)
			{
				if(*jter == constraint->obj_a)
					bone_a = rbody_to_posey[i];
				else if(*jter == constraint->obj_b)
					bone_b = rbody_to_posey[i];
			}

			if(bone_a && bone_b)
			{
				if(bone_a == bone_b->parent)
					joints.push_back(Joint(constraint, constraint->desired_ori, bone_b->name, false));
				else if(bone_b == bone_a->parent)
					joints.push_back(Joint(constraint, constraint->desired_ori, bone_a->name, true));
			}
		}
	}

	IKWalkPose::~IKWalkPose()
	{
		for(vector<EndEffector*>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
			delete *iter;
		end_effectors.clear();
	}

	void IKWalkPose::UpdatePose(TimingInfo time)
	{
		float timestep = time.elapsed;
		if(timestep > 0)
		{
			float now = time.total;
			float next_arrival = -1;

			// find the soonest upcoming arrival time
			if(now < arrive_time)
				next_arrival = arrive_time;
			for(vector<EndEffector*>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
			{
				EndEffector* ee = *iter;
				if(now < ee->arrive_time)
					if(next_arrival == -1 || ee->arrive_time < next_arrival)
						next_arrival = ee->arrive_time;
			}

			Seek(timestep, max(timestep, next_arrival - now));
		}
	}

	void IKWalkPose::AddEndEffector(RigidBody* foot)
	{
		Bone* from = rbody_to_posey[0];				// TODO: maybe have a way to select which bone to use?
		Bone* to = NULL;

		for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
			if(rigid_bodies[i] == foot)
				to = rbody_to_posey[i];

		end_effectors.push_back(new EndEffector(physics, mphys, from, to, foot));
	}

	void IKWalkPose::Seek(float timestep, float foresight)
	{
		// compute position and velocity of the dood's center of mass
		Vec3 com, com_vel;
		float total_weight;

		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
		{
			RigidBody& body = **iter;
			MassInfo mass_info = body.GetMassInfo();

			float weight = mass_info.mass;
			
			com += body.GetCenterOfMass() * weight;
			com_vel += body.GetLinearVelocity() * weight;

			total_weight += weight;
		}

		com /= total_weight;
		com_vel /= total_weight;

		// TODO: use the stuff we just computed somehow



		unsigned int num_variables = joints.size() * 3;

		vector<float> best =		vector<float>(num_variables);
		vector<float> guess =		vector<float>(num_variables);
		vector<float> rot =			vector<float>(num_variables);
		vector<float> min_extents =	vector<float>(num_variables);
		vector<float> max_extents =	vector<float>(num_variables);

		vector<float>::iterator best_iter =		best.begin();
		vector<float>::iterator guess_iter =	guess.begin();
		vector<float>::iterator rot_iter =		rot.begin();
		vector<float>::iterator min_iter =		min_extents.begin();
		vector<float>::iterator max_iter =		max_extents.begin();

		// rot = the solution we came up with last time
		for(vector<Joint>::iterator iter = joints.begin(); iter != joints.end(); ++iter)
		{
			const Joint& joint = *iter;
			const JointConstraint& constraint = *joint.constraint;

			*(min_iter++) = constraint.min_extents.x;
			*(min_iter++) = constraint.min_extents.y;
			*(min_iter++) = constraint.min_extents.z;

			*(max_iter++) = constraint.max_extents.x;
			*(max_iter++) = constraint.max_extents.y;
			*(max_iter++) = constraint.max_extents.z;

			Quaternion target_ori = joint.target_ori;
			Vec3 vec = Mat3::Invert(constraint.axes) * target_ori.ToPYR();
			*(rot_iter++) = max(constraint.min_extents.x, min(constraint.max_extents.x, vec.x));
			*(rot_iter++) = max(constraint.min_extents.y, min(constraint.max_extents.y, vec.y));
			*(rot_iter++) = max(constraint.min_extents.z, min(constraint.max_extents.z, vec.z));
		}

		float best_score = -1;

		// look for incrementally better solutions
		for(int i = 0; i < 100; ++i)
		{
			if(i == 0)
				guess = vector<float>(num_variables);			// initialize all to 0.0f
			else if(i == 1)
				guess = rot;									// try the solution we came up with last time
			else
			{
				guess = best;									// mutations off of the best solution so far

				int num_mutations = Random3D::RandInt(1, 3);
				for(int i = 0; i < num_mutations; ++i)
				{
					int index = Random3D::RandInt(num_variables);

					float min_val = min_extents[index], max_val = max_extents[index];
					float range = max_val - min_val;

					guess[index] = max(min_val, min(max_val, guess[index] + Random3D::Rand(-0.1f * range, 0.1f * range)));
				}
			}

			float score = EvaluateSolution(guess);

			if(best_score == -1 || score < best_score)
			{
				best = guess;
				best_score = score;

				// stop iterating once score is "good enough"
				if(score < 0.0001f)
					break;
			}
		}

		// apply the best solution we came up with
		best_iter = best.begin();
		for(vector<Joint>::iterator iter = joints.begin(); iter != joints.end(); ++iter)
		{
			Joint& joint = *iter;
			JointConstraint& constraint = *joint.constraint;

			float x = *(best_iter++), y = *(best_iter++), z = *(best_iter++);
			joint.target_ori = Quaternion::FromPYR(constraint.axes * Vec3(x, y, z));

			Quaternion ori = Quaternion::Reverse(constraint.obj_a->GetOrientation()) * constraint.obj_b->GetOrientation();

			Quaternion target_relative = Quaternion::Reverse(ori) * joint.target_ori;
			float a_coeff = timestep / foresight, b_coeff = 1.0f - a_coeff;

			ori *= target_relative * a_coeff + Quaternion::Identity() * b_coeff;

			SetBonePose(joint.set_pose_id, ori.ToPYR(), Vec3());
		}
	}

	float IKWalkPose::EvaluateSolution(const vector<float>& values)
	{
#if 1
		return 0.0f;
#else
		map<RigidBody*, Mat4> bone_xforms;

		Vec3 feet_pos;

		// find out positions of feet for balancing purposes... also get xforms of placed feet
		for(vector<EndEffector*>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
		{
			RigidBody* foot = (*iter)->foot;

			if((*iter)->placed)
				bone_xforms[foot] = foot->GetTransformationMatrix();

			feet_pos += foot->GetTransformationMatrix().TransformVec3_1(foot->GetMassInfo().com);
		}

		if(bone_xforms.empty())
			return 0.0f;				// oops? first argument passed will be rest pose, so give it a perfect score

		feet_pos /= (float)end_effectors.size();

		// find xforms for all the other bones
		bool progress;
		do
		{
			progress = false;

			vector<float>::const_iterator value_iter = values.begin();
			vector<Joint>::iterator joint_iter = joints.begin();
			for(vector<JointConstraint*>::iterator iter = all_joints.begin(); iter != all_joints.end(); ++iter)
			{
				Vec3 pyr;

				JointConstraint* jc = *iter;
				if(joint_iter != joints.end() && joint_iter->constraint == jc)
				{
					float x = *(value_iter++), y = *(value_iter++), z = *(value_iter++);
					pyr = Vec3(x, y, z);
				}

				RigidBody* a = jc->obj_a;
				RigidBody* b = jc->obj_b;

				map<RigidBody*, Mat4>::iterator found_a = bone_xforms.find(a);
				map<RigidBody*, Mat4>::iterator found_b = bone_xforms.find(b);
				map<RigidBody*, Mat4>::iterator found;
				

				RigidBody* looking_for = NULL;
				if(found_a == bone_xforms.end())
				{
					if(found_b == bone_xforms.end())
						continue;
					else
					{
						looking_for = a;
						found = found_b;
					}
				}
				else
				{
					if(found_b == bone_xforms.end())
					{
						looking_for = b;
						found = found_a;
					}
					else
						continue;
				}

				unsigned a_index = 0, b_index = 0;
				for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
				{
					if(rigid_bodies[i] == a)
					{
						a_index = i + 1;
						if(b_index)
							break;
					}
					else if(rigid_bodies[i] == b)
					{
						b_index = i + 1;
						if(a_index)
							break;
					}
				}

				if(a_index && b_index)
				{
					const Mat4& known_matrix = found->second;

					Quaternion ori = Quaternion::FromPYR(jc->axes * pyr);

					Bone* a_bone = rbody_to_posey[a_index - 1];
					Bone* b_bone = rbody_to_posey[b_index - 1];
					Bone& child = *(a_bone->parent == b_bone ? a_bone : b_bone);

					Mat4 to_rest_pos = Mat4::Translation(child.rest_pos);
					Mat4 from_rest_pos = Mat4::Translation(-child.rest_pos);
					Mat4 rotation_mat = Mat4::FromQuaternion(ori * child.rest_ori);
					Mat4 offset = Mat4::Translation(child.pos);

					Mat4 net = to_rest_pos * rotation_mat * offset * from_rest_pos;
					if((b_bone->parent == a_bone) ^ (looking_for == b))
						net = Mat4::Invert(net);

					bone_xforms[looking_for] = known_matrix * net;

					Vec3 com = looking_for->GetMassInfo().com;
					Vec3 xformed = bone_xforms[looking_for].TransformVec3_1(com);

					progress = true;								// keep iterating as long as we're finding new bone xforms
				}
			}
		} while(progress);

		Vec3 com;
		float total_weight = 0.0f;

		float bad_match_penalty = 0.0f;
		for(vector<EndEffector*>::iterator iter = end_effectors.begin(); iter != end_effectors.end(); ++iter)
		{
			EndEffector& ee = **iter;
			if(ee.placed)
			{
				vector<float> ee_values;				// TODO: get these values
				for(vector<IKChain::ChainNode>::iterator bone_iter = ee.chain->bones.begin(); bone_iter != ee.chain->bones.end(); ++bone_iter)
				{
					
				}
				Mat4 pelvis_mat = ee.foot->GetTransformationMatrix() * Mat4::Invert(ee.chain->GetEndTransformRelative(ee_values.data()));

				for(vector<EndEffector*>::iterator jter = iter; jter != end_effectors.end(); ++jter)
					if(iter != jter)
					{
						EndEffector& ee2 = **jter;
						if(ee2.placed)
						{
							vector<float> ee2_values;	// TODO: get these values
							Mat4 pelvis_2 = ee2.foot->GetTransformationMatrix() * Mat4::Invert(ee2.chain->GetEndTransformRelative(ee2_values.data()));

							Mat4 dif = pelvis_mat * Mat4::Invert(pelvis_2);
							Vec3 pos;
							Quaternion ori;
							dif.Decompose(pos, ori);

							bad_match_penalty += pos.ComputeMagnitudeSquared();
							bad_match_penalty += ori.NormSquared();
						}
					}
			}
		}

		// compute offset of com
		for(map<RigidBody*, Mat4>::iterator iter = bone_xforms.begin(); iter != bone_xforms.end(); ++iter)
		{
			MassInfo mass_info = iter->first->GetMassInfo();

			float weight = mass_info.mass;

			com += iter->second.TransformVec3_1(mass_info.com) * weight;
			total_weight += weight;
		}

		com /= total_weight;


		com -= feet_pos;
		float score = Vec2::MagnitudeSquared(com.x, com.z);

		// TODO: make sure all end effectors think bones are in the same places; penalize score if they disagree

		// TODO: penalize score for not having the torso upright and facing the correct direction

		/*
		float min_com_elevation = 0.9f;
		if(com.y < min_com_elevation)
		{
			float bad = min_com_elevation - com.y;
			score += bad * bad * 2.0f;
		}
		*/

		score += bad_match_penalty;

		return score;
#endif
	}
}
