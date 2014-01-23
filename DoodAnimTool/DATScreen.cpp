#include "StdAfx.h"
#include "DATScreen.h"

#include "DATBone.h"
#include "DATKeyframe.h"

#include "PoseDelta.h"

#include "JointOrientations.h"
#include "PoseChainNode.h"

#include "Constraint.h"
#include "CSkeletalJoint.h"
#include "CFixedJoint.h"
#include "CFlatFoot.h"
#include "CAlignedAxis.h"

#include "../CibraryEngine/DebugDrawMaterial.h"

#include "GCanvas.h"
#include "GColumnList.h"
#include "GCheckbox.h"

#define ENABLE_IMMEDIATE_EDIT_FEEDBACK 1

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	/*
	 * DATScreen private implementation struct
	 */
	struct DATScreen::Imp
	{
		ProgramScreen* next_screen;
		ProgramWindow* window;
		InputState* input_state;

		ModelPhysics* mphys;

		vector<DATBone> bones;
		vector<unsigned int> id_to_bones;		// given a bone's "name", returns index into the above +1

		vector<DATKeyframe> keyframes;

		float anim_timer;
		unsigned int edit_keyframe;

		vector<CSkeletalJoint*> skeletal_joints;
		vector<Constraint*> constraints;
		JointOrientations reusable_jos;
		bool jos_valid;

		vector<Vec3> debug_dots;

		float yaw, pitch;
		CameraView camera;
		SceneRenderer renderer;
		bool draw_model;

		Skeleton* skeleton;
		vector<Mat4> bone_matrices;
		SkinnedCharacterRenderInfo sk_rinfo;
		UberModel *dood_uber;

		enum BoxSelectMode { None, Disable, Waiting, Box } box_selecting;
		int box_x1, box_y1;
		int mouseover_bone;
		unsigned int selection_count;

		float now, buffered_time;

		Cursor* cursor;
		BitmapFont* font;

		class ConstraintCheckbox : public GCheckbox
		{
			public:
				Imp* imp;
				unsigned int index;
				bool default_val;

				ConstraintCheckbox(Imp* imp, unsigned int index, bool default_val) : GCheckbox(imp->font), imp(imp), index(index), default_val(default_val) { }
				void LayoutChildren() { selected = imp->keyframes[imp->edit_keyframe].enabled_constraints[index]; }
				bool OnClick(int x, int y)
				{
					DATKeyframe& keyframe = imp->keyframes[imp->edit_keyframe];
					bool& val = keyframe.enabled_constraints[index];
					val = !val;
					if(val)
						imp->ApplyConstraints(keyframe);

					return true;
				}
		};
		vector<ConstraintCheckbox*> constraint_checkboxes;
		vector<GLabel*> constraint_labels;

		class BoneLockCheckbox : public GCheckbox
		{
			public:
				Imp* imp;
				unsigned int index;

				BoneLockCheckbox(Imp* imp, unsigned int index) : GCheckbox(imp->font), imp(imp), index(index) { }
				void LayoutChildren() { selected = imp->bones[index].locked; }
				bool OnClick(int x, int y) { bool& val = imp->bones[index].locked; val = !val; return true; }
		};
		vector<BoneLockCheckbox*> lock_checkboxes;

		class BoneSelectCheckbox : public GCheckbox
		{
			public:
				Imp* imp;
				unsigned int index;

				BoneSelectCheckbox(Imp* imp, unsigned int index) : GCheckbox(imp->font), imp(imp), index(index) { }
				void LayoutChildren() { selected = imp->bones[index].selected; }
				bool OnClick(int x, int y)
				{
					bool& val = imp->bones[index].selected;
					if(val)
						--imp->selection_count;
					else
						++imp->selection_count;
					val = !val;

					return true;
				}
		};
		vector<BoneSelectCheckbox*> select_checkboxes;
		vector<GLabel*> bone_labels;

		GLabel sel_label, lock_label;

		GColumnList constraints_listbox, bones_listbox;
		GColumnList file_menu;
		GLabel debug_text;
		GCanvas canvas;

		class FileReset : public GLabel
		{
			public:
				Imp* imp;
				FileReset() { }
				FileReset(Imp* imp, BitmapFont* font) : GLabel(font, "Reset Pose", true), imp(imp) { }
				bool OnClick(int x, int y) { imp->ResetPose(); return true; }
		} file_reset;

		class FileOpen : public GLabel
		{
			public:
				Imp* imp;
				FileOpen() { }
				FileOpen(Imp* imp, BitmapFont* font) : GLabel(font, "Open Pose", true), imp(imp) { }
				bool OnClick(int x, int y) { imp->LoadPose(); return true; } 
		} file_open;

		class FileSave : public GLabel
		{
			public:
				Imp* imp;
				FileSave() { }
				FileSave(Imp* imp, BitmapFont* font) : GLabel(font, "Save Pose", true), imp(imp) { }
				bool OnClick(int x, int y) { imp->SavePose(); return true; }
		} file_save;


		Imp(ProgramWindow* window) :
			next_screen(NULL),
			window(window),
			input_state(window->input_state),
			mphys(NULL),
			camera(Mat4::Identity(), 1.0f, 1.0f),				// these values don't matter; they will be overwritten before use
			renderer(&camera),
			draw_model(true),
			skeleton(NULL),
			dood_uber(NULL),
			box_selecting(None),
			cursor(NULL),
			font(NULL),
			key_listener(),
			mouse_listener()
		{
			key_listener.imp = mouse_listener.imp = mouse_motion_listener.imp = this;

			input_state->KeyStateChanged         += &key_listener;
			input_state->MouseButtonStateChanged += &mouse_listener;
			input_state->MouseMoved              += &mouse_motion_listener;

			cursor = window->content->GetCache<Cursor>()->Load("Cursor");
			font = window->content->GetCache<BitmapFont>()->Load("../Font");

			constraints_listbox = GColumnList(5,  GColumnList::Left);
			constraints_listbox.AddColumn    (20, GColumnList::Left);
			canvas.AddChild(&constraints_listbox, GCanvas::HAlign(20, NULL), GCanvas::VAlign(20, NULL));

			bones_listbox = GColumnList(5,  GColumnList::Center);
			bones_listbox.AddColumn    (5,  GColumnList::Center);
			bones_listbox.AddColumn    (20, GColumnList::Left  );
			sel_label  = GLabel(font, "sel" );			// these get added in LoadDood
			lock_label = GLabel(font, "lock");
			canvas.AddChild(&bones_listbox, GCanvas::HAlign(NULL, 20), GCanvas::VAlign(20, NULL));

			file_menu = GColumnList(5, GColumnList::Left);
			file_menu.BeginRow();   file_reset = FileReset(this, font);   file_menu.AddToLastRow(&file_reset);
			file_menu.BeginRow();   file_open  = FileOpen (this, font);   file_menu.AddToLastRow(&file_open );
			file_menu.BeginRow();   file_save  = FileSave (this, font);   file_menu.AddToLastRow(&file_save );
			canvas.AddChild(&file_menu, GCanvas::HAlign(NULL, 20), GCanvas::VAlign(NULL, 20));

			debug_text = GLabel(font, string());
			canvas.AddChild(&debug_text, GCanvas::HAlign(20, NULL), GCanvas::VAlign(NULL, 20));

			// make sure soldier physics file is up to date... these lines should probably be removed at some point
			ScriptSystem::Init();
			ScriptSystem::GetGlobalState().DoFile("Files/Scripts/soldier_zzp.lua");

			LoadDood("soldier");

			now = buffered_time = 0.0f;
			yaw = pitch = 0.0f;
		}

		~Imp()
		{
			input_state->KeyStateChanged -= &key_listener;
			input_state->MouseButtonStateChanged -= &mouse_listener;
			input_state->MouseMoved -= &mouse_motion_listener;

			if(skeleton)				{ skeleton->Dispose();					delete skeleton;				skeleton = NULL; }
			if(sk_rinfo.bone_matrices)	{ sk_rinfo.bone_matrices->Dispose();	delete sk_rinfo.bone_matrices;	sk_rinfo.bone_matrices = NULL; }

			DeleteConstraints();

			DebugDrawMaterial::GetDebugDrawMaterial()->EmptyRecycleBin();
		}

		void DeleteConstraints()
		{
			skeletal_joints.clear();
			for(vector<Constraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
				delete *iter;
			constraints.clear();
		}



		void LoadDood(const string& dood_name)
		{
			Cache<ModelPhysics>* mphys_cache = window->content->GetCache<ModelPhysics>();
			Cache<UberModel>*    uber_cache  = window->content->GetCache<UberModel>();
			Cache<Material>*     mat_cache   = window->content->GetCache<Material>();

			mphys     = mphys_cache->Load(dood_name);
			dood_uber = uber_cache->Load (dood_name);
			dood_uber->LoadCachedMaterials(mat_cache);

			skeleton = dood_uber->CreateSkeleton();

			bones.clear();
			keyframes.clear();

			// cleanup gui items created by a previous call to LoadDood (if that were possible)
			for(vector<BoneLockCheckbox*>::iterator iter = lock_checkboxes.begin(); iter != lock_checkboxes.end(); ++iter)
				delete *iter;
			lock_checkboxes.clear();

			for(vector<BoneSelectCheckbox*>::iterator iter = select_checkboxes.begin(); iter != select_checkboxes.end(); ++iter)
				delete *iter;
			select_checkboxes.clear();

			for(vector<ConstraintCheckbox*>::iterator iter = constraint_checkboxes.begin(); iter != constraint_checkboxes.end(); ++iter)
				delete *iter;
			constraint_checkboxes.clear();

			for(vector<GLabel*>::iterator iter = constraint_labels.begin(); iter != constraint_labels.end(); ++iter)
				delete *iter;
			constraint_labels.clear();

			for(vector<GLabel*>::iterator iter = bone_labels.begin(); iter != bone_labels.end(); ++iter)
				delete *iter;
			bone_labels.clear();

			bones_listbox.rows.clear();
			constraints_listbox.rows.clear();

			DeleteConstraints();

			anim_timer = 0.0f;
			edit_keyframe = 0;
			jos_valid = false;

			mouseover_bone = -1;
			selection_count = 0;

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

			CreateSoldierSpecificHelperBones(mphys_cache, uber_cache, mat_cache);

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

			// create bone select and lock checkbox lists
			bones_listbox.BeginRow();
			bones_listbox.AddToLastRow(&sel_label);
			bones_listbox.AddToLastRow(&lock_label);

			for(unsigned int i = 0; i < bones.size(); ++i)
			{
				select_checkboxes.push_back(new BoneSelectCheckbox(this, i));
				lock_checkboxes.push_back(new BoneLockCheckbox(this, i));
				bone_labels.push_back(new GLabel(font, Bone::string_table[bones[i].name]));

				bones_listbox.BeginRow();
				
				bones_listbox.AddToLastRow(*select_checkboxes.rbegin());
				bones_listbox.AddToLastRow(*lock_checkboxes.rbegin());
				bones_listbox.AddToLastRow(*bone_labels.rbegin());
			}

			// create constraints (starting with the soldier-specific ones)
			CreateSoldierSpecificConstraints();

			// add default constraints to the constraints list
			for(unsigned int i = 0; i < mphys->joints.size(); ++i)
			{
				CSkeletalJoint* sj = new CSkeletalJoint(mphys, i, bones); 
				skeletal_joints.push_back(sj);
				constraints.push_back(sj);
			}

			DATKeyframe initial_pose = GetDefaultPose();
			keyframes.push_back(initial_pose);
		}

		int GetBoneIndex(const string& bone_name)
		{
			unsigned int id = Bone::string_table[bone_name];
			return id < id_to_bones.size() ? (signed)id_to_bones[id] - 1 : -1;
		}

		void AddSpecialConstraint(const string& name, bool default_val, Constraint* c)
		{
			constraint_checkboxes.push_back(new ConstraintCheckbox(this, constraints.size(), default_val));
			constraint_labels.push_back(new GLabel(font, name));

			constraints_listbox.BeginRow();
			constraints_listbox.AddToLastRow(*constraint_checkboxes.rbegin());
			constraints_listbox.AddToLastRow(*constraint_labels.rbegin());

			constraints.push_back(c);
		}

		void AddHelperBone(const string& bone_name, CollisionShape* shape, UberModel* uber)
		{
			unsigned int id = Bone::string_table[bone_name];
			skeleton->AddBone(id, Quaternion::Identity(), Vec3());
			bones.push_back(DATBone(skeleton->bones.size() - 1, id, shape, uber));
		}

		void CreateSoldierSpecificHelperBones(Cache<ModelPhysics>* mphys_cache, Cache<UberModel>* uber_cache, Cache<Material>* mat_cache)
		{
			CollisionShape* gun_shape        = mphys_cache->Load( "gun"         )->bones[0].collision_shape;
			CollisionShape* foothelper_shape = mphys_cache->Load( "foot_helper" )->bones[0].collision_shape;

			UberModel* gun_uber    = uber_cache->Load( "gun"         );
			UberModel* ground_uber = uber_cache->Load( "foot_helper" );

			gun_uber->LoadCachedMaterials(mat_cache);
			ground_uber->LoadCachedMaterials(mat_cache);

			AddHelperBone( "gun",      gun_shape,        gun_uber    );
			AddHelperBone( "l ground", foothelper_shape, ground_uber );
			AddHelperBone( "r ground", foothelper_shape, ground_uber );
		}

		void CreateSoldierSpecificConstraints()
		{
			// create constraints for the soldier holding the gun, and constraints between the soldier's feet and the ground under them
			int lhand   = GetBoneIndex( "l hand"   ), rhand   = GetBoneIndex( "r hand"   );
			int lfoot   = GetBoneIndex( "l foot"   ), rfoot   = GetBoneIndex( "r foot"   );
			int lground = GetBoneIndex( "l ground" ), rground = GetBoneIndex( "r ground" );
			int gun     = GetBoneIndex( "gun"      );
			int head    = GetBoneIndex( "head"     );

			if(lhand >= 0 && gun >= 0)
				AddSpecialConstraint("l grip",   false, new CFixedJoint( (unsigned int)lhand, (unsigned int)gun,     Vec3( 0.990f, 1.113f, 0.037f), Vec3(0.000f,  0.000f,  0.468f), Quaternion::FromRVec(-0.0703434f, -0.0146932f,  2.50207f)));
			if(rhand >= 0 && gun >= 0)
				AddSpecialConstraint("r grip",   true,  new CFixedJoint( (unsigned int)rhand, (unsigned int)gun,     Vec3(-0.959f, 1.098f, 0.077f), Vec3(0.000f, -0.063f, -0.152f), Quaternion::FromRVec( 1.27667f,   -0.336123f,  -0.64284f)));
			if(lfoot >= 0 && lground >= 0)
				AddSpecialConstraint("l ground", true,  new CFlatFoot(   (unsigned int)lfoot, (unsigned int)lground, Vec3( 0.238f, 0.000f, 0.065f), Vec3(), Quaternion::Identity()));
			if(rfoot >= 0 && rground >= 0)
				AddSpecialConstraint("r ground", true,  new CFlatFoot(   (unsigned int)rfoot, (unsigned int)rground, Vec3(-0.238f, 0.000f, 0.065f), Vec3(), Quaternion::Identity()));
			if(gun >= 0 && head >= 0)
				AddSpecialConstraint("aim/look", false, new CAlignedAxis((unsigned int)gun,   (unsigned int)head,    Vec3(0, 0, 1), Vec3(0, 0, 1)));
		}

		void DoSoldierSpecificKeyframeStuff(DATKeyframe& initial_pose)
		{
			// we need to position the gun in a way that won't give the soldier too violent of an initial jerk
			int gun = GetBoneIndex("gun");
			if(gun >= 0)
			{
				initial_pose.data[gun].pos = Vec3(-1.02937f, 0.989617f, 0.178852f);
				initial_pose.data[gun].ori = Quaternion::FromRVec(1.27667f, -0.336123f, -0.64284f);
			}

			// and set the appropriate positions for the objects under foot
			int lground = GetBoneIndex("l ground");
			if(lground >= 0)
				initial_pose.data[lground].pos = Vec3( 0.238f, 0.000f, 0.065f);

			int rground = GetBoneIndex("r ground");
			if(rground >= 0)
				initial_pose.data[rground].pos = Vec3(-0.238f, 0.000f, 0.065f);
		}

		DATKeyframe GetDefaultPose()
		{
			DATKeyframe pose(bones.size(), constraints.size());

			DoSoldierSpecificKeyframeStuff(pose);

			for(vector<ConstraintCheckbox*>::iterator iter = constraint_checkboxes.begin(); iter != constraint_checkboxes.end(); ++iter)
			{
				const ConstraintCheckbox& cc = **iter;
				pose.enabled_constraints[cc.index] = cc.default_val;
			}

#if 0
			int gun = GetBoneIndex("gun");
			if(gun >= 0)
			{
				const DATKeyframe::KBone& gunpose = pose.data[gun];
				const Vec3& pos = gunpose.pos;
				Vec3 ori = gunpose.ori.ToRVec();

				Debug(((stringstream&)(stringstream() << "gun pos = (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << endl)).str());
				Debug(((stringstream&)(stringstream() << "gun ori = (" << ori.x << ", " << ori.y << ", " << ori.z << ")" << endl)).str());
			}
#endif

			return pose;
		}



		void Update(TimingInfo& time)
		{
			if(time.elapsed > 0)
			{
				buffered_time += time.elapsed;
				float timestep = 1.0f / 60.0f;
				while(buffered_time >= timestep)
				{
					now += timestep;
					buffered_time -= timestep;

					TimingInfo use_time = TimingInfo(timestep, now);

					bool applied_constraints = false;

					// rotate the selected bones using QWEASD, or translate using Shift + QWEASD
					if(selection_count > 0)
					{
						Vec3 bone_controls
						(
							(float)((input_state->keys['W'] ? 1 : 0) - (input_state->keys['S'] ? 1 : 0)),
							(float)((input_state->keys['A'] ? 1 : 0) - (input_state->keys['D'] ? 1 : 0)),
							(float)((input_state->keys['Q'] ? 1 : 0) - (input_state->keys['E'] ? 1 : 0))
						);

						if(bone_controls.x != 0 || bone_controls.y != 0 || bone_controls.z != 0)							
						{
							DATKeyframe& keyframe = keyframes[edit_keyframe];

							static vector<PoseDelta> deltas;
							deltas.clear();

							if(input_state->keys[VK_SHIFT])
							{
								// translate the selected bones
								Vec3 delta = bone_controls * (0.5f * timestep);
								for(unsigned int i = 0; i < keyframe.num_bones; ++i)
								{
									if(bones[i].selected)
									{
										const DATKeyframe::KBone& oldstate = keyframe.data[i];
										deltas.push_back(PoseDelta(i, oldstate, DATKeyframe::KBone(oldstate.pos + delta, oldstate.ori)));
									}
								}
							}
							else
							{
								// find the center of the selected bones
								Quaternion delta_quat = Quaternion::FromRVec(bone_controls * timestep);

								Vec3 center;
								for(unsigned int i = 0; i < keyframe.num_bones; ++i)
									if(bones[i].selected)
										center += skeleton->bones[bones[i].bone_index]->GetTransformationMatrix().TransformVec3_1(bones[i].center);
								center /= float(selection_count);

								// rotate the selected bones around their center
								for(unsigned int i = 0; i < keyframe.num_bones; ++i)
								{
									if(bones[i].selected)
									{
										const DATKeyframe::KBone &oldstate = keyframe.data[i];

										Mat4 oldmat = Mat4::FromPositionAndOrientation(oldstate.pos, oldstate.ori);
										Vec3 oldcenter = Mat4::Invert(oldmat).TransformVec3_1(center);

										Quaternion ori = delta_quat * oldstate.ori;
										deltas.push_back(PoseDelta(i, oldstate, DATKeyframe::KBone(center - ori * oldcenter, ori)));
									}
								}
							}

#if ENABLE_IMMEDIATE_EDIT_FEEDBACK
							ApplyConstraints(keyframe, &deltas);
							applied_constraints = true;
#else
							for(vector<PoseDelta>::iterator iter = deltas.begin(); iter != deltas.end(); ++iter)
								keyframe.data[iter->bone] = iter->new_state;
#endif
						}
					}

					// control to force additional iterations of the constraint solver
					if(!applied_constraints && input_state->keys[VK_RETURN])
						ApplyConstraints(keyframes[edit_keyframe]);
				}
			}
		}

		JointOrientations JointOrientationsFromPose(const DATKeyframe& pose)
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

		void ApplyConstraints(DATKeyframe& pose, const vector<PoseDelta>* deltas = NULL)
		{
			// apply user edits
			if(deltas != NULL)
			{
				for(vector<PoseDelta>::const_iterator iter = deltas->begin(); iter != deltas->end(); ++iter)
					pose.data[iter->bone] = iter->new_state;
				jos_valid = false;
			}

			// get a condensed list of the active constraints
			vector<Constraint*> active_constraints;
			active_constraints.reserve(constraints.size());
			for(unsigned int i = 0; i < constraints.size(); ++i)
				if(pose.enabled_constraints[i])
					active_constraints.push_back(constraints[i]);
			Constraint** constraints_begin = active_constraints.data();
			Constraint** constraints_end   = constraints_begin + active_constraints.size();

			// figure out what bones to start from when going from JointOrientations to DATKeyframe
			unsigned int num_bones = mphys->bones.size();
			bool* locked_bones = new bool[num_bones];
			memset(locked_bones, 0, num_bones * sizeof(bool));
			bool any = false;
			for(vector<DATBone>::iterator iter = bones.begin(); iter != bones.end(); ++iter)		// locked bones' xforms are initially known
				if(iter->locked && iter->bone_index < num_bones)
					any = locked_bones[iter->bone_index] = true;
			for(Constraint** iter = constraints_begin; iter != constraints_end; ++iter)				// some constraints may set and lock a bones' initial xform as well
				if((*iter)->SetLockedBones(pose, locked_bones))
					any = true;
			if(!any)																				// if all else fails, start from bone 0
				locked_bones[0] = true;

			// figure out in what order the bone posing operations should be done (when going from JointOrientations to DATKeyframe)
			JointOrientations target_jos = jos_valid ? reusable_jos : JointOrientationsFromPose(pose);
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

			for(unsigned int i = 0; i < 1000; ++i)
			{
				test_jos = best_jos;
				if(i != 0)
				{
					// mutate jos
					float mutation_rate = 0.05f * sqrtf(best_score);
					float coeff = mutation_rate * 2.0f, sub = mutation_rate;
					for(unsigned int j = 0; j < 3; ++j)
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

				test_score = ScoreJOs(test_pose, test_jos, target_jos, constraints_begin, constraints_end);

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

			reusable_jos = best_jos;
			jos_valid = true;

			debug_text.SetText(((stringstream&)(stringstream() << best_score)).str());
			Debug(((stringstream&)(stringstream() << "found = " << found_count << "; ratio = " << (first_score / best_score) << "; score = " << best_score << endl)).str());
		}

		float ScoreJOs(const DATKeyframe& test_pose, const JointOrientations& jos, const JointOrientations& target, Constraint** constraints_begin, Constraint** constraints_end)
		{
			float score = 0.0f;
			for(Constraint** iter = constraints_begin; iter != constraints_end; ++iter)
				score += (*iter)->GetErrorAmount(test_pose);

#if 1
			return score;
#else
			float cost = 0.0f;
			for(const Vec3 *jos_begin = jos.data, *jos_end = jos_begin + jos.num_joints, *jos_iter = jos_begin, *target_iter = target.data; jos_iter != jos_end; ++jos_iter, ++target_iter)
				cost += (*jos_iter - *target_iter).ComputeMagnitudeSquared();

			return score * cost + score;
#endif
		}



		void PoseBones(Skeleton* skeleton) { PoseBones(skeleton, keyframes[edit_keyframe]); }

		void PoseBones(Skeleton* skeleton, const DATKeyframe& pose)
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

		void PoseBones(Skeleton* skeleton, const DATKeyframe& frame_a, const DATKeyframe& frame_b, float b_frac)
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



		void ClearSelection()
		{
			for(vector<DATBone>::iterator iter = bones.begin(); iter != bones.end(); ++iter)
				iter->selected = false;

			selection_count = 0;
		}

		void SelectAll()
		{
			for(vector<DATBone>::iterator iter = bones.begin(); iter != bones.end(); ++iter)
				iter->selected = true;

			selection_count = bones.size();
		}

		bool DoBoneFrustumTest(Vec3* eight_points, Sphere* spheres_begin, Sphere* spheres_end)
		{
			const char NUM_ITERATIONS = 50;
			const char NUM_OFFSETS = 8;


			Vec3* points_end = eight_points + 8;			// 8 vertices of the frustum

			// try to find a separating axis
			Vec3 direction;
			float score = -1;
			float search_scale = 0.6f;

			static const float x_offsets[NUM_OFFSETS] = { -1, -1, -1, -1,  1,  1,  1,  1 };
			static const float y_offsets[NUM_OFFSETS] = { -1, -1,  1,  1, -1, -1,  1,  1 };
			static const float z_offsets[NUM_OFFSETS] = { -1,  1, -1,  1, -1,  1, -1,  1 };

			for(char i = 0; i < NUM_ITERATIONS; ++i)
			{
				float best_score;
				Vec3 best_test;

				for(char j = 0; j < NUM_OFFSETS; ++j)
				{
					Vec3 dir(Vec3::Normalize(
						direction.x + x_offsets[j] * search_scale,
						direction.y + y_offsets[j] * search_scale,
						direction.z + z_offsets[j] * search_scale));

					Sphere* iter = spheres_begin;
					float max_extent = Vec3::Dot(dir, iter->center) + iter->radius;
					++iter;

					while(iter != spheres_end)
					{
						max_extent = max(max_extent, Vec3::Dot(dir, iter->center) + iter->radius);
						++iter;
					}

					Vec3* jter = eight_points;
					float min_extent = Vec3::Dot(dir, *jter);
					++jter;
					while(jter != points_end)
					{
						min_extent = min(min_extent, Vec3::Dot(dir, *jter));
						++jter;
					}

					float test_score = max_extent - min_extent;

					if(test_score < 0)							// found a separating plane? go home early
						return false;
					else if(j == 0 || test_score < best_score)
					{
						best_test = dir;
						best_score = test_score;
					}
				}

				if(i != 0 && best_score >= score)
					search_scale *= 0.75f;
				else
				{
					direction = best_test;
					score = best_score;
				}
			}

			return true;
		}

		void SelectBonesInRectangle(int x1, int y1, int x2, int y2)
		{
			if(x2 < x1)
				swap(x1, x2);
			if(y2 < y1)
				swap(y1, y2);

			vector<unsigned int> unselected_indices;
			for(unsigned int i = 0; i < bones.size(); ++i)
				if(!bones[i].selected)
					unselected_indices.push_back(i);

			int ww = window->GetWidth(), wh = window->GetHeight();	
			float inv_w = 1.0f / ww, inv_h = 1.0f / wh;
			Vec3 origin, direction;
			float t;

			float fx1 = (float)x1 * inv_w, fx2 = (float)x2 * inv_w;
			float fy1 = (float)(wh - y1) * inv_h, fy2 = (float)(wh - y2) * inv_h;

			Vec3 normal = camera.GetForward();
			float base_offset = Vec3::Dot(normal, camera.GetPosition());

			Plane near_plane(normal, base_offset);
			Plane far_plane(normal, base_offset + 1);

			Vec3 points[8];

			for(int n = 0; n < 10; ++n, ++near_plane.offset, ++far_plane.offset)
			{
				camera.GetRayFromDimCoeffs(fx1, fy1, origin, direction);
				t = Util::RayPlaneIntersect(Ray(origin, direction), near_plane);
				points[0] = origin + direction * t;
				t = Util::RayPlaneIntersect(Ray(origin, direction), far_plane);
				points[1] = origin + direction * t;

				camera.GetRayFromDimCoeffs(fx1, fy2, origin, direction);
				t = Util::RayPlaneIntersect(Ray(origin, direction), near_plane);
				points[2] = origin + direction * t;
				t = Util::RayPlaneIntersect(Ray(origin, direction), far_plane);
				points[3] = origin + direction * t;

				camera.GetRayFromDimCoeffs(fx2, fy1, origin, direction);
				t = Util::RayPlaneIntersect(Ray(origin, direction), near_plane);
				points[4] = origin + direction * t;
				t = Util::RayPlaneIntersect(Ray(origin, direction), far_plane);
				points[5] = origin + direction * t;

				camera.GetRayFromDimCoeffs(fx2, fy2, origin, direction);
				t = Util::RayPlaneIntersect(Ray(origin, direction), near_plane);
				points[6] = origin + direction * t;
				t = Util::RayPlaneIntersect(Ray(origin, direction), far_plane);
				points[7] = origin + direction * t;

				for(unsigned int i = 0; i < unselected_indices.size(); ++i)
				{
					DATBone& dat_bone = bones[unselected_indices[i]];
					Bone* bone = skeleton->bones[dat_bone.bone_index];
					CollisionShape* shape = dat_bone.shape;
					Mat4 xform = bone->GetTransformationMatrix();

					switch(shape->GetShapeType())
					{
						case ST_MultiSphere:
						{
							MultiSphereShapeInstanceCache mssic;
							ShapeInstanceCache* sic = &mssic;

							((MultiSphereShape*)shape)->ComputeCachedWorldAABB(xform, sic);
							if(DoBoneFrustumTest(points, mssic.spheres.data(), mssic.spheres.data() + mssic.spheres.size()))
							{
								dat_bone.selected = true;
								++selection_count;
							}

							break;
						}

						default:
							break;
					}
				}
			}
		}

		int FindBoneAtScreenPos(int x, int y)
		{
			int window_height = window->GetHeight();

			Vec3 origin, direction;
			camera.GetRayFromDimCoeffs((float)x / window->GetWidth(), (float)(window_height - y) / window_height, origin, direction);

			float target_t = 0.0f;
			int target = -1;

			for(unsigned int i = 0; i < bones.size(); ++i)
			{
				DATBone& dat_bone = bones[i];
				Bone* bone = skeleton->bones[dat_bone.bone_index];
				CollisionShape* shape = dat_bone.shape;
				Mat4 inv_xform = Mat4::Invert(bone->GetTransformationMatrix());

				const float ray_length = 1000.0f;			// because CollideRay only returns a fraction of the ray length; values > 1 get discarded
				Ray ray(inv_xform.TransformVec3_1(origin), inv_xform.TransformVec3_0(direction) * ray_length);
				RayResult ray_result;

				switch(shape->GetShapeType())
				{
					case ST_MultiSphere:
					{
						MultiSphereShape* mss = (MultiSphereShape*)shape;
						if(mss->CollideRay(ray, ray_result))
						{
							if(ray_result.t < target_t || target == -1)
							{
								target = (signed)i;
								target_t = ray_result.t;
							}
						}

						break;
					}

					default:
						break;
				}
			}

			return target;
		}

		void FindMouseoverBone() { mouseover_bone = FindBoneAtScreenPos(input_state->mx, input_state->my); }



		void LoadPose()
		{
			ifstream file("Files/pose.pose", ios::in | ios::binary);
			if(!file)
				Debug("Unable to load pose!\n");
			else
			{
				if(unsigned int error = keyframes[0].Read(file))
					Debug(((stringstream&)(stringstream() << "Error loading pose! DATKeyframe::Read returned error " << error << "!" << endl)).str());
				else
					Debug("Loaded pose successfully!\n");
			}

			jos_valid = false;
		}

		void SavePose()
		{
			ofstream file("Files/pose.pose", ios::out | ios::binary);
			if(!file)
				Debug("Failed to save pose!\n");
			else
			{
				keyframes[0].Write(file);
				if(!file)
					Debug("Something may have gone wrong saving pose!\n");
				else
					Debug("Saved pose successfully!\n");
			}
		}

		void ResetPose() { jos_valid = false; keyframes[edit_keyframe] = GetDefaultPose(); }



		void Draw(int width, int height)
		{
			glViewport(0, 0, width, height);

			glDepthMask(true);
			glColorMask(true, true, true, false);

			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			// set up camera
			float zoom = 2.0f;
			float aspect_ratio = (float)width / height;
			float view_distance = 3.5f;
			Vec3 view_center = Vec3(0, 1.0f, 0);
			Mat4 view_matrix = Mat4::Translation(0, 0, -view_distance) * Mat4::FromQuaternion(Quaternion::FromRVec(pitch, 0, 0) * Quaternion::FromRVec(0, yaw, 0)) * Mat4::Translation(-view_center);
			camera = CameraView(view_matrix, zoom, aspect_ratio);
			
			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf(camera.GetProjectionMatrix().Transpose().values);
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf(camera.GetViewMatrix().Transpose().values);

			DebugDrawMaterial* ddm = DebugDrawMaterial::GetDebugDrawMaterial();

			// draw grid (ground plane)
			for(short i = -2; i <= 2; ++i)
			{
				renderer.objects.push_back(RenderNode(ddm, ddm->New(Vec3(-2, 0,  i ), Vec3( 2, 0, i )), 0));
				renderer.objects.push_back(RenderNode(ddm, ddm->New(Vec3( i, 0, -2 ), Vec3( i, 0, 2 )), 0));
			}

			// draw debug dots
			for(vector<Vec3>::iterator iter = debug_dots.begin(); iter != debug_dots.end(); ++iter)
			{
				static const float R = 0.01f;
				static const Vec3 xvec(R, 0, 0);
				static const Vec3 yvec(0, R, 0);
				static const Vec3 zvec(0, 0, R);

				renderer.objects.push_back(RenderNode(ddm, ddm->New(*iter - xvec, *iter + xvec), 0));
				renderer.objects.push_back(RenderNode(ddm, ddm->New(*iter - yvec, *iter + yvec), 0));
				renderer.objects.push_back(RenderNode(ddm, ddm->New(*iter - zvec, *iter + zvec), 0));
			}

			// draw the dood
			PoseBones(skeleton);
			skeleton->InvalidateCachedBoneXforms();

			if(draw_model)
			{
				// draw the skinned character
				skeleton->GetBoneMatrices(bone_matrices);
				sk_rinfo.num_bones = bone_matrices.size();
				sk_rinfo.bone_matrices = SkinnedCharacter::MatricesToTexture1D(bone_matrices, sk_rinfo.bone_matrices);
				dood_uber->Vis(&renderer, 0, Mat4::Identity(), &sk_rinfo);

				// draw helper bones (e.g. gun, ground placeholder for placed foot constraints)
				for(unsigned int i = 0; i < bones.size(); ++i)
					bones[i].DrawHelperObject(&renderer, skeleton);
			}

			// draw outlines of bones' collision shapes
			{
				Vec3 unselected_color = Vec3(0.5f, 0.5f, 0.5f);
				Vec3 selected_color   = Vec3(1.0f, 1.0f, 0.5f) * (0.5f + 0.5f * sinf(now * float(M_PI) * 2.0f * 4.0f));

				for(unsigned int i = 0; i < bones.size(); ++i)
				{
					const DATBone& bone = bones[i];
					const Bone* skel_bone = skeleton->bones[bone.bone_index];
					bone.shape->DebugDraw(&renderer, skel_bone->pos, skel_bone->ori, bone.selected ? selected_color : unselected_color);
				}
			}

			renderer.Render();
			renderer.Cleanup();

			sk_rinfo.Invalidate();

			// 2d overlay
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0, width, height, 0, -1, 1);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			
			// draw GUI
			canvas.Layout(width, height);
			canvas.OnMouseMoved(input_state->mx, input_state->my);
			canvas.Draw(width, height);

			mouseover_bone = -1;

			if(box_selecting == Box)
			{
				glDisable(GL_TEXTURE_2D);
				glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

				glBegin(GL_LINE_LOOP);
				glVertex2i(box_x1, box_y1);
				glVertex2i(input_state->mx, box_y1);
				glVertex2i(input_state->mx, input_state->my);
				glVertex2i(box_x1, input_state->my);
				glEnd();
			}
			else
			{
				GUIComponent* comp = canvas.GetComponentAtPos(input_state->mx, input_state->my);
				if(comp == NULL || comp == &canvas)
				{
					FindMouseoverBone();
					if(mouseover_bone >= 0)
						font->Print(Bone::string_table[bones[mouseover_bone].name], float(input_state->mx), input_state->my - font->font_height);
				}
			}

			cursor->Draw(float(input_state->mx), float(input_state->my));
		}



		struct KeyListener : public EventHandler
		{
			Imp* imp;

			void HandleEvent(Event* evt)
			{
				KeyStateEvent* kse = (KeyStateEvent*)evt;
				if(kse->state)
				{
					switch(kse->key)
					{
						case VK_SPACE:	{ imp->ClearSelection();				break; }
						case VK_F3:		{ imp->draw_model = !imp->draw_model;	break; }

						case VK_ESCAPE:	{ imp->next_screen = NULL;				break; }

						default:		{ break; }
					}
				}
			}
		} key_listener;

		struct MouseListener : public EventHandler
		{
			Imp* imp;

			void HandleEvent(Event* evt)
			{
				MouseButtonStateEvent* mbse = (MouseButtonStateEvent*)evt;
				int mx = imp->input_state->mx;
				int my = imp->input_state->my;

				if(mbse->state)
				{
					switch(mbse->button)
					{
						case 0:

							if(!imp->canvas.OnClick(mx, my))
							{
								if(imp->mouseover_bone >= 0)
								{
									bool& selected = imp->bones[imp->mouseover_bone].selected;

									if(selected)
										--imp->selection_count;
									else
										++imp->selection_count;

									selected = !selected;

									imp->box_selecting = Disable;
								}
								else
								{
									imp->box_x1 = mx;
									imp->box_y1 = my;

									imp->box_selecting = Waiting;
								}
							}

							break;

						default:
							break;
					}
				}
				else
				{
					switch(mbse->button)
					{
						case 0:

							if(imp->box_selecting == Box)
								imp->SelectBonesInRectangle(imp->box_x1, imp->box_y1, mx, my);
							imp->box_selecting = None;

							break;

						default:
							break;
					}
				}
			}
		} mouse_listener;

		struct MouseMotionListener : public EventHandler
		{
			Imp* imp;

			void HandleEvent(Event* evt)
			{
				MouseMotionEvent* mme = (MouseMotionEvent*)evt;
				if(imp->input_state->mb[0])
				{
					if(imp->box_selecting == Waiting && (abs(mme->x - imp->box_x1) > 2 || abs(mme->y - imp->box_y1) > 2))
						imp->box_selecting = Box;
				}
				if(imp->input_state->mb[2])
				{
					imp->yaw   += mme->dx * 0.01f;
					imp->pitch += mme->dy * 0.01f;
				}
			}
		} mouse_motion_listener;
	};




	/*
	 * DATScreen methods
	 */
	DATScreen::DATScreen(ProgramWindow* win) : ProgramScreen(win), imp(new Imp(win)) { imp->next_screen = this; }
	DATScreen::~DATScreen() { if(imp) { delete imp; imp = NULL; } }

	ProgramScreen* DATScreen::Update(TimingInfo time)
	{
		if(imp->next_screen == this)
			imp->Update(time);

		return imp->next_screen;
	}

	void DATScreen::Draw(int width, int height) { if(width > 0 && height > 0) { imp->Draw(width, height); } }
}
