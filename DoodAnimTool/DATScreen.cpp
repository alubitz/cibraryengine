#include "StdAfx.h"
#include "DATScreen.h"

#include "DATBone.h"
#include "DATKeyframe.h"

#include "PoseSolverState.h"

#include "Constraint.h"
#include "CSkeletalJoint.h"
#include "CFixedJoint.h"
#include "CFlatFoot.h"

#include "../CibraryEngine/DebugDrawMaterial.h"

#include "GCanvas.h"
#include "GColumnList.h"
#include "GCheckbox.h"

#define MAX_SOLVER_ITERATIONS 200

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

		Skeleton* skeleton;
		vector<Mat4> bone_matrices;
		SkinnedCharacterRenderInfo sk_rinfo;

		ModelPhysics* mphys;
		UberModel *dood_uber;

		vector<DATBone> bones;
		vector<unsigned int> id_to_bones;		// given a bone's "name", returns index into the above +1

		vector<DATKeyframe> keyframes;

		float anim_timer;
		unsigned int edit_keyframe;

		vector<Constraint*> constraints;

		vector<Vec3> debug_dots;

		unsigned int selection_count;

		int mouseover_bone;

		float yaw, pitch;
		CameraView camera;
		SceneRenderer renderer;

		Cursor* cursor;
		BitmapFont* font;

		enum BoxSelectMode { None, Disable, Waiting, Box } box_selecting;

		int box_x1, box_y1;

		float now, buffered_time;

		float errors[PoseSolverState::ERROR_TYPES];

		class ConstraintCheckbox : public GCheckbox
		{
		public:
			Imp* imp;
			unsigned int index;

			ConstraintCheckbox(Imp* imp, unsigned int index) : GCheckbox(imp->font), imp(imp), index(index) { }
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
		GCanvas canvas;

		Imp(ProgramWindow* window) :
			next_screen(NULL),
			window(window),
			input_state(window->input_state),
			skeleton(NULL),
			mphys(NULL),
			dood_uber(NULL),
			camera(Mat4::Identity(), 1.0f, 1.0f),				// these values don't matter; they will be overwritten before use
			renderer(&camera),
			cursor(NULL),
			font(NULL),
			box_selecting(None),
			key_listener(),
			mouse_listener()
		{
			key_listener.imp = mouse_listener.imp = mouse_motion_listener.imp = this;

			input_state->KeyStateChanged += &key_listener;
			input_state->MouseButtonStateChanged += &mouse_listener;
			input_state->MouseMoved += &mouse_motion_listener;

			cursor = window->content->GetCache<Cursor>()->Load("Cursor");
			font = window->content->GetCache<BitmapFont>()->Load("../Font");

			constraints_listbox = GColumnList(5, GColumnList::Left);
			constraints_listbox.AddColumn(20, GColumnList::Left);

			bones_listbox = GColumnList(5, GColumnList::Center);
			bones_listbox.AddColumn(5, GColumnList::Center);
			bones_listbox.AddColumn(20, GColumnList::Left);

			sel_label = GLabel(font, "sel");
			lock_label = GLabel(font, "lock");

			canvas.AddChild(&constraints_listbox, GCanvas::HAlign(20, NULL), GCanvas::VAlign(20, NULL));
			canvas.AddChild(&bones_listbox, GCanvas::HAlign(NULL, 20), GCanvas::VAlign(20, NULL));

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

		void LoadDood(const string& dood_name)
		{
			Cache<ModelPhysics>* mphys_cache = window->content->GetCache<ModelPhysics>();
			Cache<UberModel>*    uber_cache  = window->content->GetCache<UberModel>();
			Cache<Material>*     mat_cache   = window->content->GetCache<Material>();

			mphys = mphys_cache->Load(dood_name);
			dood_uber = uber_cache->Load(dood_name);
			dood_uber->LoadCachedMaterials(mat_cache);

			skeleton = dood_uber->CreateSkeleton();

			bones.clear();
			keyframes.clear();

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

			selection_count = 0;

			mouseover_bone = -1;

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
				id_to_bones.clear();

				unsigned int biggest_bone = 0;
				for(unsigned int i = 0; i < bones.size(); ++i)
					biggest_bone = max(biggest_bone, bones[i].name);
			
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
				constraints.push_back(new CSkeletalJoint(&mphys->joints[i], bones));

			DATKeyframe initial_pose = GetDefaultPose();
			keyframes.push_back(initial_pose);
		}

		int GetBoneIndex(const string& bone_name)
		{
			unsigned int id = Bone::string_table[bone_name];
			return id < id_to_bones.size() ? (signed)id_to_bones[id] - 1 : -1;
		}

		void AddSpecialConstraint(const string& name, Constraint* c)
		{
			constraint_checkboxes.push_back(new ConstraintCheckbox(this, constraints.size()));
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

			if(lhand >= 0 && gun >= 0)
				AddSpecialConstraint("l grip", new CFixedJoint((unsigned int)lhand, (unsigned int)gun,     Vec3( 0.990f, 1.113f, 0.037f), Vec3(0.000f,  0.000f,  0.468f), Quaternion::FromPYR(-0.0703434f, -0.0146932f,  2.50207f)));
			if(rhand >= 0 && gun >= 0)
				AddSpecialConstraint("r grip", new CFixedJoint((unsigned int)rhand, (unsigned int)gun,     Vec3(-0.959f, 1.098f, 0.077f), Vec3(0.000f, -0.063f, -0.152f), Quaternion::FromPYR( 1.27667f,   -0.336123f,  -0.64284f)));
			if(lfoot >= 0 && lground >= 0)
				AddSpecialConstraint("l ground", new CFlatFoot((unsigned int)lfoot, (unsigned int)lground, Vec3( 0.238f, 0.000f, 0.065f), Vec3(), Quaternion::Identity()));
			 if(rfoot >= 0 && rground >= 0)
				AddSpecialConstraint("r ground", new CFlatFoot((unsigned int)rfoot, (unsigned int)rground, Vec3(-0.238f, 0.000f, 0.065f), Vec3(), Quaternion::Identity()));
		}

		void DoSoldierSpecificKeyframeStuff(DATKeyframe& initial_pose)
		{
			// we need to position the gun in a way that won't give the soldier too violent of an initial jerk
			int gun = GetBoneIndex("gun");
			if(gun >= 0)
			{
				initial_pose.data[gun].pos = Vec3(-0.5f, 1, 0.5f);
				initial_pose.data[gun].ori = Quaternion::FromPYR(0, 1.5f, 0);
			}

			// and set the appropriate positions for the objects under foot
			int lground = GetBoneIndex("l ground");
			if(lground >= 0)
				initial_pose.data[lground].pos = Vec3( 0.238f, 0.000f, 0.065f);

			int rground = GetBoneIndex("r ground");
			if(rground >= 0)
				initial_pose.data[rground].pos = Vec3(-0.238f, 0.000f, 0.065f);

			// we'll also want to disable one of the two grips; trying to enforce both immediately would result in some weird poses
			initial_pose.enabled_constraints[constraint_checkboxes[0]->index] = false;
		}

		DATKeyframe GetDefaultPose()
		{
			DATKeyframe pose(bones.size(), constraints.size());

			DoSoldierSpecificKeyframeStuff(pose);
			ApplyConstraints(pose);

			return pose;
		}

		void DeleteConstraints()
		{
			for(vector<Constraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
				delete *iter;
			constraints.clear();
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

							if(input_state->keys[VK_SHIFT])
							{
								// translate the selected bones
								Vec3 delta = bone_controls * (1.0f * timestep);
								for(unsigned int i = 0; i < keyframe.num_bones; ++i)
								{
									if(bones[i].selected)
										keyframe.data[i].pos += delta;
								}
							}
							else
							{
								// find the center of the selected bones
								Quaternion delta_quat = Quaternion::FromPYR(bone_controls * timestep);

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
										DATKeyframe::KBone& bone = keyframe.data[i];

										Mat4 oldmat = Mat4::FromPositionAndOrientation(bone.pos, Quaternion::Reverse(bone.ori));
										Vec3 oldcenter = Mat4::Invert(oldmat).TransformVec3_1(center);

										Quaternion& ori = bone.ori;
										ori = delta_quat * ori;

										Mat4 newmat = Mat4::FromPositionAndOrientation(bone.pos, Quaternion::Reverse(bone.ori));
										Vec3 newcenter = newmat.TransformVec3_1(oldcenter);

										bone.pos += center - newcenter;
									}
								}
							}

							// enforce constraints
							ApplyConstraints(keyframe);
							applied_constraints = true;
						}
					}

					// control to force additional iterations of the constraint solver
					if(!applied_constraints && input_state->keys[VK_RETURN])
						ApplyConstraints(keyframes[0]);
				}
			}
		}

		void ApplyConstraints(DATKeyframe& pose)
		{
			// setup
			PoseSolverState pss(pose);

			vector<Constraint*> active_constraints;
			active_constraints.reserve(constraints.size());

			for(unsigned int i = 0; i < constraints.size(); ++i)
				if(pose.enabled_constraints[i])
				{
					Constraint* c = constraints[i];

					c->InitCachedStuff(pss);
					active_constraints.push_back(c);
				}
			Constraint** constraints_begin = active_constraints.data();
			Constraint** constraints_end = constraints_begin + active_constraints.size();

			unsigned int num_bones = pose.num_bones;
			
			// actually doing the iterations
			for(unsigned int i = 0; i < MAX_SOLVER_ITERATIONS; ++i)
			{
				pss.PreIteration();

				unsigned int num_changes = 0;

				for(Constraint** jter = constraints_begin; jter != constraints_end; ++jter)
					if((*jter)->ApplyConstraint(pss))
						++num_changes;

				for(unsigned int j = 0; j < num_bones; ++j)
					if(bones[j].locked)
						pss.contrib_count[j] = 0;

				pss.PostIteration();

				if(num_changes == 0)
					break;
				else
					for(Constraint** jter = constraints_begin; jter != constraints_end; ++jter)
						(*jter)->OnAnyChanges(pss);
			}

			// getting the results
			for(int i = 0; i < PoseSolverState::ERROR_TYPES; ++i)
				errors[i] = pss.errors[i];

			pose = pss.GetFinalPose();
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

		bool DoBoneFrustumTest(Vec3* points, Sphere* spheres_begin, Sphere* spheres_end)
		{
			const char NUM_ITERATIONS = 50;
			const char NUM_OFFSETS = 8;


			Vec3* points_end = points + 8;			// 8 vertices of the frustum

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

					Vec3* jter = points;
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
			Mat4 view_matrix = Mat4::Translation(0, 0, -view_distance) * Mat4::FromQuaternion(Quaternion::FromPYR(pitch, 0, 0) * Quaternion::FromPYR(0, yaw, 0)) * Mat4::Translation(-view_center);
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

			// draw the skinned character
			PoseBones(skeleton);

			skeleton->InvalidateCachedBoneXforms();
			
			skeleton->GetBoneMatrices(bone_matrices);
			sk_rinfo.num_bones = bone_matrices.size();
			sk_rinfo.bone_matrices = SkinnedCharacter::MatricesToTexture1D(bone_matrices, sk_rinfo.bone_matrices);
			dood_uber->Vis(&renderer, 0, Mat4::Identity(), &sk_rinfo);

			// draw helper bones (e.g. gun, ground placeholder for placed foot constraints)
			for(unsigned int i = 0; i < bones.size(); ++i)
				bones[i].DrawHelperObject(&renderer, skeleton);

			// draw outlines of bones' collision shapes
			{
				Vec3 unselected_color = Vec3(0.5f, 0.5f, 0.5f);
				Vec3 selected_color   = Vec3(1.0f, 1.0f, 0.5f) * (0.5f + 0.5f * sinf(now * float(M_PI) * 2.0f * 4.0f));

				for(unsigned int i = 0; i < bones.size(); ++i)
				{
					const DATBone& bone = bones[i];
					const Bone* skel_bone = skeleton->bones[bone.bone_index];
					bone.shape->DebugDraw(&renderer, skel_bone->pos, Quaternion::Reverse(skel_bone->ori), bone.selected ? selected_color : unselected_color);
				}
			}

			skeleton->InvalidateCachedBoneXforms();

			renderer.Render();
			renderer.Cleanup();

			sk_rinfo.Invalidate();

			// 2d overlay
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0, width, height, 0, -1, 1);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			float errx = float(width) - font->font_spacing * 25;
			string errnames[PoseSolverState::ERROR_TYPES] = { "skel rot : ", "skel lim : ", "skel pos : ", " fix ori : ", " fix pos : ", "foot ori : ", "foot pos : " };
			for(int i = 0; i < PoseSolverState::ERROR_TYPES; ++i)
				font->Print(((stringstream&)(stringstream() << errnames[i] << errors[i])).str(), errx, font->font_height * i);
			
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
						case VK_SPACE:	{ imp->ClearSelection();                     break; }
						case VK_ESCAPE:	{ imp->next_screen = NULL;                   break; }

						case 'R':		{ imp->keyframes[0] = imp->GetDefaultPose(); break; }

						case VK_HOME:   { imp->LoadPose(); break; }
						case VK_END:    { imp->SavePose(); break; }

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
