#include "StdAfx.h"
#include "DATScreen.h"

#include "DATBone.h"
#include "DATKeyframe.h"

#include "PoseyDood.h"
#include "SolverInstance.h"

#include "PoseDelta.h"

#include "Constraint.h"
#include "CSkeletalJoint.h"
#include "CFixedJoint.h"
#include "CFlatFoot.h"
#include "CAlignedAxis.h"

#include "../CibraryEngine/DebugDrawMaterial.h"

#include "GCanvas.h"
#include "GColumnList.h"
#include "GCheckbox.h"

#include "FileSelection.h"

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

		class SoldierDood : public PoseyDood
		{
			public:

				Imp* imp;
				SoldierDood(Imp* imp) : PoseyDood(), imp(imp) { }

				void CreateCustomHelperBones(Cache<ModelPhysics>* mphys_cache, Cache<UberModel>* uber_cache, Cache<Material>* mat_cache)
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

				void CreateCustomConstraints()
				{
					// create constraints for the soldier holding the gun, and constraints between the soldier's feet and the ground under them
					int lhand   = GetBoneIndex( "l hand"   ), rhand   = GetBoneIndex( "r hand"   );
					int lfoot   = GetBoneIndex( "l foot"   ), rfoot   = GetBoneIndex( "r foot"   );
					int lground = GetBoneIndex( "l ground" ), rground = GetBoneIndex( "r ground" );
					int gun     = GetBoneIndex( "gun"      );
					int head    = GetBoneIndex( "head"     );

					if(lhand >= 0 && gun >= 0)
						imp->AddSpecialConstraint("l grip",   false, new CFixedJoint( (unsigned int)lhand, (unsigned int)gun,     Vec3( 0.990f, 1.113f, 0.037f), Vec3(0.000f,  0.000f,  0.468f), Quaternion::FromRVec(-0.0703434f, -0.0146932f,  2.50207f)));
					if(rhand >= 0 && gun >= 0)
						imp->AddSpecialConstraint("r grip",   true,  new CFixedJoint( (unsigned int)rhand, (unsigned int)gun,     Vec3(-0.959f, 1.098f, 0.077f), Vec3(0.000f, -0.063f, -0.152f), Quaternion::FromRVec( 1.27667f,   -0.336123f,  -0.64284f)));
					if(lfoot >= 0 && lground >= 0)
						imp->AddSpecialConstraint("l ground", true,  new CFlatFoot(   (unsigned int)lfoot, (unsigned int)lground, Vec3( 0.238f, 0.000f, 0.065f), Vec3(), Quaternion::Identity()));
					if(rfoot >= 0 && rground >= 0)
						imp->AddSpecialConstraint("r ground", true,  new CFlatFoot(   (unsigned int)rfoot, (unsigned int)rground, Vec3(-0.238f, 0.000f, 0.065f), Vec3(), Quaternion::Identity()));
					if(gun >= 0 && head >= 0)
						imp->AddSpecialConstraint("aim/look", false, new CAlignedAxis((unsigned int)gun,   (unsigned int)head,    Vec3(0, 0, 1), Vec3(0, 0, 1)));
				}

				void DoCustomKeyframeStuff(DATKeyframe& initial_pose)
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
		};
		PoseyDood* dood;
		SolverInstance* solver;

		vector<DATKeyframe> keyframes;

		float anim_timer;
		unsigned int edit_keyframe;

		vector<Vec3> debug_dots;

		float yaw, pitch;
		CameraView camera;
		SceneRenderer renderer;
		bool draw_model;



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
				unsigned int index;		// index into dood's special_constraints array

				ConstraintCheckbox(Imp* imp, unsigned int index) : GCheckbox(imp->font), imp(imp), index(index) { }
				void LayoutChildren() { selected = imp->keyframes[imp->edit_keyframe].enabled_constraints[index]; }
				bool OnClick(int x, int y)
				{
					DATKeyframe& keyframe = imp->keyframes[imp->edit_keyframe];
					bool& val = keyframe.enabled_constraints[index];
					val = !val;

					imp->solver->InvalidateCache();
					imp->dood->ApplyConstraints(*imp->solver);

					return true;
				}
		};
		vector<ConstraintCheckbox*> constraint_checkboxes;
		vector<GLabel*> constraint_labels;

		class BoneLockCheckbox : public GCheckbox
		{
			public:
				SolverInstance* solver;
				unsigned int index;

				BoneLockCheckbox(SolverInstance* solver, BitmapFont* font, unsigned int index) : GCheckbox(font), solver(solver), index(index) { }
				void LayoutChildren() { selected = solver->locked_bones[index]; }
				bool OnClick(int x, int y) { bool& val = solver->locked_bones[index]; val = !val; if(!val) { solver->InvalidateCache(); } return true; }
		};
		vector<BoneLockCheckbox*> lock_checkboxes;

		class BoneSelectCheckbox : public GCheckbox
		{
			public:
				Imp* imp;
				unsigned int index;

				BoneSelectCheckbox(Imp* imp, unsigned int index) : GCheckbox(imp->font), imp(imp), index(index) { }
				void LayoutChildren() { selected = imp->dood->bones[index].selected; }
				bool OnClick(int x, int y)
				{
					bool& val = imp->dood->bones[index].selected;
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
			dood(NULL),
			camera(Mat4::Identity(), 1.0f, 1.0f),				// these values don't matter; they will be overwritten before use
			renderer(&camera),
			draw_model(true),
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

			if(dood)   { delete dood;   dood   = NULL; }
			if(solver) { delete solver; solver = NULL; }

			DebugDrawMaterial::GetDebugDrawMaterial()->EmptyRecycleBin();
		}


		void LoadDood(const string& dood_name)
		{
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

			// actual dood loading
			dood = new SoldierDood(this);
			dood->LoadDood(dood_name, window->content);

			// prepare first keyframe
			keyframes.clear();
			anim_timer = 0.0f;
			edit_keyframe = 0;

			mouseover_bone = -1;
			selection_count = 0;

			DATKeyframe initial_pose = dood->GetDefaultPose();
			keyframes.push_back(initial_pose);

			solver = new SolverInstance(dood, &keyframes[0]);

			// create bone select and lock checkbox lists
			bones_listbox.BeginRow();
			bones_listbox.AddToLastRow(&sel_label);
			bones_listbox.AddToLastRow(&lock_label);

			for(unsigned int i = 0; i < dood->bones.size(); ++i)
			{
				select_checkboxes.push_back(new BoneSelectCheckbox(this, i));
				lock_checkboxes.push_back(new BoneLockCheckbox(solver, font, i));
				bone_labels.push_back(new GLabel(font, Bone::string_table[dood->bones[i].name]));

				bones_listbox.BeginRow();

				bones_listbox.AddToLastRow(*select_checkboxes.rbegin());
				bones_listbox.AddToLastRow(*lock_checkboxes.rbegin());
				bones_listbox.AddToLastRow(*bone_labels.rbegin());
			}
		}
		
		void AddSpecialConstraint(const string& name, bool default_val, Constraint* c)
		{
			unsigned int pos = dood->AddSpecialConstraint(name, default_val, c);

			constraint_checkboxes.push_back(new ConstraintCheckbox(this, pos));
			constraint_labels.push_back(new GLabel(font, name));

			constraints_listbox.BeginRow();
			constraints_listbox.AddToLastRow(*constraint_checkboxes.rbegin());
			constraints_listbox.AddToLastRow(*constraint_labels.rbegin());
		}

		

		void Update(const TimingInfo& time)
		{
			static const float translate_speed = 0.5f;
			static const float rotate_speed    = 1.0f;
			static const float timestep        = 1.0f / 60.0f;

			if(time.elapsed > 0)
			{
				buffered_time += time.elapsed;
				if(buffered_time >= timestep)
				{
					now += timestep;
					while(buffered_time >= timestep)
						buffered_time -= timestep;

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
								Vec3 delta = bone_controls * (translate_speed * timestep);
								for(unsigned int i = 0; i < keyframe.num_bones; ++i)
								{
									if(dood->bones[i].selected)
									{
										const DATKeyframe::KBone& oldstate = keyframe.data[i];
										deltas.push_back(PoseDelta(i, oldstate, DATKeyframe::KBone(oldstate.pos + delta, oldstate.ori), true, true));
									}
								}
							}
							else
							{
								// find the center of the selected bones
								Vec3 center;
								for(unsigned int i = 0; i < keyframe.num_bones; ++i)
									if(dood->bones[i].selected)
										center += dood->skeleton->bones[dood->bones[i].bone_index]->GetTransformationMatrix().TransformVec3_1(dood->bones[i].center);
								center /= float(selection_count);

								// rotate the selected bones around their center
								Quaternion delta_quat = Quaternion::FromRVec(bone_controls * (rotate_speed * timestep));
								for(unsigned int i = 0; i < keyframe.num_bones; ++i)
								{
									if(dood->bones[i].selected)
									{
										const DATKeyframe::KBone &oldstate = keyframe.data[i];

										Mat4 oldmat = Mat4::FromPositionAndOrientation(oldstate.pos, oldstate.ori);
										Vec3 oldcenter = Mat4::Invert(oldmat).TransformVec3_1(center);

										Quaternion ori = delta_quat * oldstate.ori;
										deltas.push_back(PoseDelta(i, oldstate, DATKeyframe::KBone(center - ori * oldcenter, ori), false, true));
									}
								}
							}

#if ENABLE_IMMEDIATE_EDIT_FEEDBACK
							solver->pose = &keyframes[edit_keyframe];
							dood->ApplyConstraints(*solver, &deltas);
							applied_constraints = true;
#else
							for(vector<PoseDelta>::iterator iter = deltas.begin(); iter != deltas.end(); ++iter)
								keyframe.data[iter->bone] = iter->new_state;
#endif
						}
					}

					bool control = input_state->keys[VK_RETURN];							// hold enter to force additional iterations of the constraint solver
					if(!applied_constraints && (control || ENABLE_IMMEDIATE_EDIT_FEEDBACK && !solver->stopped))
					{
						if(control)
						{
							solver->stopped = false;
							solver->noprogress_count = 0;
						}

						solver->pose = &keyframes[edit_keyframe];
						dood->ApplyConstraints(*solver);
					}
				}
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
			dood->Vis(&renderer, keyframes[edit_keyframe], now, draw_model);

			renderer.Render();
			renderer.Cleanup();

			// 2d overlay
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0, width, height, 0, -1, 1);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			// draw GUI
			debug_text.SetText(solver->debug_text);

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
			else if(window->IsActive())
			{
				GUIComponent* comp = canvas.GetComponentAtPos(input_state->mx, input_state->my);
				if(comp == NULL || comp == &canvas)
				{
					FindMouseoverBone();
					if(mouseover_bone >= 0)
						font->Print(Bone::string_table[dood->bones[mouseover_bone].name], float(input_state->mx), input_state->my - font->font_height);
				}
			}

			if(window->IsActive())
				cursor->Draw(float(input_state->mx), float(input_state->my));
		}


		void ClearSelection()
		{
			for(vector<DATBone>::iterator iter = dood->bones.begin(); iter != dood->bones.end(); ++iter)
				iter->selected = false;

			selection_count = 0;
		}

		void SelectAll()
		{
			for(vector<DATBone>::iterator iter = dood->bones.begin(); iter != dood->bones.end(); ++iter)
				iter->selected = true;

			selection_count = dood->bones.size();
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
			for(unsigned int i = 0; i < dood->bones.size(); ++i)
				if(!dood->bones[i].selected)
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
					DATBone& dat_bone = dood->bones[unselected_indices[i]];
					Bone* bone = dood->skeleton->bones[dat_bone.bone_index];
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

			for(unsigned int i = 0; i < dood->bones.size(); ++i)
			{
				DATBone& dat_bone = dood->bones[i];
				Bone* bone = dood->skeleton->bones[dat_bone.bone_index];
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
			string file_path_name;
			if(FileSelection::ShowLoadPoseDialog(window, file_path_name))
			{
				ifstream file(file_path_name, ios::in | ios::binary);
				if(!file)
					Debug("Unable to load pose!\n");
				else
				{
					if(unsigned int error = keyframes[edit_keyframe].Read(file))
						Debug(((stringstream&)(stringstream() << "Error loading pose! DATKeyframe::Read returned error " << error << "!" << endl)).str());
					else
						Debug("Loaded pose successfully!\n");
				}

				solver->InvalidateCache();
			}
		}

		void SavePose()
		{
			string file_path_name;
			if(FileSelection::ShowSavePoseDialog(window, file_path_name))
			{
				ofstream file(file_path_name, ios::out | ios::binary);
				if(!file)
					Debug("Failed to save pose!\n");
				else
				{
					keyframes[edit_keyframe].Write(file);
					if(!file)
						Debug("Something may have gone wrong saving pose!\n");
					else
						Debug("Saved pose successfully!\n");
				}				
			}
		}

		void ResetPose() { solver->InvalidateCache(); keyframes[edit_keyframe] = dood->GetDefaultPose(); }



		void DebugPose() const
		{
			if(solver->cache_valid)
			{
				solver->DebugJos();

				int gun = dood->GetBoneIndex("gun");
				if(gun >= 0)
				{
					int pelvis = dood->GetBoneIndex("pelvis"), torso2 = dood->GetBoneIndex("torso 2");
					/*
					if(pelvis >= 0)
					{
						const DATKeyframe& pose = *solver->pose;
						Vec3 fwd = (Quaternion::Reverse(pose.data[pelvis].ori) * pose.data[gun].ori) * Vec3(0, 0, 1);
						Debug(((stringstream&)(stringstream() << "gun fwd vec relative to pelvis  = (" << fwd.x << ", " << fwd.y << ", " << fwd.z << ")" << endl)).str());
					}
					*/
					if(torso2 >= 0)
					{
						const DATKeyframe& pose = *solver->pose;
						Vec3 fwd = (Quaternion::Reverse(pose.data[pelvis].ori) * pose.data[gun].ori) * Vec3(0, 0, 1);
						Debug(((stringstream&)(stringstream() << "gun fwd vec relative to torso 2 = (" << fwd.x << ", " << fwd.y << ", " << fwd.z << ")" << endl)).str());
					}
				}
			}
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
						case VK_SPACE:  { imp->ClearSelection();                break; }
						case VK_F3:     { imp->draw_model = !imp->draw_model;   break; }
						case VK_END:    { imp->DebugPose();                     break; }

						case VK_ESCAPE: { imp->next_screen = NULL;              break; }

						default:        { break; }
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
									bool& selected = imp->dood->bones[imp->mouseover_bone].selected;

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

	ProgramScreen* DATScreen::Update(const TimingInfo& time)
	{
		if(imp->next_screen == this)
			imp->Update(time);

		return imp->next_screen;
	}

	void DATScreen::Draw(int width, int height) { if(width > 0 && height > 0) { imp->Draw(width, height); } }
}
