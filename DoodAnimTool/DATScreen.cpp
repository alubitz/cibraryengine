#include "StdAfx.h"
#include "DATScreen.h"

#include "DATBone.h"
#include "DATKeyframe.h"

#include "PoseSolverState.h"

#include "Constraint.h"
#include "CSkeletalJoint.h"
#include "CFixedJoint.h"

#include "../CibraryEngine/DebugDrawMaterial.h"

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
		vector<Material*> materials;

		UberModel* uber;
		ModelPhysics* mphys;

		UberModel* gun_model;
		vector<Material*> gun_materials;

		vector<DATBone> bones;
		vector<DATKeyframe> keyframes;
		float anim_timer;
		unsigned int edit_keyframe;

		vector<Constraint*> constraints;

		unsigned int selection_count;

		int mouseover_bone;

		CameraView camera;
		SceneRenderer renderer;

		Cursor* cursor;
		BitmapFont* font;

		float now, buffered_time;

		float yaw, pitch;

		float errors[5];

		Imp(ProgramWindow* window) :
			next_screen(NULL),
			window(window),
			input_state(window->input_state),
			skeleton(NULL),
			uber(NULL),
			mphys(NULL),
			camera(Mat4::Identity(), 1.0f, 1.0f),				// these values don't matter; they will be overwritten before use
			renderer(&camera),
			cursor(NULL),
			font(NULL),
			key_listener(),
			mouse_listener()
		{
			key_listener.imp = mouse_listener.imp = mouse_motion_listener.imp = this;

			input_state->KeyStateChanged += &key_listener;
			input_state->MouseButtonStateChanged += &mouse_listener;
			input_state->MouseMoved += &mouse_motion_listener;


			// make sure soldier physics file is up to date... these lines should probably be removed at some point
			ScriptSystem::Init();
			ScriptSystem::GetGlobalState().DoFile("Files/Scripts/soldier_zzp.lua");

			LoadDood("soldier");

			now = buffered_time = 0.0f;
			yaw = 0.0f;
			pitch = 0.0f;

			font = window->content->GetCache<BitmapFont>()->Load("../Font");

			cursor = window->content->GetCache<Cursor>()->Load("Cursor");
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
			Cache<UberModel>* uber_cache = window->content->GetCache<UberModel>();
			Cache<Material>* mat_cache = window->content->GetCache<Material>();

			mphys = mphys_cache->Load(dood_name);
			uber = uber_cache->Load(dood_name);

			materials.clear();
			for(vector<string>::iterator iter = uber->materials.begin(); iter != uber->materials.end(); ++iter)
				materials.push_back(mat_cache->Load(*iter));

			skeleton = uber->CreateSkeleton();

			CreateSoldierSpecificHelperBones(uber_cache, mat_cache);			// adds a bone for the gun at the end of the skeleton's bones array

			bones.clear();
			keyframes.clear();

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

			CreateSoldierSpecificConstraints(mphys_cache);		// creates DATBone for gun, and adds 2 CFixedJoints at the beginning of the constraints list

			// add default constraints to the constraints list
			for(unsigned int i = 0; i < mphys->joints.size(); ++i)
				constraints.push_back(new CSkeletalJoint(&mphys->joints[i], bones));

			DATKeyframe initial_pose = GetDefaultPose();
			keyframes.push_back(initial_pose);
		}

		void CreateSoldierSpecificHelperBones(Cache<UberModel>* uber_cache, Cache<Material>* mat_cache)
		{
			skeleton->AddBone(Bone::string_table["gun"], Quaternion::Identity(), Vec3());
			gun_model = uber_cache->Load("gun");
			gun_materials.clear();

			for(vector<string>::iterator iter = gun_model->materials.begin(); iter != gun_model->materials.end(); ++iter)
				gun_materials.push_back(mat_cache->Load(*iter));
		}

		void CreateSoldierSpecificConstraints(Cache<ModelPhysics>* mphys_cache)
		{
			unsigned int lname = Bone::string_table["l hand"], rname = Bone::string_table["r hand"], gname = Bone::string_table["gun"];

			// create a DATBone for the gun (it's been tacked onto the end of the skeleton's bones array)
			bones.push_back(DATBone(skeleton->bones.size() - 1, gname, mphys_cache->Load("gun")->bones[0].collision_shape));

			// figure out the indices of the DATBones for the left hand, right hand, and gun
			int lhand = -1, rhand = -1, gun = -1;
			for(unsigned int i = 0; i < bones.size(); ++i)
			{
				if(bones[i].name == lname)
					lhand = i;
				else if(bones[i].name == rname)
					rhand = i;
				else if(bones[i].name == gname)
					gun = i;
			}

			// if we managed to find all of those, create a fixed joint contraint between each hand and the gun
			if(lhand >= 0 && rhand >= 0 && gun >= 0)
			{
				constraints.push_back(new CFixedJoint((unsigned int)lhand, (unsigned int)gun, Vec3( 0.990f, 1.113f, 0.037f), Vec3(0.000f,  0.000f,  0.468f), Quaternion::FromPYR(-0.0703434f, -0.0146932f,  2.50207f)));
				constraints.push_back(new CFixedJoint((unsigned int)rhand, (unsigned int)gun, Vec3(-0.959f, 1.098f, 0.077f), Vec3(0.000f, -0.063f, -0.152f), Quaternion::FromPYR( 1.27667f,   -0.336123f,  -0.64284f)));
			}
		}

		void DoSoldierSpecificKeyframeStuff(DATKeyframe& initial_pose)
		{
			// we need to position the gun in a way that won't give the soldier too violent of an initial jerk
			initial_pose.data[bones.size() - 1].pos = Vec3(0, 1, 0.5f);
			initial_pose.data[bones.size() - 1].ori = Quaternion::FromPYR(0, 1.5f, 0);

			// we'll also want to disable one of the two constraints between the gun and the hands; trying to enforce it immediately would result in some weird poses
			initial_pose.enabled_constraints[0] = false;
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
								Vec3 delta = bone_controls * (2.0f * timestep);
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
										center += skeleton->bones[i]->GetTransformationMatrix().TransformVec3_1(bones[i].center);
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

			unsigned int num_constraints = constraints.size();
			for(unsigned int i = 0; i < num_constraints; ++i)
				if(pose.enabled_constraints[i])
					constraints[i]->InitCachedStuff(pss);
			
			// actually doing the iterations
			for(unsigned int i = 0; i < MAX_SOLVER_ITERATIONS; ++i)
			{
				pss.PreIteration();

				unsigned int num_changes = 0;

				for(unsigned int j = 0; j < num_constraints; ++j)
					if(pose.enabled_constraints[j] && constraints[j]->ApplyConstraint(pss))
						++num_changes;

				pss.PostIteration();

				if(num_changes == 0)
					break;
				else
				{
					for(unsigned int j = 0; j < num_constraints; ++j)
						if(pose.enabled_constraints[j])
							constraints[j]->OnAnyChanges(pss);
				}
			}

			// getting the results
			for(int i = 0; i < 5; ++i)
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

			// draw the skinned character
			PoseBones(skeleton);

			skeleton->InvalidateCachedBoneXforms();
			
			skeleton->GetBoneMatrices(bone_matrices);
			sk_rinfo.num_bones = bone_matrices.size();
			sk_rinfo.bone_matrices = SkinnedCharacter::MatricesToTexture1D(bone_matrices, sk_rinfo.bone_matrices);
			uber->Vis(&renderer, 0, Mat4::Identity(), &sk_rinfo, &materials);

			// draw the gun
			gun_model->Vis(&renderer, 0, skeleton->bones[bones[bones.size() - 1].bone_index]->GetTransformationMatrix(), NULL, &gun_materials);

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

			if(selection_count != 0)
			{
				float x = font->font_spacing * 2;
				int row = -1;

				font->Print("selected bones:", 0, ++row * font->font_height);
				for(vector<DATBone>::iterator iter = bones.begin(); iter != bones.end(); ++iter)
					if(iter->selected)
						font->Print(Bone::string_table[iter->name], x, ++row * font->font_height);
			}
			else
				font->Print("nothing selected", 0, 0);

			float errx = float(width) - font->font_spacing * 25;
			string errnames[5] = { "skel rot : ", "skel lim : ", "skel pos : ", " fix ori : ", " fix pos : " };
			for(int i = 0; i < 5; ++i)
				font->Print(((stringstream&)(stringstream() << errnames[i] << errors[i])).str(), errx, font->font_height * i);

			FindMouseoverBone();
			if(mouseover_bone >= 0)
				font->Print(Bone::string_table[bones[mouseover_bone].name], float(input_state->mx), input_state->my - font->font_height);

			cursor->Draw(float(input_state->mx), float(input_state->my));
		}

		// convert cursor position to a world-space ray
		void FindMouseoverBone()
		{
			int mx = input_state->mx;
			int my = input_state->my;

			int window_height = window->GetHeight();

			Vec3 origin, direction;
			camera.GetRayFromDimCoeffs((float)mx / window->GetWidth(), (float)(window_height - my) / window_height, origin, direction);

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

			mouseover_bone = target;
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
						case VK_SPACE:	{ imp->ClearSelection();                     break; }
						case VK_ESCAPE:	{ imp->next_screen = NULL;                   break; }

						case 'R':		{ imp->keyframes[0] = imp->GetDefaultPose(); break; }

						case VK_BACK:
						{
							DATKeyframe& k = imp->keyframes[0];
							bool& b = k.enabled_constraints[0];

							b = !b;
							if(b)
								imp->ApplyConstraints(k);

							break;
						}

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
				if(mbse->state)
				{
					switch(mbse->button)
					{
						case 0:

							if(imp->mouseover_bone >= 0)
							{
								bool& selected = imp->bones[imp->mouseover_bone].selected;

								if(selected)
									--imp->selection_count;
								else
									++imp->selection_count;

								selected = !selected;
							}

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
