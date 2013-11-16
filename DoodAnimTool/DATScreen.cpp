#include "StdAfx.h"
#include "DATScreen.h"

#include "DATJoint.h"
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

		vector<DATJoint> joints;
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
			key_listener.imp = this;
			input_state->KeyStateChanged += &key_listener;

			mouse_listener.imp = this;
			input_state->MouseButtonStateChanged += &mouse_listener;

			ScriptSystem::Init();
			ScriptSystem::GetGlobalState().DoFile("Files/Scripts/soldier_zzp.lua");

			LoadDood("soldier");
			//SelectAll();

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

			if(skeleton)				{ skeleton->Dispose();					delete skeleton;				skeleton = NULL; }
			if(sk_rinfo.bone_matrices)	{ sk_rinfo.bone_matrices->Dispose();	delete sk_rinfo.bone_matrices;	sk_rinfo.bone_matrices = NULL; }

			DeleteConstraints();

			DebugDrawMaterial::GetDebugDrawMaterial()->EmptyRecycleBin();
		}

		void LoadDood(const string& dood_name)
		{
			mphys = window->content->GetCache<ModelPhysics>()->Load(dood_name);
			uber = window->content->GetCache<UberModel>()->Load(dood_name);

			materials.clear();

			Cache<Material>* mat_cache = window->content->GetCache<Material>();
			for(vector<string>::iterator iter = uber->materials.begin(); iter != uber->materials.end(); ++iter)
				materials.push_back(mat_cache->Load(*iter));

			skeleton = uber->CreateSkeleton();

			joints.clear();
			bones.clear();
			keyframes.clear();

			DeleteConstraints();

			anim_timer = 0.0f;
			edit_keyframe = 0;

			selection_count = 0;

			mouseover_bone = -1;

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
						int parent_index = -1;

						if(Bone* parent_bone = bone->parent)
						{
							DATJoint dat_joint;
							dat_joint.child_index = i;

							string parent_name = Bone::string_table[bone->parent->name];

							for(vector<ModelPhysics::JointPhysics>::iterator kter = mphys->joints.begin(); kter != mphys->joints.end(); ++kter)
							{
								ModelPhysics::JointPhysics& joint = *kter;
								ModelPhysics::BonePhysics* bone_a = joint.bone_a == 0 ? NULL : &mphys->bones[joint.bone_a - 1];
								ModelPhysics::BonePhysics* bone_b = joint.bone_b == 0 ? NULL : &mphys->bones[joint.bone_b - 1];

								if(bone_a == pbone && bone_b->bone_name == parent_name || bone_b == pbone && bone_a->bone_name == parent_name)
								{
									dat_joint.joint = &joint;
									dat_joint.child_reversed = bone_b == pbone;

									joints.push_back(dat_joint);

									break;
								}
							}

							for(unsigned int j = 0; j < num_bones; ++j)
								if(skeleton->bones[j] == parent_bone)
								{
									parent_index = j;
									break;
								}
						}

						bones.push_back(DATBone(bone, pbone->collision_shape, parent_index));
					}
				}
			}

			keyframes.push_back(DATKeyframe(bones.size(), joints.size()));

			// emancipate all the bones
			for(unsigned int i = 0; i < skeleton->bones.size(); ++i)
				skeleton->bones[i]->parent = NULL;


			// initialize constraints list
			for(unsigned int i = 0; i < joints.size(); ++i)
				constraints.push_back(new CSkeletalJoint(joints[i]));

			// create fixed joint constraint between soldier's gun hands		// TODO: make this less hard-coded
			unsigned int lname = Bone::string_table["l hand"], rname = Bone::string_table["r hand"];
			int lhand = -1, rhand = -1;
			for(unsigned int i = 0; i < num_bones; ++i)
			{
				if(bones[i].bone->name == lname)
				{
					lhand = i;
					if(rhand != -1)
						break;
				}
				else if(bones[i].bone->name == rname)
				{
					rhand = i;
					if(lhand != -1)
						break;
				}
			}
			if(lhand >= 0 && rhand >= 0)
				constraints.push_back(new CFixedJoint((unsigned int)lhand, (unsigned int)rhand, Vec3(), Quaternion::Identity()));		// TODO: use actual values
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

					// rotate the camera around the dood based on keyboard input
					if(input_state->keys[VK_LEFT])
						yaw -= timestep;
					if(input_state->keys[VK_RIGHT])
						yaw += timestep;
					if(input_state->keys[VK_UP])
						pitch -= timestep;
					if(input_state->keys[VK_DOWN])
						pitch += timestep;

					// control the selected bone
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
							Quaternion delta_quat = Quaternion::FromPYR(bone_controls * timestep);

							DATKeyframe& keyframe = keyframes[edit_keyframe];
							for(unsigned int i = 0; i < keyframe.num_bones; ++i)
							{
								if(bones[i].selected)
								{
									Quaternion& ori = keyframe.data[i].ori;
									ori = delta_quat * ori;
								}
							}

							ApplyConstraints(keyframe);
						}
					}
				}
			}
		}

		void ApplyConstraints(DATKeyframe& pose)
		{
			PoseSolverState pss(pose);

			unsigned int num_constraints = constraints.size();
			for(unsigned int i = 0; i < num_constraints; ++i)
				if(pose.enabled_constraints[i])
					constraints[i]->InitCachedStuff(pss);
			
			for(unsigned int i = 0; i < MAX_SOLVER_ITERATIONS; ++i)
			{
				unsigned int num_changes = 0;

				for(unsigned int j = 0; j < num_constraints; ++j)
					if(pose.enabled_constraints[j] && constraints[j]->ApplyConstraint(pss))
						++num_changes;

				if(num_changes == 0)
					break;
				else
				{
					pss.current = pss.next;

					for(unsigned int j = 0; j < num_constraints; ++j)
						if(pose.enabled_constraints[j])
							constraints[j]->OnAnyChanges(pss);
				}
			}

			pose = pss.GetFinalPose();
		}

		void PoseBones(Skeleton* skeleton) { PoseBones(skeleton, keyframes[edit_keyframe]); }

		void PoseBones(Skeleton* skeleton, const DATKeyframe& pose)
		{
			skeleton->InvalidateCachedBoneXforms();

			unsigned int num_bones = bones.size();
			for(unsigned int i = 0; i < num_bones; ++i)
			{
				skeleton->bones[i]->ori = pose.data[i].ori;
				skeleton->bones[i]->pos = pose.data[i].pos;
			}
		}

		void PoseBones(Skeleton* skeleton, const DATKeyframe& frame_a, const DATKeyframe& frame_b, float b_frac)
		{
			skeleton->InvalidateCachedBoneXforms();

			float a_frac = 1.0f - b_frac;

			unsigned int num_bones = bones.size();
			for(unsigned int i = 0; i < num_bones; ++i)
			{
				skeleton->bones[i]->ori = frame_a.data[i].ori * a_frac + frame_b.data[i].ori * b_frac;
				skeleton->bones[i]->pos = frame_a.data[i].pos * a_frac + frame_b.data[i].pos * b_frac;
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
			Mat4 view_matrix = Mat4::Translation(0, 0, -5) * Mat4::FromQuaternion(Quaternion::FromPYR(pitch, 0, 0) * Quaternion::FromPYR(0, yaw, 0)) * Mat4::Translation(0, -1, 0);

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

			// draw outlines of bones' collision shapes
			{
				bool show_selected_bones = ((now * 4.0f) - int(now * 4.0f)) < 0.5f;
				Vec3 bone_pos;
				Quaternion bone_ori;
				for(unsigned int i = 0; i < bones.size(); ++i)
				{
					DATBone& bone = bones[i];
					if(show_selected_bones || !bone.selected)
						bone.shape->DebugDraw(&renderer, skeleton->bones[i]->pos, Quaternion::Reverse(skeleton->bones[i]->ori));
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

			font->Print(((stringstream&)(stringstream() << "time:  " << now)).str(), 0, 0);

			if(selection_count != 0)
			{
				float x = font->font_spacing * 2;
				int row = 1;

				font->Print("selected bones:", 0, ++row * font->font_height);
				for(vector<DATBone>::iterator iter = bones.begin(); iter != bones.end(); ++iter)
					if(iter->selected)
						font->Print(Bone::string_table[iter->bone->name], x, ++row * font->font_height);
			}
			else
				font->Print("nothing selected", 0, 2 * font->font_height);

			FindMouseoverBone();
			if(mouseover_bone >= 0)
				font->Print(Bone::string_table[bones[mouseover_bone].bone->name], float(input_state->mx), input_state->my - font->font_height);

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
				Bone* bone = dat_bone.bone;
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
						case VK_SPACE:	{ imp->ClearSelection();    break; }
						case VK_ESCAPE:	{ imp->next_screen = NULL;  break; }

						case 'R':		{ imp->LoadDood("soldier"); break; }

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
