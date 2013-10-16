#include "StdAfx.h"
#include "DATScreen.h"

#include "DATJoint.h"
#include "DATBone.h"
#include "DATKeyframe.h"

#include "../CibraryEngine/DebugDrawMaterial.h"

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

		unsigned int selection_count;

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

			if(skeleton)				{ skeleton->Dispose();					delete skeleton;				skeleton = NULL; }
			if(sk_rinfo.bone_matrices)	{ sk_rinfo.bone_matrices->Dispose();	delete sk_rinfo.bone_matrices;	sk_rinfo.bone_matrices = NULL; }

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

			anim_timer = 0.0f;
			edit_keyframe = 0;

			selection_count = 0;

			for(unsigned int i = 0; i < skeleton->bones.size(); ++i)
			{
				Bone* bone = skeleton->bones[i];
				string bone_name = Bone::string_table[bone->name];

				for(vector<ModelPhysics::BonePhysics>::iterator jter = mphys->bones.begin(); jter != mphys->bones.end(); ++jter)
				{
					ModelPhysics::BonePhysics* pbone = &*jter;
					if(pbone->bone_name == bone_name)
					{
						if(bone->parent != NULL)
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
						}

						bones.push_back(DATBone(bone, pbone->collision_shape));
					}
				}
			}

			keyframes.push_back(DATKeyframe(joints.size()));
		}

		void Update(TimingInfo& time)
		{
			if(time.elapsed)
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
					/*
					if(selected_bone_index != -1)
					{
						DATKeyframe& keyframe = keyframes[edit_keyframe];

						Vec3 bone_controls
						(
							(float)((input_state->keys['W'] ? 1 : 0) - (input_state->keys['S'] ? 1 : 0)),
							(float)((input_state->keys['A'] ? 1 : 0) - (input_state->keys['D'] ? 1 : 0)),
							(float)((input_state->keys['Q'] ? 1 : 0) - (input_state->keys['E'] ? 1 : 0))
						);

						if(bone_controls.x != 0 || bone_controls.y != 0 || bone_controls.z != 0)
						{
							PoseBones(skeleton, keyframe);

							Bone* bone = bones[selected_bone_index].bone;
							if(Bone* parent = bone->parent)
							{
								Quaternion ori = bone->GetTransformationMatrix().ExtractOrientation();
								Quaternion parent_ori = parent->GetTransformationMatrix().ExtractOrientation();

								Quaternion desired_ori = ori * Quaternion::FromPYR(bone_controls * timestep);
								Quaternion ori_from_parent = desired_ori * Quaternion::Reverse(parent_ori);

								for(unsigned int i = 0; i < joints.size(); ++i)
								{
									const DATJoint& joint = joints[i];
									if(joint.child_index == selected_bone_index)
									{
										const ModelPhysics::JointPhysics& jp = *joint.joint;
										Vec3 ori_pyr = ori_from_parent.ToPYR();

										Vec3& ori_data = keyframe.ori_data[i];

										ori_data = jp.axes * (joint.child_reversed ? ori_pyr : -ori_pyr);
										jp.ClampAngles(ori_data);
									
										break;
									}
								}
							}


						//	Nudge(keyframe, selected_bone_index, Vec3(), bone_controls);
						}
					}
					*/
				}
			}
		}

		void PoseBones(Skeleton* skeleton) { PoseBones(skeleton, keyframes[edit_keyframe]); }

		void PoseBones(Skeleton* skeleeton, const DATKeyframe& pose)
		{
			Vec3 pyr;
			Quaternion quat_ori;

			unsigned int num_joints = joints.size();
			DATJoint* joint_ptr = joints.data();
			for(unsigned int i = 0; i < num_joints; ++i, ++joint_ptr)
			{
				const DATJoint& joint = *joint_ptr;

				quat_ori = Quaternion::FromPYR(joint.joint->axes.Transpose() * pose.ori_data[i]);
				skeleton->bones[joint.child_index]->ori = joint.child_reversed ? Quaternion::Reverse(quat_ori) : quat_ori;
			}
		}

		void PoseBones(Skeleton* skeleton, const DATKeyframe& frame_a, const DATKeyframe& frame_b, float b_frac)
		{
			skeleton->InvalidateCachedBoneXforms();

			float a_frac = 1.0f - b_frac;

			Vec3 pyr;
			Quaternion quat_ori;

			unsigned int num_joints = joints.size();
			DATJoint* joint_ptr = joints.data();
			for(unsigned int i = 0; i < num_joints; ++i, ++joint_ptr)
			{
				DATJoint& joint = *joint_ptr;

				const ModelPhysics::JointPhysics& jp = *joint.joint;

				pyr = frame_a.ori_data[i] * a_frac + frame_b.ori_data[i] * b_frac;
				jp.ClampAngles(pyr);

				quat_ori = Quaternion::FromPYR(jp.axes.Transpose() * pyr);
				skeleton->bones[joint.child_index]->ori = joint.child_reversed ? Quaternion::Reverse(quat_ori) : quat_ori;
			}
		}

		void ResetSelectedBone()
		{
			/*
			if(selected_bone_index != -1)
			{
				DATKeyframe& keyframe = keyframes[edit_keyframe];

				for(unsigned int i = 0; i < joints.size(); ++i)
					if(joints[i].child_index == selected_bone_index)
					{
						keyframe.ori_data[i] = Vec3();
						break;
					}
			}
			*/
		}

		void Nudge(DATKeyframe& keyframe, int bone_index, const Vec3& push_pos, const Vec3& push_ori)
		{
			assert(bone_index >= 0);

			Skeleton* original = new Skeleton(skeleton);
			Skeleton* test_skeleton = new Skeleton(skeleton);

			PoseBones(original, keyframe);

			Mat4 current_xform = original->bones[bone_index]->GetTransformationMatrix();
			Vec3 pos;
			Quaternion ori;
			current_xform.Decompose(pos, ori);

			// TODO: subtract centerpoint before rotating, and add it back afterwards
			Vec3 desired_pos = pos + push_pos;
			Quaternion desired_ori = ori * Quaternion::FromPYR(push_ori);

			// TODO: run solver thingy
		}

		void ClearSelection()
		{
			for(vector<DATBone>::iterator iter = bones.begin(); iter != bones.end(); ++iter)
				iter->selected = false;

			selection_count = 0;
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

			Bone* root_bone = skeleton->bones[0];
			Vec3 offset = root_bone->pos;
			root_bone->pos = Vec3();
			skeleton->InvalidateCachedBoneXforms();
			
			skeleton->GetBoneMatrices(bone_matrices);
			sk_rinfo.num_bones = bone_matrices.size();
			sk_rinfo.bone_matrices = SkinnedCharacter::MatricesToTexture1D(bone_matrices, sk_rinfo.bone_matrices);

			uber->Vis(&renderer, 0, Mat4::Translation(offset), &sk_rinfo, &materials);

			// draw outlines of bones' collision shapes
			{
				bool show_selected_bones = ((now * 4.0f) - int(now * 4.0f)) < 0.5f;
				Vec3 bone_pos;
				Quaternion bone_ori;
				for(unsigned int i = 0; i < bones.size(); ++i)
				{
					DATBone& bone = bones[i];
					if(show_selected_bones || !bone.selected)
					{
						bone.bone->GetTransformationMatrix().Decompose(bone_pos, bone_ori);
						bone.shape->DebugDraw(&renderer, bone_pos, bone_ori);
					}
				}
			}

			root_bone->pos = offset;
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
						case 'R':		{ imp->ResetSelectedBone(); break; }
						case VK_SPACE:	{ imp->ClearSelection();    break; }
						case VK_ESCAPE:	{ imp->next_screen = NULL;  break; }

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
					if(mbse->button == 0)
					{
						// convert mouse click position to a world-space ray
						int mx = imp->input_state->mx;
						int my = imp->input_state->my;

						int window_height = imp->window->GetHeight();

						Vec3 origin, direction;
						imp->camera.GetRayFromDimCoeffs((float)mx / imp->window->GetWidth(), (float)(window_height - my) / window_height, origin, direction);

						float target_t = 0.0f;
						int target = -1;

						for(unsigned int i = 0; i < imp->bones.size(); ++i)
						{
							DATBone& dat_bone = imp->bones[i];
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

						if(target >= 0)
						{
							bool& selected = imp->bones[target].selected;

							if(selected)
								--imp->selection_count;
							else
								++imp->selection_count;

							selected = !selected;
						}
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
