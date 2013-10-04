#include "StdAfx.h"
#include "DATScreen.h"

#include "DATJoint.h"
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
		vector<pair<Bone*, CollisionShape*>> bone_shapes;
		vector<DATKeyframe> keyframes;
		float anim_timer;

		Bone* selected_bone;

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
			selected_bone(NULL),
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
			keyframes.clear();

			anim_timer = 0.0f;

			for(vector<Bone*>::iterator iter = skeleton->bones.begin(); iter != skeleton->bones.end(); ++iter)
			{
				Bone* bone = *iter;
				string bone_name = Bone::string_table[bone->name];

				for(vector<ModelPhysics::BonePhysics>::iterator jter = mphys->bones.begin(); jter != mphys->bones.end(); ++jter)
				{
					ModelPhysics::BonePhysics* pbone = &*jter;
					if(pbone->bone_name == bone_name)
					{
						if(bone->parent != NULL)
						{
							DATJoint dat_joint;
							dat_joint.child_bone = bone;

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

						bone_shapes.push_back(pair<Bone*, CollisionShape*>(bone, pbone->collision_shape));
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
				}
			}
		}

		void PoseBones()
		{
			skeleton->InvalidateCachedBoneXforms();

			// TODO: use anim_timer to select which keyframes to lerp and what value to use for b_frac
			DATKeyframe& frame_a = keyframes[0];
			DATKeyframe& frame_b = keyframes[0];

			float b_frac = 0.0f;
			float a_frac = 1.0f - b_frac;

			Vec3 pyr;
			Quaternion quat_ori;

			unsigned int num_joints = joints.size();
			DATJoint* joint_ptr = joints.data();
			for(unsigned int i = 0; i < num_joints; ++i, ++joint_ptr)
			{
				DATJoint& joint = *joint_ptr;

				const ModelPhysics::JointPhysics& jp = *joint.joint;
				const Vec3& mins  = jp.min_extents;
				const Vec3& maxes = jp.max_extents;

				pyr = frame_a.ori_data[i] * a_frac + frame_b.ori_data[i] * b_frac;
				pyr.x = max(mins.x, min(maxes.x, pyr.x));
				pyr.y = max(mins.y, min(maxes.y, pyr.y));
				pyr.z = max(mins.z, min(maxes.z, pyr.z));

				quat_ori = Quaternion::FromPYR(jp.axes * pyr);
				joint.child_bone->ori = joint.child_reversed ? Quaternion::Reverse(quat_ori) : quat_ori;
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
			PoseBones();

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
				Vec3 bone_pos;
				Quaternion bone_ori;
				for(vector<pair<Bone*, CollisionShape*>>::iterator iter = bone_shapes.begin(); iter != bone_shapes.end(); ++iter)
				{	
					if(iter->first != selected_bone)
					{
						Mat4 xform = iter->first->GetTransformationMatrix();
						xform.Decompose(bone_pos, bone_ori);

						iter->second->DebugDraw(&renderer, bone_pos, bone_ori);
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

			stringstream ss;
			ss << "selected bone:  ";
			if(selected_bone != NULL)
				ss << Bone::string_table[selected_bone->name];
			font->Print(ss.str(), 0, font->font_height);

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
					if(kse->key == VK_ESCAPE)
						imp->next_screen = NULL;
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
						Bone* target = NULL;

						for(vector<pair<Bone*, CollisionShape*>>::iterator iter = imp->bone_shapes.begin(); iter != imp->bone_shapes.end(); ++iter)
						{
							Bone* bone = iter->first;
							CollisionShape* shape = iter->second;
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
										if(ray_result.t < target_t || target == NULL)
										{
											target = bone;
											target_t = ray_result.t;
										}
									}

									break;
								}

								default:
									break;
							}
						}

						imp->selected_bone = target;
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
