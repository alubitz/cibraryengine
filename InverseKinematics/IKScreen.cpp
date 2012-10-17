#include "StdAfx.h"
#include "IKScreen.h"

#include "../CibraryEngine/DebugDrawMaterial.h"

namespace InverseKinematics
{
	using namespace CibraryEngine;

	struct IKBone
	{
		Bone* bone;
		MultiSphereShape* shape;

		IKBone() : bone(NULL), shape(NULL) { }
		~IKBone() { if(shape) { shape->Dispose(); delete shape; shape = NULL; } }

		void Vis(SceneRenderer* renderer)
		{
			if(shape)
			{
				Vec3 pos;
				Quaternion ori;
				bone->GetTransformationMatrix().Decompose(pos, ori);

				shape->DebugDraw(renderer, pos, ori);
			}
		}
	};

	class BreathingPose : public Pose
	{
		public:

			void UpdatePose(TimingInfo time)
			{
				//SetBonePose(Bone::string_table["torso 1"], Vec3(sinf(time.total * 0.5f) * 0.1f, 0, 0), Vec3());
			}
	};

	class AimingPose : public Pose
	{
		public:

			Bone* bone;					// bone which should be aimed

			Quaternion desired_ori;


			AimingPose(Bone* bone) : bone(bone), desired_ori(Quaternion::Identity()) { }

			void UpdatePose(TimingInfo time)
			{
				// TODO: implement this for real
				//SetBonePose(Bone::string_table["r shoulder"], Vec3(0, sinf(time.total) * 0.5f + 0.5f, 0), Vec3());
			}

			void Vis(SceneRenderer* renderer)
			{
				// draw actual aiming line
				Mat4 bone_xform = bone->GetTransformationMatrix();
				Vec3 origin = bone_xform.TransformVec3_1(bone->rest_pos);
				Vec3 direction = bone_xform.TransformVec3_0(Vec3(0, 0, 1));
				renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(origin, origin + direction * 1000, Vec3(1, 0, 0)), 0));
			}
	};

	/*
	 * IKScreen private implementation struct
	 */
	struct IKScreen::Imp
	{
		ProgramScreen* next_screen;
		ProgramWindow* window;
		InputState* input_state;

		PosedCharacter* character;
		AimingPose* aiming_pose;

		Skeleton* skeleton;
		vector<IKBone*> ik_bones;
		IKBone* selection;

		UberModel* uber;

		CameraView camera;
		SceneRenderer renderer;

		Cursor* cursor;
		BitmapFont* font;

		float now, buffered_time;

		Vec3 pos, vel;
		float yaw, pitch;

		Imp(ProgramWindow* window) :
			next_screen(NULL),
			window(window),
			input_state(window->input_state),
			character(NULL),
			aiming_pose(NULL),
			skeleton(NULL),
			uber(NULL),
			ik_bones(),
			camera(Mat4::Identity(), 1.0f, 1.0f),				// these values don't matter; they will be overwritten before use
			renderer(&camera),
			cursor(NULL),
			font(NULL),
			pos(),
			vel(),
			key_listener(),
			mouse_listener()
		{
			key_listener.imp = this;
			input_state->KeyStateChanged += &key_listener;

			mouse_listener.imp = this;
			input_state->MouseButtonStateChanged += &mouse_listener;

			ScriptSystem::Init();
			ScriptSystem::GetGlobalState().DoFile("Files/Scripts/savezzp.lua");

			string filename = "soldier";
			ModelPhysics* mphys = window->content->GetCache<ModelPhysics>()->Load(filename);
			uber = window->content->GetCache<UberModel>()->Load(filename);

			skeleton = uber->CreateSkeleton();

			for(vector<ModelPhysics::BonePhysics>::iterator iter = mphys->bones.begin(); iter != mphys->bones.end(); ++iter)
				if(Bone* bone = skeleton->GetNamedBone(iter->bone_name))
					WrapBone(bone, (MultiSphereShape*)iter->collision_shape);

			character = new PosedCharacter(skeleton);
			character->active_poses.push_back(new BreathingPose());

			if(Bone* aim_bone = skeleton->GetNamedBone("r grip"))
			{
				aiming_pose = new AimingPose(aim_bone);
				character->active_poses.push_back(aiming_pose);
			}

			bool left_fwd = true;

			pos.y = -0.2f;				// otherwise feet would be at 0.2

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

			character->Dispose();
			delete character;
			character = NULL;

			// these will have been deleted by the above
			aiming_pose = NULL;

			for(vector<IKBone*>::iterator iter = ik_bones.begin(); iter != ik_bones.end(); ++iter)
				delete *iter;
			ik_bones.clear();
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

					aiming_pose->desired_ori = Quaternion::Identity();

					if(input_state->keys['A'])
						vel.x += 5.0f * timestep;
					if(input_state->keys['D'])
						vel.x -= 5.0f * timestep;
					if(input_state->keys['W'])
						vel.z += 5.0f * timestep;
					if(input_state->keys['S'])
						vel.z -= 5.0f * timestep;

					pos += vel * timestep;
					vel *= exp(-2.0f * timestep);

					skeleton->GetNamedBone("pelvis")->pos = pos;
					skeleton->InvalidateCachedBoneXforms();

					float speed = vel.ComputeMagnitude();
					Vec3 u_vel = vel / speed;

					character->UpdatePoses(use_time);
				}
			}
		}

		IKBone* WrapBone(Bone* bone, MultiSphereShape* shape)
		{
			IKBone* result = new IKBone();

			result->bone = bone;
			result->shape = shape;
			ik_bones.push_back(result);

			return result;
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
			Mat4 view_matrix = Mat4::Translation(0, 0, -5) * Mat4::FromQuaternion(Quaternion::FromPYR(pitch, 0, 0) * Quaternion::FromPYR(0, yaw, 0)) * Mat4::Translation(0, -1, 0) * Mat4::Translation(-pos);

			camera = CameraView(view_matrix, zoom, aspect_ratio);
			
			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf(camera.GetProjectionMatrix().Transpose().values);
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf(camera.GetViewMatrix().Transpose().values);

			// draw bones' collision shapes
			for(vector<IKBone*>::iterator iter = ik_bones.begin(); iter != ik_bones.end(); ++iter)
				(*iter)->Vis(&renderer);

			// draw grid (ground plane)
			for(short i = -2; i <= 2; ++i)
			{
				renderer.objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(Vec3(-2,	0, i),	Vec3(2, 0, i)),	0));
				renderer.objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(Vec3(i,	0, -2),	Vec3(i, 0, 2)),	0));
			}

			// aiming pose has some info it can display, too
			//aiming_pose->Vis(&renderer);

			// draw the skinned character
			SkinnedCharacterRenderInfo sk_rinfo;

			Bone* pelvis = skeleton->GetNamedBone("pelvis");
			Vec3 offset = pelvis->pos;
			pelvis->pos = Vec3();
			skeleton->InvalidateCachedBoneXforms();
			
			vector<Mat4> bone_matrices;
			skeleton->GetBoneMatrices(bone_matrices);
			sk_rinfo.num_bones = bone_matrices.size();
			sk_rinfo.bone_matrices = SkinnedCharacter::MatricesToTexture1D(bone_matrices);

			float blink_speed = 4.0f;
			float blink_on = 1.0;
			if(now * blink_speed - (int)(now * blink_speed) < blink_on)
				uber->Vis(&renderer, 0, Mat4::Translation(offset), &sk_rinfo, window->content->GetCache<Material>());

			pelvis->pos = offset;
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
				// TODO: maybe revive this?
			}
		} key_listener;

		struct MouseListener : public EventHandler
		{
			Imp* imp;

			void HandleEvent(Event* evt) { }
		} mouse_listener;
	};




	/*
	 * IKScreen methods
	 */
	IKScreen::IKScreen(ProgramWindow* win) : ProgramScreen(win), imp(new Imp(win)) { imp->next_screen = this; }
	IKScreen::~IKScreen() { if(imp) { delete imp; imp = NULL; } }

	ProgramScreen* IKScreen::Update(TimingInfo time)
	{
		if(imp->next_screen == this)
			imp->Update(time);

		return imp->next_screen;
	}

	void IKScreen::Draw(int width, int height) { if(width > 0 && height > 0) { imp->Draw(width, height); } }
}
