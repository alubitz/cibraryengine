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

	class DerpPose : public Pose
	{
		public:

			void UpdatePose(TimingInfo time)
			{
				// TODO: implement this

				SetBonePose(Bone::string_table["r shoulder"], Vec3(0, sinf(time.total) * 0.5f + 0.5f, 0), Vec3());
				SetBonePose(Bone::string_table["torso 1"], Vec3(sinf(time.total * 0.5f) * 0.1f, 0, 0), Vec3());
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
		DerpPose* pose;

		Skeleton* skeleton;
		vector<IKBone*> ik_bones;

		CameraView* camera;
		SceneRenderer* renderer;

		BitmapFont* font;

		float now;
		float yaw, pitch;

		Imp(ProgramWindow* window) : next_screen(NULL), window(window), input_state(window->input_state), character(NULL), pose(NULL), skeleton(NULL), ik_bones(), camera(NULL), renderer(NULL)
		{
			string filename = "soldier";
			ModelPhysics* mphys = window->content->GetCache<ModelPhysics>()->Load(filename);
			UberModel* uber = window->content->GetCache<UberModel>()->Load(filename);

			skeleton = uber->CreateSkeleton();

			for(vector<ModelPhysics::BonePhysics>::iterator iter = mphys->bones.begin(); iter != mphys->bones.end(); ++iter)
				if(Bone* bone = skeleton->GetNamedBone(iter->bone_name))
					WrapBone(bone, (MultiSphereShape*)iter->collision_shape);

			character = new PosedCharacter(skeleton);

			pose = new DerpPose();
			character->active_poses.push_back(pose);

			camera = new CameraView(Mat4::Identity(), 1.0f, 1.0f);		// these values don't matter; they will be overwritten every frame
			renderer = new SceneRenderer(camera);

			now = 0.0f;
			yaw = 0.0f;
			pitch = 0.0f;

			font = window->content->GetCache<BitmapFont>()->Load("../Font");
		}

		~Imp()
		{
			if(renderer) { delete renderer; renderer = NULL; }
			if(camera) { delete camera; camera = NULL; }
			if(character) { character->Dispose(); delete character; character = NULL; pose = NULL; }

			for(vector<IKBone*>::iterator iter = ik_bones.begin(); iter != ik_bones.end(); ++iter)
				delete *iter;
			ik_bones.clear();
		}

		void Update(TimingInfo& time)
		{
			if(input_state->keys[VK_ESCAPE])
			{
				next_screen = NULL;
				return;
			}

			if(input_state->keys[VK_LEFT])
				yaw -= time.elapsed;
			if(input_state->keys[VK_RIGHT])
				yaw += time.elapsed;
			if(input_state->keys[VK_UP])
				pitch -= time.elapsed;
			if(input_state->keys[VK_DOWN])
				pitch += time.elapsed;

			// TODO: set inputs for poses?

			now = time.total;

			character->UpdatePoses(time);
		}

		IKBone* AddRootBone(const string& name, const Vec3& pos, MultiSphereShape* shape)
		{
			IKBone* result = new IKBone();
			result->bone = skeleton->AddBone(Bone::string_table[name], Quaternion::Identity(), pos);
			result->shape = shape;

			ik_bones.push_back(result);

			return result;	
		}

		IKBone* AddChildBone(IKBone* parent, const string& name, const Vec3& pos, MultiSphereShape* shape)
		{
			IKBone* result = new IKBone();
			result->bone = skeleton->AddBone(Bone::string_table[name], parent->bone, Quaternion::Identity(), pos);
			result->shape = shape;

			ik_bones.push_back(result);

			return result;
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
			Mat4 view_matrix = Mat4::Translation(0, 0, -5) * Mat4::FromQuaternion(Quaternion::FromPYR(pitch, 0, 0) * Quaternion::FromPYR(0, yaw, 0)) * Mat4::Translation(0, -1, 0);

			*camera = CameraView(view_matrix, zoom, aspect_ratio);
			
			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf(camera->GetProjectionMatrix().Transpose().values);
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf(camera->GetViewMatrix().Transpose().values);

			// draw stuff
			for(vector<IKBone*>::iterator iter = ik_bones.begin(); iter != ik_bones.end(); ++iter)
				(*iter)->Vis(renderer);

			renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(Vec3(-2, 0, 0), Vec3(2, 0, 0)), 0));
			renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(Vec3(0, 0, -2), Vec3(0, 0, 2)), 0));

			renderer->Render();
			renderer->Cleanup();

			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0, width, height, 0, -1, 1);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			font->Print(((stringstream&)(stringstream() << "time:  " << now)).str(), 0, 0);
		}
	};




	/*
	 * IKScreen methods
	 */
	IKScreen::IKScreen(ProgramWindow* win) : ProgramScreen(win), imp(new Imp(win)) { imp->next_screen = this; }
	IKScreen::~IKScreen() { if(imp) { delete imp; imp = NULL; } }

	ProgramScreen* IKScreen::Update(TimingInfo time) { imp->Update(time); return imp->next_screen; }

	void IKScreen::Draw(int width, int height) { if(width > 0 && height > 0) { imp->Draw(width, height); } }
}
