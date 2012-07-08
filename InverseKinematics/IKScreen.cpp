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

		Vec3 vel;

		IKBone() : bone(NULL), shape(NULL), vel() { }
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

			Skeleton* skeleton;
			vector<IKBone*> ik_bones;

			DerpPose(Skeleton* skeleton) : skeleton(skeleton) { }
			~DerpPose()
			{
				for(vector<IKBone*>::iterator iter = ik_bones.begin(); iter != ik_bones.end(); ++iter)
					delete *iter;
				ik_bones.clear();
			}

			void UpdatePose(TimingInfo time)
			{
				float timestep = time.elapsed;

				for(vector<IKBone*>::iterator iter = ik_bones.begin(); iter != ik_bones.end(); ++iter)
				{
					IKBone* ik_bone = *iter;

					Vec3& vel = ik_bone->vel;
					vel = (vel + Random3D::RandomNormalizedVector(timestep)) * exp(-1.0f * timestep);

					SetBonePose(ik_bone->bone->name, ik_bone->bone->ori.ToPYR() + vel * timestep, Vec3());
				}
			}

			void Vis(SceneRenderer* renderer)
			{
				for(vector<IKBone*>::iterator iter = ik_bones.begin(); iter != ik_bones.end(); ++iter)
					(*iter)->Vis(renderer);
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
	};




	/*
	 * IKScreen private implementation struct
	 */
	struct IKScreen::Imp
	{
		ProgramScreen* next_screen;
		ProgramWindow* window;
		InputState* input_state;

		PosedCharacter* posey;
		DerpPose* pose;

		CameraView* camera;
		SceneRenderer* renderer;

		Imp(ProgramWindow* window) : next_screen(NULL), window(window), input_state(window->input_state), posey(NULL), pose(NULL), camera(NULL), renderer(NULL)
		{
			Sphere spheres[] =
			{
				Sphere(Vec3(),			0.5f),
				Sphere(Vec3(0, 1, 0),	0.5f),
				Sphere(Vec3(0, 2, 0),	0.5f),
				Sphere(Vec3(0, 3, 0),	0.5f)
			};

			Skeleton* skeleton = new Skeleton();

			pose = new DerpPose(skeleton);
			IKBone* A =	pose->AddRootBone	(	"A", Vec3(),		new MultiSphereShape(&spheres[0], 2));
			IKBone* B =	pose->AddChildBone	(A,	"B", Vec3(0, 1, 0),	new MultiSphereShape(&spheres[1], 2));
			IKBone* C =	pose->AddChildBone	(B,	"C", Vec3(0, 2, 0),	new MultiSphereShape(&spheres[2], 2));

			posey = new PosedCharacter(skeleton);
			posey->active_poses.push_back(pose);

			camera = new CameraView(Mat4::Identity(), 1.0f, 1.0f);		// these values don't matter; they will be overwritten every frame
			renderer = new SceneRenderer(camera);
		}

		~Imp()
		{
			if(renderer) { delete renderer; renderer = NULL; }
			if(camera) { delete camera; camera = NULL; }
			if(posey) { posey->Dispose(); delete posey; posey = NULL; pose = NULL; }
			if(pose) { delete pose; pose = NULL; }
		}

		void Update(TimingInfo& time)
		{
			if(input_state->keys[VK_ESCAPE])
			{
				next_screen = NULL;
				return;
			}

			// TODO: set inputs for poses?

			posey->UpdatePoses(time);
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
			Mat4 view_matrix = Mat4::Translation(0, 0, -10);

			*camera = CameraView(view_matrix, zoom, aspect_ratio);
			
			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf(camera->GetProjectionMatrix().Transpose().values);
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf(camera->GetViewMatrix().Transpose().values);

			// draw stuff
			pose->Vis(renderer);

			renderer->Render();
			renderer->Cleanup();
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
