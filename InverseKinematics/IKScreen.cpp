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

	class StepPose : public Pose
	{
		public:

			Bone* end;
			Bone* base;

			struct ChainNode
			{
				Bone* from;
				Bone* to;
				Bone* child;					// parent can be determined using child->parent

				Quaternion ori;
				Vec3 rot;

				ChainNode() : from(NULL), to(NULL), child(NULL) { }
				ChainNode(Bone* from, Bone* to, Bone* child) : from(from), to(to), child(child), ori(Quaternion::Identity()), rot() { }
			};
			vector<ChainNode> chain;				// chain of bones from base to end (including both)

			Quaternion desired_base_ori, desired_end_ori;
			Vec3 desired_base_pos, desired_end_pos;
			float arrive_time;

			StepPose(Bone* end, Bone* base) : end(end), base(base), chain(), arrive_time(-1)
			{
				// desired state defaults to initial state
				Mat4 end_xform = end->GetTransformationMatrix(), base_xform = base->GetTransformationMatrix();
				end_xform.Decompose(desired_end_pos, desired_end_ori);
				base_xform.Decompose(desired_base_pos, desired_base_ori);

				// enumerate the chain of bones to go from "base" to "end"
				vector<Bone*> end_chain, base_chain;
				Bone* cur = end;
				while(cur)
				{
					end_chain.push_back(cur);
					cur = cur->parent;
				}

				cur = base;
				while(cur)
				{
					bool any = false;
					for(vector<Bone*>::reverse_iterator iter = end_chain.rbegin(); iter != end_chain.rend(); ++iter)
						if(*iter == cur)
						{
							base_chain.insert(base_chain.end(), iter, end_chain.rend());

							any = true;
							break;
						}

					if(any)
						break;
					else
					{
						base_chain.push_back(cur);
						cur = cur->parent;
					}
				}

				// we've figured out what bones are in the chain... now to actually create it
				Bone* prev = NULL;
				for(vector<Bone*>::iterator iter = base_chain.begin(); iter != base_chain.end(); ++iter)
				{
					Bone* cur = *iter;
					if(prev)
					{
						if(prev == cur->parent)
							chain.push_back(ChainNode(prev, cur, cur));
						else if(cur == prev->parent)
							chain.push_back(ChainNode(prev, cur, prev));
						else
							Debug("ERROR! Sequential bones don't have a parent-child relationship!\n");
					}

					prev = *iter;
				}
			}

			void UpdatePose(TimingInfo time)
			{
				float timestep = time.elapsed;
				float foresight = timestep * 5.0f;

				// find the velocities which will best get us to the destination pos/ori

				int num_floats = chain.size() * 3;
				float* rot = new float[num_floats];
				float* best = new float[num_floats];
				float* guess = new float[num_floats];
				float* rot_end = &rot[num_floats];
				float* best_end = &best[num_floats];
				float* guess_end = &guess[num_floats];

				float* rot_ptr = rot;
				float* best_ptr = best;
				float* guess_ptr = guess;

				for(vector<ChainNode>::iterator iter = chain.begin(); iter != chain.end(); ++iter)
				{
					//Vec3& vec = iter->rot;
					Vec3 vec;
					
					*(best_ptr++) = *(rot_ptr++) = vec.x;
					*(best_ptr++) = *(rot_ptr++) = vec.y;
					*(best_ptr++) = *(rot_ptr++) = vec.z;
				}

				float best_score;
				for(int i = 0; i < 50; ++i)
				{
					guess_ptr = guess;
					rot_ptr = rot;
					best_ptr = best;

					if(i == 0)
						while(guess_ptr != guess_end)
							*(guess_ptr++) = 0.0f;
					else if(i == 1)
						while(guess_ptr != guess_end)
							*(guess_ptr++) = *(rot_ptr++);
					else
					{
						float* ptr = &guess_ptr[Random3D::RandInt(num_floats)];
						while(best_ptr != best_end)	
							if(guess_ptr == ptr)
								*(guess_ptr++) = *(best_ptr++) + Random3D::Rand(-1, 1);
							else
								*(guess_ptr++) = *(best_ptr++);
					}

					// figure out where this arrangement puts our end effector
					Mat4 xform = Mat4::Identity();

					guess_ptr = guess;
					for(vector<ChainNode>::iterator iter = chain.begin(); iter != chain.end(); ++iter)
					{
						Bone* child = iter->child;

						float x = *(guess_ptr++), y = *(guess_ptr++), z = *(guess_ptr++);
						Quaternion ori = iter->ori * Quaternion::FromPYR(x * foresight, y * foresight, z * foresight);

						Mat4 to_rest_pos = Mat4::Translation(child->rest_pos);
						Mat4 from_rest_pos = Mat4::Translation(-child->rest_pos);
						Mat4 rotation_mat = Mat4::FromQuaternion(ori * child->rest_ori);
						Mat4 offset = Mat4::Translation(child->pos);

						Mat4 net = to_rest_pos * rotation_mat * offset * from_rest_pos;
						if(iter->to != iter->child)
							net = Mat4::Invert(net);

						xform *= net;
					}

					// score based on how closely it matches what we're after
					Quaternion end_ori;
					Vec3 end_pos;
					xform.Decompose(end_pos, end_ori);
					end_pos -= desired_end_pos;
					end_ori -= desired_end_ori;

					float score = end_pos.ComputeMagnitudeSquared() + end_ori.w * end_ori.w + end_ori.x * end_ori.x + end_ori.y * end_ori.y + end_ori.z * end_ori.z;

					Debug(((stringstream&)(stringstream() << "score = " << score << endl)).str());

					if(i == 0 || score < best_score)
					{
						best_score = score;

						best_ptr = best;
						guess_ptr = guess;
						while(best_ptr != best_end)
							*(best_ptr++) = *(guess_ptr++);
					}
				}

				// now apply those velocities, and set the bones' poses accordingly
				best_ptr = best;
				for(vector<ChainNode>::iterator iter = chain.begin(); iter != chain.end(); ++iter)
				{
					float x = *(best_ptr++), y = *(best_ptr++), z = *(best_ptr++);
					iter->rot = Vec3(x, y, z);

					Quaternion& ori = iter->ori;
					ori *= Quaternion::FromPYR(iter->rot * timestep);

					SetBonePose(iter->to->name, ori.ToPYR(), Vec3());
				}

				delete[] rot;
				delete[] guess;
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

		BitmapFont* font;

		float now;
		float yaw, pitch;

		Imp(ProgramWindow* window) :
			next_screen(NULL),
			window(window),
			input_state(window->input_state),
			character(NULL),
			aiming_pose(NULL),
			skeleton(NULL),
			selection(NULL),
			uber(NULL),
			ik_bones(),
			camera(Mat4::Identity(), 1.0f, 1.0f),				// these values don't matter; they will be overwritten before use
			renderer(&camera),
			font(NULL),
			key_listener()
		{
			key_listener.imp = this;
			input_state->KeyStateChanged += &key_listener;

			string filename = "soldier";
			ModelPhysics* mphys = window->content->GetCache<ModelPhysics>()->Load(filename);
			uber = window->content->GetCache<UberModel>()->Load(filename);

			skeleton = uber->CreateSkeleton();

			for(vector<ModelPhysics::BonePhysics>::iterator iter = mphys->bones.begin(); iter != mphys->bones.end(); ++iter)
				if(Bone* bone = skeleton->GetNamedBone(iter->bone_name))
				{
					IKBone* ik_bone = WrapBone(bone, (MultiSphereShape*)iter->collision_shape);
					if(!selection)
						selection = ik_bone;
				}

			character = new PosedCharacter(skeleton);
			character->active_poses.push_back(new BreathingPose());

			if(Bone* aim_bone = skeleton->GetNamedBone("r grip"))
			{
				aiming_pose = new AimingPose(aim_bone);
				character->active_poses.push_back(aiming_pose);
			}

			if(Bone* pelvis = skeleton->GetNamedBone("pelvis"))
			{
				if(Bone* l_foot = skeleton->GetNamedBone("l foot"))
				{
					StepPose* step = new StepPose(l_foot, pelvis);
					step->desired_end_pos += Vec3(0, 0.1f, -0.35f);
					step->arrive_time = 1.0f;

					character->active_poses.push_back(step);
				}
				if(Bone* r_foot = skeleton->GetNamedBone("r foot"))
				{
					StepPose* step = new StepPose(r_foot, pelvis);
					step->desired_end_pos += Vec3(0, 0.3f, 0.1f);
					step->arrive_time = 1.0f;

					character->active_poses.push_back(step);
				}
			}

			now = 0.0f;
			yaw = 0.0f;
			pitch = 0.0f;

			font = window->content->GetCache<BitmapFont>()->Load("../Font");
		}

		~Imp()
		{
			input_state->KeyStateChanged -= &key_listener;

			if(character) { character->Dispose(); delete character; character = NULL; }

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

			float timestep = min(time.elapsed, 1.0f / 60.0f);
			now += timestep;

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

			character->UpdatePoses(use_time);
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

			camera = CameraView(view_matrix, zoom, aspect_ratio);
			
			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf(camera.GetProjectionMatrix().Transpose().values);
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf(camera.GetViewMatrix().Transpose().values);

			/*
			// draw bones
			float flash_rate = 4.0f;
			float flash_on = 0.5f;
			for(vector<IKBone*>::iterator iter = ik_bones.begin(); iter != ik_bones.end(); ++iter)
				if(*iter != selection || (now * flash_rate - (int)(now * flash_rate)) < flash_on)
					(*iter)->Vis(&renderer);

			// draw axis lines
			renderer.objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(Vec3(-2, 0, 0), Vec3(2, 0, 0)), 0));
			renderer.objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(Vec3(0, 0, -2), Vec3(0, 0, 2)), 0));
			*/

			// aiming pose has some info it can display, too
			aiming_pose->Vis(&renderer);

			SkinnedCharacterRenderInfo sk_rinfo;

			vector<Mat4> bone_matrices = skeleton->GetBoneMatrices();
			sk_rinfo.num_bones = bone_matrices.size();
			sk_rinfo.bone_matrices = SkinnedCharacter::MatricesToTexture1D(bone_matrices);

			uber->Vis(&renderer, 0, Mat4::Identity(), &sk_rinfo, window->content->GetCache<Material>());

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
						case VK_OEM_4:	// [
						{
							vector<IKBone*>::reverse_iterator iter = imp->ik_bones.rbegin();
							while(iter != imp->ik_bones.rend())
								if(*iter == imp->selection)
								{
									++iter;
									if(iter != imp->ik_bones.rend())
										imp->selection = *iter;
									break;
								}
								else
									++iter;

							break;
						}

						case VK_OEM_6:		// ]
						{
							vector<IKBone*>::iterator iter = imp->ik_bones.begin();
							while(iter != imp->ik_bones.end())
								if(*iter == imp->selection)
								{
									++iter;
									if(iter != imp->ik_bones.end())
										imp->selection = *iter;
									break;
								}
								else
									++iter;

							break;
						}

					}
				}
				else
				{
					// TODO: implement this
				}
			}
		} key_listener;
	};




	/*
	 * IKScreen methods
	 */
	IKScreen::IKScreen(ProgramWindow* win) : ProgramScreen(win), imp(new Imp(win)) { imp->next_screen = this; }
	IKScreen::~IKScreen() { if(imp) { delete imp; imp = NULL; } }

	ProgramScreen* IKScreen::Update(TimingInfo time) { imp->Update(time); return imp->next_screen; }

	void IKScreen::Draw(int width, int height) { if(width > 0 && height > 0) { imp->Draw(width, height); } }
}
