#include "StdAfx.h"
#include "Brambles.h"
#include "TestGame.h"
#include "ConverterWhiz.h"

namespace Test
{
	/*
	 * Bramble methods
	 */
	Bramble::Bramble(GameState* gs, const Vec3& pos) :
		Entity(gs),
		root(NULL),
		bs(),
		uber_models(),
		rigid_bodies()
	{
		root = new BrambleNode(pos, Vec3(), Quaternion::FromRVec(0, Random3D::Rand() * float(M_PI) * 2.0f, 0), 1);

		while(root->Grow()) { }					// deliberately empty while loop

		VertexBuffer* vbo = gs->content->GetCache<VertexBuffer>()->Load("bramble_node");
		unsigned int n_verts = vbo->GetNumVerts();
		float* xyz = vbo->GetFloatPointer("gl_Vertex");
		float* n   = vbo->GetFloatPointer("gl_Normal");
		float* uv  = vbo->GetFloatPointer("gl_MultiTexCoord0");

		root->GetUberModels(uber_models, n_verts, xyz, uv, n);

		for(unsigned int i = 0; i < uber_models.size(); i++)
		{
			uber_models[i]->LoadCachedMaterials(gs->content->GetCache<Material>());

			if(i == 0)
				bs = uber_models[i]->GetBoundingSphere();
			else
				bs = Sphere::Expand(bs, uber_models[i]->GetBoundingSphere());

			unsigned int n = uber_models[i]->lods[0]->vertices.size();
			Debug(((stringstream&)(stringstream() << '\t' << "brambles contains " << n << " verts = " << (n / n_verts) << " nodes" << endl)).str());
		}
	}

	void Bramble::InnerDispose()
	{
		delete root;

		for(unsigned int i = 0; i < uber_models.size(); i++)
		{
			uber_models[i]->Dispose();
			delete uber_models[i];
		}
		uber_models.clear();
	}

	void Bramble::Vis(SceneRenderer* renderer)
	{
		if(renderer->camera->CheckSphereVisibility(bs))
			for(unsigned int i = 0; i < uber_models.size(); i++)
				uber_models[i]->Vis(renderer, 0, Mat4::Identity(), NULL);
	}

	void Bramble::Spawned()
	{
		rigid_bodies.clear();
		root->GetRigidBodies(rigid_bodies);
		
		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			game_state->physics_world->AddCollisionObject(*iter);
	}

	void Bramble::DeSpawned()
	{
		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
		{
			game_state->physics_world->RemoveCollisionObject(*iter);

			MultiSphereShape* mss = (MultiSphereShape*)(*iter)->GetCollisionShape();

			(*iter)->Dispose();
			delete *iter;

			mss->Dispose();
			delete mss;
		}
		rigid_bodies.clear();
	}

	bool Bramble::GetShot(Shot* shot, const Vec3& poi, const Vec3& vel, float mass)
	{
		// TODO: make dust particles come out?
		return true;
	}




	/*
	 * BrambleNode methods
	 */
	BrambleNode::BrambleNode(const Vec3& parent_pos, const Vec3& up, const Quaternion& ori, float scale) :
		ori(ori),
		scale(scale),
		parent(NULL),
		children()
	{
		float length = Random3D::Rand(0.9f, 1.1f);
		pos = parent_pos + up * length;

		draw_xform = Mat4::FromPositionAndOrientation(pos, ori) * Mat4::Scale(scale, length, scale);

		split_ready = 0;
	}

	BrambleNode::~BrambleNode()
	{
		for(unsigned int i = 0; i < children.size(); i++)
			delete children[i];
	}

	bool BrambleNode::Grow()
	{
		if(children.size() == 0)
		{
			const float arching_factor = 0.3f;
			const float narrowing_factor = 0.95f;
			const float random_factor = 0.2f;
			const float branch_angle = 0.5f;

			//if(scale > 0.3f)
			if(scale > 0.5f)
			{
				split_ready += Random3D::Rand();

				// maybe split
				if(split_ready > 2.0f)
				{
					int num_splits = 4;
					for(int i = 0; i < num_splits; i++)
					{
						float theta = i * float(M_PI) * 2.0f / num_splits;

						Quaternion nu_ori = Quaternion::FromRVec(-cosf(theta) * branch_angle, 0, -sinf(theta) * branch_angle) * ori;

						Vec3 up = nu_ori.ToMat3() * Vec3(0, 1, 0);

						BrambleNode* child = new BrambleNode(pos, up, nu_ori, scale * narrowing_factor);
						children.push_back(child);
					}
				}
				else
				{
					Vec3 up = ori.ToMat3() * Vec3(0, 1, 0);
					Vec3 cross = Vec3::Cross(up, Vec3(0, 1, 0));

					Vec3 rot = Random3D::RandomNormalizedVector(Random3D::Rand(random_factor)) + cross * arching_factor;

					BrambleNode* child = new BrambleNode(pos, up, Quaternion::FromRVec(-rot) * ori, scale * narrowing_factor);
					child->split_ready = split_ready;
					children.push_back(child);
				}
				return true;
			}
			return false;
		}
		else
		{
			bool any = false;
			for(vector<BrambleNode*>::iterator iter = children.begin(); iter != children.end(); iter++)
			{
				BrambleNode* child = *iter;

				child->parent = this;
				any |= child->Grow();
			}

			return any;
		}
	}

	void BrambleNode::AppendVertexData(UberModel::LOD* lod, unsigned int n_verts, float* pos, float* uv, float* normal)
	{
		if(parent != NULL)
		{
			unsigned int max_i = n_verts * 3;
			for(unsigned int i = 0; i < max_i; i += 9)
			{
				VTNTT a = TransformVertexInfo((Vec3&)pos[i + 0], (Vec3&)normal[i + 0], (Vec3&)uv[i + 0]);
				VTNTT b = TransformVertexInfo((Vec3&)pos[i + 3], (Vec3&)normal[i + 3], (Vec3&)uv[i + 3]);
				VTNTT c = TransformVertexInfo((Vec3&)pos[i + 6], (Vec3&)normal[i + 6], (Vec3&)uv[i + 6]);

				UberModel::Triangle tri;
				tri.material = 0;
				tri.a.v = lod->vertices.size() + 0;
				tri.b.v = lod->vertices.size() + 1;
				tri.c.v = lod->vertices.size() + 2;
				tri.a.t = lod->texcoords.size() + 0;
				tri.b.t = lod->texcoords.size() + 1;
				tri.c.t = lod->texcoords.size() + 2;
				tri.a.n = lod->normals.size() + 0;
				tri.b.n = lod->normals.size() + 1;
				tri.c.n = lod->normals.size() + 2;
				lod->triangles.push_back(tri);

				lod->vertices.push_back(a.x);
				lod->vertices.push_back(b.x);
				lod->vertices.push_back(c.x);
				lod->bone_influences.push_back(UberModel::CompactBoneInfluence());
				lod->bone_influences.push_back(UberModel::CompactBoneInfluence());
				lod->bone_influences.push_back(UberModel::CompactBoneInfluence());
				lod->texcoords.push_back(a.uvw);
				lod->texcoords.push_back(b.uvw);
				lod->texcoords.push_back(c.uvw);
				lod->normals.push_back(a.n);
				lod->normals.push_back(b.n);
				lod->normals.push_back(c.n);
			}
		}
	}

	VTNTT BrambleNode::TransformVertexInfo(Vec3 x, Vec3 n, const Vec3& uvw)
	{
		float y = min(1.0f, max(0.0f, x.y));				// clamp lerp factor (idk, do we need this?)
		float un_y = 1.0f - y;								// opposite fraction of lerp

		// making them actually 'bend' would be quite difficult :(
		Mat4 lerped = parent->draw_xform * un_y + draw_xform * y;

		x = lerped.TransformVec3_1(x.x, 0, x.z);
		n = Vec3::Normalize(lerped.TransformVec3_0(n));

		return VTNTT(x, uvw, n);
	}

	void BrambleNode::GetRigidBodies(vector<RigidBody*>& rigid_bodies)
	{
		static const float reference_size = 0.55f;							// radius of a node with scale = 1

		if(parent != NULL)
		{
			Sphere spheres[2] = { Sphere(pos, scale * reference_size), Sphere(parent->pos, parent->scale * reference_size) };
			MultiSphereShape* mss = new MultiSphereShape(spheres, 2);

			rigid_bodies.push_back(new RigidBody(NULL, mss, MassInfo()));
		}

		for(vector<BrambleNode*>::iterator iter = children.begin(); iter != children.end(); ++iter)
			(*iter)->GetRigidBodies(rigid_bodies);
	}

	void BrambleNode::GetUberModels(vector<UberModel*>& uber_models, unsigned int n_verts, float* pos, float* uv, float* normal)
	{
		UberModel* use_model = NULL;
		if(uber_models.size() != 0)
		{
			use_model = uber_models[uber_models.size() - 1];
			//if(use_model->lods[0]->vertices.size() > 10000)
			//	use_model = NULL;
		}
		if(use_model == NULL)
		{
			use_model = new UberModel();
			uber_models.push_back(use_model);
			use_model->lods.push_back(new UberModel::LOD());
			use_model->lods[0]->lod_name = "LOD 0";
			use_model->materials.push_back("bramble");
		}

		AppendVertexData(use_model->lods[0], n_verts, pos, uv, normal);

		for(unsigned int i = 0; i < children.size(); i++)
			children[i]->GetUberModels(uber_models, n_verts, pos, uv, normal);
	}
}
