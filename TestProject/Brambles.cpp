#include "Brambles.h"

#include "TestGame.h"
#include "ConverterWhiz.h"

namespace Test
{
	/*
	 * Bramble methods
	 */
	Bramble::Bramble(GameState* gs, Vec3 pos) :
		Entity(gs),
		nodes(),
		materials(),
		uber_models()
	{
		vector<VTNModel*> node_models;
		node_models.push_back(gs->content->Load<VTNModel>("bramble_node_0"));
		VTNModel* collision_model = gs->content->Load<VTNModel>("cylinder");

		root = new BrambleNode(pos, Vec3(), Quaternion::Identity(), 1, 0, node_models, collision_model);

		while (root->Grow()) { }					// deliberately empty while loop

		materials.push_back(gs->content->Load<Material>("wood"));

		root->GetUberModels(uber_models);

		VUVNTTCVertexBuffer* collision_data = new VUVNTTCVertexBuffer();
		root->AppendCollisionData(collision_data, collision_model);
		VTNModel* vtn = new VTNModel(collision_data);
		uber_models[0]->bone_physics.push_back(UberModel::BonePhysics());
		uber_models[0]->bone_physics[0].shape = ShapeFromVTNModel(vtn);

		for(unsigned int i = 0; i < uber_models.size(); i++)
		{
			if(i == 0)
				bs = uber_models[i]->GetBoundingSphere();
			else
				bs = Sphere::Expand(bs, uber_models[i]->GetBoundingSphere());
		}
	}

	void Bramble::InnerDispose()
	{
		delete root;

		for(unsigned int i = 0; i < uber_models.size(); i++)
			delete uber_models[i];
		uber_models.clear();
	}

	void Bramble::Vis(SceneRenderer* renderer)
	{
		if(renderer->camera->CheckSphereVisibility(bs))
			for(unsigned int i = 0; i < uber_models.size(); i++)
				((TestGame*)game_state)->VisUberModel(renderer, uber_models[i], 0, Mat4::Identity(), NULL, &materials);
	}

	void Bramble::Spawned()
	{
	}

	void Bramble::DeSpawned()
	{
	}

	bool Bramble::GetShot(Shot* shot, Vec3 poi, Vec3 momentum)
	{
		return true;
	}




	/*
	 * BrambleNode methods
	 */
	BrambleNode::BrambleNode(Vec3 parent_pos, Vec3 up, Quaternion ori, float scale, float parent_vcoord, vector<VTNModel*> node_models, VTNModel* collision_model) :
		ori(ori),
		scale(scale),
		parent(NULL),
		children(),
		node_models(node_models),
		collision_model(collision_model)
	{
		float length = Random3D::Rand(0.9, 1.1);

		v_coord = parent_vcoord + length * 0.5;
		pos = parent_pos + up * length;

		draw_xform = Mat4::FromPositionAndOrientation(pos, ori) * Mat4::Scale(scale, length, scale);

		split_ready = 0;

		model = node_models[Random3D::RandInt(node_models.size())];
	}

	BrambleNode::~BrambleNode()
	{
		for(unsigned int i = 0; i < children.size(); i++)
			delete children[i];
	}

	bool BrambleNode::Grow()
	{
		if (children.size() == 0)
		{
			const double arching_factor = 0.3;
			const double narrowing_factor = 0.95;
			const double random_factor = 0.2;
			const double branch_angle = 0.5;

			//if (scale > 0.3)
			if (scale > 0.35)
			{
				split_ready += Random3D::Rand();

				// maybe split
				if (split_ready > 2.0)
				{
					int num_splits = 4;
					for (int i = 0; i < num_splits; i++)
					{
						float theta = i * M_PI * 2.0 / num_splits;

						Quaternion nu_ori = Quaternion::FromPYR(cos(theta) * branch_angle, 0, sin(theta) * branch_angle) * ori;

						Vec3 up = nu_ori.ToMat3().Transpose() * Vec3(0, 1, 0);

						BrambleNode* child = new BrambleNode(pos, up, nu_ori, scale * narrowing_factor, v_coord, node_models, collision_model);
						children.push_back(child);
					}
				}
				else
				{
					Vec3 up = ori.ToMat3().Transpose() * Vec3(0, 1, 0);
					Vec3 cross = Vec3::Cross(up, Vec3(0, 1, 0));

					Vec3 rot = Random3D::RandomNormalizedVector(Random3D::Rand(random_factor)) + cross * arching_factor;

					BrambleNode* child = new BrambleNode(pos, up, Quaternion::FromPYR(rot) * ori, scale * narrowing_factor, v_coord, node_models, collision_model);
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
			for (vector<BrambleNode*>::iterator iter = children.begin(); iter != children.end(); iter++)
			{
				BrambleNode* child = *iter;

				child->parent = this;
				any |= child->Grow();
			}

			return any;
		}
	}

	void BrambleNode::AppendVertexData(UberModel::LOD* lod)
	{
		if (parent != NULL)
		{
			VUVNTTCVertexBuffer* my_vbo = model->GetVBO();
			for (unsigned int i = 0; i < my_vbo->vertex_infos.size(); i += 3)
			{
				VUVNTTC a = TransformVertexInfo(my_vbo->vertex_infos[i + 0].x, my_vbo->vertex_infos[i + 0].n, my_vbo->vertex_infos[i + 0].uvw);
				VUVNTTC b = TransformVertexInfo(my_vbo->vertex_infos[i + 1].x, my_vbo->vertex_infos[i + 1].n, my_vbo->vertex_infos[i + 1].uvw);
				VUVNTTC c = TransformVertexInfo(my_vbo->vertex_infos[i + 2].x, my_vbo->vertex_infos[i + 2].n, my_vbo->vertex_infos[i + 2].uvw);

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

	void BrambleNode::AppendCollisionData(VUVNTTCVertexBuffer* collision_verts, VTNModel* collision_shape)
	{
		if (parent != NULL)
		{
			VUVNTTCVertexBuffer* my_vbo = collision_shape->GetVBO();
			for (unsigned int i = 0; i < my_vbo->vertex_infos.size(); i += 3)
			{
				VUVNTTC a = TransformVertexInfo(my_vbo->vertex_infos[i + 0].x, my_vbo->vertex_infos[i + 0].n, my_vbo->vertex_infos[i + 0].uvw);
				VUVNTTC b = TransformVertexInfo(my_vbo->vertex_infos[i + 1].x, my_vbo->vertex_infos[i + 1].n, my_vbo->vertex_infos[i + 1].uvw);
				VUVNTTC c = TransformVertexInfo(my_vbo->vertex_infos[i + 2].x, my_vbo->vertex_infos[i + 2].n, my_vbo->vertex_infos[i + 2].uvw);

				collision_verts->AddTriangleVertexInfo(a, b, c);
			}
		}

		for(unsigned int i = 0; i < children.size(); i++)
			children[i]->AppendCollisionData(collision_verts, collision_shape);
	}

	VUVNTTC BrambleNode::TransformVertexInfo(Vec3 x, Vec3 n, Vec3 uvw)
	{
		float y = min(1.0f, max(0.0f, x.y));				// clamp lerp factor (idk, do we need this?)
		float un_y = 1.0 - y;								// opposite fraction of lerp

		// making them actually 'bend' would be quite difficult :(
		Mat4 lerped = parent->draw_xform * un_y + draw_xform * y;

		x = lerped.TransformVec3(x.x, 0, x.z, 1);
		n = Vec3::Normalize(lerped.TransformVec3(n, 0));

		return VUVNTTC(x, uvw, n);
	}

	void BrambleNode::GetUberModels(vector<UberModel*>& uber_models)
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
			use_model->lods.push_back(new UberModel::LOD());
			use_model->lods[0]->lod_name = "LOD 0";
			use_model->materials.push_back("wood");
		}

		AppendVertexData(use_model->lods[0]);

		for(unsigned int i = 0; i < children.size(); i++)
			children[i]->GetUberModels(uber_models);
	}
}
