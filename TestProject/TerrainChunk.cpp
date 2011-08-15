#include "StdAfx.h"
#include "TerrainChunk.h"

#include "DSNMaterial.h"
#include "TestGame.h"
#include "Particle.h"

namespace Test
{
	/*
	 * TerrainChunk methods
	 */
	TerrainChunk::TerrainChunk(GameState* gs, Heightfield* heightfield, VertexBuffer* model, DSNMaterial* material) :
		Entity(gs),
		material(material),
		node_data(NULL),
		heightfield(heightfield),
		model(model),
		mesh(NULL),
		rigid_body(NULL),
		physics(NULL)
	{ }

	void TerrainChunk::InnerDispose()
	{
		VisCleanup();

		Entity::InnerDispose();

		rigid_body->Dispose();		// will delete the collision shape for us
		delete rigid_body;

		delete mesh;
	}

	void TerrainChunk::Vis(SceneRenderer* renderer)
	{
		VisCleanup();				// just in case

		Sphere bs = model->GetBoundingSphere();
		bs.center += heightfield->origin;
		if(renderer->camera->CheckSphereVisibility(bs))
		{
			node_data = new DSNMaterialNodeData(model, Mat4::Translation(heightfield->origin), bs);
			renderer->objects.push_back(RenderNode(material, node_data, Vec3::Dot(renderer->camera->GetPosition(), bs.center)));
		}
	}

	void TerrainChunk::VisCleanup()
	{
		if(node_data != NULL)
		{
			delete node_data;
			node_data = NULL;
		}
	}

	void TerrainChunk::Spawned()
	{
		physics = game_state->physics_world;

		mesh = new btTriangleMesh();
		mesh->m_weldingThreshold = 0.05f;
		for (unsigned int i = 0; i < model->vertex_infos.size(); )
		{
			SkinVInfo vinfo_a = model->vertex_infos[i++];
			SkinVInfo vinfo_b = model->vertex_infos[i++];
			SkinVInfo vinfo_c = model->vertex_infos[i++];

			btVector3 a = btVector3(vinfo_a.x.x + heightfield->origin.x, vinfo_a.x.y + heightfield->origin.y, vinfo_a.x.z + heightfield->origin.z);
			btVector3 b = btVector3(vinfo_b.x.x + heightfield->origin.x, vinfo_b.x.y + heightfield->origin.y, vinfo_b.x.z + heightfield->origin.z);
			btVector3 c = btVector3(vinfo_c.x.x + heightfield->origin.x, vinfo_c.x.y + heightfield->origin.y, vinfo_c.x.z + heightfield->origin.z);

			mesh->addTriangle(a, b, c, true);
		}
		btCollisionShape* shape = new btBvhTriangleMeshShape(mesh, false);

		RigidBodyInfo* rigid_body = new RigidBodyInfo(shape, MassInfo(), Vec3());
		rigid_body->body->setUserPointer(this);

		rigid_body->body->setFriction(1.0f);

		physics->AddRigidBody(rigid_body);
		this->rigid_body = rigid_body;
	}

	void TerrainChunk::DeSpawned()
	{
		physics->RemoveRigidBody(rigid_body);
	}

	bool TerrainChunk::GetShot(Shot* shot, Vec3 poi, Vec3 momentum)
	{
		ParticleMaterial* dirt_particle = ((TestGame*)game_state)->dirt_particle;
		for (int i = 0; i < 6; i++)
		{
			Particle* p = new Particle(game_state, poi, Random3D::RandomNormalizedVector(5), dirt_particle, 0.05f, 1);
			p->gravity = 9.8f;
			p->damp = 2.0f;
			p->angle = -M_PI * 0.5f;

			game_state->Spawn(p);
		}

		return true;
	}
}

