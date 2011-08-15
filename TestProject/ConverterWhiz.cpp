#include "StdAfx.h"
#include "ConverterWhiz.h"

namespace Test
{
	btCollisionShape* ShapeFromVertexBuffer(VertexBuffer* vbo)
	{
		btTriangleMesh* mesh = new btTriangleMesh();
		mesh->m_weldingThreshold = 0.05f;
		for(unsigned int i = 0; i < vbo->GetNumVerts();)
		{
			VTNTT vinfo_a = GetVTNTT(vbo, i++);
			VTNTT vinfo_b = GetVTNTT(vbo, i++);
			VTNTT vinfo_c = GetVTNTT(vbo, i++);

			btVector3 a = btVector3(vinfo_a.x.x, vinfo_a.x.y, vinfo_a.x.z);
			btVector3 b = btVector3(vinfo_b.x.x, vinfo_b.x.y, vinfo_b.x.z);
			btVector3 c = btVector3(vinfo_c.x.x, vinfo_c.x.y, vinfo_c.x.z);

			mesh->addTriangle(a, b, c, true);
		}
		return new btBvhTriangleMeshShape(mesh, false);
	}

	btCollisionShape* ShapeFromSkinnedModel(SkinnedModel* model)
	{
		btTriangleMesh* mesh = new btTriangleMesh();
		mesh->m_weldingThreshold = 0.05f;
		for(vector<MaterialModelPair>::iterator iter = model->material_model_pairs.begin(); iter != model->material_model_pairs.end(); iter++)
		{
			VertexBuffer* vbo = iter->vbo;

			for(unsigned int i = 0; i < vbo->GetNumVerts();)
			{
				SkinVInfo vinfo_a = GetSkinVInfo(vbo, i++);
				SkinVInfo vinfo_b = GetSkinVInfo(vbo, i++);
				SkinVInfo vinfo_c = GetSkinVInfo(vbo, i++);

				btVector3 a = btVector3(vinfo_a.x.x, vinfo_a.x.y, vinfo_a.x.z);
				btVector3 b = btVector3(vinfo_b.x.x, vinfo_b.x.y, vinfo_b.x.z);
				btVector3 c = btVector3(vinfo_c.x.x, vinfo_c.x.y, vinfo_c.x.z);

				mesh->addTriangle(a, b, c, true);
			}
		}
		return new btBvhTriangleMeshShape(mesh, false);
	}

	btCollisionShape* HullFromVertexBuffer(VertexBuffer* vbo)
	{
		
		btConvexHullShape* shape = new btConvexHullShape();

		for(unsigned int i = 0; i < vbo->GetNumVerts();)
		{
			VTNTT vinfo_a = GetVTNTT(vbo, i++);
			VTNTT vinfo_b = GetVTNTT(vbo, i++);
			VTNTT vinfo_c = GetVTNTT(vbo, i++);

			shape->addPoint(btVector3(vinfo_a.x.x, vinfo_a.x.y, vinfo_a.x.z));
			shape->addPoint(btVector3(vinfo_b.x.x, vinfo_b.x.y, vinfo_b.x.z));
			shape->addPoint(btVector3(vinfo_c.x.x, vinfo_c.x.y, vinfo_c.x.z));
		}
		return shape;
	}
}
