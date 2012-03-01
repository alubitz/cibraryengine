#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	class TriangleMeshShape : public CollisionShape
	{
		public:

			vector<Vec3> vertices;

			struct Tri { unsigned int indices[3]; };
			vector<Tri> triangles;

			TriangleMeshShape();
			TriangleMeshShape(VertexBuffer* vbo);

			void Write(ostream& stream);
			unsigned int Read(istream& stream);

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);
	};
}
