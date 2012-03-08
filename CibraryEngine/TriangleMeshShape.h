#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	struct Ray;
	struct Intersection;

	class TriangleMeshShape : public CollisionShape
	{
		private:

			bool built;

			Vec3* A;
			Vec3* B;
			Vec3* C;
			Vec3* AB;
			Vec3* AC;
			Plane* planes;
			Vec3* P;
			Vec3* Q;
			float* UOffset;
			float* VOffset;

			void InitCache();
			void DeleteCache();

			void BuildCache();

		protected:

			void InnerDispose();

		public:

			vector<Vec3> vertices;

			struct Tri { unsigned int indices[3]; };
			vector<Tri> triangles;

			TriangleMeshShape();
			TriangleMeshShape(VertexBuffer* vbo);

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);

			vector<Intersection> RayTest(const Ray& ray);


			void Write(ostream& stream);
			unsigned int Read(istream& stream);
	};
}
