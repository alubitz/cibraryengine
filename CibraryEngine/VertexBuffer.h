#pragma once

#include "StdAfx.h"
#include "Disposable.h"
#include "MathTypes.h"

namespace CibraryEngine
{
	using namespace std;

	class ShaderProgram;

	enum VertexAttributeType
	{
		BadVertexAttribute	= 0,

		Float				= GL_FLOAT,
		Int					= GL_INT
	};

	struct VertexAttribute
	{
		string name;
		VertexAttributeType type;
		int n_per_vertex;

		VertexAttribute() : name(), type(BadVertexAttribute), n_per_vertex(0) { }
		VertexAttribute(string name, VertexAttributeType type, int n_per_vertex) : name(name), type(type), n_per_vertex(n_per_vertex) { }
	};

	/** Modes for drawing primitives */
	enum DrawMode
	{
		Points			= GL_POINTS,
		LineStrip		= GL_LINE_STRIP,
		LineLoop		= GL_LINE_LOOP,
		Lines			= GL_LINES,
		TriangleStrip	= GL_TRIANGLE_STRIP,
		TriangleFan		= GL_TRIANGLE_FAN,
		Triangles		= GL_TRIANGLES,
	};

	union VertexData
	{
		float* floats;
		int* ints;

		VertexData() : floats(NULL) { }
		VertexData(float* floats) : floats(floats) { }
		VertexData(int* ints) : ints(ints) { }
	};

	/** Stores a collection of named vertex attribute data */
	struct VertexBuffer : public Disposable
	{
		protected:

			map<string, VertexAttribute> attributes;
			map<string, VertexData> attribute_data;

			DrawMode storage_mode;

			unsigned int num_verts;
			unsigned int allocated_size;

			unsigned int vbo_id;

			void InnerDispose();			// for Disposable

			/** Enabling vertex attributes, and other setup like that */
			void PreDrawEnable();
			/** Disabling vertex attributes, and other cleanup like that */
			void PostDrawDisable();

		public:

			/** Constructs a VertexBuffer */
			VertexBuffer(DrawMode storage_mode = Points);

			/** Returns the DrawMode for which this VertexBuffer's vertex data is stored */
			DrawMode GetStorageMode();

			unsigned int GetNumVerts();
			void SetNumVerts(unsigned int num_verts);

			void SetAllocatedSize(unsigned int n);

			void AddAttribute(const string& name, VertexAttributeType type, int n_per_vertex);
			void RemoveAttribute(const string& name);

			VertexAttributeType GetAttribType(const string& name);
			int GetAttribNPerVertex(const string& name);

			vector<string> GetAttributes();
			int GetVertexSize();

			/** Returns a pointer to the float data for the given attribute name, if applicable; if non-applicable, returns NULL */
			float* GetFloatPointer(const string& name);

			/** Returns a pointer to the int data for the given attribute name, if applicable; if non-applicable, returns NULL */
			int* GetIntPointer(const string& name);

			void InvalidateVBO();
			void BuildVBO();
			unsigned int GetVBO();

			/** Causes the attribute array data to be loaded from the gl VBO */
			void UpdateDataFromGL();

			/**
			 * Draws this vertex buffer.
			 *
			 * @param num_instances How many instances of the vertex buffer should be drawn
			 * @param mode The mode to draw primitives using; if not specified, primitives are drawn using storage_mode
			 * @param indices Pointer to an array of indices to be drawn
			 * @param num_indices Size of the indices array
			 */
			void Draw();
			void Draw(DrawMode mode);
			void Draw(DrawMode mode, unsigned int* indices, int num_indices);
			void Draw(unsigned int num_instances);
			void Draw(unsigned int num_instances, DrawMode mode);
			void Draw(unsigned int num_instances, DrawMode mode, unsigned int* indices, int num_indices);

			/** Creates an empty vertex buffer with the same storage mode and attributes as the one passed as argument */
			static VertexBuffer* CreateEmptyCopyAttributes(VertexBuffer* existing);
	};
}
