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

		VertexData() : floats(NULL) { }
		VertexData(float* floats) : floats(floats) { }
	};

	/** Stores a collection of named vertex attribute data */
	struct VertexBuffer : public Disposable
	{
		protected:

			boost::unordered_map<string, VertexAttribute> attributes;
			boost::unordered_map<string, VertexData> attribute_data;

			DrawMode storage_mode;

			unsigned int num_verts;
			unsigned int allocated_size;

			unsigned int vbo_id;

			void InnerDispose();			// for Disposable

		public:

			/** Constructs a VertexBuffer */
			VertexBuffer(DrawMode storage_mode);

			/** Returns the DrawMode for which this VertexBuffer's vertex data is stored */
			DrawMode GetStorageMode();

			unsigned int GetNumVerts();
			void SetNumVerts(unsigned int num_verts);

			void SetAllocatedSize(unsigned int n);

			void AddAttribute(const string& name, VertexAttributeType type, int n_per_vertex);
			void RemoveAttribute(const string& name);
			VertexAttribute GetAttribute(const string& name);

			vector<VertexAttribute> GetAttributes();
			int GetVertexSize();

			/** Returns a pointer to the float data for the given attribute name, if applicable; if non-applicable, returns NULL */
			float* GetFloatPointer(const string& name);

			void InvalidateVBO();
			void BuildVBO();
			unsigned int GetVBO();

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

			/** Draws this vertex buffer, and sends the output to the specified feedback buffer */
			void DrawToFeedbackBuffer(VertexBuffer* target, ShaderProgram* shader_program, bool keep_fragments);
	};
}
