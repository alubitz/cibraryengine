// thanks to "shawn" on #gamedev IRC for this, although ....
// LRN 2 #include-GUARD
// also, was missing glClientActiveTexture...

#pragma once

#ifdef WIN32
	#include <windows.h>
	#include <gl/GL.h>
	#include <gl/glext.h>
	#define GL2ENTRY	APIENTRY

	void GetProcFailureMessage(const char* name);		// this is mine, not shawn's

	//---------------------------------------------------------------------------
	template <typename ProcType> bool GL2GetProc(ProcType& proc, const char* name)
	{
		proc = (ProcType)wglGetProcAddress( name );
		if(!proc)
		{
			GetProcFailureMessage(name);
			return false;
		}
		return true;
	}
	#define GET_GL_PROC( name )		GL2GetProc( name, #name )
	//---------------------------------------------------------------------------
	template <typename ProcType> bool GL2ClearProc(ProcType& proc)
	{
		proc = NULL;
		return true;
	}
	#define CLEAR_GL_PROC( name )		GL2ClearProc( name )

#else // !defined( WIN32 )
#error define GL2ENTRY for this platform
#endif

#ifndef GL2LOCAL // only defined in GL2API.cpp
#define GL2DECL extern
#else
#define GL2DECL
#endif

//===========================================================================
// Error checking
//===========================================================================
extern bool DbgCheckGL( GLenum err, const char* file, int line );
#ifdef _DEBUG
# define CHECK_GL()		DbgCheckGL( glGetError(), __FILE__, __LINE__ )
#else
# define CHECK_GL()
#endif
#define VERIFY_GL()		DbgCheckGL( glGetError(), __FILE__, __LINE__ )

//===========================================================================
// Vertex Management
//===========================================================================
GL2DECL void (GL2ENTRY *glGenQueries) (GLsizei n, GLuint *ids);
GL2DECL void (GL2ENTRY *glDeleteQueries) (GLsizei n, const GLuint *ids);
GL2DECL GLboolean (GL2ENTRY *glIsQuery) (GLuint id);
GL2DECL void (GL2ENTRY *glBeginQuery) (GLenum target, GLuint id);
GL2DECL void (GL2ENTRY *glEndQuery) (GLenum target);
GL2DECL void (GL2ENTRY *glGetQueryiv) (GLenum target, GLenum pname, GLint *params);
GL2DECL void (GL2ENTRY *glGetQueryObjectiv) (GLuint id, GLenum pname, GLint *params);
GL2DECL void (GL2ENTRY *glGetQueryObjectuiv) (GLuint id, GLenum pname, GLuint *params);
GL2DECL void (GL2ENTRY *glBindBuffer) (GLenum target, GLuint buffer);
GL2DECL void (GL2ENTRY *glDeleteBuffers) (GLsizei n, const GLuint *buffers);
GL2DECL void (GL2ENTRY *glGenBuffers) (GLsizei n, GLuint *buffers);
GL2DECL GLboolean (GL2ENTRY *glIsBuffer) (GLuint buffer);
GL2DECL void (GL2ENTRY *glBufferData) (GLenum target, GLsizeiptr size, const GLvoid *data, GLenum usage);
GL2DECL void (GL2ENTRY *glBufferSubData) (GLenum target, GLintptr offset, GLsizeiptr size, const GLvoid *data);
GL2DECL void (GL2ENTRY *glGetBufferSubData) (GLenum target, GLintptr offset, GLsizeiptr size, GLvoid *data);
GL2DECL GLvoid* (GL2ENTRY *glMapBuffer) (GLenum target, GLenum access);
GL2DECL GLboolean (GL2ENTRY *glUnmapBuffer) (GLenum target);
GL2DECL void (GL2ENTRY *glGetBufferParameteriv) (GLenum target, GLenum pname, GLint *params);
GL2DECL void (GL2ENTRY *glGetBufferPointerv) (GLenum target, GLenum pname, GLvoid* *params);
//---------------------------------------------------------------------------
GL2DECL void (GL2ENTRY *glBlendColor) (GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);
GL2DECL void (GL2ENTRY *glBlendEquation) (GLenum mode);
GL2DECL void (GL2ENTRY *glDrawRangeElements) (GLenum mode, GLuint start, GLuint end, GLsizei count, GLenum type, const GLvoid *indices);
GL2DECL void (GL2ENTRY *glTexImage3D) (GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, const GLvoid *pixels);
GL2DECL void (GL2ENTRY *glTexSubImage3D) (GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLenum type, const GLvoid *pixels);
GL2DECL void (GL2ENTRY *glCopyTexSubImage3D) (GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLint x, GLint y, GLsizei width, GLsizei height);

inline bool GL2StartupBufferAPI()
{
	return 
		GET_GL_PROC(glGenQueries) &&
		GET_GL_PROC(glDeleteQueries) &&
		GET_GL_PROC(glIsQuery) &&
		GET_GL_PROC(glBeginQuery) &&
		GET_GL_PROC(glEndQuery) &&
		GET_GL_PROC(glGetQueryiv) &&
		GET_GL_PROC(glGetQueryObjectiv) &&
		GET_GL_PROC(glGetQueryObjectuiv) &&
		GET_GL_PROC(glBindBuffer) &&
		GET_GL_PROC(glDeleteBuffers) &&
		GET_GL_PROC(glGenBuffers) &&
		GET_GL_PROC(glIsBuffer) &&
		GET_GL_PROC(glBufferData) &&
		GET_GL_PROC(glBufferSubData) &&
		GET_GL_PROC(glGetBufferSubData) &&
		GET_GL_PROC(glMapBuffer) &&
		GET_GL_PROC(glUnmapBuffer) &&
		GET_GL_PROC(glGetBufferParameteriv) &&
		GET_GL_PROC(glGetBufferPointerv) &&
		//---------------------------------------------------------------------------
		GET_GL_PROC(glBlendColor) &&
		GET_GL_PROC(glBlendEquation) &&
		GET_GL_PROC(glDrawRangeElements) &&
		GET_GL_PROC(glTexImage3D) &&
		GET_GL_PROC(glTexSubImage3D) &&
		GET_GL_PROC(glCopyTexSubImage3D);

}
inline bool GL2ShutdownBufferAPI()
{
	return 
		CLEAR_GL_PROC(glGenQueries) &&
		CLEAR_GL_PROC(glDeleteQueries) &&
		CLEAR_GL_PROC(glIsQuery) &&
		CLEAR_GL_PROC(glBeginQuery) &&
		CLEAR_GL_PROC(glEndQuery) &&
		CLEAR_GL_PROC(glGetQueryiv) &&
		CLEAR_GL_PROC(glGetQueryObjectiv) &&
		CLEAR_GL_PROC(glGetQueryObjectuiv) &&
		CLEAR_GL_PROC(glBindBuffer) &&
		CLEAR_GL_PROC(glDeleteBuffers) &&
		CLEAR_GL_PROC(glGenBuffers) &&
		CLEAR_GL_PROC(glIsBuffer) &&
		CLEAR_GL_PROC(glBufferData) &&
		CLEAR_GL_PROC(glBufferSubData) &&
		CLEAR_GL_PROC(glGetBufferSubData) &&
		CLEAR_GL_PROC(glMapBuffer) &&
		CLEAR_GL_PROC(glUnmapBuffer) &&
		CLEAR_GL_PROC(glGetBufferParameteriv) &&
		CLEAR_GL_PROC(glGetBufferPointerv) &&
		//---------------------------------------------------------------------------
		CLEAR_GL_PROC(glBlendColor) &&
		CLEAR_GL_PROC(glBlendEquation) &&
		CLEAR_GL_PROC(glDrawRangeElements) &&
		CLEAR_GL_PROC(glTexImage3D) &&
		CLEAR_GL_PROC(glTexSubImage3D) &&
		CLEAR_GL_PROC(glCopyTexSubImage3D);
}

//===========================================================================
// Texture Management
//===========================================================================
GL2DECL void (GL2ENTRY *glActiveTexture) (GLenum texture);
GL2DECL void (GL2ENTRY *glClientActiveTexture) (GLenum texture);
GL2DECL void (GL2ENTRY *glSampleCoverage) (GLclampf value, GLboolean invert);
GL2DECL void (GL2ENTRY *glCompressedTexImage3D) (GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLsizei imageSize, const GLvoid *data);
GL2DECL void (GL2ENTRY *glCompressedTexImage2D) (GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLint border, GLsizei imageSize, const GLvoid *data);
GL2DECL void (GL2ENTRY *glCompressedTexImage1D) (GLenum target, GLint level, GLenum internalformat, GLsizei width, GLint border, GLsizei imageSize, const GLvoid *data);
GL2DECL void (GL2ENTRY *glCompressedTexSubImage3D) (GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLsizei imageSize, const GLvoid *data);
GL2DECL void (GL2ENTRY *glCompressedTexSubImage2D) (GLenum target, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum format, GLsizei imageSize, const GLvoid *data);
GL2DECL void (GL2ENTRY *glCompressedTexSubImage1D) (GLenum target, GLint level, GLint xoffset, GLsizei width, GLenum format, GLsizei imageSize, const GLvoid *data);
GL2DECL void (GL2ENTRY *glGetCompressedTexImage) (GLenum target, GLint level, GLvoid *img);

inline bool GL2StartupTextureAPI()
{
	return 
		GET_GL_PROC(glActiveTexture) &&
		GET_GL_PROC(glClientActiveTexture) &&
		GET_GL_PROC(glSampleCoverage) &&
		GET_GL_PROC(glCompressedTexImage3D) &&
		GET_GL_PROC(glCompressedTexImage2D) &&
		GET_GL_PROC(glCompressedTexImage1D) &&
		GET_GL_PROC(glCompressedTexSubImage3D) &&
		GET_GL_PROC(glCompressedTexSubImage2D) &&
		GET_GL_PROC(glCompressedTexSubImage1D) &&
		GET_GL_PROC(glGetCompressedTexImage);
}
inline bool GL2ShutdownTextureAPI()
{
	return 
		CLEAR_GL_PROC(glActiveTexture) &&
		CLEAR_GL_PROC(glClientActiveTexture) &&
		CLEAR_GL_PROC(glSampleCoverage) &&
		CLEAR_GL_PROC(glCompressedTexImage3D) &&
		CLEAR_GL_PROC(glCompressedTexImage2D) &&
		CLEAR_GL_PROC(glCompressedTexImage1D) &&
		CLEAR_GL_PROC(glCompressedTexSubImage3D) &&
		CLEAR_GL_PROC(glCompressedTexSubImage2D) &&
		CLEAR_GL_PROC(glCompressedTexSubImage1D) &&
		CLEAR_GL_PROC(glGetCompressedTexImage);
}
//===========================================================================
// Shader Management
//===========================================================================
GL2DECL void (GL2ENTRY *glAttachShader) (GLuint program, GLuint shader);
GL2DECL void (GL2ENTRY *glBindAttribLocation) (GLuint program, GLuint index, const GLchar *name);
GL2DECL void (GL2ENTRY *glCompileShader) (GLuint shader);
GL2DECL GLuint (GL2ENTRY *glCreateProgram) (void);
GL2DECL GLuint (GL2ENTRY *glCreateShader) (GLenum type);
GL2DECL void (GL2ENTRY *glDeleteProgram) (GLuint program);
GL2DECL void (GL2ENTRY *glDeleteShader) (GLuint shader);
GL2DECL void (GL2ENTRY *glDetachShader) (GLuint program, GLuint shader);
GL2DECL void (GL2ENTRY *glDisableVertexAttribArray) (GLuint index);
GL2DECL void (GL2ENTRY *glEnableVertexAttribArray) (GLuint index);
GL2DECL void (GL2ENTRY *glGetActiveAttrib) (GLuint program, GLuint index, GLsizei bufSize, GLsizei *length, GLint *size, GLenum *type, GLchar *name);
GL2DECL void (GL2ENTRY *glGetActiveUniform) (GLuint program, GLuint index, GLsizei bufSize, GLsizei *length, GLint *size, GLenum *type, GLchar *name);
GL2DECL void (GL2ENTRY *glGetAttachedShaders) (GLuint program, GLsizei maxCount, GLsizei *count, GLuint *obj);
GL2DECL GLint (GL2ENTRY *glGetAttribLocation) (GLuint program, const GLchar *name);
GL2DECL void (GL2ENTRY *glGetProgramiv) (GLuint program, GLenum pname, GLint *params);
GL2DECL void (GL2ENTRY *glGetProgramInfoLog) (GLuint program, GLsizei bufSize, GLsizei *length, GLchar *infoLog);
GL2DECL void (GL2ENTRY *glGetShaderiv) (GLuint shader, GLenum pname, GLint *params);
GL2DECL void (GL2ENTRY *glGetShaderInfoLog) (GLuint shader, GLsizei bufSize, GLsizei *length, GLchar *infoLog);
GL2DECL void (GL2ENTRY *glGetShaderSource) (GLuint shader, GLsizei bufSize, GLsizei *length, GLchar *source);
GL2DECL GLint (GL2ENTRY *glGetUniformLocation) (GLuint program, const GLchar *name);
GL2DECL void (GL2ENTRY *glGetUniformfv) (GLuint program, GLint location, GLfloat *params);
GL2DECL void (GL2ENTRY *glGetUniformiv) (GLuint program, GLint location, GLint *params);
GL2DECL void (GL2ENTRY *glGetVertexAttribdv) (GLuint index, GLenum pname, GLdouble *params);
GL2DECL void (GL2ENTRY *glGetVertexAttribfv) (GLuint index, GLenum pname, GLfloat *params);
GL2DECL void (GL2ENTRY *glGetVertexAttribiv) (GLuint index, GLenum pname, GLint *params);
GL2DECL void (GL2ENTRY *glGetVertexAttribPointerv) (GLuint index, GLenum pname, GLvoid* *pointer);
GL2DECL GLboolean (GL2ENTRY *glIsProgram) (GLuint program);
GL2DECL GLboolean (GL2ENTRY *glIsShader) (GLuint shader);
GL2DECL void (GL2ENTRY *glLinkProgram) (GLuint program);
GL2DECL void (GL2ENTRY *glShaderSource) (GLuint shader, GLsizei count, const GLchar* *string, const GLint *length);
GL2DECL void (GL2ENTRY *glUseProgram) (GLuint program);
GL2DECL void (GL2ENTRY *glUniform1f) (GLint location, GLfloat v0);
GL2DECL void (GL2ENTRY *glUniform2f) (GLint location, GLfloat v0, GLfloat v1);
GL2DECL void (GL2ENTRY *glUniform3f) (GLint location, GLfloat v0, GLfloat v1, GLfloat v2);
GL2DECL void (GL2ENTRY *glUniform4f) (GLint location, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3);
GL2DECL void (GL2ENTRY *glUniform1i) (GLint location, GLint v0);
GL2DECL void (GL2ENTRY *glUniform2i) (GLint location, GLint v0, GLint v1);
GL2DECL void (GL2ENTRY *glUniform3i) (GLint location, GLint v0, GLint v1, GLint v2);
GL2DECL void (GL2ENTRY *glUniform4i) (GLint location, GLint v0, GLint v1, GLint v2, GLint v3);
GL2DECL void (GL2ENTRY *glUniform1fv) (GLint location, GLsizei count, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniform2fv) (GLint location, GLsizei count, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniform3fv) (GLint location, GLsizei count, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniform4fv) (GLint location, GLsizei count, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniform1iv) (GLint location, GLsizei count, const GLint *value);
GL2DECL void (GL2ENTRY *glUniform2iv) (GLint location, GLsizei count, const GLint *value);
GL2DECL void (GL2ENTRY *glUniform3iv) (GLint location, GLsizei count, const GLint *value);
GL2DECL void (GL2ENTRY *glUniform4iv) (GLint location, GLsizei count, const GLint *value);
GL2DECL void (GL2ENTRY *glUniformMatrix2fv) (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniformMatrix3fv) (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniformMatrix4fv) (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
GL2DECL void (GL2ENTRY *glValidateProgram) (GLuint program);
GL2DECL void (GL2ENTRY *glVertexAttrib1d) (GLuint index, GLdouble x);
GL2DECL void (GL2ENTRY *glVertexAttrib1dv) (GLuint index, const GLdouble *v);
GL2DECL void (GL2ENTRY *glVertexAttrib1f) (GLuint index, GLfloat x);
GL2DECL void (GL2ENTRY *glVertexAttrib1fv) (GLuint index, const GLfloat *v);
GL2DECL void (GL2ENTRY *glVertexAttrib1s) (GLuint index, GLshort x);
GL2DECL void (GL2ENTRY *glVertexAttrib1sv) (GLuint index, const GLshort *v);
GL2DECL void (GL2ENTRY *glVertexAttrib2d) (GLuint index, GLdouble x, GLdouble y);
GL2DECL void (GL2ENTRY *glVertexAttrib2dv) (GLuint index, const GLdouble *v);
GL2DECL void (GL2ENTRY *glVertexAttrib2f) (GLuint index, GLfloat x, GLfloat y);
GL2DECL void (GL2ENTRY *glVertexAttrib2fv) (GLuint index, const GLfloat *v);
GL2DECL void (GL2ENTRY *glVertexAttrib2s) (GLuint index, GLshort x, GLshort y);
GL2DECL void (GL2ENTRY *glVertexAttrib2sv) (GLuint index, const GLshort *v);
GL2DECL void (GL2ENTRY *glVertexAttrib3d) (GLuint index, GLdouble x, GLdouble y, GLdouble z);
GL2DECL void (GL2ENTRY *glVertexAttrib3dv) (GLuint index, const GLdouble *v);
GL2DECL void (GL2ENTRY *glVertexAttrib3f) (GLuint index, GLfloat x, GLfloat y, GLfloat z);
GL2DECL void (GL2ENTRY *glVertexAttrib3fv) (GLuint index, const GLfloat *v);
GL2DECL void (GL2ENTRY *glVertexAttrib3s) (GLuint index, GLshort x, GLshort y, GLshort z);
GL2DECL void (GL2ENTRY *glVertexAttrib3sv) (GLuint index, const GLshort *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4Nbv) (GLuint index, const GLbyte *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4Niv) (GLuint index, const GLint *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4Nsv) (GLuint index, const GLshort *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4Nub) (GLuint index, GLubyte x, GLubyte y, GLubyte z, GLubyte w);
GL2DECL void (GL2ENTRY *glVertexAttrib4Nubv) (GLuint index, const GLubyte *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4Nuiv) (GLuint index, const GLuint *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4Nusv) (GLuint index, const GLushort *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4bv) (GLuint index, const GLbyte *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4d) (GLuint index, GLdouble x, GLdouble y, GLdouble z, GLdouble w);
GL2DECL void (GL2ENTRY *glVertexAttrib4dv) (GLuint index, const GLdouble *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4f) (GLuint index, GLfloat x, GLfloat y, GLfloat z, GLfloat w);
GL2DECL void (GL2ENTRY *glVertexAttrib4fv) (GLuint index, const GLfloat *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4iv) (GLuint index, const GLint *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4s) (GLuint index, GLshort x, GLshort y, GLshort z, GLshort w);
GL2DECL void (GL2ENTRY *glVertexAttrib4sv) (GLuint index, const GLshort *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4ubv) (GLuint index, const GLubyte *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4uiv) (GLuint index, const GLuint *v);
GL2DECL void (GL2ENTRY *glVertexAttrib4usv) (GLuint index, const GLushort *v);
GL2DECL void (GL2ENTRY *glVertexAttribPointer) (GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid *pointer);
//---------------------------------------------------------------------------
GL2DECL void (GL2ENTRY *glUniformMatrix2x3fv) (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniformMatrix3x2fv) (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniformMatrix2x4fv) (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniformMatrix4x2fv) (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniformMatrix3x4fv) (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
GL2DECL void (GL2ENTRY *glUniformMatrix4x3fv) (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value);
inline bool
GL2StartupShaderAPI()
{
	return 
		GET_GL_PROC(glAttachShader) &&
		GET_GL_PROC(glBindAttribLocation) &&
		GET_GL_PROC(glCompileShader) &&
		GET_GL_PROC(glCreateProgram) &&
		GET_GL_PROC(glCreateShader) &&
		GET_GL_PROC(glDeleteProgram) &&
		GET_GL_PROC(glDeleteShader) &&
		GET_GL_PROC(glDetachShader) &&
		GET_GL_PROC(glDisableVertexAttribArray) &&
		GET_GL_PROC(glEnableVertexAttribArray) &&
		GET_GL_PROC(glGetActiveAttrib) &&
		GET_GL_PROC(glGetActiveUniform) &&
		GET_GL_PROC(glGetAttachedShaders) &&
		GET_GL_PROC(glGetAttribLocation) &&
		GET_GL_PROC(glGetProgramiv) &&
		GET_GL_PROC(glGetProgramInfoLog) &&
		GET_GL_PROC(glGetShaderiv) &&
		GET_GL_PROC(glGetShaderInfoLog) &&
		GET_GL_PROC(glGetShaderSource) &&
		GET_GL_PROC(glGetUniformLocation) &&
		GET_GL_PROC(glGetUniformfv) &&
		GET_GL_PROC(glGetUniformiv) &&
		GET_GL_PROC(glGetVertexAttribdv) &&
		GET_GL_PROC(glGetVertexAttribfv) &&
		GET_GL_PROC(glGetVertexAttribiv) &&
		GET_GL_PROC(glGetVertexAttribPointerv) &&
		GET_GL_PROC(glIsProgram) &&
		GET_GL_PROC(glIsShader) &&
		GET_GL_PROC(glLinkProgram) &&
		GET_GL_PROC(glShaderSource) &&
		GET_GL_PROC(glUseProgram) &&
		GET_GL_PROC(glUniform1f) &&
		GET_GL_PROC(glUniform2f) &&
		GET_GL_PROC(glUniform3f) &&
		GET_GL_PROC(glUniform4f) &&
		GET_GL_PROC(glUniform1i) &&
		GET_GL_PROC(glUniform2i) &&
		GET_GL_PROC(glUniform3i) &&
		GET_GL_PROC(glUniform4i) &&
		GET_GL_PROC(glUniform1fv) &&
		GET_GL_PROC(glUniform2fv) &&
		GET_GL_PROC(glUniform3fv) &&
		GET_GL_PROC(glUniform4fv) &&
		GET_GL_PROC(glUniform1iv) &&
		GET_GL_PROC(glUniform2iv) &&
		GET_GL_PROC(glUniform3iv) &&
		GET_GL_PROC(glUniform4iv) &&
		GET_GL_PROC(glUniformMatrix2fv) &&
		GET_GL_PROC(glUniformMatrix3fv) &&
		GET_GL_PROC(glUniformMatrix4fv) &&
		GET_GL_PROC(glValidateProgram) &&
		GET_GL_PROC(glVertexAttrib1d) &&
		GET_GL_PROC(glVertexAttrib1dv) &&
		GET_GL_PROC(glVertexAttrib1f) &&
		GET_GL_PROC(glVertexAttrib1fv) &&
		GET_GL_PROC(glVertexAttrib1s) &&
		GET_GL_PROC(glVertexAttrib1sv) &&
		GET_GL_PROC(glVertexAttrib2d) &&
		GET_GL_PROC(glVertexAttrib2dv) &&
		GET_GL_PROC(glVertexAttrib2f) &&
		GET_GL_PROC(glVertexAttrib2fv) &&
		GET_GL_PROC(glVertexAttrib2s) &&
		GET_GL_PROC(glVertexAttrib2sv) &&
		GET_GL_PROC(glVertexAttrib3d) &&
		GET_GL_PROC(glVertexAttrib3dv) &&
		GET_GL_PROC(glVertexAttrib3f) &&
		GET_GL_PROC(glVertexAttrib3fv) &&
		GET_GL_PROC(glVertexAttrib3s) &&
		GET_GL_PROC(glVertexAttrib3sv) &&
		GET_GL_PROC(glVertexAttrib4Nbv) &&
		GET_GL_PROC(glVertexAttrib4Niv) &&
		GET_GL_PROC(glVertexAttrib4Nsv) &&
		GET_GL_PROC(glVertexAttrib4Nub) &&
		GET_GL_PROC(glVertexAttrib4Nubv) &&
		GET_GL_PROC(glVertexAttrib4Nuiv) &&
		GET_GL_PROC(glVertexAttrib4Nusv) &&
		GET_GL_PROC(glVertexAttrib4bv) &&
		GET_GL_PROC(glVertexAttrib4d) &&
		GET_GL_PROC(glVertexAttrib4dv) &&
		GET_GL_PROC(glVertexAttrib4f) &&
		GET_GL_PROC(glVertexAttrib4fv) &&
		GET_GL_PROC(glVertexAttrib4iv) &&
		GET_GL_PROC(glVertexAttrib4s) &&
		GET_GL_PROC(glVertexAttrib4sv) &&
		GET_GL_PROC(glVertexAttrib4ubv) &&
		GET_GL_PROC(glVertexAttrib4uiv) &&
		GET_GL_PROC(glVertexAttrib4usv) &&
		GET_GL_PROC(glVertexAttribPointer) &&
		//---------------------------------------------------------------------------
		GET_GL_PROC(glUniformMatrix2x3fv) &&
		GET_GL_PROC(glUniformMatrix3x2fv) &&
		GET_GL_PROC(glUniformMatrix2x4fv) &&
		GET_GL_PROC(glUniformMatrix4x2fv) &&
		GET_GL_PROC(glUniformMatrix3x4fv) &&
		GET_GL_PROC(glUniformMatrix4x3fv);
}
inline bool
GL2ShutdownShaderAPI()
{
	return 
		CLEAR_GL_PROC(glAttachShader) &&
		CLEAR_GL_PROC(glBindAttribLocation) &&
		CLEAR_GL_PROC(glCompileShader) &&
		CLEAR_GL_PROC(glCreateProgram) &&
		CLEAR_GL_PROC(glCreateShader) &&
		CLEAR_GL_PROC(glDeleteProgram) &&
		CLEAR_GL_PROC(glDeleteShader) &&
		CLEAR_GL_PROC(glDetachShader) &&
		CLEAR_GL_PROC(glDisableVertexAttribArray) &&
		CLEAR_GL_PROC(glEnableVertexAttribArray) &&
		CLEAR_GL_PROC(glGetActiveAttrib) &&
		CLEAR_GL_PROC(glGetActiveUniform) &&
		CLEAR_GL_PROC(glGetAttachedShaders) &&
		CLEAR_GL_PROC(glGetAttribLocation) &&
		CLEAR_GL_PROC(glGetProgramiv) &&
		CLEAR_GL_PROC(glGetProgramInfoLog) &&
		CLEAR_GL_PROC(glGetShaderiv) &&
		CLEAR_GL_PROC(glGetShaderInfoLog) &&
		CLEAR_GL_PROC(glGetShaderSource) &&
		CLEAR_GL_PROC(glGetUniformLocation) &&
		CLEAR_GL_PROC(glGetUniformfv) &&
		CLEAR_GL_PROC(glGetUniformiv) &&
		CLEAR_GL_PROC(glGetVertexAttribdv) &&
		CLEAR_GL_PROC(glGetVertexAttribfv) &&
		CLEAR_GL_PROC(glGetVertexAttribiv) &&
		CLEAR_GL_PROC(glGetVertexAttribPointerv) &&
		CLEAR_GL_PROC(glIsProgram) &&
		CLEAR_GL_PROC(glIsShader) &&
		CLEAR_GL_PROC(glLinkProgram) &&
		CLEAR_GL_PROC(glShaderSource) &&
		CLEAR_GL_PROC(glUseProgram) &&
		CLEAR_GL_PROC(glUniform1f) &&
		CLEAR_GL_PROC(glUniform2f) &&
		CLEAR_GL_PROC(glUniform3f) &&
		CLEAR_GL_PROC(glUniform4f) &&
		CLEAR_GL_PROC(glUniform1i) &&
		CLEAR_GL_PROC(glUniform2i) &&
		CLEAR_GL_PROC(glUniform3i) &&
		CLEAR_GL_PROC(glUniform4i) &&
		CLEAR_GL_PROC(glUniform1fv) &&
		CLEAR_GL_PROC(glUniform2fv) &&
		CLEAR_GL_PROC(glUniform3fv) &&
		CLEAR_GL_PROC(glUniform4fv) &&
		CLEAR_GL_PROC(glUniform1iv) &&
		CLEAR_GL_PROC(glUniform2iv) &&
		CLEAR_GL_PROC(glUniform3iv) &&
		CLEAR_GL_PROC(glUniform4iv) &&
		CLEAR_GL_PROC(glUniformMatrix2fv) &&
		CLEAR_GL_PROC(glUniformMatrix3fv) &&
		CLEAR_GL_PROC(glUniformMatrix4fv) &&
		CLEAR_GL_PROC(glValidateProgram) &&
		CLEAR_GL_PROC(glVertexAttrib1d) &&
		CLEAR_GL_PROC(glVertexAttrib1dv) &&
		CLEAR_GL_PROC(glVertexAttrib1f) &&
		CLEAR_GL_PROC(glVertexAttrib1fv) &&
		CLEAR_GL_PROC(glVertexAttrib1s) &&
		CLEAR_GL_PROC(glVertexAttrib1sv) &&
		CLEAR_GL_PROC(glVertexAttrib2d) &&
		CLEAR_GL_PROC(glVertexAttrib2dv) &&
		CLEAR_GL_PROC(glVertexAttrib2f) &&
		CLEAR_GL_PROC(glVertexAttrib2fv) &&
		CLEAR_GL_PROC(glVertexAttrib2s) &&
		CLEAR_GL_PROC(glVertexAttrib2sv) &&
		CLEAR_GL_PROC(glVertexAttrib3d) &&
		CLEAR_GL_PROC(glVertexAttrib3dv) &&
		CLEAR_GL_PROC(glVertexAttrib3f) &&
		CLEAR_GL_PROC(glVertexAttrib3fv) &&
		CLEAR_GL_PROC(glVertexAttrib3s) &&
		CLEAR_GL_PROC(glVertexAttrib3sv) &&
		CLEAR_GL_PROC(glVertexAttrib4Nbv) &&
		CLEAR_GL_PROC(glVertexAttrib4Niv) &&
		CLEAR_GL_PROC(glVertexAttrib4Nsv) &&
		CLEAR_GL_PROC(glVertexAttrib4Nub) &&
		CLEAR_GL_PROC(glVertexAttrib4Nubv) &&
		CLEAR_GL_PROC(glVertexAttrib4Nuiv) &&
		CLEAR_GL_PROC(glVertexAttrib4Nusv) &&
		CLEAR_GL_PROC(glVertexAttrib4bv) &&
		CLEAR_GL_PROC(glVertexAttrib4d) &&
		CLEAR_GL_PROC(glVertexAttrib4dv) &&
		CLEAR_GL_PROC(glVertexAttrib4f) &&
		CLEAR_GL_PROC(glVertexAttrib4fv) &&
		CLEAR_GL_PROC(glVertexAttrib4iv) &&
		CLEAR_GL_PROC(glVertexAttrib4s) &&
		CLEAR_GL_PROC(glVertexAttrib4sv) &&
		CLEAR_GL_PROC(glVertexAttrib4ubv) &&
		CLEAR_GL_PROC(glVertexAttrib4uiv) &&
		CLEAR_GL_PROC(glVertexAttrib4usv) &&
		CLEAR_GL_PROC(glVertexAttribPointer) &&
		//---------------------------------------------------------------------------
		CLEAR_GL_PROC(glUniformMatrix2x3fv) &&
		CLEAR_GL_PROC(glUniformMatrix3x2fv) &&
		CLEAR_GL_PROC(glUniformMatrix2x4fv) &&
		CLEAR_GL_PROC(glUniformMatrix4x2fv) &&
		CLEAR_GL_PROC(glUniformMatrix3x4fv) &&
		CLEAR_GL_PROC(glUniformMatrix4x3fv);
}

//===========================================================================
// Framebuffer Management
//	(GL_ARB_framebuffer_object)
//===========================================================================
GL2DECL GLboolean (GL2ENTRY *glIsRenderbuffer) (GLuint renderbuffer);
GL2DECL void (GL2ENTRY *glBindRenderbuffer) (GLenum target, GLuint renderbuffer);
GL2DECL void (GL2ENTRY *glDeleteRenderbuffers) (GLsizei n, const GLuint *renderbuffers);
GL2DECL void (GL2ENTRY *glGenRenderbuffers) (GLsizei n, GLuint *renderbuffers);
GL2DECL void (GL2ENTRY *glRenderbufferStorage) (GLenum target, GLenum internalformat, GLsizei width, GLsizei height);
GL2DECL void (GL2ENTRY *glGetRenderbufferParameteriv) (GLenum target, GLenum pname, GLint *params);
GL2DECL GLboolean (GL2ENTRY *glIsFramebuffer) (GLuint framebuffer);
GL2DECL void (GL2ENTRY *glBindFramebuffer) (GLenum target, GLuint framebuffer);
GL2DECL void (GL2ENTRY *glDeleteFramebuffers) (GLsizei n, const GLuint *framebuffers);
GL2DECL void (GL2ENTRY *glGenFramebuffers) (GLsizei n, GLuint *framebuffers);
GL2DECL GLenum (GL2ENTRY *glCheckFramebufferStatus) (GLenum target);
GL2DECL void (GL2ENTRY *glFramebufferTexture1D) (GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level);
GL2DECL void (GL2ENTRY *glFramebufferTexture2D) (GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level);
GL2DECL void (GL2ENTRY *glFramebufferTexture3D) (GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level, GLint zoffset);
GL2DECL void (GL2ENTRY *glFramebufferRenderbuffer) (GLenum target, GLenum attachment, GLenum renderbuffertarget, GLuint renderbuffer);
GL2DECL void (GL2ENTRY *glGetFramebufferAttachmentParameteriv) (GLenum target, GLenum attachment, GLenum pname, GLint *params);
GL2DECL void (GL2ENTRY *glGenerateMipmap) (GLenum target);
GL2DECL void (GL2ENTRY *glBlitFramebuffer) (GLint srcX0, GLint srcY0, GLint srcX1, GLint srcY1, GLint dstX0, GLint dstY0, GLint dstX1, GLint dstY1, GLbitfield mask, GLenum filter);
GL2DECL void (GL2ENTRY *glRenderbufferStorageMultisample) (GLenum target, GLsizei samples, GLenum internalformat, GLsizei width, GLsizei height);
GL2DECL void (GL2ENTRY *glFramebufferTextureLayer) (GLenum target, GLenum attachment, GLuint texture, GLint level, GLint layer);
inline bool
GL2StartupFramebufferAPI()
{
	return 
		GET_GL_PROC(glIsRenderbuffer) &&
		GET_GL_PROC(glBindRenderbuffer) &&
		GET_GL_PROC(glDeleteRenderbuffers) &&
		GET_GL_PROC(glGenRenderbuffers) &&
		GET_GL_PROC(glRenderbufferStorage) &&
		GET_GL_PROC(glGetRenderbufferParameteriv) &&
		GET_GL_PROC(glIsFramebuffer) &&
		GET_GL_PROC(glBindFramebuffer) &&
		GET_GL_PROC(glDeleteFramebuffers) &&
		GET_GL_PROC(glGenFramebuffers) &&
		GET_GL_PROC(glCheckFramebufferStatus) &&
		GET_GL_PROC(glFramebufferTexture1D) &&
		GET_GL_PROC(glFramebufferTexture2D) &&
		GET_GL_PROC(glFramebufferTexture3D) &&
		GET_GL_PROC(glFramebufferRenderbuffer) &&
		GET_GL_PROC(glGetFramebufferAttachmentParameteriv) &&
		GET_GL_PROC(glGenerateMipmap) &&
		GET_GL_PROC(glBlitFramebuffer) &&
		GET_GL_PROC(glRenderbufferStorageMultisample) &&
		GET_GL_PROC(glFramebufferTextureLayer);
}
inline bool
GL2ShutdownFramebufferAPI()
{
	return 
		CLEAR_GL_PROC(glIsRenderbuffer) &&
		CLEAR_GL_PROC(glBindRenderbuffer) &&
		CLEAR_GL_PROC(glDeleteRenderbuffers) &&
		CLEAR_GL_PROC(glGenRenderbuffers) &&
		CLEAR_GL_PROC(glRenderbufferStorage) &&
		CLEAR_GL_PROC(glGetRenderbufferParameteriv) &&
		CLEAR_GL_PROC(glIsFramebuffer) &&
		CLEAR_GL_PROC(glBindFramebuffer) &&
		CLEAR_GL_PROC(glDeleteFramebuffers) &&
		CLEAR_GL_PROC(glGenFramebuffers) &&
		CLEAR_GL_PROC(glCheckFramebufferStatus) &&
		CLEAR_GL_PROC(glFramebufferTexture1D) &&
		CLEAR_GL_PROC(glFramebufferTexture2D) &&
		CLEAR_GL_PROC(glFramebufferTexture3D) &&
		CLEAR_GL_PROC(glFramebufferRenderbuffer) &&
		CLEAR_GL_PROC(glGetFramebufferAttachmentParameteriv) &&
		CLEAR_GL_PROC(glGenerateMipmap) &&
		CLEAR_GL_PROC(glBlitFramebuffer) &&
		CLEAR_GL_PROC(glRenderbufferStorageMultisample) &&
		CLEAR_GL_PROC(glFramebufferTextureLayer);
}
