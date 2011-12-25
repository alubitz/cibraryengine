#include "StdAfx.h"
#include "UniformVariables.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * UniformInt methods
	 */
	UniformInt::UniformInt(string name) : TypedUniformVariable<int>(name), value(0) { }

	void UniformInt::ApplyValue(int location) { glUniform1i(location, value); }

	int* UniformInt::GetValue() { return &value; }
	void UniformInt::SetValue(int* v) { value = *v; }




	/*
	 * UniformFloat methods
	 */
	UniformFloat::UniformFloat(string name) : TypedUniformVariable<float>(name), value(0.0f) { }

	void UniformFloat::ApplyValue(int location) { glUniform1f(location, value); }

	float* UniformFloat::GetValue() { return &value; }
	void UniformFloat::SetValue(float* v) { value = *v; }




	/*
	 * UniformVector3 methods
	 */
	UniformVector3::UniformVector3(string name) : TypedUniformVariable<Vec3>(name), value() { }

	void UniformVector3::ApplyValue(int location) { glUniform3f(location, value.x, value.y, value.z); }

	Vec3* UniformVector3::GetValue() { return &value; }
	void UniformVector3::SetValue(Vec3* v) { value = *v; }




	/*
	 * UniformTexture1D methods
	 */
	UniformTexture1D::UniformTexture1D(string name, int which) : TypedUniformVariable<Texture1D>(name), which(which), texture(NULL) { }

	void UniformTexture1D::ApplyValue(int location)
	{
		if(texture != NULL)
		{
			GLDEBUG();
			glActiveTexture(GL_TEXTURE0 + which);
			GLDEBUG();
			glEnable(GL_TEXTURE_1D);
			GLDEBUG();
			glBindTexture(GL_TEXTURE_1D, texture->GetGLName());
			GLDEBUG();
			glUniform1i(location, which);
			GLDEBUG();
		}
	}

	Texture1D* UniformTexture1D::GetValue() { return texture; }
	void UniformTexture1D::SetValue(Texture1D* t) { texture = t; }

	void UniformTexture1D::Disable()
	{
		glActiveTexture(GL_TEXTURE0 + which);
		glDisable(GL_TEXTURE_1D);
		glActiveTexture(GL_TEXTURE0);
	}




	/*
	 * UniformTexture2D methods
	 */
	UniformTexture2D::UniformTexture2D(string name, int which) : TypedUniformVariable<Texture2D>(name), which(which), texture(NULL) { }

	void UniformTexture2D::ApplyValue(int location)
	{
		if(texture != NULL)
		{
			glActiveTexture(GL_TEXTURE0 + which);
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, texture == NULL ? 0 : texture->GetGLName());
			glUniform1i(location, which);
		}
	}

	Texture2D* UniformTexture2D::GetValue() { return texture; }
	void UniformTexture2D::SetValue(Texture2D* t) { texture = t; }

	void UniformTexture2D::Disable()
	{
		glActiveTexture(GL_TEXTURE0 + which);
		glDisable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
	}




	/*
	 * UniformTexture3D methods
	 */
	UniformTexture3D::UniformTexture3D(string name, int which) : TypedUniformVariable<Texture3D>(name), which(which), texture(NULL) { }

	void UniformTexture3D::ApplyValue(int location)
	{
		if(texture != NULL)
		{
			glActiveTexture(GL_TEXTURE0 + which);
			glEnable(GL_TEXTURE_3D);
			glBindTexture(GL_TEXTURE_3D, texture == NULL ? 0 : texture->GetGLName());
			glUniform1i(location, which);
		}
	}

	Texture3D* UniformTexture3D::GetValue() { return texture; }
	void UniformTexture3D::SetValue(Texture3D* t) { texture = t; }

	void UniformTexture3D::Disable()
	{
		glActiveTexture(GL_TEXTURE0 + which);
		glDisable(GL_TEXTURE_3D);
		glActiveTexture(GL_TEXTURE0);
	}




	/*
	 * UniformTextureCube methods
	 */
	UniformTextureCube::UniformTextureCube(string name, int which) : TypedUniformVariable<TextureCube>(name), which(which), cubemap(NULL) { }

	void UniformTextureCube::ApplyValue(int location)
	{
		if(cubemap != NULL)
		{
			GLDEBUG();
			glActiveTexture(GL_TEXTURE0 + which);
			GLDEBUG();
			glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap == NULL ? 0 : cubemap->GetGLName());
			GLDEBUG();
			glUniform1i(location, which);
			GLDEBUG();
		}
	}

	TextureCube* UniformTextureCube::GetValue() { return cubemap; }
	void UniformTextureCube::SetValue(TextureCube* c) { cubemap = c; }

	void UniformTextureCube::Disable()
	{
		glActiveTexture(GL_TEXTURE0 + which);
		glDisable(GL_TEXTURE_CUBE_MAP);
		glActiveTexture(GL_TEXTURE0);
	}


	/*
	 * UniformMatrix4 methods
	 */
	UniformMatrix4::UniformMatrix4(string name, bool transpose) : TypedUniformVariable<Mat4>(name), transpose(transpose), matrix() { }

	void UniformMatrix4::ApplyValue(int location)
	{
		glUniformMatrix4fv(location, 1, transpose, matrix.values);
	}

	Mat4* UniformMatrix4::GetValue() { return &matrix; }
	void UniformMatrix4::SetValue(Mat4* mat) { matrix = *mat; }




	/*
	 * UniformMatrix4Array methods
	 */
	UniformMatrix4Array::UniformMatrix4Array(string name, bool transpose) : UniformArray<Mat4>(name), transpose(transpose) { }

	void UniformMatrix4Array::ApplyValue(int location)
	{
		unsigned int array_size = array->size();
		float* matrices = new float[array_size * 16];

		int target_index = 0;
		for(unsigned int i = 0; i < array_size; ++i)
			for(unsigned int j = 0; j < 16; ++j)
			{
				float value = (*array)[i].values[j];
				matrices[target_index++] = value;
			}

		glUniformMatrix4fv(location, array_size, transpose, matrices);

		delete[] matrices;
	}
}
