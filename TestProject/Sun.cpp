#include "StdAfx.h"
#include "Sun.h"

#include "Shot.h"

namespace Test
{
	Sun::Sun(Vec3 position, Vec3 color, VertexBuffer* model, Texture2D* texture) : position(position), color(color), view_matrix(Mat4::Identity()), model(model), texture(texture)
	{
		distance = position.ComputeMagnitude();
		rm = Util::FindOrientationZEdge(position);
	}

	void Sun::SetLight(int which)
	{
		float ambient[] = {color.x, color.y, color.z, 1};
		float diffuse[] = {color.x, color.y, color.z, 1};
		float specular[] = {color.x, color.y, color.z, 1};
		Vec3 current_dir = view_matrix.TransformVec3(position, 0);
		float pos_f[] = { current_dir.x, current_dir.y, current_dir.z, 0 };

		glEnable(GL_LIGHTING);

		int name = GL_LIGHT0 + which;
		glEnable(name);
		glLightfv(name, GL_AMBIENT, ambient);
		glLightfv(name, GL_DIFFUSE, diffuse);
		glLightfv(name, GL_SPECULAR, specular);
		glLightf(name, GL_LINEAR_ATTENUATION, 0);
		glLightf(name, GL_QUADRATIC_ATTENUATION, 0);

		glPushMatrix();
		glLoadIdentity();
		glLightfv(name, GL_POSITION, pos_f);
		glPopMatrix();
	}

	void Sun::UnsetLight(int which) { glDisable(GL_LIGHT0 + which); }

	void Sun::Draw()
	{
		if(texture != NULL && model != NULL)
		{
			glMatrixMode(GL_MODELVIEW);

			glPushMatrix();
			glLoadIdentity();

			Mat4 m = Mat4::FromMat3(rm) * view_matrix.Transpose();
			Vec3 camera_pos = view_matrix.TransformVec3(0, 0, 0, 1);
			glTranslatef(-camera_pos.x, -camera_pos.y, -camera_pos.z);
			glMultMatrixf(&m.values[0]);
			glTranslatef(0, 0, distance);

			glDisable(GL_DEPTH_TEST);

			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, texture->GetGLName());

			glEnable(GL_BLEND);
			glBlendFunc(GL_ONE, GL_ONE);

			glDisable(GL_LIGHTING);
			glColor4f(1, 1, 1, 1);

			model->Draw();

			glPopMatrix();
		}
	}

	Mat4 Sun::GetShadowMatrix(Vec3 camera_position)
	{
		float shadow_radius = 20.0f;
		float light_distance = 256.0f;

		Mat4 light_translation = Mat4::Translation(0, 0, -light_distance);
		Mat4 scale = Mat4::Scale(1.0f / shadow_radius, 1.0f / shadow_radius, 1.0f);
		Mat4 rotate = Mat4::FromMat3(rm);
		Mat4 eye_translation = Mat4::Translation(-camera_position);

		return light_translation * scale * rotate * eye_translation;
	}
}
