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
		Vec3 current_dir = view_matrix.TransformVec3_0(position);
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
			Vec3 camera_pos = view_matrix.TransformVec3_1(0, 0, 0);
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

	/** Generates the view matrix for shadow-mapping purposes */
	Mat4 Sun::GenerateShadowMatrix(CameraView& camera)
	{
		Vec3 light_dir = Vec3::Normalize(position);

		// compute this once and remember it
		Mat4 camera_matrix = camera.GetProjectionMatrix() * camera.GetViewMatrix();
		Mat4 inv_camera_matrix = Mat4::Invert(camera_matrix);

		// we'll be doing a bunch of transforms on these same verts
		Vec3 verts[] =
		{
			Vec3(	0.0f,	0.0f,	0.0f),				// centers of near and far planes
			Vec3(	0.0f,	0.0f,	1.0f)
		};

		int n = 2;										// number of verts in the above array; if you change that, change this!

		// transform screen-space coords to world-space, and flatten onto plane
		for(int i = 0; i < n; ++i)
		{
			verts[i] = inv_camera_matrix.TransformVec3(verts[i], 1.0f);
			verts[i] -= light_dir * Vec3::Dot(verts[i], light_dir);
		}
		Vec3 forward = verts[1] - verts[0];

		Vec3 forward_f = Vec3::Normalize(forward);								// flatten that onto plane perpendicular to light direction
		Vec3 right_f = Vec3::Normalize(Vec3::Cross(light_dir, forward_f));		// the other vector in that plane

		Mat3 shadow_rm(
			-right_f.x,		-right_f.y,		-right_f.z,
			forward_f.x,	forward_f.y,	forward_f.z,
			light_dir.x,	light_dir.y,	light_dir.z
		);
		Mat4 rotation(Mat4::FromMat3(shadow_rm));

		float shadow_w = 50.0f;
		float shadow_l = 50.0f;
		Vec3 light_translation_vec(camera.GetPosition());

		Mat4 scale_mat(Mat4::Scale(1.0f / shadow_w, 1.0f / shadow_l, 1.0f));
		Mat4 light_translation(Mat4::Translation(-light_translation_vec));

		return Mat4::Translation(0, 0, -256.0f) * scale_mat * rotation * light_translation;		
	}
}
