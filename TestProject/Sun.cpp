#include "Sun.h"

namespace Test
{
	Sun::Sun(Vec3 position, Vec3 color) : position(position), color(color), view_matrix(Mat4::Identity()) { }

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
}
