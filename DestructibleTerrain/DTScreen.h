#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace std;
	using namespace CibraryEngine;

	class DTScreen : public ProgramScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			DTScreen(ProgramWindow* win);
			~DTScreen();

			void Draw(int width, int height);
			ProgramScreen* Update(TimingInfo time);	
	};
}
