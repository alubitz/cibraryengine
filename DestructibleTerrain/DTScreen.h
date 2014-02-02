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

			void Activate();
			void Deactivate();

			void Draw(int width, int height);
			ProgramScreen* Update(const TimingInfo& time);	
	};
}
