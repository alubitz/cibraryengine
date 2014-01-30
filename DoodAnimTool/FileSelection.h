#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace std;
	using namespace CibraryEngine;

	struct FileSelection
	{
		static bool ShowSavePoseDialog(ProgramWindow* window, string& chosen_file_path);
		static bool ShowLoadPoseDialog(ProgramWindow* window, string& chosen_file_path);
	};
}
