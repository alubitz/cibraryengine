#include "StdAfx.h"
#include "FileSelection.h"

#include <CommDlg.h>

namespace DoodAnimTool
{
	/*
	 * FileSelection methods; code for these is based on example here http://win32-framework.sourceforge.net/Tutorials/tutorial8.htm (slightly modified)
	 */
	bool FileSelection::ShowSavePoseDialog(ProgramWindow* window, string& chosen_file_path)
	{
		// Fill the OPENFILENAME structure
		char file_path_name[_MAX_PATH] = "";							// This will hold the file name

		OPENFILENAME ofn = {0};
		ofn.lStructSize = sizeof(OPENFILENAME);
		ofn.hwndOwner = NULL;//m_hWnd;
		ofn.lpstrFilter = "Pose Files (*.pose)\0*.pose\0\0";
		ofn.lpstrFile = file_path_name;
		ofn.lpstrDefExt = "pose";
		ofn.nMaxFile = _MAX_PATH;
		//ofn.lpstrInitialDir = "./Files/";
		ofn.lpstrTitle = "Save File";
		ofn.Flags = OFN_OVERWRITEPROMPT;

		window->SetActive(false);
		bool result = GetSaveFileName(&ofn) != 0;
		window->SetActive(true);

		if(result)
		{
			chosen_file_path = file_path_name;
			return true;
		}
		else
			return false;
	}

	bool FileSelection::ShowLoadPoseDialog(ProgramWindow* window, string& chosen_file_path)
	{
		// Fill the OPENFILENAME structure
		char file_path_name[_MAX_PATH] = "";							// This will hold the file name

		OPENFILENAME ofn = {0};
		ofn.lStructSize = sizeof(OPENFILENAME);
		ofn.hwndOwner = NULL;//m_hWnd;
		ofn.lpstrFilter = "Pose Files (*.pose)\0*.pose\0\0";
		ofn.lpstrFile = file_path_name;
		ofn.nMaxFile = _MAX_PATH;
		ofn.lpstrTitle = "Open File";
		ofn.Flags = OFN_FILEMUSTEXIST;

		window->SetActive(false);
		bool result = GetOpenFileName(&ofn) != 0;
		window->SetActive(true);

		if(result)
		{
			chosen_file_path = file_path_name;
			return true;
		}
		else
			return false;
	}
}
