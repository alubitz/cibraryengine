#include "StdAfx.h"

#include "MainMenu.h"
#include "LoadingScreen.h"

#include "../CibraryEngine/NeuralNet.h"

using namespace std;

using namespace CibraryEngine;
using namespace Test;

float fofx(float x)
{
	return 2.0f * x * x - 0.25f;
}

void DoNNStuff()
{
	srand((unsigned int)time(NULL));

	unsigned int layer_sizes[] = { 2, 6, 6, 6, 4, 1 };
	MultiLayerPerceptron* nn = new MultiLayerPerceptron(6, layer_sizes);

	for(unsigned int i = 0; i < nn->neural_nets.size(); ++i)
		nn->neural_nets[i]->Randomize();

	const float learning_rate = 0.01f;
	const float min_x = -0.75f, max_x = 0.75f;

	for(int i = 0; i < 2500; ++i)
	{
		const int per_round = 100;
		for(int j = 0; j < per_round; ++j)
		{
			float x = Random3D::Rand(min_x, max_x);
			float y = fofx(x);

			float inputs[] = { x, 1.0f };

			nn->Train(inputs, &y, learning_rate);
		}

		float error = 0;
		for(int j = 0; j < per_round; ++j)
		{
			float x = j * (max_x - min_x) / (per_round - 1) + min_x;
			float y = fofx(x);
			float z;

			float inputs[] = { x, 1.0f };

			nn->Process(inputs, &z);

			float dif = z - y;
			error += dif * dif;
		}

		Debug(((stringstream&)(stringstream() << "iteration " << (i + 1) * per_round << "; error = " << sqrtf(error / per_round) << endl)).str());
	}

	delete nn;
}

int main(int argc, char** argv)
{
	ScriptSystem::Init();

	InitEndianness();

	// DoNNStuff();

	// Network::DoTestProgram();

	ProgramWindow* win = ProgramWindow::CreateProgramWindow("C++ Game Engine", 0, 0, 0, false);
	if(win == NULL)
		return 1;

	ScreenshotGrabber grabber(win);
	win->input_state->KeyStateChanged += &grabber;

	ProgramScreen* first_screen = new MainMenu(win, NULL);

	int result = win->Run(first_screen);

	win->input_state->KeyStateChanged -= &grabber;

	ScriptSystem::Shutdown();

	delete first_screen;
	delete win;

	return result;
}

#ifdef WIN32
int WINAPI WinMain(	HINSTANCE	hInstance,		// Instance
					HINSTANCE	hPrevInstance,	// Previous Instance
					LPSTR		lpCmdLine,		// Command Line Parameters
					int			nCmdShow)		// Window Show State
{
	return main(nCmdShow, &lpCmdLine);
}
#endif
