#include "StdAfx.h"
#include "ProgramWindow.h"
#include "ProgramScreen.h"

#include "DebugLog.h"

#include <winsock2.h>

namespace CibraryEngine
{
	/*
	 * Private implementation for class ProgramWindow
	 */
	struct ProgramWindow::Imp
	{
		ProgramWindow* window;

		InputState* input_state;

		ProgramScreen* current_screen;
		bool finished;

		bool active;
		int width, height;

		float total_game_time;

		bool fullscreen;
		HGLRC gl_context;
		HDC device_context;
		HWND window_handle;
		HINSTANCE program_instance;

		Imp(bool fullscreen, HGLRC gl_context, HDC device_context, HWND window_handle, HINSTANCE program_instance) :
			window(NULL),
			input_state(NULL),
			current_screen(NULL),
			finished(false),
			active(false),
			fullscreen(fullscreen),
			gl_context(gl_context),
			device_context(device_context),
			window_handle(window_handle),
			program_instance(program_instance)
		{
		}

		~Imp() { }

		void AttachWindow(ProgramWindow* win)
		{
			window = win;
			input_state = win->input_state;

			SetActive(true);
		}

		void SetActive(bool a)
		{
			if(active != a)
			{
				ShowCursor(!a);
				active = a;

				int x, y;
				if(GetCursorClientPos(x, y))
				{
					input_state->mouse_rect_valid = false;
					input_state->SetMousePosition(x, y);
				}
			}
		}

		bool GetCursorClientPos(int& x, int& y)
		{
			POINT pos;
			GetCursorPos(&pos);
			ScreenToClient(window_handle, &pos);

			x = pos.x;
			y = pos.y;

			return pos.x >= 0 && pos.y >= 0 && pos.x < width && pos.y < height;
		}

		LRESULT CALLBACK WndProc(HWND window_handle, UINT message, WPARAM wParam, LPARAM lParam)
		{
			switch(message)
			{
				case WM_KEYDOWN:
				case WM_SYSKEYDOWN:
				{
					input_state->SetKeyState(wParam, true);
					return 0;
				}
				case WM_KEYUP:
				case WM_SYSKEYUP:
				{
					input_state->SetKeyState(wParam, false);
					return 0;
				}
				case WM_MOUSEMOVE:
				{
					if(active)
					{
						int x, y;
						if(GetCursorClientPos(x, y))
							input_state->SetMousePosition(x, y);
					}

					return 0;
				}
				case WM_LBUTTONDOWN:
				{
					if(!active)
					{
						int x, y;
						if(GetCursorClientPos(x, y))
							SetActive(true);
					}

					input_state->SetMouseButtonState(0, true);
					return 0;
				}
				case WM_LBUTTONUP:
				{
					input_state->SetMouseButtonState(0, false);
					return 0;
				}
				case WM_MBUTTONDOWN:
				{
					input_state->SetMouseButtonState(1, true);
					return 0;
				}
				case WM_MBUTTONUP:
				{
					input_state->SetMouseButtonState(1, false);
					return 0;
				}
				case WM_RBUTTONDOWN:
				{
					input_state->SetMouseButtonState(2, true);
					return 0;
				}
				case WM_RBUTTONUP:
				{
					input_state->SetMouseButtonState(2, false);
					return 0;
				}
				case WM_SIZE:
				{
					UpdateDimensions();
					return 0;
				}
				case WM_ACTIVATE:
				{
					if(LOWORD(lParam) == WA_INACTIVE)
						SetActive(false);
					return 0;
				}
				case WM_SYSCOMMAND:
				{
					if(wParam == SC_CLOSE)
					{
						WindowClosingEvent evt(window);
						window->OnWindowClosing(&evt);
						if(!evt.cancel)
							PostQuitMessage(0);
						return 0;
					}
					else
						break;
				}
			}

			return DefWindowProc(window_handle, message, wParam, lParam);
		}

		void UpdateDimensions()
		{
			RECT rect;
			GetClientRect(window_handle, &rect);

			input_state->mouse_rect_w = width	= rect.right - rect.left;
			input_state->mouse_rect_h = height	= rect.bottom - rect.top;
			input_state->mouse_rect_valid = false;
		}

		int Run(ProgramScreen* initial_screen)
		{
			if(initial_screen == NULL)
				return 1;						// lolwut

			Init();

			current_screen = initial_screen;
			current_screen->Activate();

			while(!finished)
			{
				if(CheckInput())
				{
					Update();

					Draw();

					SwapBuffers(device_context);
				}
			}

			return 0;
		}

		void Init()
		{
			SetActive(true);

			srand((unsigned int)time(NULL));
			total_game_time = -1.0f;

			UpdateDimensions();

			glShadeModel(GL_SMOOTH);

			glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
		}

		bool CheckInput()
		{
			MSG msg;
			while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
			{
				if(msg.message == WM_QUIT)
					finished = true;
				else
				{
					TranslateMessage(&msg);
					DispatchMessage(&msg);
				}
			}

			if(input_state->reset_mouse)
			{
				POINT pos;
				pos.x = width / 2;
				pos.y = height / 2;
				ClientToScreen(window_handle, &pos);

				SetCursorPos(pos.x, pos.y);

				input_state->reset_mouse = false;
			}

			return !finished;
		}

		void Update()
		{
			clock_t now = clock();
			float total = (float)now / CLOCKS_PER_SEC;
			float elapsed;
			if(total >= 0)
			{
				elapsed = total - total_game_time;
				total_game_time = total;
			}
			else
				elapsed = total_game_time = 0.0f;
			ProgramScreen* old_screen = current_screen;
			current_screen = current_screen->Update(TimingInfo(elapsed, total_game_time));

			if(current_screen != old_screen)
			{
				old_screen->Deactivate();				// TODO: figure out where this should be deleted
				if(current_screen == NULL)
					finished = true;
				else
					current_screen->Activate();
			}
		}

		void Draw()
		{
			if(current_screen != NULL)
				current_screen->Draw(width, height);
		}

		void SetTitle(string text) { SetWindowText(window_handle, text.c_str()); }
	};




	/*
	 * Unfortunately a WNDPROC cannot be a member function, so we have to do this stuff...
	 */
	map<HWND, ProgramWindow*> active_windows = map<HWND, ProgramWindow*>();
	LRESULT CALLBACK WndProc(HWND window_handle, UINT message, WPARAM wParam, LPARAM lParam)
	{
		map<HWND, ProgramWindow*>::iterator found = active_windows.find(window_handle);
		if(found != active_windows.end())
			return found->second->imp->WndProc(window_handle, message, wParam, lParam);
		else
			return DefWindowProc(window_handle, message, wParam, lParam);
	}




	/*
	 * ProgramWindow methods
	 */
	ProgramWindow::ProgramWindow(Imp* imp) :
		imp(imp),
		content(new ContentMan()),
		input_state(new InputState()),
		sound_system(new SoundSystem()),
		OnWindowClosing()
	{
		imp->AttachWindow(this);
	}

	ProgramWindow::~ProgramWindow()
	{
		delete content;
		delete input_state;
		delete sound_system;
		delete imp;
	}

	int ProgramWindow::Run(ProgramScreen* initial_screen) { return imp->Run(initial_screen); }

	void ProgramWindow::SetTitle(string text) { imp->SetTitle(text); }

	int ProgramWindow::GetWidth() { return imp->width; }
	int ProgramWindow::GetHeight() { return imp->height; }

	bool ProgramWindow::IsFinished() { return imp->finished; }
	void ProgramWindow::SetFinished() { imp->finished = true; }

	bool ProgramWindow::IsActive() { return imp->active; }
	void ProgramWindow::SetActive(bool active) { imp->SetActive(active); ShowCursor(!active); }

	bool ProgramWindow::IsFullscreen() { return imp->fullscreen; }





	/*
	 * The big "CreateProgramWindow" function
	 */
	ProgramWindow* ProgramWindow::CreateProgramWindow(string title, int w, int h, int bpp, bool fullscreen)
	{
		GLuint		pixel_format;
		WNDCLASS	window_class;
		DWORD		window_style_ex;
		DWORD		window_style;
		RECT		window_rect;

		if(bpp == 0)
			bpp = 32;

		HINSTANCE program_instance = GetModuleHandle(NULL);

		window_rect.left = (long)0;
		window_rect.right = (long)w;
		window_rect.top = (long)0;
		window_rect.bottom = (long)h;

		window_class.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;		// Redraw On Move, And Own DC For Window
		window_class.lpfnWndProc	= (WNDPROC) WndProc;
		window_class.cbClsExtra		= 0;
		window_class.cbWndExtra		= 0;
		window_class.hInstance		= program_instance;
		window_class.hIcon			= LoadIcon(NULL, IDI_WINLOGO);			// Load default icon
		window_class.hCursor		= LoadCursor(NULL, IDC_ARROW);			// Load default cursor
		window_class.hbrBackground	= NULL;
		window_class.lpszMenuName	= NULL;
		window_class.lpszClassName	= "Cibrary";

		if (!RegisterClass(&window_class))						// Attempt To Register The Window Class
			return NULL;

		if (fullscreen)
		{
			if(w != 0 && h != 0)
			{
				DEVMODE device_mode;
				memset(&device_mode, 0, sizeof(device_mode));		// clear that area of memory

				device_mode.dmSize 			= sizeof(device_mode);
				device_mode.dmPelsWidth		= w;
				device_mode.dmPelsHeight	= h;
				device_mode.dmBitsPerPel	= bpp;
				device_mode.dmFields		= DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

				if (ChangeDisplaySettings(&device_mode, CDS_FULLSCREEN) != DISP_CHANGE_SUCCESSFUL)
				{
					UnregisterClass("Cibrary", program_instance);
					return NULL;
				}
			}
			else
			{
				w = window_rect.right = GetSystemMetrics(SM_CXSCREEN);
				h = window_rect.bottom = GetSystemMetrics(SM_CYSCREEN);
			}

			window_style = WS_POPUP;						// no frame around window
			window_style_ex = WS_EX_APPWINDOW;
		}
		else
		{
			window_style = WS_OVERLAPPEDWINDOW;
			window_style_ex = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;
		}

		AdjustWindowRectEx(&window_rect, window_style, false, window_style_ex);

		HWND window = CreateWindowEx(
									window_style_ex,
									"Cibrary",
									title.c_str(),
									WS_CLIPSIBLINGS | WS_CLIPCHILDREN | window_style,
									0, 0,				// window position
									window_rect.right - window_rect.left,
									window_rect.bottom - window_rect.top,
									NULL,
									NULL,
									program_instance,
									NULL);
		if(!window)
		{
			// kill window and exit
			if(fullscreen)
				ChangeDisplaySettings(NULL, 0);
			UnregisterClass("Cibrary", program_instance);

			return NULL;
		}

		static PIXELFORMATDESCRIPTOR pfd=					// pfd Tells Windows How We Want Things To Be
		{
			sizeof(PIXELFORMATDESCRIPTOR),
			1,										// version number ???
			PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
			PFD_TYPE_RGBA,
			bpp,
			8, 0, 8, 0, 8, 0,				// ignore color bits
			0,								// no alpha buffer
			0,								// ignore shift bit ???
			0,								// no accumulation buffer
			0, 0, 0, 0,						// ignore accumulation bits
			24,								// z-buffer bits
			0,	//8,						// 8-bit stencil buffer
			0,								// no auxiliary buffer
			PFD_MAIN_PLANE,					// main drawing layer ???
			0,								// reserved
			0, 0, 0							// ignore layer masks
		};

		HDC device_context = GetDC(window);
		if (!device_context)
		{
			// kill window and exit
			if(fullscreen)
				ChangeDisplaySettings(NULL, 0);
			UnregisterClass("Cibrary", program_instance);

			return NULL;
		}

		pixel_format = ChoosePixelFormat(device_context, &pfd);
		if (!pixel_format)
		{
			// kill window and exit
			if(fullscreen)
				ChangeDisplaySettings(NULL, 0);
			UnregisterClass("Cibrary", program_instance);
			ReleaseDC(window, device_context);

			return NULL;
		}

		if(!SetPixelFormat(device_context, pixel_format, &pfd))
		{
			// kill window and exit
			if(fullscreen)
				ChangeDisplaySettings(NULL, 0);
			UnregisterClass("Cibrary", program_instance);
			ReleaseDC(window, device_context);

			return NULL;
		}

		HGLRC gl_context = wglCreateContext(device_context);
		if(!gl_context)
		{
			// kill window and exit
			if(fullscreen)
				ChangeDisplaySettings(NULL, 0);
			UnregisterClass("Cibrary", program_instance);
			ReleaseDC(window, device_context);

			return NULL;
		}

		if(!wglMakeCurrent(device_context, gl_context))
		{
			// kill window and exit
			if(fullscreen)
				ChangeDisplaySettings(NULL, 0);
			UnregisterClass("Cibrary", program_instance);
			ReleaseDC(window, device_context);
			wglMakeCurrent(NULL, NULL);

			return NULL;
		}

		if(!fullscreen && (w == 0 || h == 0))
			ShowWindow(window, SW_SHOWMAXIMIZED);
		else
			ShowWindow(window, SW_SHOWNORMAL);
		SetForegroundWindow(window);
		SetFocus(window);

		//ShowCursor(false);

		ProgramWindow* result = new ProgramWindow(new Imp(fullscreen, gl_context, device_context, window, program_instance));
		active_windows[window] = result;

		return result;
	}
}
