#include "InputState.h"
#include "Scripting.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * Input-related event type constructors
	 */
	KeyStateEvent::KeyStateEvent(int key_, bool state_) : key(key_), state(state_) { }
	MouseButtonStateEvent::MouseButtonStateEvent(int button_, bool state_) : button(button_), state(state_) { }
	MouseMotionEvent::MouseMotionEvent(int x_, int y_, int dx_, int dy_) : x(x_), y(y_), dx(dx_), dy(dy_) { }

	/*
	 * InputState methods
	 */
	InputState::InputState() : mouse_rect_valid(false), reset_mouse(false), KeyStateChanged(), MouseButtonStateChanged(), MouseMoved()
	{
		for(int i = 0; i < 1024; i++)
			keys[i] = false;
		for(int i = 0; i < 3; i++)
			mb[i] = false;
	}

	void InputState::SetKeyState(int key, bool state)
	{
		if(keys[key] != state)
		{
			KeyStateEvent evt = KeyStateEvent(key, state);
			KeyStateChanged(&evt);

			ScriptSystem::DoKeyStateCallback(key, state);

			keys[key] = state;
		}
	}

	void InputState::SetMouseButtonState(int button, bool state)
	{
		if(mb[button] != state)
		{
			MouseButtonStateEvent evt = MouseButtonStateEvent(button, state);
			MouseButtonStateChanged(&evt);

			ScriptSystem::DoMouseButtonStateCallback(button, state);

			mb[button] = state;
		}
	}

	void InputState::SetMousePosition(int x, int y)
	{
		int ox = mouse_rect_w / 2, oy = mouse_rect_h / 2;
		if(x != ox || y != oy)
		{
			int dx, dy;
			if(mouse_rect_valid)
			{
				dx = x - ox;
				dy = y - oy;
				mx += dx;
				my += dy;
			}
			else
			{
				dx = dy = 0;
				mx = x;
				my = y;
			}

			mx = max(0, min(mouse_rect_w - 1, mx));
			my = max(0, min(mouse_rect_h - 1, my));

			MouseMotionEvent evt = MouseMotionEvent(mx, my, dx, dy);
			MouseMoved(&evt);

			ScriptSystem::DoMouseMovementCallback(mx, my, dx, dy);

			reset_mouse = true;
		}
		else
			reset_mouse = false;

		mouse_rect_valid = true;
	}
}
