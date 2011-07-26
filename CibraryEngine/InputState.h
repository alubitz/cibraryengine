#pragma once

#include "StdAfx.h"
#include "Events.h"

namespace CibraryEngine
{
	/** Event which occurs when a key is pressed or released */
	class KeyStateEvent : public Event
	{
		public:
			/** They key whose state changed */
			int key;
			/** True means pressed, false means released */
			bool state;

			KeyStateEvent(int key, bool state);
	};

	/** Event which occurs when a mouse button is pressed or released */
	class MouseButtonStateEvent : public Event
	{
		public:
			/** The button whose state changed */
			int button;
			/** True means pressed, false means released */
			bool state;

			MouseButtonStateEvent(int button, bool state);
	};

	/** Event which occurs when the mouse moves */
	class MouseMotionEvent : public Event
	{
		public:
			/** The horizontal position of the virtual cursor */
			int x;
			/** The vertical position of the virtual cursor */
			int y;

			/** The horizontal motion of the cursor since last time */
			int dx;
			/** The vertical motion of the cursor since last time */
			int dy;

			MouseMotionEvent(int x_, int y_, int dx_, int dy_);
	};

	/** Class storing the state of input devices */
	class InputState
	{
		public:
			/** Whether certain keys are pressed */
			bool keys[1024];

			/** Whether certain mouse buttons are pressed */
			bool mb[3];

			/** The position of the virtual cursor */
			int mx, my;

			/** Used to filter out mouse movement on the first frame which might cause a player to rotate violently */
			bool mouse_rect_valid;

			bool reset_mouse;
			/** The horizontal size of the client area (to which the virtual cursor is limited) */
			int mouse_rect_w;
			/** The vertical size of the client area (to which the virtual cursor is limited) */
			int mouse_rect_h;

			InputState();

			/** Function called when a key is pressed or released */
			void SetKeyState(int key, bool state);
			/** Function called when a mouse button is pressed or released */
			void SetMouseButtonState(int button, bool state);
			/** Function called when the mouse moves */
			void SetMousePosition(int x, int y);

			/** Event which occurs when a key is pressed or released */
			EventDispatcher KeyStateChanged;
			/** Event which occurs when a mouse button is pressed or released */
			EventDispatcher MouseButtonStateChanged;
			/** Event which occurs when the mouse moves */
			EventDispatcher MouseMoved;
	};
}
