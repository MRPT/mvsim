/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint3D.h>

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <variant>
#include <vector>

/** Backend-agnostic description of custom GUI panels, see
 * World::add_gui_panel().
 *
 * Panels are plain data (no GUI toolkit headers needed by applications).
 * Callbacks always run in the GUI thread. Values shown or edited by widgets
 * are exchanged with the application via LiveString.
 */
namespace mvsim::gui
{
/** A string that can be written from any thread, and is read by the GUI
 * thread every frame.
 *
 * Writer side (any thread):  `live->set("text");`
 * Reader side (GUI thread, e.g. inside a button callback): `live->display`.
 */
struct LiveString
{
	using Ptr = std::shared_ptr<LiveString>;

	LiveString() = default;
	explicit LiveString(const std::string& initial) : display(initial) {}

	LiveString(const LiveString&) = delete;
	LiveString& operator=(const LiveString&) = delete;

	/** Thread-safe write. */
	void set(const std::string& s)
	{
		{
			std::lock_guard<std::mutex> lck(mtx_);
			pending_ = s;
		}
		dirty_.store(true, std::memory_order_release);
	}

	/** To be called by the GUI thread every frame: copies the last value
	 * written with set(), if any, into `display`. */
	void poll_into_display()
	{
		if (!dirty_.exchange(false, std::memory_order_acq_rel))
		{
			return;
		}
		std::lock_guard<std::mutex> lck(mtx_);
		display = pending_;
	}

	/** Current text. Only accessed from the GUI thread. Edited in place by
	 * TextBox widgets. */
	std::string display;

   private:
	std::string pending_;
	std::mutex mtx_;
	std::atomic_bool dirty_ = false;
};

/** A read-only text, updated from any thread via LiveString::set(). */
struct Label
{
	LiveString::Ptr text;  //!< Never null.
};

/** A horizontal line. */
struct Separator
{
};

/** A boolean toggle. `on_change` is called from the GUI thread. */
struct CheckBox
{
	std::string label;
	bool initial_value = false;
	std::function<void(bool)> on_change;
};

/** A push button. `on_click` is called from the GUI thread. */
struct Button
{
	std::string label;
	std::function<void()> on_click;
};

/** An editable single-line text field.
 *
 * The text is `live_value->display`: set it from any thread with
 * `live_value->set()` to replace the content, and read it from the GUI thread.
 * The optional `on_change` is called when the user edits the text.
 */
struct TextBox
{
	std::string label;	//!< Shown above the field, if not empty.
	LiveString::Ptr live_value;	 //!< Never null.
	std::function<void(const std::string&)> on_change;
};

struct Row;

using LeafWidget = std::variant<Label, Separator, CheckBox, Button, TextBox>;

/** Widgets placed on one horizontal row. Rows can not be nested. */
struct Row
{
	std::vector<LeafWidget> widgets;
};

using AnyWidget = std::variant<Label, Separator, CheckBox, Button, TextBox, Row>;

/** A tab page: a vertical list of widgets. */
struct Tab
{
	std::string title;
	std::vector<AnyWidget> widgets;
};

/** A dockable window. If it has only one tab, the tab bar is not shown. */
struct WindowDescription
{
	/// Also identifies the window in the "Window" menu.
	std::string title;

	/// Initial size hint [pixels]. Height 0 means automatic.
	std::array<int, 2> size = {300, 0};

	std::vector<Tab> tabs;
};

/** State of the mouse over the 3D view, see World::set_gui_mouse_callback(). */
struct MouseState
{
	/// Point of the ground plane (or terrain) under the cursor.
	mrpt::math::TPoint3D pt{0, 0, 0};

	/// False if the cursor is not over the 3D view, e.g. it is over a panel.
	/// Buttons are only reported as down while over the 3D view.
	bool over_scene = false;
	bool left_down = false;
	bool right_down = false;
};
}  // namespace mvsim::gui
