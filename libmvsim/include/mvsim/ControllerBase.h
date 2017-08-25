/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/basic_types.h>

namespace mvsim
{
/** Interface of ControllerBaseTempl<> for teleoperation, etc.
  * Abstract interface common to any vehicle type & any controller.
  */
class ControllerBaseInterface
{
   public:
	struct TeleopInput
	{
		int keycode;
		TeleopInput() : keycode(0) {}
	};
	struct TeleopOutput
	{
		std::string append_gui_lines;
	};

	virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out)
	{ /*default: do nothing*/}

	/** Accept a Twist command. \return true if the controller supports this
	 * kind of commands, false otherwise */
	virtual bool setTwistCommand(const double vx, const double wz)
	{
		return false; /* default: no */
	}
};

/** Virtual base for controllers of vehicles of any type (template) */
template <class VEH_DYNAMICS>
class ControllerBaseTempl : public ControllerBaseInterface
{
   public:
	ControllerBaseTempl(VEH_DYNAMICS& veh) : m_veh(veh) {}
	virtual ~ControllerBaseTempl() {}
	/** This is to handle basic need of all the controllers.*/
	virtual void teleop_interface(
		const TeleopInput& in, TeleopOutput& out) override
	{
		/*default: handle logging events*/
		static bool isRecording = false;
		switch (in.keycode)
		{
			case 'l':
			case 'L':
			{
				isRecording = !isRecording;
				setLogRecording(isRecording);
			}
			break;
			case 'c':
			case 'C':
			{
				clearLogs();
			}
			break;

			case 'n':
			case 'N':
			{
				newLogSession();
			}
			break;

			default:
				break;
		}

		out.append_gui_lines +=
			std::string(
				"Toggle logging [L]. Clear logs[C]. New log session [N]. "
				"Now ") +
			std::string(isRecording ? "logging" : "not logging") +
			std::string("\n");
	}

	/** The core of the controller: will be called at each timestep before the
	 * numeric integration of dynamical eqs */
	virtual void control_step(
		const typename VEH_DYNAMICS::TControllerInput& ci,
		typename VEH_DYNAMICS::TControllerOutput& co) = 0;

	/** Override to load class-specific options from the <controller> node */
	virtual void load_config(const rapidxml::xml_node<char>& node)
	{ /*default: do nothing*/}

	virtual void setLogRecording(bool recording)
	{
		m_veh.setRecording(recording);
	}
	virtual void clearLogs() { m_veh.clearLogs(); }
	virtual void newLogSession() { m_veh.newLogSession(); }
   protected:
	VEH_DYNAMICS& m_veh;
};
}
