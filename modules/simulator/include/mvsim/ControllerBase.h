/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/basic_types.h>

#include <optional>

namespace mvsim
{
/** Interface of ControllerBaseTempl<> for teleoperation, etc.
 * Abstract interface common to any vehicle type & any controller.
 * \ingroup mvsim_simulator_module
 */
class ControllerBaseInterface
{
   public:
	struct TeleopInput
	{
		int keycode = 0;
		std::optional<TJoyStickEvent> js;

		TeleopInput() = default;
	};

	struct TeleopOutput
	{
		std::string append_gui_lines;
	};

	virtual void teleop_interface(
		[[maybe_unused]] const TeleopInput& in, [[maybe_unused]] TeleopOutput& out)
	{
		/*default: do nothing*/
	}

	/** Accept a Twist command. \return true if the controller supports this
	 * kind of commands, false otherwise
	 *
	 *  Set these values to tell the controller the desired
	 * setpoints for linear (vx) (m/s) and angular (omega) (rad/s)
	 * velocities. Note that the "vy" component of the twist is ignored.
	 */
	virtual bool setTwistCommand([[maybe_unused]] const mrpt::math::TTwist2D& t)
	{
		return false; /* default: no */
	}
};

/** Virtual base for controllers of vehicles of any type (template) */
template <class VEH_DYNAMICS>
class ControllerBaseTempl : public ControllerBaseInterface
{
   public:
	using Ptr = std::shared_ptr<ControllerBaseTempl<VEH_DYNAMICS>>;

	ControllerBaseTempl(VEH_DYNAMICS& veh) : veh_(veh) {}
	virtual ~ControllerBaseTempl() {}
	/** This is to handle basic need of all the controllers.*/
	virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override
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

		out.append_gui_lines += std::string(
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

	virtual void on_post_step([[maybe_unused]] const TSimulContext& context) {}

	/** Override to load class-specific options from the <controller> node */
	virtual void load_config(  //
		[[maybe_unused]] const rapidxml::xml_node<char>& node)
	{ /*default: do nothing*/
	}

	virtual void setLogRecording(bool recording) { veh_.setRecording(recording); }
	virtual void clearLogs() { veh_.clearLogs(); }
	virtual void newLogSession() { veh_.newLogSession(); }

   protected:
	VEH_DYNAMICS& veh_;
};
}  // namespace mvsim
