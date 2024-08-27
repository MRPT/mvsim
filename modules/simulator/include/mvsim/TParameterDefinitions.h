/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

#include <cstdlib>
#include <map>
#include <string>

namespace mvsim
{
/** Normal case: sscanf()-like specifiers, and "void*" pointing to corresponding
 * variable type.
 * Special cases:
 *  - "%lf_deg" => "val" is a "double*". The read number will be converted
 * from degrees to radians.
 *  - "%s" => "val" is assumed to be a pointer to a std::string. Strings are
 * trimmed of whitespaces.
 *  - "%color" => Expected values: "#RRGGBB[AA]" ([00-FF] each). "val" is
 * assumed to be a pointer to a mrpt::img::TColor
 *  - "%pose2d" => Expects "X Y YAW_DEG". "Val" is a pointer to
 * mrpt::poses::CPose2D
 *  - "%pose2d_ptr3d" => Expects "X Y YAW_DEG". "Val" is a pointer to
 * mrpt::poses::CPose3D
 *  - "%pose3d" => Expects "X Y Z YAW_DEG PITCH_DEG ROLL_DEG". "Val" is a
 * pointer to mrpt::poses::CPose3D
 *  - "%point3d" => Expects "X Y [Z]". "Val" is a pointer to
 * mrpt::math::TPoint3D
 *  - "%bool" ==> bool*. Values: 'true'/'false' or '1'/'0'
 *
 * \todo Rewrite using std::variant?
 */
struct TParamEntry
{
	const char* frmt = nullptr;
	void* val = nullptr;

	TParamEntry() = default;

	template <typename T>
	TParamEntry(const char* frmt_, T* targetVariable)
		: frmt(frmt_), val(reinterpret_cast<void*>(targetVariable))
	{
	}

	/** Tries to parse the given input string according to the expected format,
	 * then store the result in "*val"
	 * \exception std::runtime_error On format errors.
	 */
	void parse(
		const std::string& str, const std::string& varName,
		const std::map<std::string, std::string>& variableNamesValues = {},
		const char* functionNameContext = "") const;
};

/** Container mapping a list of parameter names to their type definitions and
 * actual placeholder variables where to store their values when reading a
 * configuration file. */
using TParameterDefinitions = std::map<std::string, TParamEntry>;

}  // namespace mvsim
