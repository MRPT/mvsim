/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

#include <cstdlib>
#include <map>
#include <string>
#include <string_view>
#include <type_traits>

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
 *  - "%scale3d" => Expects "S" (same scale for all axes) or "SX SY SZ".
 * "Val" is a pointer to mrpt::math::TPoint3D
 *  - "%bool" ==> bool*. Values: 'true'/'false' or '1'/'0'
 *
 *  \ingroup mvsim_simulator_module
 */
struct TParamEntry
{
	const char* format = nullptr;
	void* val = nullptr;

	TParamEntry() = default;

	template <typename T>
	TParamEntry(const char* format_, T* targetVariable)
		: format(matchFloatFormat<T>(format_)), val(reinterpret_cast<void*>(targetVariable))
	{
	}

	/** sscanf() needs "%f" for float and "%lf" for double: returns the variant
	 * matching T, so a mismatched format neither leaves part of a double
	 * unset nor writes past the end of a float. */
	template <typename T>
	static const char* matchFloatFormat(const char* f)
	{
		const std::string_view s(f);
		if constexpr (std::is_same_v<T, double>)
		{
			if (s == "%f")
			{
				return "%lf";
			}
			if (s == "%f_deg")
			{
				return "%lf_deg";
			}
		}
		else if constexpr (std::is_same_v<T, float>)
		{
			if (s == "%lf")
			{
				return "%f";
			}
			if (s == "%lf_deg")
			{
				return "%f_deg";
			}
		}
		return f;
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
