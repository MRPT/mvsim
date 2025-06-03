/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/COutputLogger.h>

#include <string>
#include <tuple>

namespace mvsim
{
/**
 * Keeps a local directory with cached archives and packages from remote
 * servers.
 * The cache directory is `$HOME/.cache/mvsim-storage/` in non-Windows systems.
 *
 * See resolve_path() for the possible formats of URI addresses.
 *
 * \ingroup mvsim_simulator_module
 */
class RemoteResourcesManager : public mrpt::system::COutputLogger
{
   public:
	RemoteResourcesManager();
	~RemoteResourcesManager() = default;

	/** Processes an URI with the format described next, and returns a
	 *  local machine URI for that file.
	 *
	 *  Possible formats for `uri`:
	 *  - `/plain/path/to/file`: Then this returns the same string.
	 *  - `file://<ACTUAL_LOCAL_PATH>`: This returns the local path.
	 *  - `https://server.org/path/file.dae`: It downloads the file and returns
	 *     a local path to it.
	 *  - `https://server.org/path/package.zip/file.dae`: Downloads the ZIP
	 * package, extracts
	 */
	std::string resolve_path(const std::string& uri);

	/** Returns true if the URI starts with "http[s]://"
	 */
	static bool is_remote(const std::string& uri);

	/** Returns {true, zip_uri, internal_uri} if the URI refers to a ZIP file,
	 * or {false, uri, ""} otherwise.
	 */
	static std::tuple<bool, std::string, std::string> zip_uri_split(const std::string& uri);

	static std::string cache_directory();

   private:
	std::string handle_remote_uri(const std::string& uri);

	/// Returns the local file that internalURI refers to, possibly
	/// decompressing the ZIP package first if it is the first time.
	std::string handle_local_zip_package(
		const std::string& localZipFil, const std::string& internalURI);
};

}  // namespace mvsim
