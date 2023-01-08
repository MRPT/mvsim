/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/system/filesystem.h>
#include <mvsim/RemoteResourcesManager.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>

using namespace mvsim;

RemoteResourcesManager::RemoteResourcesManager()
	: mrpt::system::COutputLogger("mvsim::RemoteResourcesManager")
{
}

bool RemoteResourcesManager::is_remote(const std::string& url)
{
	return 0 == ::strncmp(url.c_str(), "http://", strlen("http://")) ||
		   0 == ::strncmp(url.c_str(), "https://", strlen("https://"));
}

std::string RemoteResourcesManager::resolve_path(const std::string& uri)
{
	if (is_remote(uri))
	{
		return handle_remote_uri(uri);
	}
	else if (uri.substr(0, 7) == "file://")
	{
		return uri.substr(7);
	}
	else
	{
		return uri;
	}
}

std::string RemoteResourcesManager::cache_directory()
{
	std::filesystem::path local_directory;
#ifdef _WIN32
	local_directory = std::filesystem::path(std::getenv("APPDATA"));
#else
	local_directory = std::filesystem::path(std::getenv("HOME"));
	local_directory += std::filesystem::path("/.cache/");
#endif

	local_directory += std::filesystem::path("mvsim/");
	local_directory += std::filesystem::path("storage/");

	return local_directory.string();
}

std::string RemoteResourcesManager::handle_remote_uri(const std::string& uri)
{
	using namespace std::string_literals;

	const auto localDir = cache_directory();
	std::string cacheUsageStats;

	if (!mrpt::system::directoryExists(localDir))
	{
		std::filesystem::create_directories(localDir);
		ASSERT_DIRECTORY_EXISTS_(localDir);
	}
	else
	{
		// get usage stats:
		if (0 == mrpt::system::executeCommand(
					 mrpt::format("du -sh0 \"%s\"", localDir.c_str()),
					 &cacheUsageStats))
		{
			// usage stats are valid.
		}
	}
	MRPT_LOG_ONCE_INFO(
		"Using local storage directory: '"s + localDir + "' (Usage: "s +
		cacheUsageStats + ")"s);

	const auto fileName = mrpt::system::extractFileName(uri) + "." +
						  mrpt::system::extractFileExtension(uri);
	const auto localFil = localDir + fileName;

	// Download if it does not exist already from a past download:
	if (!mrpt::system::fileExists(localFil))
	{
		const auto cmd =
			mrpt::format("wget -q -O \"%s\" %s", localFil.c_str(), uri.c_str());

		int ret = ::system(cmd.c_str());
		if (ret != 0)
		{
			THROW_EXCEPTION_FMT(
				"[mvsim] Error executing the following command trying to "
				"acquire a remote resource:\n%s",
				cmd.c_str());
		}
	}

	// Is it a simple model file, or a zip?
	if (0)
	{
		// process the ZIP package:

		return {};
	}
	else
	{
		// simple file:
		return localFil;
	}
}
