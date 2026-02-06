/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/system/filesystem.h>
#include <mvsim/RemoteResourcesManager.h>

#include <cstdlib>
#include <cstring>

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
	std::string local_directory;
#ifdef _WIN32
	local_directory = std::getenv("APPDATA");
#else
	local_directory = std::getenv("HOME");
	local_directory += "/.cache/";
#endif

	local_directory += "mvsim-storage/";

	return local_directory;
}

std::tuple<bool, std::string, std::string> RemoteResourcesManager::zip_uri_split(
	const std::string& uri)
{
	auto pos = uri.find(".zip/");

	if (pos == std::string::npos) return {false, uri, ""};

	const auto zipUri = uri.substr(0, pos + 4);
	const auto internalUri = uri.substr(pos + 5);

	return {true, zipUri, internalUri};
}

std::string RemoteResourcesManager::handle_remote_uri(const std::string& uri)
{
	using namespace std::string_literals;

	const auto localDir = cache_directory();
	std::string cacheUsageStats;

	if (!mrpt::system::directoryExists(localDir))
	{
#ifdef _WIN32
		mrpt::system::createDirectory(localDir);
#else
		const auto cmd = "mkdir -p \""s + localDir + "\""s;
		if (int ret = ::system(cmd.c_str()); ret != 0)
		{
			MRPT_LOG_ERROR_STREAM("Error executing command: " << cmd);
		}
#endif
		ASSERT_DIRECTORY_EXISTS_(localDir);
	}
	else
	{
		// get usage stats:
		if (0 == mrpt::system::executeCommand(
					 mrpt::format("du -sh0 \"%s\"", localDir.c_str()), &cacheUsageStats))
		{
			// usage stats are valid.
		}
	}
	MRPT_LOG_ONCE_INFO(
		"Using local storage directory: '"s + localDir + "' (Usage: "s + cacheUsageStats + ")"s);

	const auto [isZipPkg, zipOrFileURI, internalURI] = zip_uri_split(uri);

	MRPT_LOG_DEBUG_STREAM(
		"Split URI: isZipPkg=" << isZipPkg << " zipOrFileURI=" << zipOrFileURI
							   << " internalURI=" << internalURI);

	const auto fileName = mrpt::system::extractFileName(zipOrFileURI) + "." +
						  mrpt::system::extractFileExtension(zipOrFileURI);
	const auto localFil = localDir + fileName;

	// Download if it does not exist already from a past download:
	if (!mrpt::system::fileExists(localFil))
	{
		const auto cmd =
			mrpt::format("wget -q -O \"%s\" %s", localFil.c_str(), zipOrFileURI.c_str());

		MRPT_LOG_INFO_STREAM("Downloading remote resources from: '" << uri << "'");

		if (int ret = ::system(cmd.c_str()); ret != 0)
		{
			THROW_EXCEPTION_FMT(
				"[mvsim] Error (code=%i)  executing the following command "
				"trying to acquire a remote resource:\n%s",
				ret, cmd.c_str());
		}
	}

	// Is it a simple model file, or a zip?
	if (isZipPkg)
	{
		// process the ZIP package:
		return handle_local_zip_package(localFil, internalURI);
	}
	else
	{
		// simple file:
		return localFil;
	}
}

std::string RemoteResourcesManager::handle_local_zip_package(
	const std::string& localZipFil, const std::string& internalURI)
{
	using namespace std::string_literals;

	ASSERT_FILE_EXISTS_(localZipFil);

	const auto filExtension = mrpt::system::extractFileExtension(localZipFil);
	ASSERT_EQUAL_(filExtension, "zip");

	// already decompressed?
	const auto zipOutDir = mrpt::system::fileNameChangeExtension(localZipFil, "out");

	if (!mrpt::system::directoryExists(zipOutDir))
	{
		mrpt::system::createDirectory(zipOutDir);
		ASSERT_DIRECTORY_EXISTS_(zipOutDir);

		const std::string cmd = "unzip -q \""s + localZipFil + "\" -d \""s + zipOutDir + "\""s;

		if (int ret = ::system(cmd.c_str()); ret != 0)
		{
			THROW_EXCEPTION_FMT(
				"[mvsim] Error (code=%i) executing the following command "
				"trying to unzip a remote resource:\n%s",
				ret, cmd.c_str());
		}
	}

	return zipOutDir + "/"s + internalURI;
}
