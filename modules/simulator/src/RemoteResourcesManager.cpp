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
#ifndef _WIN32
#include <sys/wait.h>
#include <unistd.h>
#endif

#include "xml_utils.h"

using namespace mvsim;

RemoteResourcesManager::RemoteResourcesManager()
	: mrpt::system::COutputLogger("mvsim::RemoteResourcesManager")
{
}

void RemoteResourcesManager::parse_from(const rapidxml::xml_node<char>& node)
{
	parse_xmlnode_attribs(node, params_, {}, "[RemoteResourcesManager]");
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

	const auto splitResult = zip_uri_split(uri);
	const bool isZipPkg = std::get<0>(splitResult);
	const std::string zipOrFileURI = std::get<1>(splitResult);
	const std::string internalURI = std::get<2>(splitResult);

	MRPT_LOG_DEBUG_STREAM(
		"Split URI: isZipPkg=" << isZipPkg << " zipOrFileURI=" << zipOrFileURI
							   << " internalURI=" << internalURI);

	const auto fileName = mrpt::system::extractFileName(zipOrFileURI) + "." +
						  mrpt::system::extractFileExtension(zipOrFileURI);
	const auto localFil = localDir + fileName;

	auto doDownload = [&]()
	{
		MRPT_LOG_INFO_STREAM("Downloading remote resources from: '" << uri << "'");

		// Retry the wget call itself, regardless of file type, in case of transient
		// network errors leaving behind a truncated/partial file. This allows for
		// an initial attempt plus two retries:
		constexpr int maxDownloadAttempts = 3;
		int ret = -1;
		for (int attempt = 0; attempt < maxDownloadAttempts; attempt++)
		{
#ifndef _WIN32
			// Use execvp so the URI is passed as a literal argument — no shell, no injection risk.
			const pid_t pid = ::fork();
			if (pid < 0)
			{
				ret = -1;
			}
			else if (pid == 0)
			{
				if (insecure_skip_tls_verify)
				{
					::execlp(
						"wget", "wget", "-q", "--no-check-certificate", "-O", localFil.c_str(),
						zipOrFileURI.c_str(), static_cast<char*>(nullptr));
				}
				else
				{
					::execlp(
						"wget", "wget", "-q", "-O", localFil.c_str(), zipOrFileURI.c_str(),
						static_cast<char*>(nullptr));
				}
				::_exit(127);
			}
			else
			{
				int status = 0;
				::waitpid(pid, &status, 0);
				ret = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
			}
#else
			const auto cmd = mrpt::format(
				"wget -q %s-O \"%s\" \"%s\"",
				insecure_skip_tls_verify ? "--no-check-certificate " : "", localFil.c_str(),
				zipOrFileURI.c_str());
			ret = ::system(cmd.c_str());
#endif
			if (ret == 0) break;

			MRPT_LOG_WARN_STREAM(
				"Attempt " << (attempt + 1) << "/" << maxDownloadAttempts << " failed (code=" << ret
						   << ") running wget to acquire remote "
							  "resource: "
						   << zipOrFileURI);
		}
		if (ret != 0)
		{
			THROW_EXCEPTION_FMT(
				"[mvsim] Error (code=%i) running wget to acquire remote resource: %s", ret,
				zipOrFileURI.c_str());
		}
	};

	// Download if it does not exist already from a past download, or if a previous
	// download was interrupted leaving behind a truncated (zero-size) cached file:
	if (!mrpt::system::fileExists(localFil) || mrpt::system::getFileSize(localFil) == 0)
	{
		doDownload();
	}

	// Is it a simple model file, or a zip?
	if (isZipPkg)
	{
		// process the ZIP package, retrying once on failure (handles corrupt/partial downloads):
		for (int attempt = 0; attempt < 2; attempt++)
		{
			try
			{
				return handle_local_zip_package(localFil, internalURI);
			}
			catch (const std::exception& e)
			{
				if (attempt == 0)
				{
					MRPT_LOG_WARN_STREAM(
						"Failed to use cached zip (possibly corrupted), re-downloading '"
						<< zipOrFileURI << "': " << e.what());
					// handle_local_zip_package already deleted the corrupt zip and output dir
					doDownload();
				}
				else
				{
					throw;
				}
			}
		}
		THROW_EXCEPTION("unreachable");
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
			// Clean up partial output dir and corrupt zip so the caller can retry the download
			mrpt::system::deleteFilesInDirectory(zipOutDir, /*deleteDirectoryAsWell=*/true);
			mrpt::system::deleteFile(localZipFil);

			THROW_EXCEPTION_FMT(
				"[mvsim] Error (code=%i) executing the following command "
				"trying to unzip a remote resource:\n%s",
				ret, cmd.c_str());
		}
	}

	return zipOutDir + "/"s + internalURI;
}
