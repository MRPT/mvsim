/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "mvsim/CsvLogger.h"

#include <filesystem>
#include <iostream>

CSVLogger::CSVLogger() { file_ = std::make_shared<std::ofstream>(); }

CSVLogger::~CSVLogger() { close(); }

void CSVLogger::updateColumn(const std::string_view& name, double value) { columns_[name] = value; }

bool CSVLogger::writeHeader()
{
	if (!file_->is_open())
	{
		return false;
	}

	for (auto it = columns_.begin(); it != columns_.end();)
	{
		*file_ << it->first;

		if (++it != columns_.end())
		{
			*file_ << ", ";
		}
	}

	*file_ << "\n";	 // most CSV readers don't use \r\n

	return !!file_;
}

bool CSVLogger::writeRow()
{
	// Always notify registered listeners, regardless of file recording:
	for (const auto& cb : onRowCallbacks_)
	{
		cb(columns_);
	}

	if (!isRecording_ || !fileWritingEnabled_)
	{
		return true;
	}

	if (!isOpen())
	{
		if (openFailed_)
		{
			return false;
		}
		clear();
	}

	for (auto it = columns_.begin(); it != columns_.end();)
	{
		*file_ << it->second;

		if (++it != columns_.end())
		{
			*file_ << ", ";
		}
	}

	*file_ << "\n";

	return !!file_;
}

bool CSVLogger::open()
{
	using std::literals::string_literals::operator""s;

	if (file_)
	{
		// The session prefix goes in front of the file name, not of its
		// directory, which is created if needed:
		const std::filesystem::path path(filepath_);
		const std::filesystem::path dir = path.parent_path();
		if (!dir.empty())
		{
			std::error_code ec;
			std::filesystem::create_directories(dir, ec);
		}
		const std::string fileName =
			"session_"s + std::to_string(::time(nullptr)) + "-"s + path.filename().string();
		file_->open((dir / fileName).string());
		if (!isOpen() && !openFailed_)
		{
			std::cerr << "[CSVLogger] Cannot write the log file: " << (dir / fileName).string()
					  << "\n";
		}
		openFailed_ = !isOpen();
		return isOpen();
	}
	return false;
}

bool CSVLogger::isOpen() { return file_->is_open(); }
bool CSVLogger::close()
{
	if (file_)
	{
		file_->close();
		return !isOpen();
	}
	return false;
}

bool CSVLogger::clear()
{
	if (isOpen())
	{
		close();
	}

	if (open())
	{
		return writeHeader();
	}

	return false;
}

void CSVLogger::setRecording(bool recording)
{
	// A new recording may retry an open that failed before:
	if (recording && !isRecording_)
	{
		openFailed_ = false;
	}
	isRecording_ = recording;
}

void CSVLogger::newSession()
{
	close();
	// A new session may succeed where a previous one failed to open its file:
	openFailed_ = false;
}