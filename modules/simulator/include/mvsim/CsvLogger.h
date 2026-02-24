/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

/** \ingroup mvsim_simulator_module */
class CSVLogger
{
   public:
	CSVLogger();
	virtual ~CSVLogger();

	using Ptr = std::shared_ptr<CSVLogger>;

	// deleted copy/move constructors and assignment operators:
	CSVLogger(const CSVLogger&) = delete;
	CSVLogger& operator=(const CSVLogger&) = delete;
	CSVLogger(CSVLogger&&) = delete;
	CSVLogger& operator=(CSVLogger&&) = delete;

	/// Callback type invoked on each writeRow(), receiving the current
	/// column name→value snapshot. Fired regardless of file-recording state.
	using on_row_callback_t =
		std::function<void(const std::map<std::string_view, double>& /*columns*/)>;

   public:
	void updateColumn(const std::string_view& name, double value);
	bool writeHeader();
	bool writeRow();

	void setFilepath(const std::string& path) { filepath_ = path; }
	bool open();
	bool isOpen();
	bool close();
	bool clear();

	void setRecording(bool recording) { isRecording_ = recording; }
	[[nodiscard]] bool isRecording() const { return isRecording_; }

	/// When disabled, writeRow() still fires callbacks but skips all file
	/// I/O. Useful when only in-memory callbacks are needed (e.g. unit tests,
	/// auto-tuning) to avoid generating useless .csv files on disk.
	void setFileWritingEnabled(bool enabled) { fileWritingEnabled_ = enabled; }
	[[nodiscard]] bool isFileWritingEnabled() const { return fileWritingEnabled_; }
	void newSession();

	/// Register a callback to be invoked on every writeRow().
	/// \sa isActive()
	void registerOnRowCallback(const on_row_callback_t& cb) { onRowCallbacks_.emplace_back(cb); }

	/// Returns true when this logger should be "active", i.e. columns
	/// should be updated, because either file recording is on **or** at
	/// least one listener callback is registered.
	[[nodiscard]] bool isActive() const { return isRecording_ || !onRowCallbacks_.empty(); }

	/// Read-only access to the current column values.
	[[nodiscard]] const std::map<std::string_view, double>& getColumns() const { return columns_; }

   private:
	std::map<std::string_view, double> columns_;
	std::shared_ptr<std::ofstream> file_;
	std::string filepath_;
	bool isRecording_ = false;
	bool fileWritingEnabled_ = true;

	std::vector<on_row_callback_t> onRowCallbacks_;
};