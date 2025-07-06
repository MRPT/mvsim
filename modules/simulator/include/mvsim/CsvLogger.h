/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <string>

/** \ingroup mvsim_simulator_module */
class CSVLogger
{
   public:
	CSVLogger();
	virtual ~CSVLogger();

	using Ptr = std::shared_ptr<CSVLogger>;

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
	void newSession();

   private:
	std::map<std::string_view, double> columns_;
	std::shared_ptr<std::ofstream> file_;
	std::string filepath_;
	bool isRecording_ = false;
};
