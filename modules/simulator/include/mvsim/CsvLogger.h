#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <string>

/** \ingroup mvsim_simulator_module */
class CSVLogger
{
	typedef std::map<std::string, double> columns_type;

   public:
	CSVLogger();
	virtual ~CSVLogger();

   public:
	void addColumn(std::string name);
	void updateColumn(std::string name, double value);
	bool writeHeader();
	bool writeRow();

	void setFilepath(std::string path) { filepath_ = path; }
	bool open();
	bool isOpen();
	bool close();
	bool clear();

	void setRecording(bool recording) { isRecording = recording; }
	void newSession();

   private:
	columns_type columns_;
	std::shared_ptr<std::ofstream> file_;
	std::string filepath_;
	bool isRecording = false;
	unsigned int currentSession = 1;
};
