#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <string>

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

	void setFilepath(std::string path) { m_filepath = path; }
	bool open();
	bool isOpen();
	bool close();
	bool clear();

	void setRecording(bool recording) { isRecording = recording; }
	void newSession();

   private:
	columns_type m_columns;
	std::shared_ptr<std::ofstream> m_file;
	std::string m_filepath;
	bool isRecording = false;
	unsigned int currentSession = 1;
};
