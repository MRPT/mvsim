#include "mvsim/CsvLogger.h"

CSVLogger::CSVLogger()
{
	m_file = std::make_shared<std::ofstream>(std::ofstream());
}

CSVLogger::~CSVLogger() { close(); }
void CSVLogger::addColumn(std::string name) { m_columns[name] = 0.0; }
void CSVLogger::updateColumn(std::string name, double value)
{
	m_columns[name] = value;
}

bool CSVLogger::writeHeader()
{
	columns_type::iterator it;
	for (it = m_columns.begin(); it != m_columns.end();)
	{
		*m_file << it->first;

		if (++it != m_columns.end())
		{
			*m_file << ", ";
		}
	}

	*m_file << "\n";  // most CSV readers don't use \r\n

	return !!m_file;
}

bool CSVLogger::writeRow()
{
	if (!isRecording) return true;

	if (!isOpen()) clear();

	columns_type::iterator it;
	for (it = m_columns.begin(); it != m_columns.end();)
	{
		*m_file << it->second;

		if (++it != m_columns.end())
		{
			*m_file << ", ";
		}
	}

	*m_file << "\n";

	return !!m_file;
}

bool CSVLogger::open()
{
	if (m_file)
	{
		m_file->open((std::string("session") + std::to_string(currentSession) +
					  std::string("-") + m_filepath)
						 .c_str());
		return isOpen();
	}
	return false;
}

bool CSVLogger::isOpen() { return m_file->is_open(); }
bool CSVLogger::close()
{
	if (m_file)
	{
		m_file->close();
		return !isOpen();
	}
	return false;
}

bool CSVLogger::clear()
{
	if (isOpen()) close();

	if (open())
	{
		return writeHeader();
	}

	return false;
}

void CSVLogger::newSession()
{
	currentSession++;
	close();
}
