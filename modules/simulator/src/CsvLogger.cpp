#include "mvsim/CsvLogger.h"

CSVLogger::CSVLogger()
{
	file_ = std::make_shared<std::ofstream>(std::ofstream());
}

CSVLogger::~CSVLogger() { close(); }
void CSVLogger::addColumn(std::string name) { columns_[name] = 0.0; }
void CSVLogger::updateColumn(std::string name, double value)
{
	columns_[name] = value;
}

bool CSVLogger::writeHeader()
{
	columns_type::iterator it;
	for (it = columns_.begin(); it != columns_.end();)
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
	if (!isRecording) return true;

	if (!isOpen()) clear();

	columns_type::iterator it;
	for (it = columns_.begin(); it != columns_.end();)
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
	if (file_)
	{
		file_->open((std::string("session") + std::to_string(currentSession) +
					 std::string("-") + filepath_)
						.c_str());
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
