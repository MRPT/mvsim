#include "mvsim/CsvLogger.h"

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
	if (!isRecording_) return true;

	if (!isOpen()) clear();

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
		file_->open("session_"s + std::to_string(::time(nullptr)) + "-"s + filepath_);
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

void CSVLogger::newSession() { close(); }
