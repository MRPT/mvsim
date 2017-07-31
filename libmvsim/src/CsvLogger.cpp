#include "mvsim/CsvLogger.h"

CSVLogger::CSVLogger()
{
  m_file = new std::ofstream();
}

CSVLogger::~CSVLogger()
{
  close();
}

void CSVLogger::addColumn(std::string name)
{
  m_columns[name] = 0.0;
}

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

    if(++it != m_columns.end())
    {
      *m_file << ", ";
    }
  }

  *m_file << "\n"; //most CSV readers don't use \r\n

  return m_file;
}

bool CSVLogger::writeRow()
{
  columns_type::iterator it;
  for (it = m_columns.begin(); it != m_columns.end();)
  {
    *m_file << it->second;

    if(++it != m_columns.end())
    {
      *m_file << ", ";
    }
  }

  *m_file << "\n";

  return m_file;
}

bool CSVLogger::open(std::string path)
{
  if(m_file)
  {
    m_file->open(path.c_str());
    return m_file->is_open();
  }
  return false;
}

bool CSVLogger::close()
{
  if(m_file)
  {
    m_file->close();
    return m_file->is_open();
  }
  return false;
}
