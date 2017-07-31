#pragma once

#include <map>
#include <fstream>
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

  bool open(std::string path);
  bool close();

private:
  columns_type m_columns;
  std::ofstream *m_file;
};
