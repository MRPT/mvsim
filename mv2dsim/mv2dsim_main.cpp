/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/mv2dsim.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>

void parse(const char* filename)
{
	using namespace std;
	using namespace rapidxml;

	rapidxml::file<> fil(filename);
	rapidxml::xml_document<> doc;    // character type defaults to char

	try 
	{
		doc.parse<0>(fil.data());

		// Print to stream using operator <<
		std::cout << doc; 

		xml_node<> *root = doc.first_node();

		cout << "Name of my first node is: " << root->name() << "\n";

		xml_node<> *node = root->first_node("vehicle");
		cout << "Node vehicle has value " << node->value() << "\n";
		for (xml_attribute<> *attr = node->first_attribute();
			 attr; attr = attr->next_attribute())
		{
			cout << "Node foobar has attribute " << attr->name() << " ";
			cout << "with value " << attr->value() << "\n";
		}
	}
	catch (rapidxml::parse_error &e)
	{
		long line = static_cast<long>(std::count(fil.data(), e.where<char>(), '\n') + 1);
		std::cerr << "Line " << line << ": " << e.what() << std::endl;
	}
}

	
int main(int argc, char **argv) 
{
    try 
	{
		if (argc!=2) return -1;

		parse(argv[1]);
				

    } catch (const std::exception& e) 
	{
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
