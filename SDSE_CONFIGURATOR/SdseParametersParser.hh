// ----------------------------------------------------------------------------
// OpenSDSE - HLA Compliant Distributed Aircraft Simulation
// Copyright (C) 2017  ISAE
//
// This program is free software ; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation ; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY ; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program ; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
// ----------------------------------------------------------------------------

#ifndef __SDSE_PARAMETERS_PARSER_HH_DEF__
#define __SDSE_PARAMETERS_PARSER_HH_DEF__
#include "tinyxml2.h"
#include <string>
#include <vector>

// XML file copied to the bin directory
#define DEFAULT_XML_PATH "sdse_init_parameters.xml"

using namespace tinyxml2;
using std::string;
using std::vector;

// ----------------------------------------------------------------------------
// tiny xml specific methods
// ---------------------------------------------------------------------------- 
namespace parser
{

	// Load a file into a document and return it.
	// -> file_url location of the file to load.
	// -> RETURN: the loaded document.
	XMLDocument* loadFileToDoc(string file_url);

	// Find the next sibling (including the current node or not) having a given name.
	// ->  initialNode The root of search.
	// ->  name to search for.
	// ->  inclusive Includes the initialNode or not
	// -> RETURN:  The next sibling having the given name.
	XMLNode* getNextSibling(XMLNode* initialNode, string name, bool inclusive);

	// Find the first child having a given name
	// ->  node Parent node.
	// ->  name Name of the child.
	// -> RETURN:  The first child having the given name.
	XMLNode* getFirstChild(XMLNode* node, string name);

	//  Find the first federation node in a tree given by its root
	// ->  root of the tree to search.
	// -> RETURN: The first federation found.
	XMLNode* getFirstFederation(XMLNode* root);

	//Returns the federate corresponding to the provided id.
	// -> root of the document to search in.
	// -> id of the Federate to search (example "MAIN_CONTROLLER")
	XMLNode* getFirstFederate(XMLNode* root, string id);

	// Returns the value of the parameter, in text format .
	// -> doc The document where to search for the value.
	// ->  path Contains the name of each node in descendant order, 
	//          starting with the name of the federate and finishing with the
	//          name of the parameter.
	// -> RETURN: The value of the parameter in text format
	const char* getTextValue(XMLDocument* doc, vector<string> path);

	// Returns the value of the parameter, casted to int.
	// -> doc The document where to search for the value.
	// ->  path Contains the name of each node in descendant order, 
	//          starting with the name of the federate and finishing with the
	//          name of the parameter.
	// -> RETURN: The value of the parameter casted to int .
	int getIntValue(XMLDocument* doc, vector<string> path);


	// Returns the value of the parameter, casted to double.
	// -> doc The document where to search for the value.
	// ->  path Contains the name of each node in descendant order, 
	//          starting with the name of the federate and finishing with the
	//          name of the parameter.
	// -> RETURN: The value of the parameter casted to double .
	double getDoubleValue(XMLDocument* doc, vector<string> path);


	// Returns the Federation Name
	string getFederationName(XMLDocument* doc);

	// Returns the name of the Federate specified by its id (example : "MAIN_CONTROLLER").
	string getFederateName(XMLDocument* doc, string id);


	// Returns the name of the Federation file
	string getFedFile(XMLDocument* doc);

}

#endif // __SDSE_PARAMETERS_PARSER_HH_DEF__
