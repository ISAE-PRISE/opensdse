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

#include "SdseParametersParser.hh"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#define FEDERATION_TAG "federation"
#define FEDERATION_NAME_TAG "federationName"
#define FEDERATE_NAME_TAG "federateName"
#define FEDERATION_FILE_TAG "fedFile"
#define DEBUG false

using namespace tinyxml2;
using std::cout;
using std::endl;


/////////////////////////////////
// TINY XML 2 SPECIFIC METHODS //
/////////////////////////////////

namespace parser
{

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

XMLDocument* loadFileToDoc(string file_url){
    XMLDocument* xml_parameters = new XMLDocument;
    if ( xml_parameters->LoadFile(file_url.c_str()) != 0 ) {
        std::cerr << "Loading of \"" << file_url;
        std::cerr << "\" failed." << endl;
        std::cerr << "Check the location of the file." << endl;
        throw string("File : \"" + file_url + "\" not found ");
    }
    return xml_parameters;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

XMLNode* getNextSibling(XMLNode* initialNode, string name, bool inclusive){

	XMLNode* node = initialNode;

	// if initial node is not included, skip it
	if(!inclusive)
		node = node->NextSibling();

	// find the node name
	const char* value = node->Value();

	// while the node name is not the given one
	while(strcmp(value, name.c_str())) {

		// find next sibling
		node = node->NextSibling();

		// if there is no other sibling, return NULL
		if(node==NULL)
			return NULL;

		// else update node name
		value = node->Value();
	}
	// return the found node
	return node;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

XMLNode* getFirstChild(XMLNode* node, string name){
	// find the potential first child
	if(node == NULL)
		return NULL;	
	XMLNode* child = node->FirstChild();
	// if the node has no child, return NULL
	if(child==NULL)
		return NULL;
	// find the first sibling (including that child) having the given name
    return getNextSibling(child, name.c_str(), true);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

XMLNode* getFirstFederation(XMLNode* root){

	// nodes to be browsed 
	XMLNode* federation;

	// find federation node (can be root)
    federation = getNextSibling(root, (char*) FEDERATION_TAG, true);
    if (DEBUG)
	    printf("federation found: %s\n", federation->Value()); //federation

	return federation;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

XMLNode* getFirstFederate(XMLNode* root, string id){

	// nodes to be browsed
	XMLNode* federation;
	XMLNode* federate;

	// find federation node (can be root)
    federation = getFirstFederation(root);
    federate = getFirstChild(federation, id);
    if (DEBUG)
	    printf("federate found: %s\n", federate->ToElement()->Attribute("id")); //federate

	return federate;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

const char* getTextValue(XMLDocument* doc, vector<string> path){
  XMLNode* root = doc->FirstChild();
    XMLNode* current_node = getFirstFederation(root);
    for(unsigned long i = 0 ; i < path.size(); i++){
        current_node = getFirstChild(current_node, path[i]);
	}
    XMLElement *param = current_node->ToElement();
    if (param->FirstChildElement("value")){
        param = param->FirstChildElement("value");
    }
    return param->GetText();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

int getIntValue(XMLDocument* doc, vector<string> path){
  return atoi(getTextValue(doc, path));
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

double getDoubleValue(XMLDocument* doc, vector<string> path){
  return atof(getTextValue(doc, path));
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

string getFederationName(XMLDocument* doc){
  XMLNode* fed = getFirstFederation(doc->FirstChild());
  return getFirstChild(fed,FEDERATION_NAME_TAG)->ToElement()->FirstChildElement("value")->GetText();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

string getFederateName(XMLDocument* doc, string id){
  return getFirstFederate(doc->FirstChild(), id)->ToElement()->Attribute(FEDERATE_NAME_TAG);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//

string getFedFile(XMLDocument* doc){
	XMLNode* fed = getFirstFederation(doc->FirstChild());
  return getFirstChild(fed,FEDERATION_FILE_TAG)->ToElement()->FirstChildElement("value")->GetText();
}
}
