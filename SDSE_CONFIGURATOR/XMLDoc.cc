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

#include "XMLDoc.hh"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#define FEDERATION_TAG "federation"
#define FEDERATION_NAME_TAG "federationName"
#define DEBUG false

using namespace tinyxml2;
using namespace parser;
using std::cout;
using std::endl;

// ----------------------------------------------------------------------------
// Constructor
XMLDoc::XMLDoc()
{
    mDocument = loadFileToDoc(DEFAULT_XML_PATH);
    mFileUrl = DEFAULT_XML_PATH;
}

// ----------------------------------------------------------------------------
// Constructor with file_url provided
XMLDoc::XMLDoc(string file_url)
{
    mDocument = loadFileToDoc(file_url);
    mFileUrl = file_url;
}

// ----------------------------------------------------------------------------
// Destructor
XMLDoc::~XMLDoc()
{
    delete mDocument;
}

// ----------------------------------------------------------------------------
//
const char* XMLDoc::getFederationName()
{
    return parser::getFederationName(mDocument).c_str();
}

// ----------------------------------------------------------------------------
//find the first federation node
XMLElement* XMLDoc::getFederation()
{
    return getFirstFederation(mDocument->FirstChild())->ToElement();
}

// ----------------------------------------------------------------------------
//
void XMLDoc::saveToFile(string file_url)
{
    mDocument->SaveFile(file_url.c_str());
}

// ----------------------------------------------------------------------------
//
void XMLDoc::saveToFile()
{
    mDocument->SaveFile(mFileUrl.c_str());
}
