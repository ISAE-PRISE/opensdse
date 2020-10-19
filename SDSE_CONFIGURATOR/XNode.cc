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

#include "XNode.hh"
#include <stdlib.h>
#include <stdio.h>
#include <sstream>

using namespace std;
using namespace tinyxml2;

// ----------------------------------------------------------------------------
// Constructor
XNode::XNode(XMLElement *element):
    validator(NULL)
{
    mElement = element;

    if (element->Attribute("federateName")){
        nodeType = Federate;
    } else if (element->FirstChildElement("value")){
        nodeType = Param;
    } else if (element->GetText()){
        nodeType = SimpleParam;
    } else {
        nodeType = NodeOnly;
    }

    if ((nodeType==Param) && element->FirstChildElement("possibleValues")){
        size_t pos = 0;
        string delimiter(",");
        string values(element->FirstChildElement("possibleValues")->GetText());
        if (values.find("[[")!=string::npos) {
            // An interval of integers is provided : "[[intMin,intMax]]"
            contentType = Integer;
            values = values.substr(2,values.length()-4);
            pos = values.find(delimiter);
            intMin = stoi(values.substr(0,pos));
            intMax = stoi(values.erase(0,pos+delimiter.length()));
            validator = new QIntValidator(intMin,intMax);
        } else if (values.find("[")!=string::npos){
            // An interval of double is provided : "[dMin,dMax]"
            contentType = Double;
            values = values.substr(1,values.length()-2);
            pos = values.find(delimiter);
            dMin = stod(values.substr(0,pos));
            dMax = stod(values.erase(0,pos+delimiter.length()));
            validator = new CustomValidator(dMin,dMax,2);
            // Set the rules to only accept "." as separator and not ","
            QLocale loc=QLocale::c();
            loc.setNumberOptions(QLocale::RejectGroupSeparator | QLocale::OmitGroupSeparator);
            validator->setLocale(loc);
        } else if (element->FirstChildElement("possibleValues")->Attribute("type","enum")){
            contentType = Enum;
            while ((pos = values.find(delimiter))!=string::npos){
                possibleValues << QString::fromStdString(values.substr(0,pos));
                values.erase(0,pos+delimiter.length());
            }
            possibleValues << QString::fromStdString(values); // Last element of the enum
        }
    }else {
        contentType = Text;
    }
}

// ----------------------------------------------------------------------------
// Destructor
XNode::~XNode()
{

}

// ----------------------------------------------------------------------------
//
bool XNode::NoChildrenNode()
{
    if (mElement->NoChildren()||(nodeType==Param)){
        return true;
    } else {
        return false;
    }
}

// ----------------------------------------------------------------------------
//
XMLElement* XNode::FirstChildElement()
{
    return mElement->FirstChildElement();
}

// ----------------------------------------------------------------------------
//
bool XNode::hasValue()
{
    switch (nodeType)
    {
		case Federate:
		case Param:
		case SimpleParam:
			return true;
			break;
		default:
			return false;
			break;
    }
}

// ----------------------------------------------------------------------------
//
bool XNode::hasUnit()
{
    if (mElement->Attribute("unit"))
    {
        return true;
    } 
    else 
    {
        return false;
    }
}

// ----------------------------------------------------------------------------
//
bool XNode::isEnumeration()
{
    if (contentType==Enum)
    {
        return true;
    } 
    else 
    {
        return false;
    }
}

// ----------------------------------------------------------------------------
//
bool XNode::isConstrained()
{
    if (validator)
    {
        return true;
    } 
    else 
    {
        return false;
    }
}

// ----------------------------------------------------------------------------
//
const char* XNode::getName()
{
    return mElement->Value();
}

// ----------------------------------------------------------------------------
//
const char* XNode::getValue()
{
    switch (nodeType)
    {
		case Param:
			return mElement->FirstChildElement("value")->GetText();
			break;
		case SimpleParam:
			return mElement->GetText();
			break;
		case Federate:
			return mElement->Attribute("federateName");
			break;
		default:
			return NULL;
    }
}

// ----------------------------------------------------------------------------
//
const char* XNode::getUnit()
{
    if (const char* unit = mElement->Attribute("unit"))
    {
        return unit;
    } 
    else 
    {
        return "";
    }
}

// ----------------------------------------------------------------------------
//
QStringList XNode::getEnumValues()
{
    return possibleValues;
}

// ----------------------------------------------------------------------------
//
const QValidator *XNode::getValidator()
{
    return validator;
}

// ----------------------------------------------------------------------------
//
const char* XNode::getDocumentation()
{
    if (mElement->FirstChildElement("documentation"))
    {
        return mElement->FirstChildElement("documentation")->GetText();
    } 
    else if (nodeType==Federate)
    {
        return "Federate Name";
    } 
    else 
    {
        return "";
    }
}

// ----------------------------------------------------------------------------
//
QString XNode::getFullDocumentation()
{
    ostringstream doc;
    switch (contentType)
    {
		case Integer:
			doc << "Type : Integer" << endl;
			break;
		case Double:
			doc << "Type : Real" << endl;
			break;
		case Enum:
			doc << "Type : Enumeration" << endl;
			break;
		default:
			break;
    }
    if (isConstrained())
    {
        switch (contentType)
        {
			case Integer:
				doc << "Min : " << intMin << endl; // getUnit can be added if necessary
				doc << "Max : " << intMax << endl; // getUnit can be added if necessary
				break;
			case Double:
				doc << "Min : " << dMin << " " << getUnit() << endl;
				doc << "Max : " << dMax << " " << getUnit() << endl;
				break;
			default:
				break;
        }
    }
    if (contentType!=Text)
        doc << endl;
    doc << getDocumentation();
    return QString::fromStdString(doc.str());
}

// ----------------------------------------------------------------------------
//
void XNode::setValue(std::string value)
{
    switch (nodeType) 
    {
		case Param:
			mElement->FirstChildElement("value")->SetText(value.c_str());
			break;
		case SimpleParam:
			mElement->SetText(value.c_str());
			break;
		case Federate:
			mElement->SetAttribute("federateName",value.c_str());
			break;
		default:
			break;
    }
}
