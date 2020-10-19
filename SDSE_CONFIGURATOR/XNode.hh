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


#ifndef __XNODE_HH_DEF__
#define __XNODE_HH_DEF__

#include "tinyxml2.h"
#include <stdlib.h>
#include <QMetaType>
#include <QString>
#include <QStringList>
#include <QValidator>
#include "CustomValidator.hh"

// ----------------------------------------------------------------------------
// Wrapper for XMLElement nodes that allows to collect and store more informations
// on the content.
// ----------------------------------------------------------------------------
class XNode
{  
	public:

		XNode(tinyxml2::XMLElement* element);
		~XNode();

		// Return true if the XNode has no children which means : 
		// same as in XML standard except for Params that never have children nodes.
		bool NoChildrenNode();

		// Wrapper for the same function applied to the XMLElement contained by the XNode.
		tinyxml2::XMLElement* FirstChildElement();

		// Return true if the node has a value.
		bool hasValue();

		// Return true if the node has a unit.
		bool hasUnit();

		// Return true if the node is an enumeration (C enum).
		bool isEnumeration();

		// Return true if the parameter has constraints, 
		// these must have been specified in the possibleValues sub-element of the parameter.
		bool isConstrained();

		// Return the name of the parameter.
		const char* getName();

		// Return the value of the parameter.
		const char* getValue();

		// Return the unit of the parameter.
		const char* getUnit();

		// Return the list of possible values of the parameter 
		// when it is an enumeration.
		QStringList getEnumValues();

		// Return a QValidator mainly used to constrain user's input (in a QLineEdit).
		// The constraints are read from the xml file, see XNode::isConstrained for the
		// format used.
		const QValidator* getValidator();
        
        // Return the text contained in the <documentation> xml element.
		const char* getDocumentation();

		// Return a more complete documentation (containing the one from getDocumentation)
		// with added information on the type when available
		// (extrapolated from \p possibleValues for the time being), as well as
		// min and max values.
		QString getFullDocumentation();

		// Sets the value of the XMLElement contained in the XNode.
		void setValue(std::string value);

	private:

		// pointer to the element in the DOM, containing all informations.
		tinyxml2::XMLElement* mElement;
		enum NodeType {NodeOnly,Federate,Param,SimpleParam} nodeType;
		enum ContentType {Text,Integer,Double,Enum} contentType;
		QStringList possibleValues;
		QValidator* validator;
		int intMin;
		int intMax;
		double dMin;
		double dMax;
};

// Declare XNode* as a metatype to include it
// in QVariant object, needed to provide custom data
// to the QStandardItem used in the model */
Q_DECLARE_METATYPE(XNode*);

#endif // __XNODE_HH_DEF__
