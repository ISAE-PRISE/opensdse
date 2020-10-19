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


#ifndef __XPARAM_MAIN_WINDOW_HH_DEF__
#define __XPARAM_MAIN_WINDOW_HH_DEF__

#include <QMainWindow>
#include <QMessageBox>
#include <QStandardItemModel>
#include <QModelIndexList>
#include <QItemSelectionModel>
#include <QItemSelection>
#include <QValidator>
#include <QLineEdit>
#include <QFileDialog>
#include <stdlib.h>

#include "XMLDoc.hh"

namespace Ui 
{
	class XParamMainWindow;
}

// GUI Window and model representing a XML file.
// Parse the XML file to obtain an internal representation (XMLDoc),
// create a QStandardItemModel to display the content of the XML file in a TreeView.
// Allow to modify values of parameters with the GUI.
class XParamMainWindow : public QMainWindow
{
    Q_OBJECT

	public:

		explicit XParamMainWindow(QWidget *parent = 0);
		~XParamMainWindow();

		// Create a tree out of the XML document, and complete the model 
		// with a QStandardItem for each XMLElement encountered
		void createTree();

		// Add all children Elements of the currentItem to the QStandardItem model.
		// This function is designed to be called recursively.
		void addChildrenToTree(QStandardItem *currentItem);

		// Check wether the value contained in the QLineEdit field displayed 
		// by the GUI comply to the constraints of the
		// param represented by the XNode provided in the call.
		bool complyToConstraint(XNode *node);

	public slots:

		// Update the right pane of the GUI depending on the
		// selected Element in the treeView.
		void updateGUI(QModelIndex index);

		
		//Update the value of the item selected, and save
		//it to the XML file on disk.
		void updateItemValue();

		// Updates the Saved Indicator label in the GUI.
        // Check the value provided against the one stored in the 
        // XML file and update the indicator to "Not Saved" if different.
		void modifySavedIndicator(QString value);

	private:
		Ui::XParamMainWindow *ui;
		XMLDoc *params;
		QStandardItemModel *model;
};

#endif // __XPARAM_MAIN_WINDOW_HH_DEF__
