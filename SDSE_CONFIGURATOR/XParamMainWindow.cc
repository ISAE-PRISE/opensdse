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

#include "XParamMainWindow.hh"
#include "ui_XParamMainWindow.h"

// ----------------------------------------------------------------------------
// Constructor
XParamMainWindow::XParamMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::XParamMainWindow)
{
    // Create the Window
    ui->setupUi(this);

    // Create the model
    model = new QStandardItemModel;
    ui->mainTree->setModel(model);

    // Load the XML file into a XMLDoc
    bool xmlLoaded = false;
    while (!xmlLoaded)
    {
		try 
		{
			params = new XMLDoc;
			// Build the model with the XML content
			createTree();
			xmlLoaded = true;
		}
		catch (string const& e)
		{
			QString file_url = QFileDialog::getOpenFileName(this, "Open File", QString(), "XML (*.xml)");
			try 
			{
				params = new XMLDoc(file_url.toStdString());
				// Build the model with the XML content
				createTree();
				xmlLoaded = true;
			}
			catch (string const& e)
			{
				QMessageBox::information(this, "No File Opened", "No suitable file was provided, please select a suitable file to open this program.");
			}
		}
    }

    // Expand the tree
    ui->mainTree->expandToDepth(1);

    // Signal to Slots connections
    // The user wants to save its changes
    connect(ui->mainTree->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)), this, SLOT(updateGUI(QModelIndex)));
    connect(ui->changeValueButton, SIGNAL(clicked()), this, SLOT(updateItemValue()));
    connect(ui->valueField, SIGNAL(returnPressed()), this, SLOT(updateItemValue()));
    connect(ui->valueField_1, SIGNAL(returnPressed()), this, SLOT(updateItemValue()));
    // Detection of modifications to notify that the parameter is not saved
    connect(ui->valueField, SIGNAL(textEdited(QString)), this, SLOT(modifySavedIndicator(QString)));
    connect(ui->valueField_1, SIGNAL(textEdited(QString)), this, SLOT(modifySavedIndicator(QString)));
    connect(ui->valueBox, SIGNAL(highlighted(QString)), this, SLOT(modifySavedIndicator(QString)));
}

// ----------------------------------------------------------------------------
// Destructor
XParamMainWindow::~XParamMainWindow()
{
    delete ui;
    delete model;
    delete params;
}

// ----------------------------------------------------------------------------
//
void XParamMainWindow::createTree()
{
    // Get the first federation
    XMLElement *xmlnode = params->getFederation();
    // Create the root item of the model
    QStandardItem *item = new QStandardItem(xmlnode->Value());
    item->setEditable(false);
    // Create and attach the XNode to the item
    XNode* node = new XNode(xmlnode);
    item->setData(QVariant::fromValue(node));
    // Add the item to the model
    model->appendRow(item);

    // Recursively add children to the tree
    addChildrenToTree(item);
}

// ----------------------------------------------------------------------------
//
void XParamMainWindow::addChildrenToTree(QStandardItem *currentItem)
{
    XNode* parentNode = currentItem->data(Qt::UserRole+1).value<XNode*>();
    if (parentNode->NoChildrenNode())
    {
        return;
    } 
    else 
    {
        XMLElement* elem = parentNode->FirstChildElement();
        QString name;
        QStandardItem* item;
        while (elem!=NULL)
        {
            name = elem->Value();
            // Create the new model item
            item = new QStandardItem(name);
            item->setEditable(false);
            // Create and add the XNode to the model item
            XNode* node = new XNode(elem);
            item->setData(QVariant::fromValue(node));
            // Add the item to the model
            currentItem->appendRow(item);
            // Look for children of this node
            addChildrenToTree(item);
            elem = elem->NextSiblingElement();
        }
    }
}

// ----------------------------------------------------------------------------
//
bool XParamMainWindow::complyToConstraint(XNode *node) 
{
    if (node->isConstrained()) 
    {
        QString value;
        switch (ui->entryField->currentIndex()){
        case 0: // Text
            value = ui->valueField->text();
            break;
        case 1: // Param with Unit
            value = ui->valueField_1->text();
            break;
        }
        int pos = 0;
        QValidator::State state = params->getSelectedNode()->getValidator()->validate(value,pos);
        if (state==QValidator::Invalid || state==QValidator::Intermediate)
        {
            return false;
        }
        else 
        {
            return true;
        }
    } 
    else 
    {
        return true;
    }
}

// ----------------------------------------------------------------------------
//
void XParamMainWindow::updateGUI(QModelIndex index)
{
    ui->saveStatusLabel->setText("<font color='green'>Saved</font>");
    XNode* node = index.data(Qt::UserRole+1).value<XNode*>();
    if (node->hasValue())
    {
        params->setSelectedNode(node);
        // Update common properties
        ui->attributeLabel->setText(node->getName());
        ui->documentationField->setText(node->getFullDocumentation());
        // Enable interractions previously disabled if no parameter was selected
        ui->valueField->setEnabled(true);
        ui->changeValueButton->setEnabled(true);

        if (node->hasUnit())
        {
            // Display page with unit field
            ui->entryField->setCurrentIndex(1);
            ui->unitLabel->setText(node->getUnit());
            ui->valueField_1->setText(node->getValue());
            if (node->isConstrained())
            {
                ui->valueField_1->setValidator(node->getValidator());
            }
        } 
        else if (node->isEnumeration())
        {
            // Display page with value Box
            ui->entryField->setCurrentIndex(2);
            ui->valueBox->clear();
            ui->valueBox->addItems(node->getEnumValues());
            ui->valueBox->setCurrentIndex(atoi(node->getValue()));
        } 
        else 
        {
            // Display page with text field only
            ui->entryField->setCurrentIndex(0);
            ui->valueField->setText(node->getValue());
            if (node->isConstrained())
            {
                ui->valueField->setValidator(node->getValidator());
            }
        }
    } 
    else 
    {
        // Disable the right page
        ui->entryField->setCurrentIndex(0);
        ui->attributeLabel->setText("No Parameter selected");
        ui->saveStatusLabel->setText("");
        ui->valueField->setText("");
        ui->documentationField->setText("");
        ui->valueField->setEnabled(false);
        ui->changeValueButton->setEnabled(false);
    }
}

// ----------------------------------------------------------------------------
//
void XParamMainWindow::updateItemValue()
{
    switch (ui->entryField->currentIndex())
    {
		case 0: // Text
			if (complyToConstraint(params->getSelectedNode()))
			{
				params->getSelectedNode()->setValue(ui->valueField->text().toStdString());
				ui->saveStatusLabel->setText("<font color='green'>Saved</font>");
			} 
			else 
			{
				QMessageBox::critical(this, "Invalid Argument.",
									  "The argument is invalid, it is not within the allowed range or has an invalid syntax.");
				ui->valueField->setText(params->getSelectedNode()->getValue());
			}
			break;
		case 1: // Param with Unit
			if (complyToConstraint(params->getSelectedNode()))
			{
				params->getSelectedNode()->setValue(ui->valueField_1->text().toStdString());
				ui->saveStatusLabel->setText("<font color='green'>Saved</font>");
			} 
			else 
			{
				QMessageBox::critical(this, "Invalid Argument.",
									  "The argument is invalid, it is not within the allowed range or has an invalid syntax.");
				ui->valueField_1->setText(params->getSelectedNode()->getValue());
			}
			break;
		case 2: // Enumeration
			params->getSelectedNode()->setValue(std::to_string(ui->valueBox->currentIndex()));
			ui->saveStatusLabel->setText("<font color='green'>Saved</font>");
			break;
    }
    params->saveToFile();
}

// ----------------------------------------------------------------------------
//
void XParamMainWindow::modifySavedIndicator(QString value)
{
    if ( value != params->getSelectedNode()->getValue())
    {
        ui->saveStatusLabel->setText("<font color='red'>Not Saved</font>");
    }
}
