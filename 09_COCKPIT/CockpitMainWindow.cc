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

#include "CockpitMainWindow.hh"
#include "ui_CockpitMainWindow.h"
#include <iostream>
#include <QDebug>
#include <QDesktopWidget>

using namespace std;

// ----------------------------------------------------------------------------
// Constructor 
CockpitMainWindow::CockpitMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::CockpitMainWindow)
{
    ui->setupUi(this);
    fcuw=new FCUwindow(ui->FCU_widget);
    ecamw=new ECAMwindow(ui->ECAM_widget);

    QDesktopWidget *desktop = QApplication::desktop();
    QRect rect = desktop->screenGeometry(0);

    if (rect.width() < 1900 || rect.height() < 1000) 
    { // Screen too small -> add a scrollbar

        QWidget *ghostwidget = new QWidget();   // Ghost container widget
        ghostwidget->resize(QSize(1920,1080));

        QScrollArea *sa = new QScrollArea(this);// scrollable zone
            sa->setWidgetResizable( false );
            sa->setAcceptDrops(0);
            sa->setWidget(ghostwidget);
            sa->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
            sa->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
            sa->viewport()->setAutoFillBackground(true);

        ui->centralWidget->setParent(ghostwidget);
        this->setCentralWidget(sa);

    }

}

// ----------------------------------------------------------------------------
// Destructor 
CockpitMainWindow::~CockpitMainWindow()
{
  delete ui;
}
