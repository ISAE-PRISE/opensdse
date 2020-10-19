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

#include <QApplication>
#include <QDesktopWidget>
#include "CockpitMainWindow.hh"
#include "CockpitFederateThreadHla1516e.hh"


// ----------------------------------------------------------------------------
// Instantiate the CockpitFederateThread and CockpitMainWindow,
// then link them (Qt signal/slot) to display information from the simulation.
// ----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    CockpitMainWindow w;
    CockpitMainWindow *win;
    win=&w;


    // screenGeometry(0) is full screen
    QDesktopWidget *desktop = QApplication::desktop();
    QRect rect = desktop->screenGeometry(0);
    w.move(rect.topLeft());

    if (rect.width() < 1900 || rect.height() < 1000) w.show();
    else w.showFullScreen();
    
	CockpitFederateThreadHla1516e thread(&w);
	CockpitFederateThreadHla1516e* th;
	th=&thread;
	CockpitMainWindow::connect(th, SIGNAL(gps_update_signal()), win, SLOT(new_data_for_GPS()));
	CockpitMainWindow::connect(th, SIGNAL(pfd_update_signal()), win, SLOT(new_data_for_PFD()));
	CockpitMainWindow::connect(th, SIGNAL(ecam_update_signal()), win, SLOT(new_data_for_ECAM()));
	thread.start();
	
    return a.exec();
}
