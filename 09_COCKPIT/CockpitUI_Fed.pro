#-------------------------------------------------
#
# Project created by QtCreator 2012-05-16T13:40:38
#
#-------------------------------------------------

QT       += core gui \
            network \
            webkit \
            svg

TARGET = CockpitUI_Fed
TEMPLATE = app


SOURCES +=\
    main.cc \
    CockpitMainWindow.cc \
    ECAM/ECAMwindow.cc \
    ECAM/N1display.cc \
    FCU/FCUwindow.cc \
    GPS/GPSwindow.cc \
    PFD/PFDwindow.cc


HEADERS  += \
    CockpitMainWindow.hh \
    ECAM/ECAMwindow.hh \
    ECAM/N1display.hh \
    FCU/FCUwindow.hh \
    GPS/GPSwindow.hh \
    PFD/PFDwindow.hh

FORMS    += \
    CockpitMainWindow.ui \
    ECAM/ECAMwindow.ui \
    FCU/FCUwindow.ui \
    GPS/GPSwindow.ui

RESOURCES += \
    Ressources.qrc \
    FCU/fcu_rc.qrc \
    GPS/gps_rc.qrc \
    PFD/pfd_rc.qrc \
    ECAM/ecam_rc.qrc
