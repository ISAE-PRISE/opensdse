#-------------------------------------------------
#
# Project created by QtCreator 2015-06-10T13:54:07
#
#-------------------------------------------------

QT       += core gui

CONFIG += c++11

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = XParam
TEMPLATE = app

INCLUDEPATH += Includes

SOURCES += Sources/main.cc\
    Sources/XParamMainWindow.cc \
    Sources/XMLDoc.cc \
    Sources/XNode.cc \
    Sources/SdseParametersParser.cc \
    Sources/tinyxml2.cpp

HEADERS  += Includes/XParamMainWindow.hh \
    Includes/XMLDoc.hh \
    Includes/XNode.hh \
    Includes/CustomValidator.hh \
    Includes/SdseParametersParser.hh \
    Includes/tinyxml2.h \

FORMS    += Sources/XParamMainWindow.ui
