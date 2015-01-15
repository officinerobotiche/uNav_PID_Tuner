#-------------------------------------------------
#
# Project created by QtCreator 2015-01-15T08:41:04
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

include(../../orblibcpp/orblibcpp/orblibcpp.pri)
include(3rdParty/qcustomplot-1.3.0/qcustomplot.pri)

TARGET = uNav_PID_Tuner
TEMPLATE = app

INCLUDEPATH += \
    include

SOURCES += \
    src/main.cpp \
    src/mainwindow.cpp

HEADERS  += \
    include/mainwindow.h

FORMS    += \
    gui/mainwindow.ui

RESOURCES += \
    gui/resources.qrc
