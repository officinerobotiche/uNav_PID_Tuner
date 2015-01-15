message("Added qcustomplot")

QT += printsupport

PATH = $$PWD

INCLUDEPATH += $$PATH/include

HEADERS += \
    $$PATH/include/qcustomplot.h

SOURCES += \
    $$PATH/src/qcustomplot.cpp
