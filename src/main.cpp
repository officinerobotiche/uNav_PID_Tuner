#include "mainwindow.h"
#include <QApplication>
#include <QFile>
#include <QLocale>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

//    // >>>>> Load an application style
//    QFile styleFile( ":/stylesheet/darkgreen.qss" );
//    styleFile.open( QFile::ReadOnly );
//    // <<<<< Load an application style

//    // >>>>> Apply the loaded stylesheet
//    QString style( styleFile.readAll() );
//    a.setStyleSheet( style );
//    // <<<<< Apply the loaded stylesheet

    QLocale::setDefault(QLocale(QLocale::English, QLocale::UnitedStates));

    MainWindow w;
    w.show();

    return a.exec();
}
