#include "main_window.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication::setDesktopSettingsAware(false);
    QApplication a(argc, argv);
    Main_Window w;
    w.show();

    return a.exec();
}
