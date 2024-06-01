#include "mainwindow.h"

#include <QApplication>
#include <vtkFileOutputWindow.h>
#include <QSurfaceFormat>
#include "QVTKOpenGLNativeWidget.h"

int main(int argc, char *argv[])
{
    vtkNew<vtkFileOutputWindow> fileOutputWindow;
    fileOutputWindow->SetFileName("output.txt");
    vtkOutputWindow::SetInstance(fileOutputWindow);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
