#include <vtkFileOutputWindow.h>

#include <QApplication>
#include <QSurfaceFormat>

#include "QVTKOpenGLNativeWidget.h"
#include "mainwindow.h"

int main(int argc, char *argv[]) {
    vtkNew<vtkFileOutputWindow> fileOutputWindow;
    fileOutputWindow->SetFileName("./data/output.txt");
    vtkOutputWindow::SetInstance(fileOutputWindow);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
