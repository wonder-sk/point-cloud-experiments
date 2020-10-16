#include <Qt3DQuickExtras/qt3dquickwindow.h>
#include <Qt3DQuick/QQmlAspectEngine>
#include <QGuiApplication>
#include <QQmlContext>
#include <QQmlEngine>
#include <QFileInfo>

#include "pointcloudgeometry.h"

int main(int argc, char* argv[])
{
    QGuiApplication app(argc, argv);

    QStringList filenames;
    filenames << ":/data/1-0-0-0.bin";
    filenames << ":/data/2-1-1-0.bin";
    PointCloudGeometry *pcg = new PointCloudGeometry(filenames);

    // TODO: limiting FPS does not work for me this way
//    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
//    qDebug() << "surface format" << format.swapBehavior() << format.swapInterval();
//    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
//    format.setSwapInterval(0);
//    QSurfaceFormat::setDefaultFormat( format );

    Qt3DExtras::Quick::Qt3DQuickWindow view;
    qDebug() << "real interval" << view.format().swapInterval();
    view.setTitle("Point Cloud 3D Renderer");
    view.resize(1600, 800);
    view.engine()->qmlEngine()->rootContext()->setContextProperty("_window", &view);
    view.engine()->qmlEngine()->rootContext()->setContextProperty("_bbg", pcg);
    view.setSource(QUrl("qrc:/main.qml"));
    view.show();

    return app.exec();
}
