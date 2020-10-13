#include "pointcloudrenderer.h"

#include <QTime>
#include <QtDebug>

#include "math.h"


void PointCloudRenderer::drawData( const QVector<qint32> &data )
{
  QTime tD;
  tD.start();
  QRgb* rgb = (QRgb*) img.bits();
  const qint32* ptr = data.constData();
  int count = data.count() / 3;
  for ( int i = 0; i < count; ++i )
  {
    qint32 ix = ptr[i*3+0];
    qint32 iy = ptr[i*3+1];
    qint32 iz = ptr[i*3+2];
    int imgx = round(double( ix - xmin ) / ( xmax - xmin ) * (imgW-1));
    int imgy = round(double( iy - ymin ) / ( ymax - ymin ) * (imgH-1));
    int imgz = round(double( iz - zmin ) / ( zmax - zmin ) * 155 );
    int c = 100+imgz;
    imgy = imgH - imgy - 1;  // Y in QImage is 0 at the top an increases towards bottom - need to invert it
    if ( imgx >= 0 && imgx < imgW && imgy >= 0 && imgy < imgH )
      rgb[imgx+imgy*imgW] = qRgb(c,c,c);
    //qDebug() << imgx << imgy << imgz;
    //drawImg.img.setPixelColor( imgx, imgy, QColor( c, c, c ) ); // Qt::yellow );
  }

  // stats
  ++nodesDrawn;
  pointsDrawn += count;
  int timeDraw = tD.elapsed();
  drawingTime += timeDraw;
  qDebug() << "time draw" << timeDraw << "ms";
}
