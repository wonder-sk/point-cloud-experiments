#ifndef POINTCLOUDRENDERER_H
#define POINTCLOUDRENDERER_H

#include <QImage>


struct PointCloudRenderer
{
  QImage img;
  int imgW, imgH;
  qint32 xmin,ymin,xmax,ymax;  // view extent
  qint32 zmin, zmax;

  // some stats
  int nodesDrawn = 0;
  int pointsDrawn = 0;
  int drawingTime = 0;  // in msec

  void drawData( const QVector<qint32> &data );
};


#endif // POINTCLOUDRENDERER_H
