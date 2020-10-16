#ifndef POINTCLOUDGEOMETRY_H
#define POINTCLOUDGEOMETRY_H


#include <Qt3DRender/QGeometry>
#include <Qt3DRender/QBuffer>

#include <QVector3D>


struct Point3D {
  inline Point3D() {}
  inline Point3D( int _x, int _y, int _z ): x(_x), y(_y), z(_z) {}

  qint32 x; qint32 y; qint32 z;
};


class PointCloudGeometry : public Qt3DRender::QGeometry
{
  Q_OBJECT

  Q_PROPERTY(int count READ count NOTIFY countChanged)

public:
  PointCloudGeometry( QStringList filenames, Qt3DCore::QNode *parent = nullptr );

  //void setPoints( const QVector<QVector3D> &vertices );

  int count();

signals:
    void countChanged(int count);

private:
  void makeVertexBuffer( std::vector<Point3D> &vertices, std::vector<char> *classes );

private:
  Qt3DRender::QAttribute *mPositionAttribute = nullptr;
  Qt3DRender::QAttribute *mClassAttribute = nullptr;
  Qt3DRender::QBuffer *mVertexBuffer = nullptr;
  int mVertexCount = 0;
};


#endif // POINTCLOUDGEOMETRY_H
