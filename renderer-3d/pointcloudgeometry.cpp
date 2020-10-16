#include "pointcloudgeometry.h"

#include <Qt3DRender/QAttribute>
#include <QFile>

#include <QElapsedTimer>



PointCloudGeometry::PointCloudGeometry( QStringList filenames, Qt3DCore::QNode *parent )
  : Qt3DRender::QGeometry( parent )
  , mPositionAttribute( new Qt3DRender::QAttribute( this ) )
  , mClassAttribute( new Qt3DRender::QAttribute( this ) )
  , mVertexBuffer( new Qt3DRender::QBuffer( Qt3DRender::QBuffer::VertexBuffer, this ) )
{

  mPositionAttribute->setAttributeType( Qt3DRender::QAttribute::VertexAttribute );
  mPositionAttribute->setBuffer( mVertexBuffer );
  mPositionAttribute->setVertexBaseType( Qt3DRender::QAttribute::Float );
  mPositionAttribute->setVertexSize( 3 );
  mPositionAttribute->setName( Qt3DRender::QAttribute::defaultPositionAttributeName() );
  mPositionAttribute->setByteOffset( 0 );
  mPositionAttribute->setByteStride( 16 );

  mClassAttribute->setAttributeType( Qt3DRender::QAttribute::VertexAttribute );
  mClassAttribute->setBuffer( mVertexBuffer );
  mClassAttribute->setVertexBaseType( Qt3DRender::QAttribute::Float );
  mClassAttribute->setVertexSize( 1 );
  mClassAttribute->setName( "cls" );
  mClassAttribute->setByteOffset( 12 );
  mClassAttribute->setByteStride( 16 );

  addAttribute( mPositionAttribute );
  addAttribute( mClassAttribute );

  // TODO: how long does it take to read file with LAZ data?

  ///

  std::vector<Point3D> vertices;
  std::vector<char> classes;

  for ( auto filename : filenames )
  {
    QFile f(filename);
    bool res = f.open(QIODevice::ReadOnly);
    Q_ASSERT(res);
    char ptData[46];
    int count = 0;
    QMap<char, int> histogram;
    while (!f.atEnd())
    {
      f.read(ptData, 46);
      qint32 x = *(double*)(ptData);
      qint32 y = *(double*)(ptData+8);
      qint32 z = *(double*)(ptData+16);
      char cls = ptData[30];
      vertices.push_back( Point3D( x, y, z ) );
      classes.push_back( cls );
      ++count;
    }
    qDebug() << "point count:" << count;
  }

  makeVertexBuffer( vertices, &classes );
}

  /*
  class 2  1892114
  class 3   684757
  class 4   537068
  class 5  6222985
  class 6     3374
  class 7     6566
  class 12 4578351
  class 15     491
  class 17     276
  */

  /*
  Standard LIDAR classes:
    0 Created, never classified
    1 Unclassified
    2 Ground
    3 Low Vegetation
    4 Medium Vegetation
    5 High Vegetation
    6 Building
    7 Low Point (noise)
    8 Model Key-point (mass point)
    9 Water
    10 Reserved for ASPRS Definition
    11 Reserved for ASPRS Definition
    12 Overlap Points
    13-31 Reserved for ASPRS Definition
  */

void PointCloudGeometry::makeVertexBuffer( std::vector<Point3D> &vertices, std::vector<char> *classes )
{
  float scale = 0.00025;

  QByteArray vertexBufferData;
  vertexBufferData.resize( vertices.size() * 4 * sizeof( float ) );
  float *rawVertexArray = reinterpret_cast<float *>( vertexBufferData.data() );
  int idx = 0;
  int i = 0;
  for ( const auto &v : vertices )
  {
    rawVertexArray[idx++] = v.x * scale;
    rawVertexArray[idx++] = v.z * scale;
    rawVertexArray[idx++] = v.y * scale;
    rawVertexArray[idx++] = ( classes ? (*classes)[i++] :  2 );
  }

  mVertexCount = vertices.size();
  mVertexBuffer->setData( vertexBufferData );

}

int PointCloudGeometry::count()
{
  return mVertexCount;
}
