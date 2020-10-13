#include "indexedpointcloud.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTime>
#include <QtDebug>

uint qHash( const NodeID &id )
{
  return id.d + id.x + id.y + id.z;
}


IndexedPointCloud::IndexedPointCloud()
{

}


bool IndexedPointCloud::load(const QString &directory)
{
    mDirectory = directory;
    QFile f(directory + "/ept.json");
    if ( !f.open(QIODevice::ReadOnly) )
      return false;
    QByteArray dataJson = f.readAll();
    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(dataJson, &err);
    if ( err.error != QJsonParseError::NoError )
      return false;

    QString dataType = doc["dataType"].toString();  // "binary" or "laszip"
    if ( dataType != "laszip" && dataType != "binary" )
      return false;

    mLaszip = (dataType == "laszip");

    QString hierarchyType = doc["hierarchyType"].toString();  // "json" or "gzip"
    if ( hierarchyType != "json" )
      return false;

    mSpan = doc["span"].toInt();

    QJsonArray bounds = doc["bounds"].toArray();
    if ( bounds.size() != 6 )
      return false;

    QJsonArray schemaArray = doc["schema"].toArray();

    for ( QJsonValue schemaItem : schemaArray )
    {
      QJsonObject schemaObj = schemaItem.toObject();
      QString name = schemaObj["name"].toString();
      QString type = schemaObj["type"].toString();
      int size = schemaObj["size"].toInt();

      float scale = 1.f;
      if ( schemaObj.contains("scale") )
        scale = schemaObj["scale"].toDouble();

      float offset = 0.f;
      if ( schemaObj.contains("offset") )
        offset = schemaObj["offset"].toDouble();

      if ( name == "X" )
      {
        mScaleOffset.ox = offset;
        mScaleOffset.sx = scale;
      }
      else if ( name == "Y" )
      {
        mScaleOffset.oy = offset;
        mScaleOffset.sy = scale;
      }
      else if ( name == "Z" )
      {
        mScaleOffset.oz = offset;
        mScaleOffset.sz = scale;
      }

      // TODO: can parse also stats: "count", "minimum", "maximum", "mean", "stddev", "variance"
    }

    // save mRootBounds

    // bounds (cube - octree volume)
    double xmin = bounds[0].toDouble();
    double ymin = bounds[1].toDouble();
    double zmin = bounds[2].toDouble();
    double xmax = bounds[3].toDouble();
    double ymax = bounds[4].toDouble();
    double zmax = bounds[5].toDouble();

    mRootBounds.xmin = (xmin - mScaleOffset.ox) / mScaleOffset.sx;
    mRootBounds.xmax = (xmax - mScaleOffset.ox) / mScaleOffset.sx;
    mRootBounds.ymin = (ymin - mScaleOffset.oy) / mScaleOffset.sy;
    mRootBounds.ymax = (ymax - mScaleOffset.oy) / mScaleOffset.sy;
    mRootBounds.zmin = (zmin - mScaleOffset.oz) / mScaleOffset.sz;
    mRootBounds.zmax = (zmax - mScaleOffset.oz) / mScaleOffset.sz;

    double dx = xmax-xmin, dy = ymax-ymin, dz = zmax-zmin;
    qDebug() << "lvl0 node size in CRS units:" << dx << dy << dz;   // all dims should be the same
    qDebug() << "res at lvl0" << dx/mSpan;
    qDebug() << "res at lvl1" << dx/mSpan/2;
    qDebug() << "res at lvl2" << dx/mSpan/4 << "with node size" << dx/4;

    // load hierarchy

    QFile fH(directory + "/ept-hierarchy/0-0-0-0.json");
    if ( !fH.open(QIODevice::ReadOnly))
      return false;

    QByteArray dataJsonH = fH.readAll();
    QJsonParseError errH;
    QJsonDocument docH = QJsonDocument::fromJson(dataJsonH, &errH);
    if ( errH.error != QJsonParseError::NoError )
      return false;

    QJsonObject rootHObj = docH.object();
    for ( auto it = rootHObj.constBegin(); it != rootHObj.constEnd(); ++it )
    {
      QString nodeIdStr = it.key();
      int nodePointCount = it.value().toInt();
      NodeID nodeId = NodeID::fromString(nodeIdStr);
      mHierarchy[nodeId] = nodePointCount;
    }

    return true;
}

QList<NodeID> IndexedPointCloud::children(const NodeID &n)
{
  Q_ASSERT( mHierarchy.contains( n ) );
  QList<NodeID> lst;
  int d = n.d + 1;
  int x = n.x * 2;
  int y = n.y * 2;
  int z = n.z * 2;

  for (int i = 0; i < 8; ++i )
  {
    int dx = i & 1, dy = !!(i & 2), dz = !!(i & 4);
    NodeID n2( d, x+dx, y+dy, z+dz );
    if ( mHierarchy.contains( n2 ) )
      lst.append( n2 );
  }
  return lst;
}

#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/Options.hpp>

#include <QtDebug>


class ReadDataStage : public pdal::Streamable
{
public:
  explicit ReadDataStage( QVector<qint32> &d ): data(d) {}

  std::string getName() const override { return "hello world"; }

  bool processOne(pdal::PointRef& point) override
  {

    using namespace pdal::Dimension;
    double x = point.getFieldAs<double>(Id::X);
    double y = point.getFieldAs<double>(Id::Y);
    double z = point.getFieldAs<double>(Id::Z);

    //qDebug() << xd << yd << zd << "---" << x << y << z;

    qint32 ix = round( (x - so.ox) / so.sx );
    qint32 iy = round( (y - so.oy) / so.sy );
    qint32 iz = round( (z - so.oz) / so.sz );

    //qDebug() << ix << iy << iz;

    data[i*3+0] = ix;
    data[i*3+1] = iy;
    data[i*3+2] = iz;

    double x2 = ix * so.sx + so.ox;
    Q_ASSERT( x == x2 );

    ++i;

    return true;
  }

  int i = 0;
  QVector<qint32> &data;

public:
  ScaleOffset so;
};



QVector<qint32> IndexedPointCloud::nodePositionDataAsInt32(const NodeID &n, ScaleOffset &so, DataBounds &db)
{
  Q_ASSERT( mHierarchy.contains( n ) );
  int count = mHierarchy[n];
  //qDebug() << "count" << count;


  if ( !mLaszip )
  {
    QString filename = QString( "%1/ept-data/%2.bin" ).arg( mDirectory ).arg( n.toString() );
    Q_ASSERT( QFile::exists( filename ) );

    QFile f( filename );
    bool r = f.open(QIODevice::ReadOnly);
    Q_ASSERT(r);

    // WHY??? per-record should be 18 based on schema, not 46
    int stride = 46; //18;
    int count = f.size() / stride;
    db.xmax = db.ymax = db.zmax = -999999999;
    db.xmin = db.ymin = db.zmin =  999999999;
    QVector<qint32> data( count * 3 );
    for ( int i = 0; i < count; ++i )
    {
      QByteArray bytes = f.read( stride );
      // WHY??? X,Y,Z are int32 values stored as doubles
      double *bytesD = (double*) bytes.constData();
      data[i*3+0] = (bytesD[0]);
      data[i*3+1] = (bytesD[1]);
      data[i*3+2] = (bytesD[2]);

      db.xmin = min( db.xmin, data[i*3+0]);
      db.xmax = max( db.xmax, data[i*3+0]);
      db.ymin = min( db.ymin, data[i*3+1]);
      db.ymax = max( db.ymax, data[i*3+1]);
      db.zmin = min( db.zmin, data[i*3+2]);
      db.zmax = max( db.zmax, data[i*3+2]);
    }
    return data;
  }


  QString filename = QString( "%1/ept-data/%2.laz" ).arg( mDirectory ).arg( n.toString() );
  Q_ASSERT( QFile::exists( filename ) );

  pdal::Option las_opt("filename", filename.toStdString() );
  pdal::Options las_opts;
  las_opts.add(las_opt);

  pdal::LasReader las_reader;
  las_reader.setOptions(las_opts);

  QVector<qint32> data( count * 3 );

  pdal::point_count_t streamLimit = 10000;
  std::unique_ptr<pdal::FixedPointTable> streamTablePtr(new pdal::FixedPointTable(streamLimit));

  ReadDataStage ppp( data );
  ppp.setInput(las_reader);
  ppp.prepare(*streamTablePtr);

  const pdal::LasHeader& h = las_reader.header();


  so.ox = h.offsetX(); so.oy = h.offsetY(); so.oz = h.offsetZ();
  so.sx = h.scaleX();  so.sy = h.scaleY();  so.sz = h.scaleZ();
  ppp.so = so;

  db.xmin = (h.minX() - h.offsetX())/h.scaleX();
  db.xmax = (h.maxX() - h.offsetX())/h.scaleX();
  db.ymin = (h.minY() - h.offsetY())/h.scaleY();
  db.ymax = (h.maxY() - h.offsetY())/h.scaleY();
  db.zmin = (h.minZ() - h.offsetZ())/h.scaleZ();
  db.zmax = (h.maxZ() - h.offsetZ())/h.scaleZ();

#if 0
  qDebug() << "min" << h.minX() << h.minY() << h.minZ();
  qDebug() << "max" << h.maxX() << h.maxY() << h.maxZ();

  qDebug() << "span x" << ( h.maxX() - h.minX() ) / so.sx;
  qDebug() << "span y" << ( h.maxY() - h.minY() ) / so.sy;
  qDebug() << "span z" << ( h.maxZ() - h.minZ() ) / so.sz;

  qDebug() << "scale" << h.scaleX() << h.scaleY() << h.scaleZ();
  qDebug() << "offset" << h.offsetX() << h.offsetY() << h.offsetZ();
#endif

  QTime t;
  t.start();
  ppp.execute(*streamTablePtr);
  qDebug() << "time read" << t.elapsed() << "ms";

  return data;
}

DataBounds IndexedPointCloud::nodeBounds(const NodeID &n)
{
  DataBounds db;
  int d = mRootBounds.xmax - mRootBounds.xmin;
  double dLevel = (double)d / pow( 2, n.d );

  db.xmin = round( mRootBounds.xmin + dLevel * n.x );
  db.xmax = round( mRootBounds.xmin + dLevel * (n.x + 1) );
  db.ymin = round( mRootBounds.ymin + dLevel * n.y );
  db.ymax = round( mRootBounds.ymin + dLevel * (n.y + 1) );
  db.zmin = round( mRootBounds.zmin + dLevel * n.z );
  db.zmax = round( mRootBounds.zmin + dLevel * (n.z + 1) );
  return db;
}
