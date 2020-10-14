#ifndef INDEXEDPOINTCLOUD_H
#define INDEXEDPOINTCLOUD_H

#include <QHash>
#include <QStringList>
#include <QVector>

struct NodeID
{
  NodeID(): d(-1), x(0), y(0), z(0) {}   // invalid node ID
  NodeID(int _d, int _x, int _y, int _z): d(_d), x(_x), y(_y), z(_z) {}

  bool isValid() const { return d >= 0; }

  bool operator==( const NodeID &other ) const { return d == other.d && x == other.x && y == other.y && z == other.z; }

  static NodeID fromString( const QString &str )
  {
    QStringList lst = str.split('-');
    if ( lst.count() != 4 )
      return NodeID();
    return NodeID( lst[0].toInt(), lst[1].toInt(), lst[2].toInt(), lst[3].toInt() );
  }

  QString toString() const
  {
    return QString( "%1-%2-%3-%4" ).arg( d ).arg( x ).arg( y ).arg( z );
  }

//  NodeID parentNode() const
//  {
//    return NodeID( d-1, x/2, y/2, z/2 );
//  }

  int d, x, y, z;
};

uint qHash( const NodeID &id );


class IndexedPointCloud;


// not really used now
class IndexedPointCloudNode
{
public:
  IndexedPointCloudNode( NodeID id, IndexedPointCloud *pc ): mId(id), mPC(pc) {}


  NodeID mId;
  IndexedPointCloud *mPC;  // not owned
};

#include <QImage>



struct ScaleOffset
{
  double sx = 1, sy = 1, sz = 1, ox = 0, oy = 0, oz = 0;
};

// what are the min/max to expect in the piece of data
struct DataBounds
{
  qint32 xmin,ymin,xmax,ymax, zmin, zmax;
};


class IndexedPointCloud
{
public:
  IndexedPointCloud();

  bool load(const QString &filename);

  IndexedPointCloudNode root() { return IndexedPointCloudNode( NodeID(0,0,0,0), this ); }

  QList<NodeID> children( const NodeID &n );

  QVector<qint32> nodePositionDataAsInt32( const NodeID &n, ScaleOffset &so, DataBounds &db );

  DataBounds nodeBounds( const NodeID &n );

private:
  QString mDirectory;
  QString mDataType;
  QHash<NodeID, int> mHierarchy;
  ScaleOffset mScaleOffset;  //!< scale and offset of our int32 coordinates compared to CRS coords
  DataBounds mRootBounds;  //!< bounds of the root node's cube (in int32 coordinates)
  int mSpan;  //!< number of points in one direction in a single node
};



#endif // INDEXEDPOINTCLOUD_H
