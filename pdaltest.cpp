#include <memory>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pdal/Options.hpp>

#include <pdal/Stage.hpp>
#include <pdal/Streamable.hpp>

#include <chrono>

#include <QTime>

class MyStage : public pdal::Streamable
{
public:
  std::string getName() const override { return "hello world"; }

  bool processOne(pdal::PointRef& point) override
  {
    ++i;
    if ( i % 1000000 == 0 )
      std::cout << "reading " << i << std::endl;

    using namespace pdal::Dimension;
    double x = point.getFieldAs<double>(Id::X);
    double y = point.getFieldAs<double>(Id::Y);
    double z = point.getFieldAs<double>(Id::Z);

    return true;
  }

  int i = 0;
};


void readWithStreaming(pdal::LasReader &las_reader)
{
  pdal::point_count_t streamLimit = 10000;
  std::unique_ptr<pdal::FixedPointTable> streamTablePtr(new pdal::FixedPointTable(streamLimit));

  MyStage ppp;
  ppp.setInput(las_reader);
  ppp.prepare(*streamTablePtr);
  ppp.execute(*streamTablePtr);
}


void readWithoutStreaming(pdal::LasReader &las_reader)
{
  pdal::PointTable table;
  las_reader.prepare(table);
  pdal::PointViewSet point_view_set = las_reader.execute(table);

  pdal::PointViewPtr point_view = *point_view_set.begin();
  pdal::Dimension::IdList dims = point_view->dims();

  for (pdal::PointId idx = 0; idx < point_view->size(); ++idx) {

      if ( idx % 1000000 == 0)
        std::cout << "reading " << idx << std::endl;

      using namespace pdal::Dimension;
      double x = point_view->getFieldAs<double>(Id::X, idx);
      double y = point_view->getFieldAs<double>(Id::Y, idx);
      double z = point_view->getFieldAs<double>(Id::Z, idx);

//        int return_no = point_view->getFieldAs<int>(Id::ReturnNumber, idx);
//        int n_returns = point_view->getFieldAs<int>(Id::NumberOfReturns, idx);
//        int point_class = point_view->getFieldAs<int>(Id::Classification, idx);
//        int red = point_view->getFieldAs<int>(Id::Red, idx);
//        int green = point_view->getFieldAs<int>(Id::Green, idx);
//        int blue = point_view->getFieldAs<int>(Id::Blue, idx);
//        double time = point_view->getFieldAs<double>(Id::GpsTime, idx);

  }
}

#include "indexedpointcloud.h"
#include "pointcloudrenderer.h"

#include <QtDebug>

QDebug operator<<(QDebug debug, const NodeID &n)
{
    QDebugStateSaver saver(debug);
    debug.nospace() << n.d << "-" << n.x << "-" << n.y << "-" << n.z;
    return debug;
}

QDebug operator<<(QDebug debug, const DataBounds &db)
{
    QDebugStateSaver saver(debug);
    debug/*.nospace()*/ << db.xmin << db.xmax << db.ymin << db.ymax << db.zmin << db.zmax;
    return debug;
}


void dumpHierarchy( IndexedPointCloud& pc, NodeID n = NodeID(0,0,0,0), int ident = 0 )
{
  qDebug() << QString( ident, ' ' ) << n;
  QList<NodeID> ch = pc.children( n );
  for ( auto n2 : ch )
  {
    dumpHierarchy( pc, n2, ident+1 );
    //qDebug() << n2 << " -> " << pc.children( n2 );
  }
}

QList<NodeID> traverseTree( IndexedPointCloud &pc, NodeID n, int maxDepth )
{
  QList<NodeID> nodes;
  nodes.append( n );

  for ( auto nn : pc.children( n ) )
  {
    if ( maxDepth > 0 )
      nodes += traverseTree( pc, nn, maxDepth - 1 );
  }

  return nodes;
}

void do_2d_rendering()
{
  // quick tests: rendering 7 nodes with ~366K points:
  // - total time with "laszip" ~630ms
  // - total time with "binary" ~180ms
  // - rendering itself is fast: ~12ms
  // - big difference in size: "laszip" ~82 MB  /  "binary" ~650 MB
  // - orig. LAZ  ~81 MB  /  equivalent LAS  ~484 MB

  IndexedPointCloud pc;
  //bool res = pc.load("/home/martin/tmp/las/entwine_26850-bin");  // test using "binary" encoding
  bool res = pc.load("/home/martin/tmp/las/entwine_26850");   // test using "laszip" encoding
  Q_ASSERT( res );

  QList<NodeID> lvl1 = pc.children( NodeID(0,0,0,0) );
  qDebug() << lvl1;
  Q_ASSERT( lvl1.count() == 4 );

  ScaleOffset so;
  DataBounds db;

  //dumpHierarchy( pc );
  NodeID n0(0,0,0,0);
  NodeID n1(1,0,0,0);
  NodeID n2(2,2,2,0);
  NodeID n4( 4, 6, 7, 1 );
  NodeID n5( 5, 14, 14, 3 );
  QVector<qint32> nodeData = pc.nodePositionDataAsInt32( n1, so, db );

  qDebug() << pc.nodeBounds( n0 );
  qDebug() << pc.nodeBounds( n1 );

  //for ( auto n : lvl1 )
  //  qDebug() << n << " -> " << pc.children( n );

  PointCloudRenderer drawImg;
  drawImg.imgW = 256;
  drawImg.imgH = drawImg.imgW;
  drawImg.img = QImage( drawImg.imgW, drawImg.imgH, QImage::Format_RGB32 );
  drawImg.img.fill( Qt::black );
  drawImg.xmin = db.xmin;
  drawImg.xmax = db.xmax;
  drawImg.ymin = db.ymin;
  drawImg.ymax = db.ymax;
  drawImg.zmin = db.zmin;
  drawImg.zmax = db.zmax;

  // add some margins
  int dx = drawImg.xmax - drawImg.xmin;
  drawImg.xmin -= dx/10;
  drawImg.xmax += dx/10;
  int dy = drawImg.ymax - drawImg.ymin;
  drawImg.ymin -= dy/10;
  drawImg.ymax += dy/10;

  QTime t;
  t.start();

  // TODO: traverse with spatial filter

  QList<NodeID> nodes = traverseTree( pc, n1, 2 );
  qDebug() << nodes;

  // drawing
  for ( auto n : nodes )
  {
    drawImg.drawData( pc.nodePositionDataAsInt32( n, so, db ) );
  }

  qDebug() << "grand total incl loading " << t.elapsed() << "ms";

  qDebug() << "totals:" << drawImg.nodesDrawn << "nodes | " << drawImg.pointsDrawn << " points | " << drawImg.drawingTime << "ms";

  //drawData( drawImg, nodeData );

  drawImg.img.save( "/tmp/chmura.png" );
}

void simple_reading_test()
{
  std::chrono::time_point<std::chrono::system_clock> start, end;

  start = std::chrono::system_clock::now();

  std::string filename = "/home/martin/tmp/las/26850_12580.laz";
  //std::string filename =  "/home/martin/tmp/las/19_VelkaFatra_18_353893_5427743_j_c.laz";
  //std::string filename2 = "/home/martin/tmp/las/19_VelkaFatra_18_362680_5421014_c_c.laz";

  // timing:
  // ~10s to read 81MB LAZ file (~15M points)
  // ~20s to read 200MB LAZ file
  // ~40s to read 41M points ... ~1s / million points

  // ~12s to read 81MB LAZ file (15M points)
  // ~10s to read it in streaming mode

  pdal::Option las_opt("filename", filename);
  pdal::Options las_opts;
  las_opts.add(las_opt);

  pdal::LasReader las_reader;
  las_reader.setOptions(las_opts);

  std::cout << "streamable " << las_reader.pipelineStreamable() << std::endl;
  std::cout << "starting read..." << std::endl;

  readWithStreaming(las_reader);
  //readWithoutStreaming(las_reader);

  std::cout << "read point set." << std::endl;

  pdal::LasHeader las_header = las_reader.header();

  double scale_x = las_header.scaleX();
  double scale_y = las_header.scaleY();
  double scale_z = las_header.scaleZ();
  std::cout << "scale " << scale_x << " " << scale_y << " " << scale_z << std::endl;

  double offset_x = las_header.offsetX();
  double offset_y = las_header.offsetY();
  double offset_z = las_header.offsetZ();
  std::cout << "offset " << offset_x << " " << offset_y << " " << offset_z << std::endl;

  double xmin = las_header.minX();
  double xmax = las_header.maxX();
  double ymin = las_header.minY();
  double ymax = las_header.maxY();
  std::cout << "extent " << xmin << " " << ymin << " " << xmax << " " << ymax << std::endl;

  unsigned int n_features = las_header.pointCount();
  std::cout << "points " << n_features << std::endl;

  std::cout << "wkt " << las_reader.getSpatialReference().getWKT() << std::endl;

  bool have_time = las_header.hasTime();
  bool have_color = las_header.hasColor();
  std::cout << "time " << have_time << std::endl;
  std::cout << "color " << have_color << std::endl;

  end = std::chrono::system_clock::now();
  std::cout << "elapsed time: " << std::chrono::duration<double>(end - start).count() << "s\n";
}


int main()
{
  do_2d_rendering();

  //simple_reading_test();

  return 0;
}
