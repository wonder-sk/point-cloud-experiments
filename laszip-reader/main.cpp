
#include <iostream>

// laz-perf
#include "io.hpp"
#include "common/common.hpp"

// laszip
#include "/home/martin/inst/LASzip-inst/include/laszip/laszip_api.h"

laszip_POINTER laszip_reader;

void handleLaszip(int result)
{
    if (result)
    {
        char *buf;
        laszip_I32 i = laszip_get_error(laszip_reader, &buf);
        std::cerr << "ERR:"  << i << "  " << buf <<std::endl;
        //throwError(buf);
    }
}

void read_lazperf(std::string filename)
{
  std::ifstream file(filename, std::ios::binary);
  if (!file.good())
  {
    std::cerr << "Could not open file for reading: " << filename << std::endl;
    return;
  }

  auto start = common::tick();

  laszip::io::reader::file f(file);

  size_t count = f.get_header().point_count;
  char buf[256]; // a buffer large enough to hold our point

  for(size_t i = 0 ; i < count ; i ++) {
          f.readPoint(buf); // read the point out
          laszip::formats::las::point10 p = laszip::formats::packers<laszip::formats::las::point10>::unpack(buf);

          //std::cout << p.x << ", " << p.y << ", " << p.z << std::endl;
  }

  float t = common::since(start);
  std::cout << "LAZ-PERF Read through the points in " << t << " seconds." << std::endl;

}

void read_laszip(std::string filename)
{

  laszip_BOOL is_compressed = true;

  laszip_header* header;

  int load_i = laszip_load_dll();
  std::cerr << "load " << load_i << std::endl;

  laszip_U8 major, minor;
  laszip_U16 rev;
  laszip_U32 build;
  laszip_get_version(&major, &minor, &rev, &build);
  std::cerr << "version: " << (int)major << "." << (int)minor << "." << rev << " build " << build << std::endl;

  handleLaszip(laszip_create(&laszip_reader));

  laszip_BOOL request_reader = 1;
  handleLaszip(laszip_request_compatibility_mode(laszip_reader, request_reader));

  bool hasClassification = false;
  bool hasGpsTime = false;
  bool hasIntensity = false;
  bool hasNumberOfReturns = false;
  bool hasReturnNumber = false;
  bool hasPointSourceId = false;

  auto start = common::tick();

  {
          //laszip_BOOL is_compressed = iEndsWith(file, ".laz") ? 1 : 0;
          handleLaszip(laszip_open_reader(laszip_reader, filename.c_str(), &is_compressed));

          handleLaszip(laszip_get_header_pointer(laszip_reader, &header));

          long long npoints = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

	  laszip_point* point;
	  laszip_get_point_pointer(laszip_reader, &point);

	  for (int i = 0; i < npoints; i++) {
		  laszip_read_point(laszip_reader);

//		  hasClassification |= point->classification != 0;
//		  hasGpsTime |= point->gps_time != 0;
//		  hasIntensity |= point->intensity != 0;
//		  hasNumberOfReturns |= point->number_of_returns != 0;
//		  hasReturnNumber |= point->return_number != 0;
//		  hasPointSourceId |= point->point_source_ID != 0;
	  }
  }

  float t = common::since(start);
  std::cout << "LASZIP Read through the points in " << t << " seconds." << std::endl;

  laszip_close_reader(laszip_reader);
  laszip_destroy(laszip_reader);
}



int main()
{
  std::string file = "/home/martin/tmp/las/26850_12580.laz";

  read_laszip(file);

  read_lazperf(file);

  return 0;
}
