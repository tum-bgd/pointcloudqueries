/*(c) 2018 Martin Werner
 ALL RIGHTS RESERVED
*/
#ifndef PCL_INCLUDE
#define PCL_INCLUDE
#include<vector>
#include<fstream>
#include<sstream>
#include<cstdlib>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/graph_utility.hpp>

//#define HAVE_HDF5

/*
This has been stripped for the demo for my colleagues. More functionaly has been there.

 */

using namespace boost;

typedef float cost;
typedef adjacency_list<vecS, vecS, bidirectionalS> graph_t; // property<vertex_color_t,default_color_type>,    property<edge_weight_t, cost, property<edge_index_t,size_t>>

//typedef property_map<graph_t, edge_weight_t>::type WeightMap;
//typedef property_map<graph_t, vertex_color_t>::type ColorMap;
//typedef color_traits<property_traits<ColorMap>::value_type> Color;
//typedef property_map<graph_t, edge_index_t>::type IndexMap;
typedef graph_t::vertex_descriptor vertex_descriptor;
typedef graph_t::edge_descriptor edge_descriptor;
typedef graph_t::vertex_iterator vertex_iterator;


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/algorithms/distance.hpp>

// Boost.Range
#include <boost/range.hpp>
// adaptors
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/function_output_iterator.hpp>


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;


struct flat_point3{
double x,y,z;
flat_point3(){}
flat_point3(double _x,double  _y,double _z):x(_x),y(_y),z(_z){};
template<typename Q>
flat_point3( Q& p){x = p.x; y = p.y; z=0;}
};

struct flat_point2
{
double x,y,z;
flat_point2(){}
flat_point2(double _x,double  _y):x(_x),y(_y){};
template<typename Q>
flat_point2( Q& p){x = p.x; y = p.y;}
};

BOOST_GEOMETRY_REGISTER_POINT_3D(flat_point3, double, bg::cs::cartesian,x,y,z);
BOOST_GEOMETRY_REGISTER_POINT_2D(flat_point2, double, bg::cs::cartesian,x,y);
typedef flat_point3 point3;
typedef flat_point2 point2;
typedef point3 point;

typedef bg::model::multi_point<point3>  multipoint; //boost::geometry::model::d2::point_xy<double>
typedef bg::model::box<point3> box3;
typedef bg::model::box<point2> box2;
typedef bg::model::polygon<point3> polygon; // ccw, open polygon
typedef bg::model::polygon<point2> polygon2; // ccw, open polygon
typedef bg::model::multi_polygon<polygon> multipolygon; // ccw, open polygon
typedef bg::model::multi_polygon<polygon2> multipolygon2; // ccw, open polygon
typedef bg::model::linestring<point3> linestring;
typedef std::pair<box3, unsigned> value3;

typedef bgi::rtree< value3, bgi::rstar<16, 4> > rtree3;	


// a functional turning points into values

struct value_maker3
{
    template<typename T>
    inline value3 operator()(T const& v) const
    {
	box3 b(v.value(), v.value()); // make point to box
        return value3(b, v.index());
    }
};


std::ostream &operator<<(std::ostream &os, point3 const &p) {
    return os << "(" << bg::get<0> (p) << ";" <<bg::get<1> (p) << ";" <<bg::get<2> (p) << ")";
}
std::ostream &operator<<(std::ostream &os, point2 const &p) {
    return os << "(" << bg::get<0> (p) << ";" <<bg::get<1> (p)  << ")";
}


namespace util{



class ZonalKey
{
    std::map<std::string, polygon2> zones;
    typedef std::pair<box2, std::string> zk_value;
    typedef bgi::rtree< zk_value, bgi::rstar<16, 4> > zk_rtree;	
    zk_rtree rt;
    public:
	ZonalKey(std::string filename)
	{
	    std::vector<zk_value> values;
	   // a zone definition file contains
	   std::ifstream ifs(filename);
	   std::string line;
	   while (std::getline(ifs,line))
	   {
	       auto pos = line.find(" ");
	       if (pos == std::string::npos)
		   continue;
	       std::string key = line.substr(0,pos);
	       std::string wkt = line.substr(pos+1);
	       polygon2 p;
	       bg::read_wkt(wkt,p);
       
	       
	       bg::correct(p);
	       zones[key] = p;
	       box2 b;
	       bg::envelope(p,b);
	   #ifdef DEBUG_ZONAL
	       std::cout << "adding a polygon" << std::endl;
	       std::cout << "WKT: " << wkt << std::endl;
	       std::cout << "Parsed: " << bg::wkt(p) << std::endl;
	       std::cout << "BBox:" << bg::wkt(b) << std::endl;
	       std::cout << "Key: " << key << std::endl;
	   #endif
	       values.push_back(std::make_pair(b,key));
	       
	   }
	   std::cout << "Using " << values.size() << " polygons. " << std::endl;
	   // build an R tree for this dataset
	   rt = zk_rtree(values.begin(), values.end());
	   
	}
    // get spatial key
    std::vector<std::string> operator()(point2 p)
    {
#ifdef DEBUG_ZONAL
       std::cout << "Intersecting point " << p << std::endl;
#endif
        std::vector<std::string> out;
	std::vector<zk_value> result;
	rt.query(bgi::intersects(p), boost::make_function_output_iterator([&](zk_value const& val)
	{
	   if (bg::within(p,zones[val.second])){
#ifdef DEBUG_ZONAL
           std::cout << "Found it to be part of "<< val.second << std::endl;
#endif
	
	    out.push_back(val.second);
	    }
	}));
	return out;
    }

};





class ZonalKeyMulti
{
    std::multimap<std::string, polygon2> zones;
    typedef std::pair<box2, std::string> zk_value;
    typedef bgi::rtree< zk_value, bgi::rstar<16, 4> > zk_rtree;	
    zk_rtree rt;
    public:
	ZonalKeyMulti(std::string filename)
	{
	    std::vector<zk_value> values;
	   // a zone definition file contains
	   std::ifstream ifs(filename);
	   std::string line;
	   while (std::getline(ifs,line))
	   {
	       auto pos = line.find(" ");
	       if (pos == std::string::npos)
		   continue;
	       std::string key = line.substr(0,pos);
	       std::string wkt = line.substr(pos+1);
	       polygon2 rp;
	       multipolygon2 mp;
	       try{
		   bg::read_wkt(wkt,rp);
		   mp.push_back(rp);
		}catch(...)
		{
		   bg::read_wkt(wkt,mp);
		}
	      for (auto &p: mp){
		   bg::correct(p);
    //	       zones[key] = p;
	       zones.insert(std::make_pair(key,p));
	       box2 b;
	       bg::envelope(p,b);
	   #ifdef DEBUG_ZONAL
	       std::cout << "adding a polygon" << std::endl;
	       std::cout << "WKT: " << wkt << std::endl;
	       std::cout << "Parsed: " << bg::wkt(p) << std::endl;
	       std::cout << "BBox:" << bg::wkt(b) << std::endl;
	       std::cout << "Key: " << key << std::endl;
	   #endif
	       values.push_back(std::make_pair(b,key));
	     }
	   }
	   std::cout << "Using " << values.size() << " polygons. " << std::endl;
	   // build an R tree for this dataset
	   rt = zk_rtree(values.begin(), values.end());
	   
	}
    // get spatial key
    std::vector<std::string> operator()(point2 p)
    {
#ifdef DEBUG_ZONAL
       std::cout << "Intersecting point " << p << std::endl;
#endif
        std::vector<std::string> out;
	std::vector<zk_value> result;
	rt.query(bgi::intersects(p), boost::make_function_output_iterator([&](zk_value const& val)
	{
	    auto r = zones.equal_range(val.second);
	    polygon2 poly; std::string key;
	    for (;r.first != r.second; r.first++ ){
	    
	    if (bg::within(p,r.first->second)){
#ifdef DEBUG_ZONAL
           std::cout << "Found it to be part of "<< r->first << std::endl;
#endif
	
	    out.push_back(r.first->first);
	    }
	    }
	
/*	   if (bg::within(p,zones[val.second])){
#ifdef DEBUG_ZONAL
           std::cout << "Found it to be part of "<< val.second << std::endl;
#endif
	
	    out.push_back(val.second);
	    }*/
	}));
	return out;
    }




    
};





}




namespace mpcl{

  /*
Features based on Eigenvalues need the Eigen library which I did not have on my Windows host
Can be added rather easily (by administrative tasks
    
struct tensor_features
{
   const std::vector<std::string> names = {"linearity","planarity","scattering" ,
        "omnivariance",
        "anisotropy"  ,
        "eigenentropy",
        "trace_by_l0" ,
        "chg_of_curv" };

    template<typename vec3>
    std::vector<double> operator()(const vec3 &l)
    {
         return {
	 	(l[0] - l[1])/l[0], // linearity
         (l[1] - l[2])/l[0],        //planarity  
         l[2]/l[0],		    //scattering
	 cbrt(l[0] *l[1]*l[2]),	    //omnivarianc
          (l[0]-l[2])/l[0],	    //anisotropy 
	  -l[0]*log2(l[0])-l[1]*log2(l[1])-l[2]*log2(l[2]),  //eigenentrop
          (l[0]+l[1]+l[2]) / l[0],  //trace_by_l0
          l[2] / (l[0]+l[1]+l[2])	    //chg_of_curv
	 };
     };

};// class



  */
  template<typename T>
  struct notmyself {
    T myself;
    notmyself(T _myself):myself(_myself){};
    template <typename Value>
    bool operator()(Value const& v)const {
      return v.second != myself;
    }
  };

    

class pointcloud
{
   public:
      std::vector<point> cloud;
      std::vector<unsigned> classes;
      std::vector<float> certainty;
  //  std::vector<std::vector<double>> features;
  //    std::vector<std::tuple<unsigned char, unsigned char, unsigned char>> color;
  //  std::map<unsigned, std::tuple<float,float,float>> colormap;
      graph_t g;

      rtree3 rt;
      

      point center;
      pointcloud()
      {
    
      }

      size_t size() { return cloud.size();};


  void addFromArray(double *pArray, int N)
  {
    for(size_t i=0; i<N; i++)
      cloud.push_back(
		      point(
			    pArray[3*i+0],
			    pArray[3*i+1],
			    pArray[3*i+2]
			    ));

	std::cout << "Have " << cloud.size() << " points" << std::endl;
	// Compute center
	multipoint &mp = (multipoint &)cloud;
	bg::centroid(mp,center);
	std::cout << "Center: " << bg::get<0> (center) << ", " << bg::get<1> (center) << ", " << bg::get<2> (center) << std::endl;

  }

   void buildIndex(){
	rt = rtree3(cloud | boost::adaptors::indexed()
                    | boost::adaptors::transformed(value_maker3()));
   
//	rt = rtree(cloud.begin(), cloud.end());
   }

  std::vector<double> nndistancemap()
  {
    std::vector<double> ret;
    ret.resize(cloud.size());
    	for (size_t i=0; i <cloud.size(); i++)
	{
	    const auto &p = cloud[i];
	    notmyself nms(i);
	    rt.query(bgi::nearest(p,1) && bgi::satisfies(nms), boost::make_function_output_iterator([&](value3 const& v) {
	      ret[i] = bg::distance(cloud[i], cloud[v.second]);
	    }
		));
	}
	return ret;
  }

  
  /*
   size_t buildGraph(size_t k=7){ // will get parameters
        g = graph_t(cloud.size()); // create a fresh empty graph
	#pragma omp parallel for
	for (size_t i=0; i <cloud.size(); i++)
	{
	    const auto &p = cloud[i];
	    rt.query(bgi::nearest(p,k), boost::make_function_output_iterator([&](value3 const& v) {
//	        #neighbors.push_back(v.first.min_corner());
                if (i == v.second) return;
		#pragma omp critical
                add_edge(i,v.second,g); // add an edge from i to second
	    }));
	}
	return num_edges(g);
   }

    void createVertexIndexArray(std::vector<uint32_t> &arr)
    {
       auto c = num_edges(g);
       if (2*c > arr.size()) throw(std::runtime_error("vertex index array buffer smaller than max elements"));
       
        graph_traits<graph_t>::edge_iterator ei, ei_end;
	uint32_t *p = &arr[0];
	for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
	{
	    // modification for mutual kNN
	    auto s = source(*ei,g);
	    auto e = target(*ei,g);
	    if (edge(e,s,g).second){ // mutual kNN
		   *p = s; p++;
		   *p = e; p++;
	    }
	}
    }


    void applyKNNZ(size_t k=6)
    {
	#pragma omp parallel for
	for (size_t i=0; i <cloud.size(); i++)
	{
	    const auto &p = cloud[i];
	    double d = 0;
	    rt.query(bgi::nearest(p,k), boost::make_function_output_iterator([&](value3 const& v) {
//	        #neighbors.push_back(v.first.min_corner());
                if (i == v.second) return;
		double _d = bg::distance(cloud[i], cloud[v.second]);
		#pragma omp critical
		if (_d > d)
		  d = _d;
 	    }));
	    bg::set<2>(cloud[i],-d);
	}

    }
    
   

   void extractKNN(size_t k=6)
   {
      features.clear();
      // our first basic extractor: for each point, extract kNN, extract features from it and write to CSV
      tensor_features tf;
      
      for (size_t i=0; i< cloud.size(); i++)
      {
          if (i % 1000 == 0)
	     std::cout << i << "/" << cloud.size() << std::endl;
          const point &p =  cloud[i];
//	  std::cout << "Extract " << p << std::endl;
	  multipoint neighbors;
	  neighbors.push_back(p);
	  rt.query(bgi::nearest(p,k), boost::make_function_output_iterator([&neighbors](value3 const& v) {neighbors.push_back(v.first.min_corner()); }));
	  point neighbors_centroid;
	  bg::centroid(neighbors,neighbors_centroid);

//	  for (auto &p: neighbors)
//	    std::cout << "Neighbor: " << p<<std::endl;
//	  std::cout << "Center: " << neighbors_centroid << std::endl;
  
	  	  
	  MatrixXd mp(k,3);
	  for (size_t i = 0; i < k; i++)  // assert cloud.size() > k
	  {
	      mp(i,0) = bg::get<0>(neighbors[i]) - bg::get<0>( neighbors_centroid);
	      mp(i,1) = bg::get<1>(neighbors[i])- bg::get<1>( neighbors_centroid);
	      mp(i,2) = bg::get<2>(neighbors[i])- bg::get<2>( neighbors_centroid);
	  }
//	  std::cout << mp << std::endl;
	  mp = mp.transpose() * mp;
//	  std::cout << "--" << std::endl;
//	  std::cout << mp << std::endl;
//	  std::cout << "--" << std::endl;
	  Eigen::SelfAdjointEigenSolver<MatrixXd> solver (mp,false);
	  auto ev = solver.eigenvalues().reverse();
	  ev = ev / ev.sum();

	  features.push_back(tf(ev));
 
      } // endfor

   }
   
  */

};







};
#endif
