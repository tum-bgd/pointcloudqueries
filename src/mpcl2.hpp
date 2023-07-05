/*
A redesign of MPCL for better utilization of floats and using local types
*/

#include<unordered_map>


#include <boost/geometry.hpp>
#include <boost/range.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/function_output_iterator.hpp>

#ifdef MPCL_USE_EIGEN
// related to X11 #define clashing with Eigen's enum value Success
#ifdef Success
  #undef Success
#endif
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
	  
using Eigen::MatrixXd;

#endif


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;


/* Section 1:    Utilities*/
/* Section 1.1:  Structure Tensor Features */

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


/* Section 1.2: Output Helper (empty as templating needs to recurse on dimension)*/


/* Section 2: The Point Cloud Class */


template<typename _coord_type=float, size_t DIM=3>
class pointcloud{
  public:
  /*SECTION 1: TYPES and TYPE DERIVATION*/
  /* Point and coordinates*/
  typedef _coord_type coord_type;
  typedef bg::model::point<coord_type, DIM, bg::cs::cartesian> point_type;
  
  /* Deriving a full OGC Simple Features frameowrk of types */
  typedef bg::model::multi_point<point_type>  multipoint_type; 
  typedef bg::model::box<point_type> box_type;
  typedef bg::model::polygon<point_type> polygon_type; 
  typedef bg::model::multi_polygon<polygon_type> multipolygon_type; 
  typedef bg::model::linestring<point_type> linestring_type;
  typedef std::pair<box_type, unsigned> value_type;

  /* An indexing engine */
  typedef bgi::rtree< value_type, bgi::rstar<16, 4> > rtree_type;	

  /* Type Conversion Tools */
  struct value_maker
  {
    template<typename T>
    inline value_type operator()(T const& v) const
    {
        box_type b(v.value(), v.value()); // make point to box
        return value_type(b, v.index());
    }
   };
   /* Public Application Programming Interface (API) */

   std::vector<double> &get_attrib_by_name(std::string name){return named_attributes[name];};

   void buildIndex(){
	rt = rtree_type(cloud | boost::adaptors::indexed()
                    | boost::adaptors::transformed(value_maker()));
      }

     
    std::vector<double> & computeKNNradius(std::string name, size_t k)
    {
	named_attributes[name] = std::vector<double>(cloud.size());
	std::vector<double> &ref_attrib = named_attributes[name]; 
	
	#pragma omp parallel for
	for (size_t i=0; i <cloud.size(); i++)
	{
	    const auto &p = cloud[i];
	    double d = 0;
	    rt.query(bgi::nearest(p,k), boost::make_function_output_iterator([&](value_type const& v) {
                if (i == v.second) return;
		double _d = bg::distance(cloud[i], cloud[v.second]);
		#pragma omp critical
		if (_d > d)
		  d = _d;
 	    }));
	    ref_attrib[i] = d;
	}
	return ref_attrib;
    };

    /* extractKNN */
   /*void extractKNN(size_t k=6)
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

   }*/



      
   /* Data Section*/
    std::vector<point_type> cloud;
 private:
      std::unordered_map<std::string, std::vector<double>> named_attributes;
      rtree_type rt;
      point_type center;
      box_type bounds;
};

#ifdef TEST_MAIN
int main()
{
   pointcloud<double,4> pcl;
    
   return 0;
}
#endif
