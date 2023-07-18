/*
A redesign of MPCL for better utilization of floats and using local types
*/
#define MPCL_USE_EIGEN

#include<unordered_map>

#include<iostream> //debug only
#include <type_traits>

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


/* Sectopm 1.1     Compile-time recursion through dimension with a functor*/

template <int I>
using int_ = std::integral_constant<int, I>;

//recursive call that iterates the point and calls F on its coordinate
template <class Point, class F, std::size_t I = 0>
struct apply {

    static void call(Point& point, F& f) {
        f(point, int_<I>());
        apply<Point, F, I+1>::call(point, f);
    }
};

//specialisation to end the recursion
template <class CT, std::size_t DIM, class S, template <class, std::size_t, class> class Point, class F>
struct apply<Point<CT, DIM, S>, F, DIM> {

    static void call([[maybe_unused]] Point<CT, DIM, S>& point,[[maybe_unused]]  F& f){
	
    }
};

//interface for calling the function
template <class Point, class F>
void apply_functor(Point& point, F& f) {
    apply<Point, F>::call(point, f);
}

//example functor
template <class Point>
struct add_radius {
     typedef typename boost::geometry::traits::coordinate_type<Point>::type CoordinateType;
     CoordinateType r;
    add_radius(CoordinateType _r): r(_r){};
    template <class Index>
    void operator()(Point& point, [[maybe_unused]] Index I) {
	auto coord_value = bg::get<Index::value>(point)+r;
	bg::set<Index::value>(point,coord_value);
    }
    
/*  example for specialization
    void operator()(Point& point, int_<2>) {
        std::cout << "I am coordinate " << 2 << " and I am specialised with value " << bg::get<2>(point) << std::endl;
    }
*/
};



/* Section 1.0: Exceptions and Event Classes*/


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

/* Section 1.3: Function that are only reaosonable in selected dimensions*/

template<typename pointcloud>
void  boxfilter_4d(pointcloud &pcl,
		    std::string prefix, double radius, double time_radius)
{

   pcl.named_attributes[prefix+"_boxfilter4d"] = std::vector<double>(pcl.cloud.size());
   auto &a = pcl.named_attributes[prefix + "_boxfilter4d"];
   auto & cloud = pcl.cloud;
   #pragma omp parallel for
   
   for (size_t i=0; i< cloud.size(); i++)
   {
       typename pointcloud::point_type minPoint, maxPoint;
       bg::set<0>(minPoint, bg::get<0> (cloud[i]) - radius);
       bg::set<1>(minPoint, bg::get<1> (cloud[i]) - radius);
       bg::set<2>(minPoint, -std::numeric_limits<double>::infinity());
       bg::set<3>(minPoint, bg::get<3> (cloud[i]) - time_radius);
       bg::set<0>(maxPoint, bg::get<0> (cloud[i]) + radius);
       bg::set<1>(maxPoint, bg::get<1> (cloud[i]) + radius);
       bg::set<2>(maxPoint, std::numeric_limits<double>::infinity());
       bg::set<3>(maxPoint, bg::get<3> (cloud[i]) + time_radius);
//       std::cout << bg::wkt(minPoint) << "==>" << bg::wkt(maxPoint) << std::endl;
       typename pointcloud::box_type query_box(minPoint,maxPoint);
       double sum=0, N=0; 
       pcl.rt.query(bgi::intersects(query_box), boost::make_function_output_iterator([&](typename pointcloud::value_type const& v) {
       N++;
       sum += bg::get<2>(v.first.min_corner()); // z coordinate
       }));
       double mean = sum/N;
       a[i] = mean;
   }
}

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
  /*Attributes*/
  typedef std::unordered_map<std::string, std::vector<double>> named_attrib_type;

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
		if (_d > d)
		  d = _d;
 	    }));
	    ref_attrib[i] = d;
	}
	return ref_attrib;
    };


    void map_kNN_features(size_t k, std::string prefix)
    {
       // this function runs a functional for each kNN neighborhood of each point
	static tensor_features tf;
        for (size_t j=0; j < tf.names.size(); j++)
	  named_attributes[prefix+"_"+tf.names[j]]=std::vector<double> (cloud.size());
    
       size_t computed = 0;   
       #pragma omp parallel for
      for (size_t i=0; i< cloud.size(); i++)
      {
	  // compute neighbors
	  if (computed % 10000 == 0)
	  #pragma omp critical
              std::cout << computed << "/" << cloud.size() <<  "(" << (double)computed/cloud.size() << ")" << "\r";
	#pragma omp atomic
	  computed ++;
	  
          const point_type &p =  cloud[i];
	  multipoint_type neighbors;
	  neighbors.push_back(p);
	  rt.query(bgi::nearest(p,k), boost::make_function_output_iterator([&neighbors](value_type const& v) {neighbors.push_back(v.first.min_corner()); }));
	  point_type neighbors_centroid;
	  bg::centroid(neighbors,neighbors_centroid);
	
	  MatrixXd mp(k,3);
	  for (size_t i = 0; i < k; i++)  // assert cloud.size() > k
	  {
	      mp(i,0) = bg::get<0>(neighbors[i]) - bg::get<0>( neighbors_centroid);
	      mp(i,1) = bg::get<1>(neighbors[i])- bg::get<1>( neighbors_centroid);
	      mp(i,2) = bg::get<2>(neighbors[i])- bg::get<2>( neighbors_centroid);
	  }
	  mp = mp.transpose() * mp;
	  Eigen::SelfAdjointEigenSolver<MatrixXd> solver (mp,false);
	  auto ev = solver.eigenvalues().reverse();
	  ev = ev / ev.sum();

	  auto features = tf(ev);
	  for (size_t j=0; j < features.size(); j++)
	  {
	      named_attributes[prefix+"_"+tf.names[j]][i] =features[j];
	  }
      } // endfor
       

    }

    void map_3drange_features(double radius, std::string prefix)
    {
       // this function runs a functional for each kNN neighborhood of each point
	static tensor_features tf;
        for (size_t j=0; j < tf.names.size(); j++)
	  named_attributes[prefix+"_"+tf.names[j]]=std::vector<double> (cloud.size());
	
       size_t computed = 0;   
       #pragma omp parallel for
      for (size_t i=0; i< cloud.size(); i++)
      {
	  // compute neighbors
	  if (computed % 10000 == 0)
	  #pragma omp critical
              std::cout << computed << "/" << cloud.size() <<  "(" << (double)computed/cloud.size() << ")" << "\r";
	#pragma omp atomic
	  computed ++;
	  
          const point_type &p =  cloud[i];
	  multipoint_type neighbors;
	  neighbors.push_back(p);
	  point_type min_pt=p, max_pt=p;
	  add_radius<point_type> posadder(radius), negadder(-radius);
	  apply_functor(min_pt, negadder);
	  apply_functor(max_pt, posadder);
	  box_type query_box(min_pt,max_pt);
//	  std::cout << bg::wkt(query_box.min_corner())<<std::endl;;
	  rt.query(bgi::intersects(query_box), boost::make_function_output_iterator([&neighbors,radius,&p](value_type const& v) {
	    if(true)//bg::distance(v.first.min_corner(), p) < radius)
		neighbors.push_back(v.first.min_corner());
	  }));
	  point_type neighbors_centroid;
	  bg::centroid(neighbors,neighbors_centroid);
	  MatrixXd mp(neighbors.size(),3);
	  for (size_t i = 0; i < neighbors.size(); i++)  // assert cloud.size() > k
	  {
	      mp(i,0) = bg::get<0>(neighbors[i]) - bg::get<0>( neighbors_centroid);
	      mp(i,1) = bg::get<1>(neighbors[i])- bg::get<1>( neighbors_centroid);
	      mp(i,2) = bg::get<2>(neighbors[i])- bg::get<2>( neighbors_centroid);
	  }
	  mp = mp.transpose() * mp;
	  Eigen::SelfAdjointEigenSolver<MatrixXd> solver (mp,false);
	  auto ev = solver.eigenvalues().reverse();
	  ev = ev / ev.sum();

	  auto features = tf(ev);
	  for (size_t j=0; j < features.size(); j++)
	  {
	      named_attributes[prefix+"_"+tf.names[j]][i] =features[j];
	  }
      } // endfor
       

    }
      
   /* Data Section*/
    std::vector<point_type> cloud;
 rtree_type rt;
      named_attrib_type named_attributes;
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
