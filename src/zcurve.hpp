#ifndef ZCURVE_HPP_INC
#define ZCURVE_HPP_INC

#include<iostream>
#include<functional>
#include<bitset>



/*Section 0: Some Geometry Definitions*/
/*#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/algorithms/distance.hpp> 


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;


typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef bg::model::polygon<point, false, false> polygon; // ccw, open polygon
typedef bg::model::linestring<point> linestring;
typedef std::pair<point, size_t> value;
*/

/*
  Section 1: 
  A general discretizer for uniform ranges specified during construction.
*/


struct discretizer 
{
  double vmin, vmax;
  size_t steps;
public:
  discretizer(double _vmin, double _vmax, size_t _steps):vmin(_vmin),vmax(_vmax),steps(_steps){}
  size_t operator() (double p) const{
    return static_cast<size_t> ((p - vmin)/(vmax-vmin)*steps);

  }
};


/*
  Function: mix
  
  A function for mixing a varying number of bitsets bit by bit as needed for the Z curve.
  This function has the drawback of that it needs to be told the size for the return value
  at compile time. It defaults to 32 bit.  
*/


template<typename... Ts, size_t return_size = 32>
size_t mix(Ts... args)
{
   const int size = sizeof...(args) ;
   std::bitset<return_size> ret(0); 
   typename std::tuple_element<0,std::tuple<Ts...>>::type set[size] = {args...};
   size_t l = 0;
   for (size_t k = 0; k < set[0].size(); k++)
   {
     for (size_t i=0; i < size; i++)
     {
        ret[l++] = set[i][k];
     }
   }
   return ret.to_ulong();
}


/*
  Function: mix_into
  
  A function for mixing a varying number of bitsets bit by bit as needed for the Z curve.
  This function clears the bitset given as the first argument and writes the results to this
  existing bitset.   
*/


template<typename target_bitset_type, typename... Ts>
size_t mix_into(target_bitset_type &ret , Ts... args)
{
   ret = 0;
   const int size = sizeof...(args) ;
   // assume the first type in Ts... is the type for all args
   typename std::tuple_element<0,std::tuple<Ts...>>::type set[size] = {args...};
   size_t l = 0;
   for (size_t k = 0; k < set[0].size(); k++)
   {
     for (size_t i=0; i < size; i++)
     {
        ret[l++] = set[i][k];
     }
   }
   return ret.to_ulong();
}

// SOME VARIADIC HELPERS
template<unsigned int index, unsigned int In, unsigned int... remPack> struct getVal
{
    static const unsigned int val = getVal<index-1, remPack...>::val;
};
template<unsigned int In, unsigned int...remPack> struct getVal<0,In,remPack...>
{
    static const unsigned int val = In;
};


    //go to any arg by :
    //getVal<Some_Unsigned_Index, T...>::val;



template<typename target_bitset_type, typename ...Ts>
size_t demix_from(target_bitset_type from , Ts & ... args)
{
   const int size = sizeof...(args) ;
   // assume the first type in Ts... is the type for all args
   typename std::tuple_element<0,std::tuple<Ts  ...>>::type  set[size] = {args...};
   size_t l = 0;
   for (size_t k = 0; k < set[0].size(); k++)
   {
      // This essentially calls the args[k] on each argument abusing brace initializer expansion
    int dummy[sizeof...(Ts)] =     { (args[k] = from [l++],0) ...};
    ((void)dummy); // suppress the warning that dummy is, well, a dummy...
   }
   return 0;
}




/*
    ZCurve2D: The final class

    This class is initialized with 2 discretizers and can be used
    to transform spatial points into cells or the associated Z curve key.

   TODO: It is possible / should be possible to rewrite this for varying
         the number of dimensions. However, we stick with the simple case 
	  here.
*/

template<typename point>
class ZCurve2D
{
public:
  discretizer x_disc,y_disc;
  ZCurve2D():x_disc(0,1,256),y_disc(0,1,256)
  {}
  ZCurve2D(discretizer _x, discretizer _y): x_disc(_x),y_disc(_y){}
  ZCurve2D(double lx,double hx, double ly, double hy, size_t steps):x_disc(lx,hx,steps),y_disc(ly,hy,steps){}

  std::pair<size_t, size_t> discrete(point p) const
  {
    return std::make_pair(x_disc(bg::get<0>(p)),y_disc(bg::get<1>(p)));
  }

  const discretizer &get_xdisc() const { return x_disc;} 
  const discretizer &get_ydisc() const { return y_disc;} 
  
  
   template<size_t BIT_PER_DIMENSION=16>
  size_t key(std::pair<size_t, size_t > dp) const
  {
      
      return(mix(
	    std::bitset<BIT_PER_DIMENSION> (dp.first),
	    std::bitset<BIT_PER_DIMENSION> (dp.second)
	    ));
  }

template<size_t BIT_PER_DIMENSION=16>
  size_t key(point p) const
  {
      auto dp = discrete(p);
      return(key(dp));
      
  }

  
  template<size_t BIT_PER_DIMENSION=16>
  std::pair<size_t, size_t> dekey(size_t key)
  {
      std::bitset<BIT_PER_DIMENSION> a,b;
      demix_from(std::bitset<2*BIT_PER_DIMENSION> (key),a,b);
      return std::make_pair(a.to_ulong(),b.to_ulong());
  }

  size_t neighbor(size_t keyin, int x_offset, int y_offset)
  {
      auto dp = dekey(keyin);
      dp.first += x_offset;
      dp.second += y_offset;
      return key(dp);
  }

  
};


#endif
