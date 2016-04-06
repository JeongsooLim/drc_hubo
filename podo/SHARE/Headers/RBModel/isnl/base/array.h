#ifndef MYLIB_ARRAY_H
#define MYLIB_ARRAY_H
#include <vector>
#include <assert.h>
#include <string>

namespace isnl{
template<class T> class Array2;
}

typedef std::vector<int>                  ints;
typedef std::vector<long>                 longs;
typedef std::vector<float>                floats;
typedef std::vector<double>               doubles;
typedef std::vector<std::string>          strings;

typedef isnl::Array2<int>    int2;
typedef isnl::Array2<long>   long2;
typedef isnl::Array2<float>  float2;
typedef isnl::Array2<double> double2;
typedef isnl::Array2<std::string> string2;

namespace isnl{

template<class T>
class Array2{
private:
    int sz[2];
    std::vector<T> data;
public:
    typedef typename std::vector<T>::iterator       titerator;
    typedef typename std::vector<T>::const_iterator const_titerator;
    Array2()                                   :data()         {sz[0]=0;sz[1]=0;}
    Array2(int s0, int s1)                     :data(s0*s1)    {sz[0]=s0;sz[1]=s1;}
    Array2(int s0, int s1, const T& val)       :data(s0*s1,val){sz[0]=s0;sz[1]=s1;}
    Array2(const Array2& arr)                  :data(arr.data) {sz[0]=arr.sz[0];sz[1]=arr.sz[1];}

          T& operator() (int i0, int i1)       {return data[i0*sz[1]+i1];}
    const T& operator() (int i0, int i1) const {return data[i0*sz[1]+i1];}

	int size(int i=0)                    const {assert(i==0 || i==1); return sz[i];}
    int length()                         const {return sz[0]*sz[1];}
    void resize(int s0, int s1)                {sz[0] = s0; sz[1] = s1; data.resize(s0*s1);}
    void clear()                               {sz[0] = 0; sz[1] = 0; data.clear();}
	bool empty(){return data.empty();}

          titerator tbegin()                   {return data.begin();}// traverse iterator
    const_titerator tbegin()             const {return data.begin();}// traverse iterator
          titerator tend()                     {return data.end();}// traverse iterator
    const_titerator tend()               const {return data.end();}// traverse iterator
};

}
#endif
