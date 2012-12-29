#include "Triangle.h"
#include <vector>
#include <algorithm>
using namespace std;

Triangle::Triangle(int d1,int d2,int d3){

    vector<int> vertices;
    vertices.push_back(d1);
    vertices.push_back(d2);
    vertices.push_back(d3);
    sort(vertices.begin(), vertices.end());
    v1 = vertices.at(0);
    v2 = vertices.at(1);
    v3 = vertices.at(2);
}
  


Triangle::~Triangle(void)
{
}
  
bool Triangle::operator < (const Triangle& other) const {
    if(v1 != other.v1) return (v1<other.v1);
    else if( v2 != other.v2) return (v2<other.v2);
    else if( v3 != other.v3) return (v3<other.v3);
    else return false;
  
}

