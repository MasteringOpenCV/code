#pragma once
class Triangle
{
public:
    int v1,v2,v3;  

    Triangle(int d1,int d2,int d3);
    ~Triangle(void);
    bool operator < (const Triangle& other) const;
};

