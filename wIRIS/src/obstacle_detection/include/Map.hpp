/**THIS FILE CONTAINS A 2D MAP CLASS THAT CAN BE EASILY VIEWED BY HUMANS**/
//written by Leon Frickensmith and Max Archer
//leonfrickensmith@gmail.com

#ifndef MAP_HPP
#define MAP_HPP

#include <iostream>//cout
#include <fstream>//filestream
#include <stdlib.h>//abs

#include <memory.h>

#if _DEBUG
    #define ASSERT(x) if(!(x)) DebugBreak()
#else
    #define ASSERT(x) 
#endif

template<typename T> struct matrix_tag
{
public:
    matrix_tag(matrix_tag & m)
    {
        d=0;
        nx=0;ny=0;
        *this=m;
    }
    matrix_tag()
    {
        d=0;
        nx=0;ny=0;
    }
    int dx,dy;
    matrix_tag(int width, int height)
    {
        d=0;
        nx=0;ny=0;
        create(width,height);
    }
private:
    int nx,ny;
    T* d;
    T* g;
public:
    void create(int ax,int bx,int ay,int by)
    {
        int oldsize=nx*ny;
        dx=-ax;dy=-ay;
        nx=bx-ax+1;ny=by-ay+1;
        ASSERT(nx>0);
        ASSERT(ny>0);
        int newsize=nx*ny;
        if(d && oldsize!=newsize)
        {
            delete[] d;
            d=0;
        }
        if(!d)
            d=new T [newsize];
        g=d+dx+nx*dy;
    }
    void create(int width, int height)
    {
        create(0,width-1,0,height-1);
    }
    matrix_tag(int ax,int bx,int ay,int by)
    {
        d=0;
        nx=0;ny=0;
        create(ax,bx,ay,by);
    }
#if _DEBUG
    T & operator() (int ax,int ay)
    {
        int kx=ax+dx;int ky=ay+dy;
        ASSERT(kx<nx);
        ASSERT(ky<ny);
        ASSERT(kx>=0);
        ASSERT(ky>=0);
        return d[kx+nx*ky];
    }
#else
    T & operator() (int ax,int ay)
    {
        return g[ax+nx*ay];
    }
#endif
    matrix_tag & operator= (const matrix_tag & m)
    {
        if(d && nx*ny!=m.nx*m.ny)
        {
            delete[] d;
            d=0;
        }
        if(!d)
            d=new T [m.nx*m.ny];
        nx=m.nx;
        ny=m.ny;
        dx=m.dx;
        dy=m.dy;
        g=d+dx+nx*dy;
        memcpy(d,m.d,nx * ny * sizeof(T));
        return *this;
    }
    int xSize()
    {
        return nx;
    }
    int ySize()
    {
        return ny;
    }
    operator void*()
    {
        return (void*)d;
    }
    T* data()
    {
        return d;
    }
    void zeroMem()
    {
        memset(d,0,nx * ny * sizeof(T));
    }
    ~matrix_tag()
    {
        if(d)
            delete[] d;
        d=0;
    }
    void fill(T val)
    {
        for(int i=0;i<nx*ny;i++)
            d[i]=val;
    }
} ;

typedef matrix_tag<double> MATRIX;
typedef matrix_tag<unsigned char> MAT_GRAYSCALE;
typedef matrix_tag<unsigned char[3]> MAT_RGB;

const float map_defaultValue = -9999.0f;//default value of map pieces, DO NOT REFERENCE THIS
const float map_occupied = 1;
const float map_unoccupied = 0;
const float map_unknown = startValue;

/**======================**/
/**TELLS US THE STEEPNESS FACTOR OF A PIECE OF TERRAIN BY COMPARING THE HEIGHT OF ADJACENT PIECES**/
/**======================**/
template <typename T>
int f_isSteep(T origin, T up, T left, T down, T right, T tolerance)//written by Max Archer 9/17/2014
{
    if(origin!=map_defaultValue)
    {
        if(up==map_defaultValue)
            up = origin;
        if(left==map_defaultValue)
            left = origin;
        if(down==map_defaultValue)
            down = origin;
        if(right==map_defaultValue)
            right = origin;

        if(up-origin >= tolerance || origin-up >= tolerance)
            return map_occupied;
        else if(left-origin >= tolerance || origin-left >= tolerance)
            return map_occupied;
        else if(down-origin >= tolerance || origin-down >= tolerance)
            return map_occupied;
        else if(right-origin >= tolerance || origin-right >= tolerance)
            return map_occupied;
    }
    return map_unoccupied;
}

/**======================**/
/**Uses the input map to produce a gradient of this map**/
/**======================**/
void makeGradient(MATRIX & output, const MATRIX& input, const float tolerance)//takes a map and gives it the gradient data
{
    
    //const float tolerance = 0.5f;
    for(int y = -m_halfSize.x; y <= m_halfSize.y; ++y)
    {
        for(int x = -m_halfSize.x; x <= m_halfSize.x; ++x)
        {
            if(input(x,y) != map_defaultValue)//get the point, and all points around it!
                output(x,y) = f_isSteep(input(x  , y  ),
                                        input(x  , y+1),
                                        input(x  , y-1),
                                        input(x-1, y  ),
                                        input(x+1, y  ),
                                        tolerance);
        }
    }
}

#endif // MAP_HPP
