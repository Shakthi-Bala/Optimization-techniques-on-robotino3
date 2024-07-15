
#include "pso_set.h"

#include <vector>

using namespace std;

Pso_set::Pso_set()
{
}
Pso_set::Pso_set(float originX, float originY, float resolution, int width, int height, unsigned char *costs)
{
    setoriginX(originX);
    setoriginY(originY);
    setresolution(resolution);
    setwidth(width);
    setheight(height);
    setCosts(costs);
}

Pso_set::~Pso_set()
{
}
/**********************************************************/
//Function: Mutators and Accessors
/**********************************************************/

void Pso_set::setoriginX(float originx)
{
    originX = originx;
}
float Pso_set::getoriginX()
{
    return originX;
}
void Pso_set::setoriginY(float originy)
{
    originY = originy;
}
float Pso_set::getoriginY()
{
    return originY;
}
void Pso_set::setresolution(float Resolution)
{
    resolution = Resolution;
}
float Pso_set::getresolution()
{
    return resolution;
}
void Pso_set::setwidth(int Width)
{
    width = Width;
}
int Pso_set::getwidth()
{
    return width;
}
void Pso_set::setheight(int Height)
{
    height = Height;
}
int Pso_set::getheight()
{
    return height;
}
void Pso_set::setCosts(unsigned char *Costs)
{
    costs = Costs;
}
unsigned char *Pso_set::getCosts()
{
    return costs;
}
