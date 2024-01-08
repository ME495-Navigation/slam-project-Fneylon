#include "geometry2d.hpp"

using namespace turtlelib;

int main()

{

float x1 = 4.500000000000001;
float x2 = 4.500000000000002;

if (almost_equal(x1, x2))
{
    std::cout << "x1 and x2 are equal" << std::endl;
}
else
{
    std::cout << "x1 and x2 are not equal" << std::endl;

}
}