#include "regression.h"


Regression::Regression(void)
{
    reset();
}

void Regression::reset(void)
{
    this->n = 0;
    this->sum_x = 0;
    this->sum_y = 0;
    this->sum_xx = 0;
    this->sum_xy = 0;
}

void Regression::add(double x, double y)
{
    n++;
    this->sum_x += x;
    this->sum_y += y;
    this->sum_xx += x * x;
    this->sum_xy += x * y;
}

void Regression::calculate(void)
{
    double nom = (n * sum_xy) - (sum_x * sum_y);
    double denom = (n * sum_xx) - (sum_x * sum_x);
    this->b = nom / denom;
    this->a = (sum_y - b * sum_x) / n;
    this->c = (sum_x - sum_y / b) / n;
}

double Regression::getA(void)
{
    return a;
}

double Regression::getB(void)
{
    return b;
}

double Regression::getC(void)
{
    return c;
}

