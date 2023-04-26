#include <iostream>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

double fit(double x)
{
    vector<double> coefficients  {26.96, -19.75, 88.01, -175.6, 170.9, -96.81, 
                                  35.18, -8.703, 1.517, -0.19, 0.01718, -0.001112,
                                  5.032e-05, -1.511e-6, 2.705e-08, -2.186e-10};

    double value = 0,power_x = 1;
    for (int i = 0; i < coefficients.size(); i++)
    {
        value += coefficients[i] * power_x;
        power_x *= x;
    }
    
    return value;
}

// Define main function
int main()
{
    //Call fit function
    double irradiance = fit(1);

    cout << "Irradiance: " << irradiance <<endl;

}