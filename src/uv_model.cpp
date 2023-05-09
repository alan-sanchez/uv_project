/*
CPP program to find the irradiance value for
a given distance, x, value
*/

#include "uv_project/uv_model.h"

/*
Constructor of the `uv_model` class used to define variables and constants
*/
uv_model::uv_model(){
    coefficients = {26.96, -19.75, 88.01, -175.6, 170.9, -96.81, 
                    35.18, -8.703, 1.517, -0.19, 0.01718, -0.001112,
                    5.032e-05, -1.511e-6, 2.705e-08, -2.186e-10};
    ir_value = 0;
    power_x = 1;
}


/*
This function computes the irradiance from the uv model
@param x: double 

@returns ir_value: double
*/
double uv_model::model(double x){
    // // For loop to compute irradiance value  
    for (int i = 0; i < coefficients.size(); i++)
    {
        ir_value += coefficients[i] * power_x;
        power_x *= x;
    }
    
    // // return irradiance value. Units are (mW/cm^2)
    return ir_value; 
}

/*
Main function that runs the `model` function 
*/
// int main()
// {
//     // Call `model` function with argument value of 1
//     double irradiance = model(1);

//     // Print out irradiance value
//     cout << "Irradiance: " << irradiance <<endl;
// }