#ifndef UV_MODEL_H
#define UV_MODEL_H

#include <iostream>
#include <vector>

using namespace std;

class uv_model {
    private:
        vector<double> coefficients;
        double ir_value;
        double power_x;

    public:
        uv_model();

        double model(double x);
};

#endif