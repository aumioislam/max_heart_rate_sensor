#include <iostream>
#include "../src/ampd/ampd.hpp"

int main() {
    uint32_t *buf = (uint32_t *) malloc(8*sizeof(uint32_t));

    buf[0] = 30;
    buf[1] = 45;
    buf[2] = 51;
    buf[3] = 57;
    buf[4] = 60;
    buf[5] = 65;
    buf[6] = 70;
    buf[7] = 71;

    reg_coeff vals = determine_coefficients(buf, 8);

    std::cout << vals.slope << std::endl << vals.intercept << std::endl;

}
