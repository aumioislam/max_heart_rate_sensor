#include <cstdint>
#include <iostream>
#include <climits>
#include <time.h>
#include <cmath>
#include <random>

#include "ampd.hpp"

/*
 * This function is used to determine the coefficients of a linear regression 
 * for a given array
 */
reg_coeff determine_coefficients(uint32_t *buf, const uint16_t n) {
    float sum_y = 0, sum_x = 0, sum_x_squared = 0, sum_xy = 0, squared_sum_x = 0;
    float m = 0, b = 0;

    for (int i = 0; i < n; i++) {
        sum_y += (float) buf[i];
        sum_x += (float) i;
        sum_x_squared += (float) i * i;
        sum_xy += (float) buf[i] * i;
    }

    squared_sum_x = sum_x * sum_x;

    m = n*sum_xy - (sum_x*sum_y);
    m /= n*sum_x_squared - squared_sum_x;

    b = sum_y - m*sum_x;
    b /= n;

    reg_coeff ret_struct = { .slope = m, .intercept = b};

    return ret_struct;
}

/*
 * This function linearly detrends the data by subtracting the value of the 
 * linear regression for each array element
 */
float *linearly_detrend(uint32_t *buf, const uint16_t n) {
    reg_coeff lin_reg = determine_coefficients(buf, n);
    float *detrend_buf = (float *) malloc(n*sizeof(float));
    float temp;

    for (int i = 0; i < n; i++) {
        temp = (float) buf[i];
        detrend_buf[i] = temp - (i*lin_reg.slope + lin_reg.intercept);
    }

    return detrend_buf;
}

/*
 * This is an implementation of an algorithm for peak dectection in a signal
 * For more infomation regarding the algorithm: 
 * DOI: 10.3390/a5040588
 */
uint16_t count_peaks_ampd(uint32_t *buf, const uint16_t N) {
    //used to generate uniform distribution of 
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    uint16_t L;

    //Find the value of L - Ceiling of (N/2) - 1
    if (N % 2 == 1) {
        L = (N+1)/2;
    } else {
        L = N/2;
    } 

    L -= 1;

    //linearly detrend the data
    float *x = linearly_detrend(buf, N);

    //initial LMS matrix of size LxN
    float **LMS = (float **) calloc(L, sizeof(float *));
    for (int i = 0; i < L; i++) {
        LMS[i] = (float *) calloc(N, sizeof(float));
    }

    //initialize gamma vector
    float gamma[L] = {0};
    int lambda = 0, min = INT_MAX;
    
    //loop through data array
    for (int k = 0; k < L; k++) {
        for (int i = 0; i < N; i++) {
            
            //if i = k+2,...,N-k+1
            if ((i >= (k+2)) && (i <= (N - k + 1))) {

                //if x[i-1] is a local maxima wrt the sliding window 2k
                if ((x[i-1] > x[i-k-1]) && (x[i-1] > x[i+k-1])) {
                    LMS[k][i] = 0;
                } else {
                    LMS[k][i] = (float) dis(gen) + 1;
                }

            } else {
                LMS[k][i] = (float) dis(gen) + 1;
            }

            //gamma is equal to the row-wise summation of the LMS matrix
            gamma[k] += LMS[k][i];
        }

        //lambda equals the index of the smallest element of gammaj:w
        if (gamma[k] < min) {
            min = gamma[k];
            lambda = k;
        }
    }

    //lambda must be at least 2
    if (lambda < 1) {
        return 0;
    }

    //copy LMS matrix to reshaped M_r matrix of size lambda by N 
    float **M_r = (float **) calloc(lambda, sizeof(float *));
    for (int i = 0; i < lambda; i++) {
        M_r[i] = (float *) calloc(N, sizeof(float));
    }

    float sigma[N] = {0};

    for (int k = 0; k < lambda; k++) {
        for (int i = 0; i < N; i++) {
            M_r[k][i] = LMS[k][i];
        }
        free(LMS[k]);
    }

    free(LMS);

    uint16_t peaks = 0;
    float mean, std_dev;

    //compute the column-wise standard deviation for M_r
    for (int i = 0; i < N; i++) {
        std_dev = 0;
        for (int k_1 = 0; k_1 < lambda; k_1++) {
            mean = 0;
            for (int k_2 = 0; k_2 < lambda; k_2++) {
                mean += M_r[k_2][i];
            }
            mean /= lambda;

            
            std_dev += (float) sqrt(pow(M_r[k_1][i] - mean, 2));
        }

        std_dev /= (lambda - 1);
        sigma[i] = std_dev;

        //Original algorithm called for only std devs equal to zero to indicate a peak
        //After trial and error, an bound of 0.005 was found to provide the best results
        if (sigma[i] <= 0.005) {
            peaks++;
        }
    }

    //free memory
    for (int i = 0; i < lambda; i++) {
        free(M_r[i]);
    }
    free(M_r);

    //return peaks
    return peaks;
}
