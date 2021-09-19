#ifndef _AMPD_HPP
#define _AMPD_HPP

typedef struct {
    float slope;
    float intercept;
} reg_coeff;

reg_coeff determine_coefficients(uint32_t *buf, const uint16_t n);
float *linearly_detrend(uint32_t *buf, const uint16_t n);
uint16_t count_peaks_ampd(uint32_t *buf, const uint16_t N);
#endif
