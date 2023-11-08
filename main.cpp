#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <Eigen/Dense>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Noise ile sanal olcumler yarat
std::vector<double> generateMeasurements(double true_value, double noise_std, int num_samples) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(0, noise_std);

    std::vector<double> measurements;
    for (int i = 0; i < num_samples; ++i) {
        double measurement = true_value + d(gen);  // True value with noise
        measurements.push_back(measurement);
    }
    return measurements;
}


std::vector<double> kalmanFilter(const std::vector<double>& measurements, double initial_estimate, double estimate_uncertainty, double measurement_uncertainty) {
    std::vector<double> estimates;

    double estimate = initial_estimate;

    double estimate_error = estimate_uncertainty;
    double kalman_gain;

    for (double measurement : measurements) {

        kalman_gain = estimate_error / (estimate_error + measurement_uncertainty);
        estimate = estimate + kalman_gain * (measurement - estimate);
        estimate_error = (1 - kalman_gain) * estimate_error;

        estimates.push_back(estimate);
    }

    return estimates;
}

int main() {
    // Istenen gercek deger
    const double true_value = 25;

    // Noise icin standart sapma
    const double noise_std = 3.0;

    // Olcum sayisi
    const int num_samples = 100;

    // Noisy olcum yarat
    std::vector<double> measurements = generateMeasurements(true_value, noise_std, num_samples);

    double initial_estimate = 60.0; // Rastgele deger
    double estimate_uncertainty = 2.0;
    double measurement_uncertainty = std::pow(noise_std, 2);

    //Kalman Filter
    std::vector<double> estimates = kalmanFilter(measurements, initial_estimate, estimate_uncertainty, measurement_uncertainty);

    // Plotting the results using matplotlib-cpp
    plt::figure_size(1200, 780);
    plt::named_plot("True Value", std::vector<double>(num_samples, true_value));
    plt::named_plot("Measurements", measurements, "ro");
    plt::named_plot("Kalman Filter Estimate", estimates, "g-");
    plt::legend();
    plt::title("Kalman Filter Example");
    plt::show();

    return 0;
}
