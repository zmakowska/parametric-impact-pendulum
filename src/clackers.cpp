#include "config.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ------------------------------------------------------------
// Numerical constants
// ------------------------------------------------------------
static constexpr double POSITION_EPS = 1e-10;
static constexpr double VELOCITY_EPS = 1e-10;
static constexpr double COLLISION_POSITION_EPS = 1e-9;
static constexpr double RELATIVE_STEP_EPS = 1e-14;
static constexpr double DENOMINATOR_EPS = 1e-30;

static constexpr int MAX_COLLISION_SUBSTEPS = 1000;
static constexpr double EXPLOSION_RMS = 1e6;

// Dimensionless damping coefficient used in the equation of motion.
// Equation:
// alpha'' + DAMPING * alpha' + (1/delta^2 + epsilon sin(t)) sin(alpha) = 0
static constexpr double DAMPING = 0.02;

// ------------------------------------------------------------
// Data structures
// ------------------------------------------------------------
struct State {
    double alpha;
    double alpha_prime;
};

struct Result {
    double delta;
    double epsilon;
    double final_alpha;
    double final_alpha_prime;
    int steps;
    double rms;
};

// ------------------------------------------------------------
// Equation of motion
// ------------------------------------------------------------
inline State derivative(
    double t,
    const State& state,
    double frequency_factor,
    double epsilon
) {
    const double stiffness = frequency_factor + epsilon * std::sin(t);

    return {
        state.alpha_prime,
        -stiffness * std::sin(state.alpha) - DAMPING * state.alpha_prime
    };
}

// ------------------------------------------------------------
// One RK4 step
// ------------------------------------------------------------
inline State rk4_step(
    double t,
    const State& state,
    double h,
    double frequency_factor,
    double epsilon
) {
    State k1 = derivative(t, state, frequency_factor, epsilon);
    k1.alpha *= h;
    k1.alpha_prime *= h;

    State temp{
        state.alpha + 0.5 * k1.alpha,
        state.alpha_prime + 0.5 * k1.alpha_prime
    };

    State k2 = derivative(t + 0.5 * h, temp, frequency_factor, epsilon);
    k2.alpha *= h;
    k2.alpha_prime *= h;

    temp = {
        state.alpha + 0.5 * k2.alpha,
        state.alpha_prime + 0.5 * k2.alpha_prime
    };

    State k3 = derivative(t + 0.5 * h, temp, frequency_factor, epsilon);
    k3.alpha *= h;
    k3.alpha_prime *= h;

    temp = {
        state.alpha + k3.alpha,
        state.alpha_prime + k3.alpha_prime
    };

    State k4 = derivative(t + h, temp, frequency_factor, epsilon);
    k4.alpha *= h;
    k4.alpha_prime *= h;

    return {
        state.alpha + (k1.alpha + 2.0 * k2.alpha + 2.0 * k3.alpha + k4.alpha) / 6.0,
        state.alpha_prime + (k1.alpha_prime + 2.0 * k2.alpha_prime + 2.0 * k3.alpha_prime + k4.alpha_prime) / 6.0
    };
}

// ------------------------------------------------------------
// Velocity-dependent restitution models
// ------------------------------------------------------------

inline double large_steel_function(double v) {
    return std::exp(-0.169348 * std::pow(std::fabs(v), 0.526615));
}

inline double medium_steel_function(double v) {
    return std::exp(-0.346827 * std::pow(std::fabs(v), 0.533857));
}

inline double small_steel_function(double v) {
    return std::exp(-0.332746 * std::pow(std::fabs(v), 0.534113));
}

inline double aluminium_function(double v) {
    return std::exp(-0.169052 * std::pow(std::fabs(v), 0.550565));
}

inline double titanium_function(double v) {
    return std::exp(-0.0637493 * std::pow(std::fabs(v), 0.521426));
}

inline double brass_function(double v) {
    return std::exp(-0.439974 * std::pow(std::fabs(v), 0.53989));
}

inline double restitution_coefficient(double velocity, const Config& cfg) {
    if (cfg.material == "large_steel") {
        return large_steel_function(velocity);
    }
    if (cfg.material == "medium_steel") {
        return medium_steel_function(velocity);
    }
    if (cfg.material == "small_steel") {
        return small_steel_function(velocity);
    }
    if (cfg.material == "aluminium") {
        return aluminium_function(velocity);
    }
    if (cfg.material == "titanium") {
        return titanium_function(velocity);
    }
    if (cfg.material == "brass") {
        return brass_function(velocity);
    }

    std::cerr << "Warning: unknown material '" << cfg.material
              << "'. Using medium_steel model.\n";

    return medium_steel_function(velocity);
}

// ------------------------------------------------------------
// Oblique collision model
// ------------------------------------------------------------
// Because the balls have finite radius, collision occurs at alpha_c,
// not exactly on the symmetry axis. The effective post-collision
// angular velocity is:
//
// alpha_prime_after = alpha_prime_before * (-e cos^2(alpha_c) + sin^2(alpha_c))
//
// where e is a velocity-dependent coefficient of restitution.
inline double collision_factor(
    double velocity_before_collision,
    double collision_angle,
    const Config& cfg
) {
    const double e = restitution_coefficient(velocity_before_collision, cfg);

    const double cos_c = std::cos(collision_angle);
    const double sin_c = std::sin(collision_angle);

    return -e * cos_c * cos_c + sin_c * sin_c;
}

inline double rebound_velocity(
    double velocity_before_collision,
    double collision_angle,
    double min_rebound_velocity,
    const Config& cfg
) {
    double velocity_after_collision =
        collision_factor(velocity_before_collision, collision_angle, cfg)
        * velocity_before_collision;

    if (std::fabs(velocity_after_collision) < min_rebound_velocity) {
        velocity_after_collision =
            (velocity_after_collision >= 0.0 ? min_rebound_velocity : -min_rebound_velocity);
    }

    return velocity_after_collision;
}

// ------------------------------------------------------------
// RK4 step with collision detection
// ------------------------------------------------------------
// The allowed angular interval is:
//
//     [collision_angle, 2*pi - collision_angle]
//
// A collision occurs when alpha reaches either boundary while moving
// outward. The function can resolve multiple collision events within
// a single RK4 step.
void step_with_collisions(
    double t,
    const State& initial_state,
    double h,
    double frequency_factor,
    double epsilon,
    double collision_angle,
    double min_rebound_velocity,
    const Config& cfg,
    State& output_state,
    double& used_h,
    bool& collision_occurred
) {
    collision_occurred = false;

    const double lower_boundary = collision_angle;
    const double upper_boundary = 2.0 * M_PI - collision_angle;

    double remaining_h = h;
    double local_t = t;
    State current_state = initial_state;

    int collision_substeps = 0;

    while (remaining_h > RELATIVE_STEP_EPS) {
        collision_substeps++;

        if (collision_substeps > MAX_COLLISION_SUBSTEPS) {
            throw std::runtime_error(
                "Exceeded maximum number of collision substeps. "
                "The simulation may be stuck near a collision boundary."
            );
        }

        const bool near_lower =
            std::fabs(current_state.alpha - lower_boundary) < COLLISION_POSITION_EPS;

        const bool near_upper =
            std::fabs(current_state.alpha - upper_boundary) < COLLISION_POSITION_EPS;

        const bool moving_into_lower =
            near_lower && current_state.alpha_prime < 0.0;

        const bool moving_into_upper =
            near_upper && current_state.alpha_prime > 0.0;

        if (moving_into_lower || moving_into_upper) {
            collision_occurred = true;

            current_state.alpha =
                moving_into_lower ? lower_boundary : upper_boundary;

            current_state.alpha_prime = rebound_velocity(
                current_state.alpha_prime,
                collision_angle,
                min_rebound_velocity,
                cfg
            );

            remaining_h = 0.0;
            break;
        }

        State tentative_state = rk4_step(
            local_t,
            current_state,
            remaining_h,
            frequency_factor,
            epsilon
        );

        const bool crossed_lower =
            (current_state.alpha - lower_boundary) *
            (tentative_state.alpha - lower_boundary) < 0.0;

        const bool crossed_upper =
            (current_state.alpha - upper_boundary) *
            (tentative_state.alpha - upper_boundary) < 0.0;

        if (!crossed_lower && !crossed_upper) {
            current_state = tentative_state;
            local_t += remaining_h;
            remaining_h = 0.0;
            continue;
        }

        collision_occurred = true;

        const double collision_boundary =
            crossed_lower ? lower_boundary : upper_boundary;

        const double denominator =
            current_state.alpha - tentative_state.alpha;

        double collision_fraction = 0.5;

        if (std::fabs(denominator) > DENOMINATOR_EPS) {
            collision_fraction =
                (current_state.alpha - collision_boundary) / denominator;
        }

        collision_fraction =
            std::max(0.0, std::min(1.0, collision_fraction));

        const double h_until_collision =
            remaining_h * collision_fraction;

        const double h_after_collision =
            remaining_h - h_until_collision;

        double velocity_at_collision = current_state.alpha_prime;

        if (std::fabs(denominator) > DENOMINATOR_EPS) {
            velocity_at_collision =
                current_state.alpha_prime +
                collision_fraction *
                (tentative_state.alpha_prime - current_state.alpha_prime);
        }

        State collision_state{
            collision_boundary,
            rebound_velocity(
                velocity_at_collision,
                collision_angle,
                min_rebound_velocity,
                cfg
            )
        };

        local_t += h_until_collision;
        remaining_h = h_after_collision;
        current_state = collision_state;

        if (remaining_h < RELATIVE_STEP_EPS) {
            remaining_h = 0.0;
        }
    }

    output_state = current_state;
    used_h = local_t - t;
}

// ------------------------------------------------------------
// Adaptive RK4 simulation for one pair of parameters
// ------------------------------------------------------------
Result solve_equation(
    double delta,
    double epsilon,
    const Config& cfg
) {
    double t = 0.0;
    double h = cfg.initial_h;

    int step_count = 0;
    bool exploded = false;
    bool pinned = false;

    State state{
        cfg.initial_alpha,
        cfg.initial_alphaprime
    };

    const double frequency_factor =
        (delta == 0.0) ? 1e12 : 1.0 / (delta * delta);

    double integrated_alpha_squared = 0.0;
    double integrated_time = 0.0;

    while (t < cfg.t_end) {
        if (h < cfg.h_min) {
            break;
        }

        if (t + h > cfg.t_end) {
            h = cfg.t_end - t;
        }

        const double state_norm =
            std::sqrt(state.alpha * state.alpha +
                      state.alpha_prime * state.alpha_prime);

        if (state_norm > cfg.max_val) {
            exploded = true;
            break;
        }

        const double old_h = h;

        bool collision_big = false;
        bool collision_half_1 = false;
        bool collision_half_2 = false;

        State state_big;
        State state_half_1;
        State state_half_2;

        double used_big = 0.0;
        double used_half_1 = 0.0;
        double used_half_2 = 0.0;

        try {
            step_with_collisions(
                t,
                state,
                h,
                frequency_factor,
                epsilon,
                cfg.collision_a0,
                cfg.min_rebound,
                cfg,
                state_big,
                used_big,
                collision_big
            );

            step_with_collisions(
                t,
                state,
                0.5 * h,
                frequency_factor,
                epsilon,
                cfg.collision_a0,
                cfg.min_rebound,
                cfg,
                state_half_1,
                used_half_1,
                collision_half_1
            );

            step_with_collisions(
                t + 0.5 * h,
                state_half_1,
                0.5 * h,
                frequency_factor,
                epsilon,
                cfg.collision_a0,
                cfg.min_rebound,
                cfg,
                state_half_2,
                used_half_2,
                collision_half_2
            );
        } catch (const std::runtime_error& error) {
            throw std::runtime_error(
                std::string("Collision-handling error: ") + error.what()
            );
        }

        const double diff_alpha =
            state_half_2.alpha - state_big.alpha;

        const double diff_alpha_prime =
            state_half_2.alpha_prime - state_big.alpha_prime;

        const double error_estimate =
            std::sqrt(diff_alpha * diff_alpha +
                      diff_alpha_prime * diff_alpha_prime) / 15.0;

        if (error_estimate > cfg.tol) {
            const double scale =
                0.9 * std::pow(cfg.tol / error_estimate, 0.25);

            h *= scale;
            continue;
        }

        state = state_half_2;
        t += h;
        step_count++;

        const double rms_interval_start =
            std::max(t - h, cfg.rms_start);

        const double rms_interval_end =
            std::min(t, cfg.t_end);

        if (rms_interval_end > rms_interval_start) {
            const double dt = rms_interval_end - rms_interval_start;
            integrated_alpha_squared += state.alpha * state.alpha * dt;
            integrated_time += dt;
        }

        const double lower_boundary = cfg.collision_a0;
        const double upper_boundary = 2.0 * M_PI - cfg.collision_a0;

        if (state.alpha < lower_boundary || state.alpha > upper_boundary) {
            throw std::runtime_error(
                "Invalid state: alpha outside allowed boundaries."
            );
        }

        const bool pinned_lower =
            std::fabs(state.alpha - lower_boundary) < POSITION_EPS &&
            std::fabs(state.alpha_prime) < VELOCITY_EPS;

        const bool pinned_upper =
            std::fabs(state.alpha - upper_boundary) < POSITION_EPS &&
            std::fabs(state.alpha_prime) < VELOCITY_EPS;

        if (pinned_lower || pinned_upper) {
            std::cerr << "Stopping early: system pinned at collision boundary.\n";

            state.alpha = pinned_lower ? lower_boundary : upper_boundary;
            state.alpha_prime = 0.0;
            pinned = true;
            break;
        }

        const bool any_collision =
            collision_big || collision_half_1 || collision_half_2;

        if (any_collision) {
            h = old_h;
        } else {
            double scale = 2.0;

            if (error_estimate > 0.0) {
                scale = 0.9 * std::pow(cfg.tol / error_estimate, 0.25);
                scale = std::min(2.0, scale);
            }

            h *= scale;
        }
    }

    Result result;
    result.delta = delta;
    result.epsilon = epsilon;
    result.final_alpha = state.alpha;
    result.final_alpha_prime = state.alpha_prime;
    result.steps = step_count;

    if (pinned) {
        result.rms = 0.0;
    } else if (exploded) {
        result.rms = EXPLOSION_RMS;
    } else {
        result.rms =
            (integrated_time > 1e-15)
                ? std::sqrt(integrated_alpha_squared / integrated_time)
                : 0.0;
    }

    return result;
}

// ------------------------------------------------------------
// Build parameter grid
// ------------------------------------------------------------
std::vector<std::pair<double, double>> build_parameter_grid(const Config& cfg) {
    std::vector<std::pair<double, double>> grid;
    grid.reserve(cfg.num_delta * cfg.num_epsilon);

    const double delta_step =
        (cfg.num_delta > 1)
            ? (cfg.delta_max - cfg.delta_min) / (cfg.num_delta - 1)
            : 0.0;

    const double epsilon_step =
        (cfg.num_epsilon > 1)
            ? (cfg.epsilon_max - cfg.epsilon_min) / (cfg.num_epsilon - 1)
            : 0.0;

    for (int i = 0; i < cfg.num_delta; ++i) {
        const double delta = cfg.delta_min + i * delta_step;

        for (int j = 0; j < cfg.num_epsilon; ++j) {
            const double epsilon = cfg.epsilon_min + j * epsilon_step;
            grid.emplace_back(delta, epsilon);
        }
    }

    return grid;
}

// ------------------------------------------------------------
// Run grid simulation in parallel
// ------------------------------------------------------------
std::vector<std::string> run_grid_simulation(
    const std::vector<std::pair<double, double>>& grid,
    const Config& cfg
) {
    std::vector<std::string> results(grid.size());

    std::atomic<size_t> processed_count(0);
    const size_t total = grid.size();

    unsigned int number_of_threads = static_cast<unsigned int>(cfg.num_threads);

    if (number_of_threads == 0) {
        number_of_threads = std::thread::hardware_concurrency();
    }

    if (number_of_threads == 0) {
        number_of_threads = 1;
    }

    number_of_threads =
        std::min(number_of_threads, static_cast<unsigned int>(total));

    auto worker = [&](size_t start_index, size_t end_index) {
        for (size_t index = start_index; index < end_index; ++index) {
            const double delta = grid[index].first;
            const double epsilon = grid[index].second;

            try {
                const Result result =
                    solve_equation(delta, epsilon, cfg);

                std::ostringstream line;
                line << result.delta << ","
                     << result.epsilon << ","
                     << result.final_alpha << ","
                     << result.final_alpha_prime << ","
                     << result.steps << ","
                     << result.rms;

                results[index] = line.str();
            } catch (const std::runtime_error&) {
                std::ostringstream line;
                line << delta << ","
                     << epsilon << ","
                     << cfg.collision_a0 << ","
                     << 0.0 << ","
                     << -1 << ","
                     << 0.0;

                results[index] = line.str();
            }

            processed_count++;
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(number_of_threads);

    const size_t base_chunk_size = total / number_of_threads;
    const size_t remainder = total % number_of_threads;

    size_t start_index = 0;

    for (unsigned int i = 0; i < number_of_threads; ++i) {
        const size_t chunk_size =
            base_chunk_size + (i < remainder ? 1 : 0);

        const size_t end_index = start_index + chunk_size;

        threads.emplace_back(worker, start_index, end_index);
        start_index = end_index;
    }

    std::thread progress_thread([&]() {
        while (processed_count.load() < total) {
            const double percent =
                100.0 * processed_count.load() / static_cast<double>(total);

            std::cout << "Progress: "
                      << processed_count.load()
                      << " / "
                      << total
                      << " ("
                      << percent
                      << "%)\r"
                      << std::flush;

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        std::cout << "Progress: "
                  << total
                  << " / "
                  << total
                  << " (100%)             "
                  << std::endl;
    });

    for (auto& thread : threads) {
        thread.join();
    }

    progress_thread.join();

    return results;
}

// ------------------------------------------------------------
// Save CSV file
// ------------------------------------------------------------
void write_results_to_csv(
    const std::string& filename,
    const std::vector<std::string>& results
) {
    std::ofstream file(filename);

    if (!file) {
        throw std::runtime_error("Could not open output file: " + filename);
    }

    file << "delta,epsilon,final_alpha,final_alpha_prime,steps,rms\n";

    for (const std::string& line : results) {
        file << line << "\n";
    }
}

// ------------------------------------------------------------
// Main
// ------------------------------------------------------------
int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: "
                  << argv[0]
                  << " <config_filename> <result_filename>\n";

        return 1;
    }

    const std::string config_filename = argv[1];
    const std::string result_filename = argv[2];

    try {
        Config cfg = read_config(config_filename);

        std::vector<std::pair<double, double>> grid =
            build_parameter_grid(cfg);

        std::random_device random_device;
        std::mt19937 generator(random_device());
        std::shuffle(grid.begin(), grid.end(), generator);

        std::vector<std::string> results =
            run_grid_simulation(grid, cfg);

        write_results_to_csv(result_filename, results);

        std::cout << "Grid simulation completed. Results saved to "
                  << result_filename
                  << std::endl;

    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << std::endl;
        return 1;
    }

    return 0;
}