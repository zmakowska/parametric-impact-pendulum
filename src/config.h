#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

struct Config {
    // Grid parameters
    int num_delta;
    int num_epsilon;
    double delta_min;
    double delta_max;
    double epsilon_min;
    double epsilon_max;

    // Simulation parameters
    double t_end;
    double tol;
    double initial_h;
    double h_min;
    double max_val;
    double rms_start;

    // Collision parameters
    double collision_a0;
    double min_rebound;
    std::string material;

    // Initial conditions
    double initial_alpha;
    double initial_alphaprime;

    // Parallelism
    int num_threads;
};

Config read_config(const std::string &filename) {
    Config cfg;

    // Default values:
    cfg.num_delta    = 501;
    cfg.num_epsilon  = 501;
    cfg.delta_min    = 0.0;
    cfg.delta_max    = 5.0;
    cfg.epsilon_min  = 0.0;
    cfg.epsilon_max  = 5.0;
    cfg.t_end        = 5000.0;
    cfg.tol          = 1e-6;
    cfg.initial_h    = 0.1;
    cfg.h_min        = 1e-10;
    cfg.max_val      = 1e8;
    cfg.rms_start    = 4500.0;
    cfg.collision_a0 = 0.05;
    cfg.min_rebound  = 1e-6;
    cfg.num_threads  = 64;
    cfg.material = "medium_steel";
    cfg.initial_alpha = 1.0;
    cfg.initial_alphaprime = 0.0;

    std::ifstream infile(filename);
    if (!infile) {
        std::cerr << "Could not open config file: " << filename << ". Using default configuration.\n";
        return cfg;
    }

    std::string line;
    while (std::getline(infile, line)) {
        // Remove comments
        size_t pos = line.find('#');
        if (pos != std::string::npos)
            line = line.substr(0, pos);
        if (line.empty())
            continue;
        
        pos = line.find('=');
        if (pos == std::string::npos)
            continue;
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        
        auto trim = [](const std::string &s) -> std::string {
            size_t start = s.find_first_not_of(" \t");
            if (start == std::string::npos) return "";
            size_t end = s.find_last_not_of(" \t");
            return s.substr(start, end - start + 1);
        };
        key = trim(key);
        value = trim(value);

        try {
            if (key == "num_delta")           cfg.num_delta = std::stoi(value);
            else if (key == "num_epsilon")      cfg.num_epsilon = std::stoi(value);
            else if (key == "delta_min")        cfg.delta_min = std::stod(value);
            else if (key == "delta_max")        cfg.delta_max = std::stod(value);
            else if (key == "epsilon_min")      cfg.epsilon_min = std::stod(value);
            else if (key == "epsilon_max")      cfg.epsilon_max = std::stod(value);
            else if (key == "t_end")            cfg.t_end = std::stod(value);
            else if (key == "tol")              cfg.tol = std::stod(value);
            else if (key == "initial_h")        cfg.initial_h = std::stod(value);
            else if (key == "h_min")            cfg.h_min = std::stod(value);
            else if (key == "max_val")          cfg.max_val = std::stod(value);
            else if (key == "rms_start")        cfg.rms_start = std::stod(value);
            else if (key == "collision_a0")     cfg.collision_a0 = std::stod(value);
            else if (key == "min_rebound")      cfg.min_rebound = std::stod(value);
            else if (key == "num_threads")      cfg.num_threads = std::stoi(value);
            else if (key == "initial_alpha")    cfg.initial_alpha = std::stod(value);
            else if (key == "initial_alphaprime") cfg.initial_alphaprime = std::stod(value);
        } catch (...) {
            std::cerr << "Error reading configuration for key " << key << std::endl;
        }
    }

    return cfg;
}

#endif // CONFIG_H