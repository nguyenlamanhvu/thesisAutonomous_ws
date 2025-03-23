#include "../include/robot_navigation/GA.hpp"

using json = nlohmann::json;

PathPlanningGA::PathPlanningGA(void)
        : populationSize(1000), generations(10000), mutationRate(0.01), crossoverRate(0.8), stopGA(false) {
    std::srand(std::time(nullptr));
        
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.setCallbackQueue(&high_priority_queue);

    GA_planning_server = nh.advertiseService("GA_optimize", &PathPlanningGA::handleGARequest, this);
    Stop_GA_flag_sub = private_nh.subscribe("/GA_stop_flag", 10, &PathPlanningGA::stopGAFlag, this);

    spinner = std::make_unique<ros::AsyncSpinner>(1, &high_priority_queue);
    spinner->start();
}

PathPlanningGA::~PathPlanningGA() {
    if (spinner) {
        spinner->stop();
    }
}

void PathPlanningGA::loadCostData(const std::string& start, const std::vector<std::string>& destinations) {
    locationNames.clear();
    costMap.clear();
    locationToIndex.clear();
    indexToLocation.clear();
    
    // Add start location and destinations to locationNames
    locationNames.push_back(start);
    locationNames.insert(locationNames.end(), destinations.begin(), destinations.end());
    if (hasEndLocation) {
        locationNames.push_back(endLocation);
    }
    
    // Create index mappings
    for (size_t i = 0; i < locationNames.size(); ++i) {
        locationToIndex[locationNames[i]] = i;
        indexToLocation[i] = locationNames[i];
    }
    
    // Load costs between all pairs of locations
    for (size_t i = 0; i < locationNames.size(); i++) {
        for (size_t j = i + 1; j < locationNames.size(); j++) {
            const std::string& from = locationNames[i];
            const std::string& to = locationNames[j];
            
            double cost = getCostFromJson(from, to);
            costMap[{from, to}] = cost;
            costMap[{to, from}] = cost; // Assuming symmetric costs
            // cost = getCostFromJson(to, from);
            // costMap[{to, from}] = cost;
        }
    }
    
    // Print loaded costs for verification
    std::cout << "Loaded costs:\n";
    for (const auto& cost : costMap) {
        std::cout << cost.first.first << " -> " 
                    << cost.first.second << ": " 
                    << cost.second << "\n";
    }
}

void PathPlanningGA::initializePopulation(const std::string& start, const std::vector<std::string>& destinations) {
    startLocation = start;
    population.clear();
    
    // Create a base chromosome with destination indices
    std::vector<int> baseChromosome;
    for (const auto& dest : destinations) {
        baseChromosome.push_back(locationToIndex[dest]);
    }

    // Generate initial population
    for (int i = 0; i < populationSize; ++i) {
        Individual ind;
        ind.chromosome = baseChromosome;
        std::random_shuffle(ind.chromosome.begin(), ind.chromosome.end());
        ind.fitness = calculateFitness(ind.chromosome);
        population.push_back(ind);
    }
}

double PathPlanningGA::calculateFitness(const std::vector<int>& chromosome) {
    double totalCost = 0.0;
    std::string currentLoc = startLocation;

    // Calculate path through destinations
    for (int index : chromosome) {
        std::string nextLoc = indexToLocation[index];
        auto costPair = std::make_pair(currentLoc, nextLoc);
        
        if (costMap.find(costPair) != costMap.end()) {
            totalCost += costMap[costPair];
        } else {
            totalCost += 99999999; // Large penalty for invalid paths
        }
        
        currentLoc = nextLoc;
    }

    // Add cost to end location if specified
    if (hasEndLocation) {
        auto finalCostPair = std::make_pair(currentLoc, endLocation);
        if (costMap.find(finalCostPair) != costMap.end()) {
            totalCost += costMap[finalCostPair];
        } else {
            totalCost += 99999999;
        }
    }

    return 1.0 / (totalCost + 1.0);
}

std::vector<int> PathPlanningGA::crossover(const std::vector<int>& parent1, const std::vector<int>& parent2) {
    if (static_cast<double>(rand()) / RAND_MAX >= crossoverRate) {
        return parent1;
    }

    // Order Crossover (OX)
    int size = parent1.size();
    std::vector<int> child(size, -1);
    
    // Select random segment
    int start = rand() % size;
    int end = rand() % size;
    if (start > end) std::swap(start, end);

    // Copy segment from parent1
    for (int i = start; i <= end; ++i) {
        child[i] = parent1[i];
    }

    // Fill remaining positions with parent2's genes
    int j = 0;
    for (int i = 0; i < size; ++i) {
        if (i >= start && i <= end) continue;
        
        while (std::find(child.begin(), child.end(), parent2[j]) != child.end()) {
            j++;
        }
        child[i] = parent2[j++];
    }

    return child;
}

void PathPlanningGA::mutate(std::vector<int>& chromosome) {
    for (size_t i = 0; i < chromosome.size(); ++i) {
        if (static_cast<double>(rand()) / RAND_MAX < mutationRate) {
            int j = rand() % chromosome.size();
            std::swap(chromosome[i], chromosome[j]);
        }
    }
}

std::vector<std::string> PathPlanningGA::optimize() {
    for (int gen = 0; gen < generations; ++gen) {
        std::vector<Individual> newPopulation;

        // Elitism: keep the best individual
        auto elite = std::max_element(population.begin(), population.end(),
            [](const Individual& a, const Individual& b) { return a.fitness < b.fitness; });
        newPopulation.push_back(*elite);

        // Generate new population
        while (newPopulation.size() < populationSize) {
            // Tournament selection
            auto parent1 = tournament();
            auto parent2 = tournament();

            // Crossover
            auto childChromosome = crossover(parent1.chromosome, parent2.chromosome);

            // Mutation
            mutate(childChromosome);

            // Add new individual
            Individual child;
            child.chromosome = childChromosome;
            child.fitness = calculateFitness(childChromosome);
            newPopulation.push_back(child);

            if(stopGA) {
                break;
            }
        }

        population = newPopulation;
        if(stopGA) {
            break;
        }
    }

    // Get best solution
    auto best = std::max_element(population.begin(), population.end(),
        [](const Individual& a, const Individual& b) { return a.fitness < b.fitness; });

    // Convert solution back to location names
    std::vector<std::string> optimalPath;
    for (int index : best->chromosome) {
        optimalPath.push_back(indexToLocation[index]);
    }

    // Add end location to path if specified
    if (hasEndLocation) {
        optimalPath.push_back(endLocation);
    }

    return optimalPath;
}

PathPlanningGA::Individual PathPlanningGA::tournament(int tournamentSize) {
    Individual best = population[rand() % populationSize];
    
    for (int i = 1; i < tournamentSize; ++i) {
        Individual contestant = population[rand() % populationSize];
        if (contestant.fitness > best.fitness) {
            best = contestant;
        }
    }
    
    return best;
}

double PathPlanningGA::getCostFromJson(const std::string& from, const std::string& to) {
    std::string filename = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/AStarResult/" + from + "To" + to + ".json";
    std::string altFilename = "/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/AStarResult/" + to + "To" + from + ".json";
    
    std::ifstream f(filename);
    if (f.good()) {
        json data = json::parse(f);
        return data["cost"];
    }
    
    std::ifstream altF(altFilename);
    if (altF.good()) {
        json data = json::parse(altF);
        return data["cost"];
    }
    
    return 999999; // Return large cost if path not found
}

void PathPlanningGA::setEndPosition(const std::string& end) {
    endLocation = end;
    hasEndLocation = true;
}

// Update the stopGAFlag function to use mutex
void PathPlanningGA::stopGAFlag(const std_msgs::Bool::ConstPtr &msg) {
    ROS_INFO("Get GA (algorithm) stop flag");
    stopGA = msg->data;
}

bool PathPlanningGA::handleGARequest(robot_navigation::GARequest::Request &req,
                                     robot_navigation::GARequest::Response &res) {
    std::string startLocation = req.start;
    ROS_INFO_STREAM("Receive request:" << startLocation);
    std::vector<std::string> destinations = req.destinations;
    for (const auto &s : req.destinations)
    {
        ROS_INFO("  - %s", s.c_str());
    }

    for (const auto &s : req.destinations)
    {
                if (s == "CheckoutCounter") {
                    setEndPosition(s);
            
            auto it = std::find(destinations.begin(), destinations.end(), s);
            if (it != destinations.end()) {
                destinations.erase(it);
                    }
            
                    break;
                }
            }


    loadCostData(startLocation, destinations);
    initializePopulation(startLocation, destinations);
            std::vector<std::string> optimalPath = optimize();

    std::cout << "\nOptimal path from " << startLocation << ":\n";
    // res.GA_result.push_back(startLocation);
    for (const auto& location : optimalPath) {
        std::cout << location << " -> ";
        res.GA_result.push_back(location);
    }
    std::cout << "End\n";
    // res.GA_result.push_back("End");
    hasEndLocation = false;

    return true;
}
