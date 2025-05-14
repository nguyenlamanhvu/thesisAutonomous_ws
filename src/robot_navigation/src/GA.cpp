#include "../include/robot_navigation/GA.hpp"

using json = nlohmann::json;
namespace fs = std::filesystem;

// Calculate Euclidean distance between two points
double distance(const Good& p1, const Good& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

void KMeans::addGood(const Good& g) {
    goods.push_back(g);
}

Good KMeans::getCentroid(int index) {
    if (index >= 0 && index < k) {
        return centroids[index];
    }
    return Good();
}

void KMeans::initializeCentroidsPlusPlus() {
    std::random_device rd;
    std::mt19937 gen(rd());

    // Choose first centroid randomly
    std::uniform_int_distribution<> dis(0, goods.size() - 1);
    centroids.clear();
    centroids.push_back(goods[dis(gen)]);

    // Choose remaining centroids
    for (int i = 1; i < k; i++) {
        std::vector<double> distances(goods.size());
        double sum = 0.0;

        // Calculate distances to nearest existing centroid
        for (size_t j = 0; j < goods.size(); j++) {
            double minDist = std::numeric_limits<double>::max();
            for (const auto& centroid : centroids) {
                minDist = std::min(minDist, distance(goods[j], centroid));
            }
            distances[j] = minDist * minDist;
            sum += distances[j];
        }

        // Choose next centroid with probability proportional to D²
        std::uniform_real_distribution<> dis_prob(0.0, sum);
        double rand_val = dis_prob(gen);
        double cumsum = 0.0;
        size_t chosen_idx = 0;

        for (size_t j = 0; j < goods.size(); j++) {
            cumsum += distances[j];
            if (cumsum >= rand_val) {
                chosen_idx = j;
                break;
            }
        }

        centroids.push_back(goods[chosen_idx]);
    }
}

bool KMeans::assignClusters() {
    bool changed = false;
    std::vector<int> newAssignments(goods.size());
    double new_inertia = 0.0;
    
    for (size_t i = 0; i < goods.size(); ++i) {
        double minDist = std::numeric_limits<double>::max();
        int closestCentroid = 0;
        
        for (int j = 0; j < k; ++j) {
            double dist = distance(goods[i], centroids[j]);
            if (dist < minDist) {
                minDist = dist;
                closestCentroid = j;
            }
        }
        
        // Add penalty if distance exceeds maxRadius
        if (minDist > maxRadius) {
            new_inertia += std::numeric_limits<double>::max(); // Large penalty
        } else {
            new_inertia += minDist * minDist;
        }

        newAssignments[i] = closestCentroid;
        if (assignments.empty() || newAssignments[i] != assignments[i]) {
            changed = true;
        }
    }
    
    assignments = newAssignments;
    inertia = new_inertia;
    return changed;
}

void KMeans::updateCentroids() {
    std::vector<Good> newCentroids(k, Good(0, 0));
    std::vector<int> counts(k, 0);
    
    for (size_t i = 0; i < goods.size(); ++i) {
        int cluster = assignments[i];
        newCentroids[cluster].x += goods[i].x;
        newCentroids[cluster].y += goods[i].y;
        counts[cluster]++;
    }
    
    for (int i = 0; i < k; ++i) {
        if (counts[i] > 0) {
            newCentroids[i].x /= counts[i];
            newCentroids[i].y /= counts[i];
            newCentroids[i].name = "centroid" + std::to_string(i);
        }
    }
    
    centroids = newCentroids;
}

double KMeans::getInertia() {
    return inertia;
}

void KMeans::printClusterResults() {
    std::vector<std::vector<std::string>> clusters(k);
    for (size_t i = 0; i < goods.size(); ++i) {
        clusters[assignments[i]].push_back(goods[i].name);
    }
    
    std::cout << "\nClustering Results:\n";
    std::cout << "==================\n";
    for (size_t i = 0; i < clusters.size(); ++i) {
        std::cout << "Cluster " << i + 1 << ": ";
        for (size_t j = 0; j < clusters[i].size(); ++j) {
            std::cout << clusters[i][j];
            if (j < clusters[i].size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
    }
    std::cout << "Inertia: " << inertia << std::endl;
    std::cout << "==================\n";
}

void KMeans::run(int maxIterations) {
    if (goods.empty() || k <= 0) return;
    
    initializeCentroidsPlusPlus();
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        bool changed = assignClusters();
        if (!changed) break;
        updateCentroids();
    }
}

// Get all goods in each cluster
std::vector<std::vector<std::string>> KMeans::getClusterContents() {
    std::vector<std::vector<std::string>> clusters(k);
    for (size_t i = 0; i < goods.size(); ++i) {
        clusters[assignments[i]].push_back(goods[i].name);
    }
    return clusters;
}

// Function to read goods data from JSON files
std::vector<Good> readGoodsFromFiles(const std::string& folderPath) {
    std::vector<Good> goods;
    
    try {
        for (const auto& entry : fs::directory_iterator(folderPath)) {
            if (entry.path().extension() == ".json") {
                std::ifstream file(entry.path());
                if (file.is_open()) {
                    json j;
                    file >> j;
                    
                    if (j.contains("positon") && j["positon"].is_array() && j["positon"].size() >= 2) {
                        double x = j["positon"][0];
                        double y = j["positon"][1];
                        std::string name = entry.path().stem().string();
                        goods.emplace_back(x, y, name);
                    }
                }
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error accessing directory: " << e.what() << std::endl;
        return goods;
    }
    
    return goods;
}

// Function to find optimal k using elbow method
int findOptimalK(const std::vector<Good>& goods, double maxRadius, int maxK) {
    std::vector<double> inertias;
    
    // Try different values of k
    for (int k = 1; k <= maxK; ++k) {
        KMeans kmeans(k, maxRadius);
        for (const auto& good : goods) {
            kmeans.addGood(good);
        }
        kmeans.run();
        inertias.push_back(kmeans.getInertia());
    }
    
    // Find the elbow point using the maximum curvature
    int optimalK = 1;
    double maxCurvature = 0.0;
    
    for (int i = 1; i < maxK - 1; ++i) {
        double curvature = std::abs(inertias[i-1] - 2*inertias[i] + inertias[i+1]);
        if (curvature > maxCurvature) {
            maxCurvature = curvature;
            optimalK = i + 1;
        }
    }
    
    // Print elbow curve
    std::cout << "\nElbow Curve:\n";
    std::cout << "============\n";
    for (int i = 0; i < maxK; ++i) {
        std::cout << "K=" << i+1 << ": Inertia=" << inertias[i] << std::endl;
    }
    std::cout << "============\n";
    
    return optimalK;
}

PathPlanningGA::PathPlanningGA(void)
        : populationSize(100), generations(25000), mutationRate(0.01), crossoverRate(0.8), stopGA(false) {
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
    const int maxRecent = 10;
    const double epsilon = 1e-5;

    std::deque<double> recentFitnesses;

    for (int gen = 0; gen < generations; ++gen) {
        std::vector<Individual> newPopulation;

        // Elitism: keep the best individual
        auto elite = std::max_element(population.begin(), population.end(),
            [](const Individual& a, const Individual& b) { return a.fitness < b.fitness; });
        newPopulation.push_back(*elite);

        std::cout << "Generation: " << gen << " Best Fitness: " << elite->fitness << std::endl;

        // Store recent fitnesses for early stopping
        recentFitnesses.push_back(elite->fitness);
        if (recentFitnesses.size() > maxRecent)
            recentFitnesses.pop_front();

        // Check for early stopping
        if (recentFitnesses.size() == maxRecent) {
            double maxF = *std::max_element(recentFitnesses.begin(), recentFitnesses.end());
            double minF = *std::min_element(recentFitnesses.begin(), recentFitnesses.end());
            if (maxF - minF < epsilon) {
                break;
            }
        }

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
    // if (hasEndLocation) {
    //     optimalPath.push_back(endLocation);
    // }

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

// Function to select goods by name
std::vector<Good> selectGoods(const std::vector<Good>& allGoods, const std::vector<std::string>& selectedNames) {
    std::vector<Good> selectedGoods;
    for (const auto& name : selectedNames) {
        auto it = std::find_if(allGoods.begin(), allGoods.end(),
            [&name](const Good& g) { return g.name == name; });
        if (it != allGoods.end()) {
            selectedGoods.push_back(*it);
        }
    }
    return selectedGoods;
}

// Update the stopGAFlag function to use mutex
void PathPlanningGA::stopGAFlag(const std_msgs::Bool::ConstPtr &msg) {
    ROS_INFO("Get GA (algorithm) stop flag");
    stopGA = msg->data;
}

bool PathPlanningGA::handleGARequest(robot_navigation::GARequest::Request &req,
                                     robot_navigation::GARequest::Response &res) {
    std::string startLocation = req.start;
    std::string realEndLocation = "";
    ROS_INFO_STREAM("Receive request:" << startLocation);
    std::vector<std::string> destinations = req.destinations;
    for (const auto &s : req.destinations)
    {
        ROS_INFO("  - %s", s.c_str());
    }

    // 1. Read goods from Pose folder
    std::vector<Good> allGoods = readGoodsFromFiles("/home/lamanhvu/thesisAutonomous_ws/src/robot_navigation/ProductPoseReal");

    // Print available goods
    std::cout << "\nAvailable goods:\n";
    for (const auto& good : allGoods) {
        std::cout << good.name << " ";
    }
    std::cout << "\n\n";

    for (const auto &s : req.destinations)
    {
        if (s == "CheckoutCounter") {
            // setEndPosition(s);
            realEndLocation = s;

            auto it = std::find(destinations.begin(), destinations.end(), s);
            if (it != destinations.end()) {
                destinations.erase(it);
            }
            break;
        }
    }

    // Get selected goods (excluding start and end locations)
    std::vector<Good> goods = selectGoods(allGoods, destinations);

    if (goods.empty()) {
        std::cerr << "No valid goods selected. Exiting.\n";
        return false;
    }
    
    std::cout << "\nSelected goods for clustering:\n";
    for (const auto& good : goods) {
        std::cout << good.name << " ";
    }
    std::cout << "\n\n";

    // 3. Find optimal number of clusters with radius limit
    double maxClusterRadius = 1.4; // Set your desired maximum cluster radius
    int optimalK = findOptimalK(goods, maxClusterRadius);
    //int optimalK = 3; // For testing, set a fixed number of clusters    
    std::cout << "Optimal number of clusters: " << optimalK << std::endl;
    
    // 4. Run K-means clustering without radius limit
    KMeans kmeans(optimalK);  // No radius limit for fixed k
    for (const auto& good : goods) {
        kmeans.addGood(good);
    }
    kmeans.run();
    kmeans.printClusterResults();
    
    // 5. Get cluster centers and cluster contents
    std::vector<std::string> clusterCenters;
    std::vector<std::vector<std::string>> clusterContents = kmeans.getClusterContents();
    for (int i = 0; i < optimalK; i++) {
        Good center = kmeans.getCentroid(i);
        // Find nearest good to this center
        double minDist = std::numeric_limits<double>::max();
        std::string nearestGood;
        
        for (const auto& good : goods) {
            double dist = distance(center, good);
            if (dist < minDist) {
                minDist = dist;
                nearestGood = good.name;
            }
        }
        clusterCenters.push_back(nearestGood);
    }

    ROS_INFO_STREAM("Finished clustering");

    // 6. Run GA with start location, cluster centers, and end location
    loadCostData(startLocation, clusterCenters);
    initializePopulation(startLocation, clusterCenters);
    // loadCostData(startLocation, destinations);
    // initializePopulation(startLocation, destinations);
    std::vector<std::string> optimalPath = optimize();

    // Create 1D array of goods arranged by clusters
    std::vector<std::string> arrangedGoods;
    // std::vector<int> goodsIndices;

    // goodsIndices.push_back(0);
    // int cntGoods = 0;
    for (size_t i = 0; i < optimalPath.size(); i++) {
        const std::string& center = optimalPath[i];
        // Find cluster index for this center
        for (size_t j = 0; j < clusterCenters.size(); j++) {
            if (clusterCenters[j] == center) {
                // Add all goods from this cluster
                std::vector<std::string> clusterGoods = clusterContents[j];
                
                if(clusterGoods.size() > 1) {
                    if(i < optimalPath.size() - 1) {
                        // Set end position for next cluster
                        setEndPosition(optimalPath[i+1]);
                    } else {
                        if(realEndLocation != "") {
                            // Set end position to end location
                            setEndPosition(realEndLocation);
                        }
                    }
                    loadCostData(startLocation, clusterGoods);
                    initializePopulation(startLocation, clusterGoods);
                    clusterGoods.clear();
                    clusterGoods = optimize();
                    hasEndLocation = false;
                }
                // Randomly shuffle goods within cluster
                // std::random_shuffle(clusterGoods.begin(), clusterGoods.end());
                // Add to arranged goods
                // cntGoods += clusterGoods.size();
                // goodsIndices.push_back(cntGoods);
                arrangedGoods.insert(arrangedGoods.end(), clusterGoods.begin(), clusterGoods.end());
                startLocation = arrangedGoods.back();
                break;
            }
        }
    }
    if(realEndLocation != "") {
        // goodsIndices.push_back(cntGoods+1);
        arrangedGoods.push_back(realEndLocation);
    }

    std::cout << "\n1. Optimal path from " << startLocation << ":\n";
    // res.GA_result.push_back(startLocation);
    // for (const auto& location : optimalPath) {
    //     std::cout << location << " -> ";
    //     res.GA_result.push_back(location);
    // }
    for (const auto& location : arrangedGoods) {
        std::cout << location << " -> ";
        res.GA_result.push_back(location);
    }
    std::cout << "End\n";
    // res.GA_result.push_back("End");
    hasEndLocation = false;
    realEndLocation = "";

    std::cout << "2. Arranged Goods (by cluster order):\n";
    std::cout << "[ ";
    for (size_t i = 0; i < arrangedGoods.size(); i++) {
        std::cout << arrangedGoods[i];
        res.Products.push_back(arrangedGoods[i]);
        if (i < arrangedGoods.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << " ]\n";

    std::cout << "3. Goods Indices:\n";
    // std::cout << "[ ";
    // for (size_t i = 0; i < goodsIndices.size(); i++) {
    //     std::cout << goodsIndices[i];
    //     res.Products_indices.push_back(goodsIndices[i]);
    //     if (i < goodsIndices.size() - 1) {
    //         std::cout << ", ";
    //     }
    // }
    // std::cout << " ]\n";
    std::cout << "[ ";
    for (size_t i = 0; i <= arrangedGoods.size(); i++) {
        std::cout << i;
        res.Products_indices.push_back(i);
        if (i < arrangedGoods.size()) {
            std::cout << ", ";
        }
    }
    std::cout << " ]\n";

    return true;
}
