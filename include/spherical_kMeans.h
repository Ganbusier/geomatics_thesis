#ifndef SPHERICAL_KMEANS_H
#define SPHERICAL_KMEANS_H

#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <cstdlib> // for std::abs
#include "easy3d/core/point_cloud.h"  // assuming easy3d::vec3 has defined operations like add, subtract, divide, dot product, etc.

using std::vector;
using namespace easy3d;

// calculate cosine similarity (since vectors are normalized, directly use dot product)
inline float cosine_similarity(const vec3 &a, const vec3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/*
 * spherical k-means algorithm (single run)
 * parameters:
 *   directions: set of normalized edge directions
 *   k: number of clusters
 *   maxIterations: maximum number of iterations
 *   tolerance: convergence tolerance (considered converged if inertia change is below this value)
 * return value:
 *   labels: cluster label for each direction, and outputs the inertia value (sum of errors) for this run
 */
inline vector<int> sphericalKMeans(const vector<vec3>& directions,
                                   int k,
                                   int maxIterations,
                                   float tolerance,
                                   float &inertia)
{
    size_t n = directions.size();
    vector<int> labels(n, -1);
    vector<vec3> centers(k);

    // randomly initialize centers (randomly select k data points)
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, n-1);
    for (int i = 0; i < k; i++) {
        centers[i] = directions[dist(rng)];
    }

    float prevInertia = std::numeric_limits<float>::max();
    for (int iter = 0; iter < maxIterations; iter++) {
        // assignment step: for each data point, choose the center with highest cosine similarity
        for (size_t i = 0; i < n; i++) {
            float bestSim = -std::numeric_limits<float>::max();
            int bestCluster = 0;
            for (int j = 0; j < k; j++) {
                float sim = cosine_similarity(directions[i], centers[j]);
                if (sim > bestSim) {
                    bestSim = sim;
                    bestCluster = j;
                }
            }
            labels[i] = bestCluster;
        }
        
        // update step: for each cluster, update its center as the normalized sum of all assigned vectors
        vector<vec3> newCenters(k, vec3(0, 0, 0));
        vector<int> counts(k, 0);
        for (size_t i = 0; i < n; i++) {
            newCenters[labels[i]] = newCenters[labels[i]] + directions[i];
            counts[labels[i]]++;
        }
        for (int j = 0; j < k; j++) {
            if (counts[j] > 0) {
                newCenters[j] = newCenters[j].normalize();
            } else {
                // if a cluster is empty, reinitialize randomly
                newCenters[j] = directions[dist(rng)];
            }
        }
        centers = newCenters;
        
        // calculate inertia: we use sum(1 - cosine similarity) as error measure
        inertia = 0.0f;
        for (size_t i = 0; i < n; i++) {
            float sim = cosine_similarity(directions[i], centers[labels[i]]);
            inertia += (1.0f - sim);
        }
        
        if (std::abs(prevInertia - inertia) < tolerance) {
            break;
        }
        prevInertia = inertia;
    }
    return labels;
}

// evaluation method enum for clustering
enum class ClusterEvaluationMethod {
    ELBOW,          // original elbow method
    SILHOUETTE,     // silhouette coefficient method
    COMPREHENSIVE   // comprehensive evaluation method
};

// metrics structure for cluster evaluation
struct ClusteringMetrics {
    float inertia;
    float separation;
    float silhouette;
    
    ClusteringMetrics() : inertia(0), separation(0), silhouette(0) {}
    
    // comprehensive score with adjustable weights
    float comprehensive_score() const {
        const float w1 = 0.4f;  // weight for inertia
        const float w2 = 0.3f;  // weight for separation
        const float w3 = 0.3f;  // weight for silhouette
        return -(w1 * inertia) + w2 * separation + w3 * silhouette;
    }
};

// calculate separation between clusters
inline float calculate_cluster_separation(const vector<vec3>& centers) {
    float separation = 0;
    for(size_t i = 0; i < centers.size(); i++) {
        for(size_t j = i + 1; j < centers.size(); j++) {
            float dot_product = centers[i].x * centers[j].x + 
                              centers[i].y * centers[j].y + 
                              centers[i].z * centers[j].z;
            dot_product = std::max(std::min(dot_product, 1.0f), -1.0f);
            float angle = std::acos(std::abs(dot_product));
            separation += angle;
        }
    }
    return centers.size() > 1 ? separation / (centers.size() * (centers.size() - 1) / 2) : 0;
}

// calculate silhouette coefficient
inline float calculate_silhouette(const vector<vec3>& directions,
                                const vector<int>& labels,
                                const vector<vec3>& centers) {
    float total_silhouette = 0;
    for(size_t i = 0; i < directions.size(); i++) {
        // calculate a(i): average distance to points in the same cluster
        float a_i = 0;
        int count_same_cluster = 0;
        for(size_t j = 0; j < directions.size(); j++) {
            if(i != j && labels[i] == labels[j]) {
                float dot_product = directions[i].x * directions[j].x + 
                                  directions[i].y * directions[j].y + 
                                  directions[i].z * directions[j].z;
                a_i += 1.0f - std::abs(dot_product);  // use 1-cos as distance measure
                count_same_cluster++;
            }
        }
        a_i = count_same_cluster > 0 ? a_i / count_same_cluster : 0;

        // calculate b(i): average distance to points in the nearest other cluster
        float b_i = std::numeric_limits<float>::max();
        for(size_t c = 0; c < centers.size(); c++) {
            if(c != labels[i]) {
                float cluster_dist = 0;
                int count_other_cluster = 0;
                for(size_t j = 0; j < directions.size(); j++) {
                    if(labels[j] == c) {
                        float dot_product = directions[i].x * directions[j].x + 
                                          directions[i].y * directions[j].y + 
                                          directions[i].z * directions[j].z;
                        cluster_dist += 1.0f - std::abs(dot_product);
                        count_other_cluster++;
                    }
                }
                if(count_other_cluster > 0) {
                    cluster_dist /= count_other_cluster;
                    b_i = std::min(b_i, cluster_dist);
                }
            }
        }
        
        if(b_i != std::numeric_limits<float>::max()) {
            float silhouette_i = (b_i - a_i) / std::max(a_i, b_i);
            total_silhouette += silhouette_i;
        }
    }
    return directions.size() > 0 ? total_silhouette / directions.size() : 0;
}

/*
 * run spherical k-means multiple times and select optimal k using specified evaluation method
 * parameters:
 *   directions   : set of edge direction vectors (normalized and preprocessed for consistent orientation)
 *   k_min, k_max : range of candidate cluster numbers
 *   maxIterations: maximum iterations for each clustering
 *   tolerance    : convergence tolerance
 *   eval_method  : method to evaluate clustering quality (default: elbow method)
 * output:
 *   bestLabels   : cluster labels for each data point in optimal clustering
 *   bestCenters  : cluster centers (representing main directions)
 *   bestInertia  : inertia value for optimal clustering
 * return value:
 *   optimal number of clusters k
 */
inline int chooseOptimalK(const vector<vec3>& directions,
                         int k_min,
                         int k_max,
                         int maxIterations,
                         float tolerance,
                         vector<int>& bestLabels,
                         vector<vec3>& bestCenters,
                         float &bestInertia,
                         ClusterEvaluationMethod eval_method = ClusterEvaluationMethod::ELBOW) {
    int optimalK = k_min;
    float prevInertia = std::numeric_limits<float>::max();
    float bestMetric = eval_method == ClusterEvaluationMethod::ELBOW ? 
                      std::numeric_limits<float>::max() : 
                      -std::numeric_limits<float>::max();
    bestInertia = std::numeric_limits<float>::max();
    vector<int> labelsOptimal;
    
    const int numRuns = 5;  // run multiple times for each k to get optimal result
    for (int k = k_min; k <= k_max; k++) {
        float inertia_k = std::numeric_limits<float>::max();
        vector<int> labels_k;
        vector<vec3> centers_k;
        
        // multiple runs, record best result
        for (int run = 0; run < numRuns; run++) {
            float inertia = 0.0f;
            vector<int> labels = sphericalKMeans(directions, k, maxIterations, tolerance, inertia);
            if (inertia < inertia_k) {
                inertia_k = inertia;
                labels_k = labels;
                
                // calculate cluster centers
                centers_k.clear();
                centers_k.resize(k);
                vector<int> counts(k, 0);
                for (size_t i = 0; i < directions.size(); i++) {
                    centers_k[labels[i]] = centers_k[labels[i]] + directions[i];
                    counts[labels[i]]++;
                }
                for (int j = 0; j < k; j++) {
                    if (counts[j] > 0) {
                        centers_k[j] = centers_k[j].normalize();
                    }
                }
            }
        }
        
        // calculate metric based on chosen evaluation method
        float current_metric;
        switch(eval_method) {
            case ClusterEvaluationMethod::ELBOW: {
                float delta = prevInertia - inertia_k;
                current_metric = delta;
                break;
            }
            case ClusterEvaluationMethod::SILHOUETTE: {
                current_metric = calculate_silhouette(directions, labels_k, centers_k);
                break;
            }
            case ClusterEvaluationMethod::COMPREHENSIVE: {
                ClusteringMetrics metrics;
                metrics.inertia = inertia_k;
                metrics.separation = calculate_cluster_separation(centers_k);
                metrics.silhouette = calculate_silhouette(directions, labels_k, centers_k);
                current_metric = metrics.comprehensive_score();
                break;
            }
        }
        
        // update best result
        bool is_better = (eval_method == ClusterEvaluationMethod::ELBOW) ? 
                        (current_metric < bestMetric) : 
                        (current_metric > bestMetric);
        
        if (is_better) {
            bestMetric = current_metric;
            optimalK = k;
            bestInertia = inertia_k;
            bestLabels = labels_k;
            bestCenters = centers_k;
        }
        
        prevInertia = inertia_k;
    }
    
    return optimalK;
}

#endif // SPHERICAL_KMEANS_H