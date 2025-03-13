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

/*
 * run spherical k-means multiple times and use simple "elbow method" model selection to automatically determine the optimal k.
 * parameters:
 *   directions   : set of edge direction vectors (normalized and preprocessed for consistent orientation)
 *   k_min, k_max : range of candidate cluster numbers
 *   maxIterations: maximum iterations for each clustering
 *   tolerance    : convergence tolerance
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
                          float &bestInertia)
{
    int optimalK = k_min;
    float prevInertia = std::numeric_limits<float>::max();
    float bestDelta = std::numeric_limits<float>::max();
    bestInertia = std::numeric_limits<float>::max();
    vector<int> labelsOptimal;

    const int numRuns = 5; // run multiple times for each k to get optimal result
    for (int k = k_min; k <= k_max; k++) {
        float inertia_k = std::numeric_limits<float>::max();
        vector<int> labels_k;
        
        // multiple runs, record minimum inertia
        for (int run = 0; run < numRuns; run++) {
            float inertia = 0.0f;
            vector<int> labels = sphericalKMeans(directions, k, maxIterations, tolerance, inertia);
            if (inertia < inertia_k) {
                inertia_k = inertia;
                labels_k = labels;
            }
        }
        
        if (k > k_min) {
            float delta = prevInertia - inertia_k;
            if (delta < bestDelta) {
                bestDelta = delta;
                optimalK = k;
                bestInertia = inertia_k;
                bestLabels = labels_k;
            }
        } else {
            bestInertia = inertia_k;
            bestLabels = labels_k;
        }
        prevInertia = inertia_k;
    }

    // recalculate centers for optimal clustering
    int n = directions.size();
    bestCenters.resize(optimalK);
    vector<int> counts(optimalK, 0);

    // calculate new center for each cluster
    for (int j = 0; j < optimalK; j++) {
        vec3 centroid(0, 0, 0);
        
        // accumulate all direction vectors in this cluster
        for (int i = 0; i < n; i++) {
            if (bestLabels[i] == j) {
                centroid = centroid + directions[i];
                counts[j]++;
            }
        }

        // if cluster is not empty, normalize to get main direction
        if (counts[j] > 0) {
            bestCenters[j] = centroid.normalize();
        }
    }

    return optimalK;
}

#endif // SPHERICAL_KMEANS_H