#ifndef SPHERICAL_KMEANS_H
#define SPHERICAL_KMEANS_H

#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <cstdlib> // for std::abs
#include "easy3d/core/point_cloud.h"  // 假设easy3d::vec3已经定义了加、减、除法、点乘等操作

using std::vector;
using namespace easy3d;

// 计算余弦相似度（由于向量均归一化，直接使用点积即可）
inline float cosine_similarity(const vec3 &a, const vec3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/*
 * 球面k-means算法（单次运行）
 * 参数：
 *   directions：已归一化的边方向集合
 *   k：聚类数
 *   maxIterations：最大迭代次数
 *   tolerance：收敛容差（惯性变化低于该值则认为收敛）
 * 返回值：
 *   labels：每个方向对应的聚类标签，并通过 inertia 输出该次运行的惯性值（误差之和）
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

    // 随机初始化中心（随机选取k个数据点）
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, n-1);
    for (int i = 0; i < k; i++) {
        centers[i] = directions[dist(rng)];
    }

    float prevInertia = std::numeric_limits<float>::max();
    for (int iter = 0; iter < maxIterations; iter++) {
        // 分配步骤：对于每个数据点，选择与它余弦相似度最高的中心
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
        
        // 更新步骤：对每个聚类，将其中心更新为所有归类向量的和后归一化
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
                // 如果某簇为空，则重新随机初始化
                newCenters[j] = directions[dist(rng)];
            }
        }
        centers = newCenters;
        
        // 计算惯性：我们这里用 sum(1 - 余弦相似度) 作为误差度量
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
 * 多次运行球面k-means并使用简单的“肘部法”模型选择来自动确定最佳簇数k。
 * 参数：
 *   directions   : 边方向向量集合（归一化且预处理使得正反向一致）
 *   k_min, k_max : 候选聚类个数范围
 *   maxIterations: 每次聚类的最大迭代次数
 *   tolerance    : 收敛容差
 * 输出：
 *   bestLabels   : 最佳聚类时每个数据点的簇标签
 *   bestCenters  : 最佳聚类的聚类中心（代表主要方向）
 *   bestInertia  : 最佳聚类对应的惯性值
 * 返回值：
 *   最佳的聚类个数k
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
    // 用于记录“肘部”处的惯性降低幅度
    float bestDelta = std::numeric_limits<float>::max();
    bestInertia = std::numeric_limits<float>::max();
    vector<int> labelsOptimal;
    vector<vec3> centersOptimal;

    const int numRuns = 5; // 每个k运行多次取最优结果
    for (int k = k_min; k <= k_max; k++) {
        float inertia_k = std::numeric_limits<float>::max();
        vector<int> labels_k;
        vector<vec3> centers_k; // 后面重新计算
        // 多次运行，记录最小惯性
        for (int run = 0; run < numRuns; run++) {
            float inertia = 0.0f;
            vector<int> labels = sphericalKMeans(directions, k, maxIterations, tolerance, inertia);
            if (inertia < inertia_k) {
                inertia_k = inertia;
                labels_k = labels;
            }
        }
        // 如果不是第一个k，计算惯性降低差值
        if (k > k_min) {
            float delta = prevInertia - inertia_k;
            // 当delta下降不明显时，选k=min时的值作为最佳
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
    // 根据最佳聚类标签重新计算各聚类中心
    int n = directions.size();
    bestCenters.resize(optimalK, vec3(0, 0, 0));
    vector<int> counts(optimalK, 0);
    for (int i = 0; i < n; i++) {
        bestCenters[ bestLabels[i] ] = bestCenters[ bestLabels[i] ] + directions[i];
        counts[ bestLabels[i] ]++;
    }
    for (int j = 0; j < optimalK; j++) {
        if (counts[j] > 0)
            bestCenters[j] = bestCenters[j].normalize();
    }
    return optimalK;
}

#endif // SPHERICAL_KMEANS_H