#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <utility>
#include <map>
#include <algorithm>

// 무한대 값 설정
const double INF = std::numeric_limits<double>::infinity();

// 그래프에서 사용하는 간선 구조체 정의
struct Edge {
    int node;
    double cost;
};

// 다익스트라 알고리즘 함수
std::vector<int> dijkstra(int start, int target, const std::map<int, std::vector<Edge>>& graph) {
    std::map<int, double> distance;    // start 노드로부터의 최단 거리를 저장할 map
    std::map<int, int> previous;       // 경로를 추적하기 위한 이전 노드 저장 map
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;  // 우선순위 큐

    // 초기화
    for (const auto& entry : graph) {
        distance[entry.first] = INF;
    }
    distance[start] = 0;
    pq.push({0, start});

    // 다익스트라 알고리즘 실행
    while (!pq.empty()) {
        double current_distance = pq.top().first;
        int current_node = pq.top().second;
        pq.pop();

        // 이미 처리된 노드 무시
        if (current_distance > distance[current_node]) continue;

        // 인접한 모든 노드를 확인
        for (const auto& edge : graph.at(current_node)) {
            int next_node = edge.node;
            double new_distance = current_distance + edge.cost;

            // 더 짧은 경로 발견 시 업데이트
            if (new_distance < distance[next_node]) {
                distance[next_node] = new_distance;
                previous[next_node] = current_node;
                pq.push({new_distance, next_node});
            }
        }
    }

    // target 노드로 가는 경로가 없는 경우
    if (distance[target] == INF) {
        std::cout << "No path exists from node " << start << " to node " << target << std::endl;
        return {};
    }

    // 경로 복원
    std::vector<int> path;
    for (int at = target; at != start; at = previous[at]) {
        path.push_back(at);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

int main() {
    // 그래프 정의 (노드 번호는 0부터 시작한다고 가정)
    std::map<int, std::vector<Edge>> graph;
    graph[0] = {{1, 4.0}, {2, 1.0}};
    graph[1] = {{3, 1.0}};
    graph[2] = {{1, 2.0}, {3, 5.0}};
    graph[3] = {};

    int start = 0;  // 시작 노드
    int target = 3; // 목적지 노드

    // 다익스트라 알고리즘 실행 및 경로 출력
    std::vector<int> shortest_path = dijkstra(start, target, graph);

    std::cout << "Shortest path from node " << start << " to node " << target << ": ";
    for (int node : shortest_path) {
        std::cout << node << " ";
    }
    std::cout << std::endl;

    return 0;
}
