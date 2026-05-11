#include <algorithm>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

using std::cout;
using std::endl;
using std::string;
using std::vector;

struct PathResult {
    int distance;
    vector<int> path; // Хранит вершины в индексации 0..N-1
    bool exists;
};

class Graph {
private:
    int vertexCount;
    vector<vector<int>> matrix;
    const int INF = 1000000000;

    PathResult buildPathResult(int start, int finish, const vector<int>& parent, const vector<int>& dist) const {
        PathResult result{INF, {}, false};

        if (dist[finish] >= INF) {
            return result;
        }

        vector<int> path;
        for (int cur = finish; cur != -1; cur = parent[cur]) {
            path.push_back(cur);
        }
        std::reverse(path.begin(), path.end());

        if (path.empty() || path.front() != start) {
            return result;
        }

        result.distance = dist[finish];
        result.path = path;
        result.exists = true;
        return result;
    }

public:
    Graph(int vertexCount) : vertexCount(vertexCount), matrix(vertexCount, vector<int>(vertexCount, INF)) {
        for (int i = 0; i < vertexCount; ++i) {
            matrix[i][i] = 0;
        }
    }

    void addEdge(int from, int to, int weight) {
        int u = from - 1;
        int v = to - 1;
        if (u < 0 || u >= vertexCount || v < 0 || v >= vertexCount || weight < 0) {
            return;
        }
        matrix[u][v] = weight;
    }

    void addUndirectedEdge(int from, int to, int weight) {
        addEdge(from, to, weight);
        addEdge(to, from, weight);
    }

    void printMatrix() const {
        cout << "\nМатрица смежности графа:\n\n";
        cout << "    ";
        for (int i = 0; i < vertexCount; ++i) {
            cout << std::setw(4) << (i + 1);
        }
        cout << "\n";

        for (int i = 0; i < vertexCount; ++i) {
            cout << std::setw(3) << (i + 1) << " ";
            for (int j = 0; j < vertexCount; ++j) {
                if (matrix[i][j] >= INF) {
                    cout << std::setw(4) << "-";
                } else {
                    cout << std::setw(4) << matrix[i][j];
                }
            }
            cout << "\n";
        }
        cout << "\n";
    }

    PathResult dijkstra(int start, int finish, int brokenNode = -1) const {
        // Входные вершины передаются в индексации 1..N
        int s = start - 1;
        int f = finish - 1;
        int broken = (brokenNode == -1) ? -1 : brokenNode - 1;

        if (s < 0 || s >= vertexCount || f < 0 || f >= vertexCount) {
            return {INF, {}, false};
        }
        if (broken != -1 && (s == broken || f == broken)) {
            return {INF, {}, false};
        }

        vector<int> dist(vertexCount, INF);
        vector<int> parent(vertexCount, -1);
        vector<bool> visited(vertexCount, false);
        dist[s] = 0;

        for (int step = 0; step < vertexCount; ++step) {
            int v = -1;
            for (int i = 0; i < vertexCount; ++i) {
                if (visited[i]) continue;
                if (i == broken) continue;
                if (v == -1 || dist[i] < dist[v]) {
                    v = i;
                }
            }

            if (v == -1 || dist[v] >= INF) {
                break;
            }

            visited[v] = true;

            for (int to = 0; to < vertexCount; ++to) {
                if (to == broken) continue;
                if (matrix[v][to] >= INF) continue;
                if (dist[v] + matrix[v][to] < dist[to]) {
                    dist[to] = dist[v] + matrix[v][to];
                    parent[to] = v;
                }
            }
        }

        return buildPathResult(s, f, parent, dist);
    }

    PathResult floydWarshall(int start, int finish, int brokenNode = -1) const {
        int s = start - 1;
        int f = finish - 1;
        int broken = (brokenNode == -1) ? -1 : brokenNode - 1;

        if (s < 0 || s >= vertexCount || f < 0 || f >= vertexCount) {
            return {INF, {}, false};
        }
        if (broken != -1 && (s == broken || f == broken)) {
            return {INF, {}, false};
        }

        vector<vector<int>> dist = matrix;
        vector<vector<int>> next(vertexCount, vector<int>(vertexCount, -1));

        for (int i = 0; i < vertexCount; ++i) {
            if (i == broken) continue;
            for (int j = 0; j < vertexCount; ++j) {
                if (j == broken) continue;
                if (dist[i][j] < INF && i != j) {
                    next[i][j] = j;
                }
            }
        }

        for (int k = 0; k < vertexCount; ++k) {
            if (k == broken) continue;
            for (int i = 0; i < vertexCount; ++i) {
                if (i == broken || dist[i][k] >= INF) continue;
                for (int j = 0; j < vertexCount; ++j) {
                    if (j == broken || dist[k][j] >= INF) continue;
                    if (dist[i][k] + dist[k][j] < dist[i][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        next[i][j] = next[i][k];
                    }
                }
            }
        }

        if (dist[s][f] >= INF || next[s][f] == -1) {
            if (s == f) {
                return {0, {s}, true};
            }
            return {INF, {}, false};
        }

        vector<int> path{ s };
        int cur = s;
        while (cur != f) {
            cur = next[cur][f];
            if (cur == -1) {
                return {INF, {}, false};
            }
            path.push_back(cur);
            if (static_cast<int>(path.size()) > vertexCount + 1) {
                return {INF, {}, false};
            }
        }

        return {dist[s][f], path, true};
    }
};

string pathToString(const vector<int>& path) {
    if (path.empty()) return "-";
    string out;
    for (size_t i = 0; i < path.size(); ++i) {
        out += std::to_string(path[i] + 1); // перевод в нумерацию 1..N
        if (i + 1 != path.size()) out += " -> ";
    }
    return out;
}

void printResultBlock(const string& title, const PathResult& result) {
    cout << title << "\n";
    if (!result.exists) {
        cout << "Путь отсутствует.\n\n";
        return;
    }
    cout << "Путь: " << pathToString(result.path) << "\n";
    cout << "Расстояние: " << result.distance << "\n\n";
}

bool compareWithExpected(const PathResult& actual, int expectedDistance, const vector<int>& expectedPathOneBased) {
    if (!actual.exists) return false;
    if (actual.distance != expectedDistance) return false;
    if (actual.path.size() != expectedPathOneBased.size()) return false;
    for (size_t i = 0; i < actual.path.size(); ++i) {
        if (actual.path[i] + 1 != expectedPathOneBased[i]) {
            return false;
        }
    }
    return true;
}

void runControlTest() {
    cout << "\n==================== КОНТРОЛЬНЫЙ ТЕСТ ====================\n";

    cout << "[Контрольная точка] Граф создан\n";
    Graph testGraph(4);

    testGraph.addUndirectedEdge(1, 2, 3);
    testGraph.addUndirectedEdge(2, 3, 2);
    testGraph.addUndirectedEdge(1, 3, 10);
    testGraph.addUndirectedEdge(3, 4, 1);
    testGraph.addUndirectedEdge(2, 4, 8);
    cout << "[Контрольная точка] Рёбра добавлены\n";

    int K = 1, M = 4, T = 3;
    cout << "[Контрольная точка] Выбраны K, M и T\n";
    cout << "K = " << K << "\nM = " << M << "\nT = " << T << "\n\n";

    cout << "Ожидаемый результат до сбоя:\n";
    cout << "Путь: 1 -> 2 -> 3 -> 4\n";
    cout << "Расстояние: 6\n\n";

    cout << "Ожидаемый результат после сбоя:\n";
    cout << "Путь: 1 -> 2 -> 4\n";
    cout << "Расстояние: 11\n\n";

    cout << "[Контрольная точка] Запуск алгоритма Дейкстры до сбоя\n";
    PathResult dBefore = testGraph.dijkstra(K, M);
    cout << "[Контрольная точка] Запуск алгоритма Дейкстры после сбоя\n";
    PathResult dAfter = testGraph.dijkstra(K, M, T);

    cout << "[Контрольная точка] Запуск алгоритма Флойда до сбоя\n";
    PathResult fBefore = testGraph.floydWarshall(K, M);
    cout << "[Контрольная точка] Запуск алгоритма Флойда после сбоя\n";
    PathResult fAfter = testGraph.floydWarshall(K, M, T);

    cout << "Полученный результат до сбоя (Дейкстра):\n";
    printResultBlock("", dBefore);
    cout << "Полученный результат после сбоя (Дейкстра):\n";
    printResultBlock("", dAfter);

    cout << "Полученный результат до сбоя (Флойд-Уоршелл):\n";
    printResultBlock("", fBefore);
    cout << "Полученный результат после сбоя (Флойд-Уоршелл):\n";
    printResultBlock("", fAfter);

    bool dBeforeOk = compareWithExpected(dBefore, 6, {1, 2, 3, 4});
    bool dAfterOk = compareWithExpected(dAfter, 11, {1, 2, 4});
    bool fBeforeOk = compareWithExpected(fBefore, 6, {1, 2, 3, 4});
    bool fAfterOk = compareWithExpected(fAfter, 11, {1, 2, 4});

    cout << "[Контрольная точка] Выполнено сравнение результата с ожидаемым\n\n";
    cout << "Проверка:\n";
    cout << "Дейкстра до сбоя: " << (dBeforeOk ? "OK" : "Ошибка") << "\n";
    cout << "Дейкстра после сбоя: " << (dAfterOk ? "OK" : "Ошибка") << "\n";
    cout << "Флойд до сбоя: " << (fBeforeOk ? "OK" : "Ошибка") << "\n";
    cout << "Флойд после сбоя: " << (fAfterOk ? "OK" : "Ошибка") << "\n";
    cout << "==========================================================\n\n";
}

void runMainTask() {
    cout << "\n==================== ОСНОВНАЯ ЗАДАЧА ====================\n";
    cout << "[Контрольная точка] Граф создан\n";

    Graph graph(8);

    // Блок рёбер для варианта 13 (можно изменить под ваш рисунок варианта)
    graph.addUndirectedEdge(1, 2, 3);
    graph.addUndirectedEdge(2, 3, 6);
    graph.addUndirectedEdge(2, 4, 4);
    graph.addUndirectedEdge(2, 5, 2);
    graph.addUndirectedEdge(3, 4, 2);
    graph.addUndirectedEdge(4, 5, 1);
    graph.addUndirectedEdge(5, 6, 1);
    graph.addUndirectedEdge(6, 7, 5);
    graph.addUndirectedEdge(7, 8, 3);
    graph.addUndirectedEdge(5, 8, 3);

    // Пример того, как добавлять остальные рёбра варианта 13:
    // graph.addUndirectedEdge(u, v, w);

    cout << "[Контрольная точка] Рёбра добавлены\n";
    graph.printMatrix();

    std::mt19937 rng(static_cast<unsigned int>(std::time(nullptr)));
    std::uniform_int_distribution<int> pick(1, 8);

    int K = pick(rng);
    int M = pick(rng);
    while (M == K) {
        M = pick(rng);
    }

    int T = pick(rng);
    while (T == K || T == M) {
        T = pick(rng);
    }

    cout << "[Контрольная точка] Выбраны K, M и T\n";
    cout << "Случайно выбранные вершины:\n";
    cout << "K = " << K << "\n";
    cout << "M = " << M << "\n";
    cout << "T = " << T << "\n\n";

    cout << "[Контрольная точка] Запуск алгоритма Дейкстры\n";
    PathResult dBefore = graph.dijkstra(K, M);
    PathResult dAfter = graph.dijkstra(K, M, T);
    cout << "[Контрольная точка] Расчёт Дейкстры завершён\n";

    cout << "[Контрольная точка] Запуск алгоритма Флойда\n";
    PathResult fBefore = graph.floydWarshall(K, M);
    PathResult fAfter = graph.floydWarshall(K, M, T);
    cout << "[Контрольная точка] Расчёт Флойда завершён\n\n";

    cout << "Алгоритм Дейкстры:\n";
    printResultBlock("До сбоя:", dBefore);
    cout << "После сбоя:\n";
    cout << "Неисправная вершина: " << T << "\n";
    if (!dAfter.exists) {
        cout << "Путь отсутствует.\n\n";
    } else {
        cout << "Путь: " << pathToString(dAfter.path) << "\n";
        cout << "Расстояние: " << dAfter.distance << "\n\n";
    }

    cout << "Алгоритм Флойда-Уоршелла:\n";
    printResultBlock("До сбоя:", fBefore);
    cout << "После сбоя:\n";
    cout << "Неисправная вершина: " << T << "\n";
    if (!fAfter.exists) {
        cout << "Путь отсутствует.\n\n";
    } else {
        cout << "Путь: " << pathToString(fAfter.path) << "\n";
        cout << "Расстояние: " << fAfter.distance << "\n\n";
    }

    cout << "=========================================================\n\n";
}

int main() {
    cout << "Лабораторная работа: графы и кратчайшие пути\n";
    cout << "1 — выполнить контрольный тест\n";
    cout << "2 — выполнить основную задачу\n";
    cout << "3 — выполнить оба режима\n";
    cout << "Выберите режим: ";

    int mode = 0;
    std::cin >> mode;

    if (!std::cin) {
        cout << "Ошибка ввода. Завершение программы.\n";
        return 0;
    }

    if (mode == 1) {
        runControlTest();
    } else if (mode == 2) {
        runMainTask();
    } else if (mode == 3) {
        runControlTest();
        runMainTask();
    } else {
        cout << "Неизвестный режим. Завершение программы.\n";
    }

    return 0;
}
