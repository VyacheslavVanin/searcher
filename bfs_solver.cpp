#include <algorithm>
#include <iostream>
#include <queue>
#include <set>
#include <stack>
#include <unordered_set>
#include <vector>

char grid[][8] = {
    {0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 0, 1},
    {1, 1, 1, 1, 1, 0, 0, 0},
    {0, 1, 0, 0, 0, 0, 1, 0},
    {0, 0, 0, 1, 1, 1, 1, 0},
    {1, 1, 0, 0, 0, 0, 1, 1},
    {0, 0, 0, 0, 1, 0, 0, 0},
};

template <typename T>
T extract(std::stack<T>& c)
{
    T ret = std::move(c.top());
    c.pop();
    return ret;
}

template <typename T>
T extract(std::queue<T>& c)
{
    T ret = std::move(c.front());
    c.pop();
    return ret;
}

template <typename C, typename V>
bool contain(const C& c, const V& v)
{
    return c.find(v) != c.end();
}

template <typename State, typename IsGoal, typename GenSuccessors>
std::vector<State> dfs_search(const State& start, const IsGoal& isGoal,
                              const GenSuccessors& successors)
{
    std::stack<std::pair<State, int>> nodes_to_visit;
    std::set<State> visited;
    std::vector<State> ret;

    nodes_to_visit.push(std::make_pair(start, 0));

    while (!nodes_to_visit.empty()) {
        const auto & [ currentState, currentLvl ] = extract(nodes_to_visit);

        ret.resize(currentLvl);
        ret.push_back(currentState);

        if (isGoal(currentState))
            return ret;

        visited.insert(currentState);
        for (const auto& s : successors(currentState))
            if (!contain(visited, s))
                nodes_to_visit.push(std::make_pair(s, currentLvl + 1));
    }

    return {};
}


enum class DFS_RESULT {
    FOUND,
    NOT_FOUND,
    CUTOFF
};

std::ostream& operator << (std::ostream& str, const DFS_RESULT& r)
{
    switch (r) {
        case DFS_RESULT::FOUND: str << "FOUND"; break;
        case DFS_RESULT::NOT_FOUND: str << "NOT FOUND"; break;
        case DFS_RESULT::CUTOFF: str << "CUTOFF"; break;
    }
    return str;
}

template <typename State>
struct dfs_ret {
    std::vector<State> path;
    DFS_RESULT result;
};


template <typename State, typename IsGoal, typename GenSuccessors>
dfs_ret<State> dfs_search(const State& start, const IsGoal& isGoal,
                          const GenSuccessors& successors, size_t maxDepth)
{
    std::stack<std::pair<State, int>> nodes_to_visit;
    std::set<State> visited;
    std::vector<State> ret;
    bool cutoff = false;

    nodes_to_visit.push(std::make_pair(start, 0));

    while (true) {
        if (nodes_to_visit.empty()) {
            if (cutoff)
                return {{}, DFS_RESULT::CUTOFF};
            else
                return {{}, DFS_RESULT::NOT_FOUND};
        }

        const auto & [ currentState, currentLvl ] = extract(nodes_to_visit);

        ret.resize(currentLvl);
        ret.push_back(currentState);

        if (isGoal(currentState))
            return {std::move(ret), DFS_RESULT::FOUND};

        visited.insert(currentState);
        if (currentLvl >= maxDepth) {
            cutoff = true;
            continue;
        }

        for (const auto& s : successors(currentState))
            if (!contain(visited, s))
                nodes_to_visit.push(std::make_pair(s, currentLvl + 1));
    }
}

template <typename State, typename IsGoal, typename GenSuccessors>
dfs_ret<State> idfs_search(const State& start, const IsGoal& isGoal,
                           const GenSuccessors& successors, size_t maxDepth)
{
    dfs_ret<State> ret{{}, DFS_RESULT::NOT_FOUND};
    for (size_t depth = 1; depth < maxDepth; ++depth) {
        ret = dfs_search(start, isGoal, successors, depth);
        if (ret.result == DFS_RESULT::FOUND ||
            ret.result == DFS_RESULT::NOT_FOUND)
            break;
    }
    return ret;
}

template <typename State, typename Goal, typename Successors>
std::vector<State> bfs_search(const State& start, const Goal& isGoal,
                              const Successors& successors)
{
    using path = std::vector<State>;
    std::queue<path> posibleways;
    posibleways.push({start});
    std::set<State> visited;
    visited.insert(start);

    while (!posibleways.empty()) {
        const auto current_path = extract(posibleways);
        const auto& pos         = current_path.back();

        if (isGoal(pos))
            return current_path;

        for (const auto& s : successors(pos))
            if (!contain(visited, s)) {
                auto new_path{current_path};
                new_path.push_back(s);
                posibleways.push(std::move(new_path));
                visited.insert(s);
            }
    }

    return {};
}

std::ostream& operator<<(std::ostream& out, const std::pair<int, int>& p)
{
    return out << p.first << "; " << p.second << '\n';
}

const auto R = 7;
const auto C = 8;

int main()
{
    using point = std::pair<int, int>;

    auto succ = [](const point& p) {
        const auto maxr = R - 1;
        const auto maxc = C - 1;
        const auto r    = p.first;
        const auto c    = p.second;

        std::vector<point> ret;
        if (r < maxr && grid[r + 1][c] == 0)
            ret.push_back({r + 1, c});
        if (c < maxc && grid[r][c + 1] == 0)
            ret.push_back({r, c + 1});

        if (r > 0 && grid[r - 1][c] == 0)
            ret.push_back({r - 1, c});
        if (c > 0 && grid[r][c - 1] == 0)
            ret.push_back({r, c - 1});

        return ret;
    };

    auto goal = [](const point& p) {
        const auto maxr = R - 1;
        const auto maxc = C - 1;
        const auto r    = p.first;
        const auto c    = p.second;

        return r == maxr && c == maxc;
    };

    const auto start = std::make_pair(0, 0);
    auto path        = bfs_search(start, goal, succ);
    for (const auto& p : path)
        std::cout << p;
    std::cout << "-----------------\n";
    auto path2 = dfs_search(start, goal, succ);
    for (const auto& p : path2)
        std::cout << p;

    const auto path3 = idfs_search(start, goal, succ, 200);
    if (path3.result == DFS_RESULT::FOUND)
        for (const auto& p : path3.path)
            std::cout << p;
}