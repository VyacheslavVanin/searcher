#include <algorithm>
#include <iostream>
#include <queue>
#include <set>
#include <stack>
#include <unordered_set>
#include <vector>
#include "dfs_searcher.hpp"

char grid[][8] = {
    {0, 1, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 1, 1},
    {1, 1, 1, 1, 1, 0, 0, 0},
    {0, 1, 0, 0, 0, 0, 1, 0},
    {0, 0, 0, 1, 1, 1, 1, 0},
    {1, 1, 0, 0, 0, 0, 1, 1},
    {0, 0, 0, 0, 1, 0, 0, 0},
};

template <typename State, typename Goal, typename Successors>
std::vector<State> bfs_search(const State& start, const Goal& isGoal,
                              const Successors& successors)
{
    using namespace vvv::helpers;

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
    return out << p.second << "; " << p.first;
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
        std::cout << p << "\n";
    std::cout << "-----------------\n";
    const auto path3 = idfs_search(start, goal, succ, 200);
    if (path3.result == DFS_RESULT::FOUND)
        for (const auto& p : path3.path)
            std::cout << p << "\n";

    std::cout << "Searcher !!!!!!!!!!!!!!!!!\n";
    DFS_Searcher searcher(start, goal, succ);
    const auto path4 = searcher.find_next(21);
    const auto path5 = searcher.find_next(21);
    const auto path6 = searcher.find_next(21);
    const auto path7 = searcher.find_next(22);
    const auto path8 = searcher.find_next(22);
    const auto path9 = searcher.find_next(22);
    const auto path10 = searcher.find_next(25);
    const auto path11 = searcher.find_next(25);

    std::cout << "path4 " << path4.result << "\n";
    std::cout << "path5 " << path5.result << "\n";
    std::cout << "path6 " << path6.result << "\n";
    std::cout << "path7 " << path7.result << "\n";
    std::cout << "path8 " << path8.result << "\n";
    std::cout << "path9 " << path9.result << "\n";
    std::cout << "path10 " << path10.result << "\n";
    std::cout << "path11 " << path11.result << "\n";
    for (const auto& p : path11.path)
        std::cout << p << "\n";
}
