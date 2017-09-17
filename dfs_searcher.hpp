#pragma once
#include "stdhelper/containerhelper.hpp"
#include <stack>
#include <stdint.h>
#include <vector>

enum class DFS_RESULT { FOUND, NOT_FOUND, CUTOFF };

template <typename State, typename IsGoal, typename GenSuccessors>
class DFS_Searcher {
public:
    struct dfs_ret {
        std::vector<State> path;
        DFS_RESULT result;
    };

    DFS_Searcher(const State& start, const IsGoal& isGoal,
                 const GenSuccessors& successors);

    dfs_ret find_next(uint64_t maxDepth);

private:
    const State start;
    const IsGoal isGoal;
    const GenSuccessors successors;
    std::stack<std::pair<State, uint64_t>> nodes_to_visit;
    std::vector<State> path;
};

template <typename State, typename IsGoal, typename GenSuccessors>
DFS_Searcher<State, IsGoal, GenSuccessors>::DFS_Searcher(
    const State& start, const IsGoal& isGoal, const GenSuccessors& successors)
    : start(start), isGoal(isGoal), successors(successors),
      nodes_to_visit({std::make_pair(start, 0)}), path()
{
}

template <typename State, typename IsGoal, typename GenSuccessors>
typename DFS_Searcher<State, IsGoal, GenSuccessors>::dfs_ret
DFS_Searcher<State, IsGoal, GenSuccessors>::find_next(uint64_t maxDepth)
{
    using namespace vvv::helpers;

    bool cutoff = false;

    while (true) {
        if (nodes_to_visit.empty()) {
            if (cutoff)
                return {{}, DFS_RESULT::CUTOFF};
            else
                return {{}, DFS_RESULT::NOT_FOUND};
        }

        const auto& statelvl     = extract(nodes_to_visit);
        const auto& currentState = statelvl.first;
        const auto& currentLvl   = statelvl.second;

        path.resize(currentLvl);
        path.push_back(currentState);

        if (isGoal(currentState))
            return {path, DFS_RESULT::FOUND};

        if (currentLvl >= maxDepth) {
            cutoff = true;
            continue;
        }

        for (const auto& s : successors(currentState))
            if (!contain(path, s))
                nodes_to_visit.push(std::make_pair(s, currentLvl + 1));
    }
}
