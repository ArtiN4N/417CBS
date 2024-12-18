#pragma once

#include "defs.h"
#include "cbs.h"
#include <list>

class MDDNode
{
public:
	MDDNode(AStarLocation loc, MDDNode* par)
	{
		location = loc; 
		if (par == nullptr)
			level = 0;
		else
		{
			level = par->level + 1;
			pars.push_back(par);
		}
	}
	AStarLocation location;
	int level;
	std::list<MDDNode*> childs;
	std::list<MDDNode*> pars;
};


class MDD
{
public:
	std::vector<std::list<MDDNode*>> locsAtTime;

	bool createMDD(
		AStarLocation start, int time, uint agent,
		const std::vector<Constraint>& constraints,
		Map map, HeuristicTable hTable
	);
    void printMDD() const;

    ~MDD();
};

bool detectCardinalConflict(const MDD &mdd1, const MDD &mdd2);

void createAllMDDs(
	std::vector<MDD>& mdds, std::vector<AStarPath> &paths,
	std::vector<Constraint> &constraints, Map &map, std::vector<HeuristicTable> heuristics,
	bool parallel, uint nthreads
);

void grabAllConflictingPairs(
	std::vector<MDD>& mdds, Map& map, std::vector<std::pair<int, int>>& conflictingAgentPairs, bool parallel, uint nthreads
);

void grabAllDGConflictingPairs(
    std::vector<MDD> &mdds, Map &map, std::vector<std::pair<int, int>> &conflictingAgentPairs, bool parallel, uint nthreads
);

bool detectDependency(const MDD &mdd1, const MDD &mdd2);
