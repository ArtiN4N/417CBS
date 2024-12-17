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

	bool createMDD(AStarLocation start, int time, uint agent, const std::vector<Constraint>& constraints, Map map, HeuristicTable hTable);
    void printMDD() const;

    ~MDD();
};
