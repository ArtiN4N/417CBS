#include "../include/cbstools.h"




/*
void printAStarPath(AStarPath path) {
    uint ret = 0;
    for (auto loc : path)
        std::cout << "(" << loc.second << "," << loc.first << ")->";
    std::cout << std::endl;
}

void printCollisionLocation(CollisionLocation l) {
    std::cout << "[ (" << l.l1.first << "," << l.l1.second << ")";
    if (l.isEdgeCollision)
        std::cout << ", (" << l.l2.first << "," << l.l2.second << ")";
    std::cout << " ]";
}

void printConstraint(Constraint c) {
    std::cout << " id=" << c.agentId << " time=" << c.timeStep << " location=";
    printCollisionLocation(c.location);
    std::cout << std::endl;
}

void printResults(std::vector<AStarPath> paths, uint nExpanded, uint nGenerated, double elapsed, std::string filename) {
    std::ofstream file;

    //std::string text = std::to_string(elapsed) + "," + std::to_string(getSumOfCost(paths));
    std::string text = std::to_string(elapsed) + "," + std::to_string(0);

    file.open("autotest/tests/" + filename, std::ios::app); // open to append mode
    file << text << std::endl;
    file.close();

    //std::cout << "Found a solution!" << std::endl;
    //std::cout << "Time elapsed : " << elapsed << std::endl;
    //std::cout << "Sum of costs : " << getSumOfCost(paths) << std::endl;
    //std::cout << "Expanded nodes : " << nExpanded << std::endl;
    //std::cout << "Generated nodes : " << nGenerated << std::endl;
}
*/