#include "au_uav_ros/gridSquare.h"
using namespace au_uav_ros;

/* (from header)
	struct DangerDescriptor
	{
		unsigned int planeID;
		double danger;
	};

	double totalDanger;
	std::vector<DangerDescriptor> dangers;
*/

GridSquare::GridSquare() {
	totalDanger = 0;
}

GridSquare::GridSquare(double value) {
	totalDanger = value;
}

double GridSquare::getTotalDanger() const {
	return totalDanger;
}

// Adds danger to a square. Can only have 1 DangerDescripto for one plane in a grid. 
// So addDanger will replace the descritor for the plane if it already exists
void GridSquare::addDanger(DangerDescriptor newDanger) {
	removeDanger(newDanger.planeID);
	dangers.push_back(newDanger);
	totalDanger += newDanger.danger;
}

// Remove danger for specified plane
bool GridSquare::removeDanger(unsigned int planeID) {
	int i;
	// finds i position of the planeID. Never more than 1 danger for 1 planeID in a grid
	for (i = 0; i < dangers.size() && dangers[i].planeID != planeID; i++);
	// swap to end of array and remove
	if (i < dangers.size()) {
		dangers[i] = dangers[dangers.size() - 1];
		dangers.erase(dangers.end() - 1);
	}
}
