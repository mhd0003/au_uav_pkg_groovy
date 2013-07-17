// gridSquare.h
// Individual grid square in the air space grid. Also contains a list
// of DangerDescriptors for each plane that "dangers" that square.
// Chris Tate

#ifndef GRID_SQUARE
#define GRID_SQUARE

#include <time.h>
#include <iostream>


struct DangerDescriptor
{
	unsigned int planeID;
	double danger;
};

class GridSquare
{
private:
	double totalDanger;
	std::vector<DangerDescriptor> dangers;

public:
	GridSquare()
	{	
		totalDanger = 0;
	}

	GridSquare(double value)
	{
		totalDanger = value;
	}

	double getTotalDanger() const
	{
		return totalDanger;
	}

	// Adds danger to a square. Can only have 1 DangerDescripto for one plane in a grid. 
	// So addDanger will replace the descritor for the plane if it already exists
	void addDanger(DangerDescriptor newDanger)
	{
		removeDanger(newDanger.planeID);
		dangers.push_back(newDanger);
		totalDanger += newDanger.danger;
	}
	
	// Remove danger for specified plane
	bool removeDanger(unsigned int planeID) 
	{
		int i;
		// finds i position of the planeID. Never more than 1 danger for 1 planeID in a grid
		for (i = 0; i < dangers.size() && dangers[i].planeID != planeID; i++);
		// swap to end of array and remove
		if (i < dangers.size())
		{
			dangers[i] = dangers[dangers.size() - 1];
			dangers.erase(dangers.end() - 1);
		}
	}
};

#endif
