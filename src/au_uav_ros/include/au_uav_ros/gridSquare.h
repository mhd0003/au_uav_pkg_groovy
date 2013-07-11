// gridSquare.h
// Individual grid square in the air space grid. Also contains a list
// of DangerDescriptors for each plane that "dangers" that square.
// Chris Tate

#ifndef _GRID_SQUARE_H_
#define _GRID_SQUARE_H_

// #include <time.h>
// #include <iostream>

namespace au_uav_ros {
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
		GridSquare();

		GridSquare(double value);

		double getTotalDanger() const;
		
		// Adds danger to a square. Can only have 1 DangerDescripto for one plane in a grid. 
		// So addDanger will replace the descritor for the plane if it already exists
		void addDanger(DangerDescriptor newDanger);
		
		// Remove danger for specified plane
		bool removeDanger(unsigned int planeID);
	};
};

#endif
