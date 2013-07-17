// Used to output logs of the planned A* path in fun ASCII format!

#ifndef PATH_LOGGER
#define PATH_LOGGER

#include "map_tools.h"
#include "dangerGrid.h"
#include "Position.h"


#include <queue>
#include <iostream>
#include <string>
#include <sstream>

// protos
void printDanger(DangerGrid *bc, int currentTime);

void printPath(std::queue<point> fullPath, int planeID, int startx, int starty, int endx, int endy, DangerGrid *bc) {
	int widthX = bc->get_width_in_squares();
	int heightY = bc->get_height_in_squares();
	string gridToPrint[widthX][heightY];

	vector<point> vFullPath;
	while (!fullPath.empty()) {
		vFullPath.push_back(fullPath.front());
		fullPath.pop();
	}

	// initialize grid
	for (int i = 0; i < widthX; i++)
		for (int j = 0; j < heightY; j++)
			gridToPrint[i][j] = "-";
	
	// place start and end, denoted with $ and @
	gridToPrint[startx][starty] = "$";
	gridToPrint[endx][endy] = "@";

	// place path onto grid, denoted with *
	for (int i = 0; i < vFullPath.size(); i++) {
		if (vFullPath[i].x > 0 && vFullPath[i].y > 0 && vFullPath[i].x < widthX && vFullPath[i].y < heightY) {
			gridToPrint[vFullPath[i].x][vFullPath[i].y] = "*";
		}
	}

	// Print to cout
	// First header and top row of numbers
	std::cout << "Full Path for plane: " << planeID << "\n";

	// Now print each row
	for (int i = 0; i < widthX; i++) {
		for (int j = 0; j < heightY; j++) {
			std::cout << gridToPrint[i][j] << " ";
		}
		std::cout << "\n";
	}
}

void printPathAndDanger(std::queue<point> fullPath, int planeID, int startx, int starty, int endx, int endy, DangerGrid *bc) {
	int widthX = bc->get_width_in_squares();
	int heightY = bc->get_height_in_squares();
	string gridToPrint[widthX][heightY];

	vector<point> vFullPath;
	while (!fullPath.empty()) {
		vFullPath.push_back(fullPath.front());
		fullPath.pop();
	}

	for (int goThroughTime = 0; goThroughTime < vFullPath.size(); goThroughTime++) {
		// initialize grid
		for (int i = 0; i < widthX; i++)
			for (int j = 0; j < heightY; j++)
				gridToPrint[i][j] = "-";
		
		// place start and end, denoted with $ and @
		gridToPrint[startx][starty] = "$";
		gridToPrint[endx][endy] = "@";

		// place path onto grid, denoted with *
		for (int i = 0; i < goThroughTime; i++) {
			if (vFullPath[i].x > 0 && vFullPath[i].y > 0 && vFullPath[i].x < widthX && vFullPath[i].y < heightY) {
				gridToPrint[vFullPath[i].x][vFullPath[i].y] = "*";
			}
		}

		// Print to cout
		// First header and top row of numbers
		std::cout << "Partial path for plane " << planeID << " at time " << goThroughTime << "\n";

		// Now print each row
		for (int i = 0; i < widthX; i++) {
			for (int j = 0; j < heightY; j++) {
				std::cout << gridToPrint[i][j] << " ";
			}
			std::cout << "\n";
		}
		printDanger(bc, goThroughTime);
	}
}

void printDanger(DangerGrid *bc, int currentTime) {
	int widthX = bc->get_width_in_squares();
	int heightY = bc->get_height_in_squares();
	string gridToPrint[widthX][heightY];
	
	// initialize grid
	for (int i = 0; i < widthX; i++) {
		for (int j = 0; j < heightY; j++) {
			std::ostringstream strs;
			strs << (int) bc->get_pos(i, j, currentTime);
			gridToPrint[i][j] = strs.str(); 
		}
	}
	
	

	std::cout << "Danger at time " << currentTime << "\n";
	// Now print each row
	for (int i = 0; i < widthX; i++) {
		for (int j = 0; j < heightY; j++) {
			std::cout << gridToPrint[i][j] << " ";
		}
		std::cout << "\n";
	}
}


#endif
