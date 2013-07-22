////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones (stlastar.h and fsa.h, primarily)
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

// All code in except that directly included in the original was developed and implemented by Thomas Crescenzi, Andrew Kaizer, and Tyler Young

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stlastar.h" // See header for copyright and usage information
#include "DiscretizedPlane.h"
#include "dangerGrid.h"
#include <iostream>
// pow, abs, round, sqrt used in this code; mainly in Euclidean distance calculations or Chebyshev square distance
#include <math.h>
// Used for infinity values in cost comparison
#include <limits>

// Used for storing our optimal paths/other paths
#include <queue>
// Used in determining a backwards optimal path (from goal to start) in is_sparse()
#include <stack>
// Used to hold a quick converter between map bearing_t -> a-star integer bearing and its reverse
#include <map>

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0
#define OUTPUT_NODES 0 // use this to output text version of goal states

using namespace std;
using namespace map_tools;

// Global data
// The world map

/**
 *
 *Global Variables Below
 *
 */

// MAP_WIDTH and MAP_HEIGHT are assigned when astar_point is called.
// They are used throughout the entire code navigate through gridspace
int MAP_WIDTH = -1;
int MAP_HEIGHT = -1;

//TATE
int timey = 0;
bool toggle2 = false;

// This is a debugging value that can be found in the function other_main()
const int ROUTE_ABANDONED = 0; // if 1, says if search terminated

// This is our global pointer to the Best Cost/Danger Grid  so that all functions have access to it, as well as MapNode methods
DangerGrid *bc_grid;

// 0-7 are actual moves, 8 is goal state, 9 is stall state
/**
 * movex and movey are the way to quickly determine a move that A* specified in grid space
 *
 * The moves correspond to [0] = E, [1] = SE, [2] = S, [3] = SW, [4] = W, [5] = NW, [6] = N, [7] = NE, [8] = found a goal state (no move ordered), [9] = did not find goal state (no move ordered)
 */
const int movex[10] = {1, 1, 0, -1, -1, -1, 0, 1, 0, 0};
const int movey[10] = {0, 1, 1, 1, 0, -1, -1, -1, 0, 0};

// movegoals is a companion variable to movex/movey; it is used to MOD certain values when calculating similar bearings throughout the code
int movegoals = 8; // 0 - 7

/**
 * sparse_expansion determines how much additional grid spaces our planes consider outside of the rectangle created by their Start Position and End Position
 * A higher sparse value (such as 20) will usually produce less optimal, but safer paths.
 * A lower sparse value raises the risk of conflict/collision, but the plane will always take faster routes.
 */
int sparse_expansion = 20;

/**
 * The initial_bearing of our plane; vitally important to determine how A* starts its node expansions
 * Depending on your implementation, we recommend:
 *    1) Using the planes given bearing from the DiscretizedPlane object
 *    2) Using the planes bearing to its goal (runs risk of collision due to mismatch between actual and assumed bearing)
 */
bearing_t initial_bearing;

// Start and End positions for the DiscretizedPlane
int s_x = -1;
int s_y = -1;
int e_x = -1;
int e_y = -1;

/**
 * A quick and dirty way to convert between A* int bearings and Map class direction bearings (N, S, E, W)
 */
std::map<int, bearing_t> astar_to_map;
std::map<bearing_t, int> map_to_astar;


// This struct is used to store (and reply) with the next "destination" that the plane is flying to
/**
 * The point struct specifies a point in time, along with the assumed bearing.
 * It is primarily used to store the paths predicted by A* and optimal path calculations
 */
struct point {
    int x;
    int y;
    int t;
    bearing_t b;
} ;

// TODO don't include this here, put it under cause it depends on point CURRENTLY, not forever though.
#include "pathLogger.h"

/**
 * Is the path that A* recommends our plane take
 */
queue<point> a_path;


/**
 * DEPRECATED: In the initial version of this code, astar_map held the map values
 * GetMap would return the value at a specific location in the map or return a value of 9 if the spot was unreachable or off the map (9 was the highest value)
 * In our implementation, the cost is purely associated with the Best-Cost/Danger-Grids and is therefore not needed here.
 *
 * Code will be removed in future revisions.
 */
int astar_map[ 1 ][ 1 ];
int GetMap( int x, int y )
{
    return 0;
}


/*
 * This is a helper function to assist A* in determining grid bearing.
 * It is used heavily throughout setting up the code to run A* in is_sparse, astar_point
 *
 * @param currentx -- The starting object's x position
 * @param currenty -- The starting object's y position
 * @param dest_x -- The destination object's x position
 * @param dest_y -- The destination object's y position
 */
int getGridBearing( int current_x, int current_y, int dest_x, int dest_y ){
    if (current_y-dest_y == 0 && current_x-dest_x < 0){
        return 0; // goal is to your right
    }
    else if (current_y-dest_y < 0 && current_x-dest_x < 0){
        return 1; // down right
    }
    else if (current_y-dest_y < 0 && current_x-dest_x == 0){
        return 2; // down
    }
    else if (current_y-dest_y < 0 && current_x-dest_x > 0){
        return 3; // down left
    }
    else if (current_y-dest_y == 0 && current_x-dest_x >0){
        return 4; // left
    }
    else if (current_y-dest_y > 0 && current_x-dest_x > 0){
        return 5; // top left
    }
    else if (current_y-dest_y > 0 && current_x-dest_x == 0){
        return 6; // top
    }
    else if (current_y-dest_y > 0 && current_x-dest_x < 0){
        return 7; // top right
    }
}


/**
 * Used to determine if the bearing of two objects is similar.
 * In our case, we generously state that as long as we are not facing away from the target, we are facing it.
 * Example: If initial_bear = N, then similar_bearing = { W, NW, N, NE, E }
 * Example: If initial_bear = SE, then similar_bearing = { SW, S, SE, E, NE }
 *
 * By changing the range of i, you can change how generous the similarities are.
 * Be warned, a similarity of 0 can lead to planes being unable to turn around without flying off the map (crashing A*)
 *
 * @param initial_bear Usually the current bearing of our plane
 * @param target_bear The necessary bearing to some target (such a goal or a threat)
 */
bool similar_bearing(bearing_t initial_bear, bearing_t target_bear){
    for (int i = -2; i <=2; i++){
        int a_star_bearing = map_to_astar[target_bear];
        a_star_bearing = (a_star_bearing + i) % movegoals;
        if (a_star_bearing < 0){
            a_star_bearing = movegoals + a_star_bearing;
        }
        
        if (astar_to_map[a_star_bearing] == initial_bear)
            return true;
    }
    
    return false;
}

/**
 * Helper function to get the exact opposite bearing of some object (DiscretizedPlane, Target, Blimp, etc.)
 *
 * @param bear the current bearing of some object
 */
bearing_t opposite_bearing(bearing_t bear){
    if (bear == N)
        return S;
    else if (bear == NE)
        return SW;
    else if (bear == E)
        return W;
    else if (bear == SE)
        return NW;
    else if (bear == S)
        return N;
    else if (bear == SW)
        return NE;
    else if (bear == W)
        return E;
    else if (bear == NW)
        return SE;
    else
        return N;
}


/**
 * The heavily modified MapSearchNode.
 * This is the code that is used to expand A*, calculate the weight/cost of a given node, and check various conditions (legal move, end state, etc.)
 * It currently has a maximum expansion of 200,000 (as set in stlastar.h)
 */
class MapSearchNode
{
public:
    int x;	 // the (x,y) positions of the node
    int y;
    int timestep; // AK: the timestep that the node is being considered under (equivalent to the t value in our point struct)
	bool parentWentDiagonal;
    /**
     maneuver is a tricky concept...
     Since we have a turn radius of 22.5*, it takes us 2 grid spaces to make a turn (i.e., we cannot do a 45* turn from East to North East, it has to be East (t=0), EastNorthEast(t=1), North East(t=2)).
     However, if we did include a maneuver bearing, then our planes would be stuck going in a single direction (East to East to East, they could never be biased towards East North East!
     We also need to grab a DiscretizedPlanes actual position at time 0 in their grid (are they in the middle, to a side, etc.).  This is important because of these expansions:
     IF 0 == TOP_HALF OF GRID
     - - - 3
     - - 2 3
     0 1 2 3
     - - 2 3
     - - - -
     ELSE IF 0 == LOWER_HALF OF GRID
     - - - -
     - - 2 3
     0 1 2 3
     - - 2 3
     - - - 3
     ELSE 0 == MIDDLE OF GRID (lucky us)
     - - - 3
     - - 2 3
     0 1 2 3
     - - 2 3
     - - - 3
     
     NOTE: As of the present time, we have not fully implented this; we just assume that the plane always has 3 possible expansions after their first move.
     It would be advantageous to get this issue resolved since it would produce considerably better/safer/legal paths.
     */
    
    // The bearing of the parent of the child node -- especially important if implementing the above concept (of strictly enforcing legal moves)
    bearing_t parent_bearing; // AK: Used for children node to understand where their parent is coming from
    
    /**
     * Every child's expansions are actually determined by its parent when the child is determine to be a legal move.
     * When you run the GetSuccessors method the parent determines the child's moves based on its bearing and the childs bearing.
     *
     * This action, however, could be performed directly in the child by taking its current_bearing and its parent_bearing...it just takes more effort.
     */
    queue<bearing_t> legal_expansions_for_child;
	
    /**
     * The default constructor for the MapSearchNode
     * This should not be used beyond creating the first Node -- the values are installed from s_x, s_y, and initial_bearing when they are retrieved from astar_point setting up the search
     */
    MapSearchNode() {
        x = y = 0;
        timestep = 0;
		parentWentDiagonal = false;
        parent_bearing = initial_bearing;
    }
    
    /**
     * The constructor used to expand from our starting node to all other nodes
     * It initializes a node with all the necessary information
     *
     * @param px nodes x position
     * @param py nodes y position
     * @param t_step notes t value
     * @param parent node's parent's bearing
     * @param my_maneuver a queue of possible moves that the parent is allowing the child node
     */
    MapSearchNode( int px, int py, int t_step, bearing_t parent, queue<bearing_t> my_maneuver, bool wentDiagonal)
    {
        x=px;
        y=py;
        parent_bearing = parent;
        legal_expansions_for_child=my_maneuver;
		parentWentDiagonal = wentDiagonal;
        
        // Depending on how much you expand the Best Cost/Danger Grid determines what value is here; in this case 19
        if (t_step > bc_grid->getTimeLength())
            timestep = bc_grid->getTimeLength();
        else
            timestep = t_step;
    } // AK
    
    /**
     * Destructor for MapSearchNode to plug a bug in the original code (in the original code memory was not released from the nodes)
     * The destructor is called in stlasatr.h
     * Without this destructor our memory is eventually completely consumed by bearing_t's that are not properly released
     */
    ~MapSearchNode(){
        while (!legal_expansions_for_child.empty()){
            legal_expansions_for_child.pop();
        }
    }
    
    /**
     * This is the cost to move from one node to another as determined by the Best Cost/Danger Grid
     *
     * @param nodeGoal the node whose cost is to be examined
     */
    float GoalDistanceEstimate( MapSearchNode &nodeGoal );
    
    /**
     * Checks to see if this node is the goal, if it is we have found a legal path from start to goal
     *
     * @nodeGoal
     */
    bool IsGoal( MapSearchNode &nodeGoal );
    
    /**
     * Gets the children of a certain node.
     *
     * @param astarsearch
     * @parent_node - the current node being investigated
     */
    bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
    
    /**
     * DEPRECATED: Uses the GetCost function described earlier in the code, which only returns 0 (no weight)
     *
     * @param successor - the node to succeed our current parent
     */
    float GetCost( MapSearchNode &successor );
    
    /**
     * A very important method that determines if we are trying to expand the same node twice based on x, y, and t
     * The values you allow for this expansion determine how many nodes you need to allow A* to expand in stlastar.h
     * Example: On a 100x100 grid with a Best Cost/Danger Grid up to time 20 you need to have 200,000 nodes (100x's * 100 y's * 20 time steps)
     *
     * Additionally you can include the bearing_t of a grid space in the calculation, however the node expansions necessary explodes from 200k to 1.6 million
     * Example: Using the same grid above but adding the 8 bearings (N, NE, ...) would mean 100*100*20*8 = 1,600,000
     * This dramatically slows down A* to a point where it struggles to update 16 planes in one second
     * The only way to safely cut down on the number of expansions is to either remove the amount of bearings allowed (such as restricting it to only opposite bearings) OR
     *   to cut down on the number of time steps in the Best Cost/ Danger Grid
     *  Example: On a 100*100*10*2 = 200,000 expansions
     *
     * Also: a lower sparseness can help this by placing a O(n) on the maximum while lowering the average expansions to <= ~n/2
     *
     * @param rhs
     */
    bool IsSameState( MapSearchNode &rhs );
    
    /**
     * Helper method to print out a nodes values
     */
    void PrintNodeInfo();
    
    /**
     * Getter methods for x pos, y pos, timestep, and bearing
     */
    int getX();
    int getY();
    int getT();
    bearing_t getB();
    
};

/**
 * See detailed explanation above about the time and space complexities
 */
bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{
    if( (x == rhs.x) &&
       (y == rhs.y) &&
       (timestep == rhs.timestep) )
    {
        return true;
    }
    else
    {
        return false;
    }
    
}

void MapSearchNode::PrintNodeInfo()
{
    cout << "Node position : (" << x << ", " << y << ")" << " at " << timestep << " with a bearing of " << parent_bearing << endl;
}

// Added to get node X pos -- Andrew Kaizer
int MapSearchNode::getX(){
    return x;
}

// Added to get node Y pos -- Andrew Kaizer
int MapSearchNode::getY(){
    return y;
}

// Added to get a node T pos -- Andrew Kaizer
int MapSearchNode::getT(){
    return timestep;
}

// Added to get a node B pos -- Andrew Kaizer
bearing_t MapSearchNode::getB(){
    return parent_bearing;
}

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
	// TODO Understand what that is doing
    /**
     * The Average Method -- Used to provide a fuzzy value for any given square
     * This is useful if we are not entirely positive that our Best Cost/Danger Grid is completely accurate (ex: when dealing with numerous turns)
     * It produces less optimal, but safer results
     */
//    double cost = bc_grid->get_pos(x, y, timestep);
//    double count = 1;
//    for (int i = 0; i < 8; i++){
//        if (x + movex[i] >= 0 && x + movex[i] < MAP_WIDTH &&
//            y + movey[i] >= 0 && y + movey[i] < MAP_HEIGHT){
//            for (int t = 0; t <= 2; t++){
//                cost += bc_grid->get_pos(x+movex[i], y+movey[i], timestep);
//                count++;
//            }
//        }
//    }
//    
//    return cost/count;
    
    /**
     * In normal circumstances, when the Best Cost/Danger Grid and A* are fully tuned to each other, we would use this value.
     * This grabs the weight of this square at this timestep
     */
     return bc_grid->get_pos(x, y, timestep);
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{
    
    if( (x == nodeGoal.x) &&
       (y == nodeGoal.y) )
    {
        return true;
    }
    
    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{
    // Our time_z starts at -1, if it continues to be -1 then something has gone wrong in the code that must be resolved
    int time_z = -1;
    
    if ( !parent_node ){
        // if we have a parent, we actually only have one really legal move...that is going in the same direction as we current are
        // a change in direction occurs by which children we expand
        legal_expansions_for_child.push(initial_bearing);
        parent_bearing = initial_bearing;
    } else {
        
    }
    // update our time step -- if the time exceeds our predined value, give it a max time instead (maximum is number of time in Best Cost/Danger Grid
    time_z = timestep+1>bc_grid->getTimeLength() ? bc_grid->getTimeLength() : 1+timestep;
 
 // Throwing all the nodes for it to look at	
//    MapSearchNode NewNode;
//    queue<bearing_t> child_expansions;
//	if (x-1 >= 0 && x-1 < 50 && y >= 0 && y < 50) {
//		NewNode = MapSearchNode( x-1, y, time_z, N, child_expansions);
//		astarsearch->AddSuccessor( NewNode );
//	}
//	if (x+1 >= 0 && x+1 < 50 && y >= 0 && y < 50) {
//		NewNode = MapSearchNode( x+1, y, time_z, N, child_expansions);
//		astarsearch->AddSuccessor( NewNode );
//	}
//	if (x >= 0 && x < 50 && y-1 >= 0 && y-1 < 50) {
//		NewNode = MapSearchNode( x, y-1, time_z, N, child_expansions);
//		astarsearch->AddSuccessor( NewNode );
//	}
//	if (x >= 0 && x < 50 && y+1 >= 0 && y+1 < 50) {
//		NewNode = MapSearchNode( x, y+1, time_z, N, child_expansions);
//		astarsearch->AddSuccessor( NewNode );
//	}
//	if (x-1 >= 0 && x-1 < 50 && y-1 >= 0 && y-1 < 50) {
//		NewNode = MapSearchNode( x-1, y-1, time_z, N, child_expansions);
//		astarsearch->AddSuccessor( NewNode );
//	}
//	if (x-1 >= 0 && x-1 < 50 && y+1 >= 0 && y+1 < 50) {
//		NewNode = MapSearchNode( x-1, y+1, time_z, N, child_expansions);
//		astarsearch->AddSuccessor( NewNode );
//	}
//	if (x+1 >= 0 && x+1 < 50 && y-1 >= 0 && y-1 < 50) {
//		NewNode = MapSearchNode( x+1, y-1, time_z, N, child_expansions);
//		astarsearch->AddSuccessor( NewNode );
//	}
//	if (x+1 >= 0 && x+1 < 50 && y+1 >= 0 && y+1 < 50) {
//		NewNode = MapSearchNode( x+1, y+1, time_z, N, child_expansions);
//		astarsearch->AddSuccessor( NewNode );
//	}
//
    
    /**
     * The following three blocks of code determine the amount of range our field can consider (Sparse A*)
     * We do not allow A* to consider a node outside of this sparse range
     */
    int greater_x = 0;
    int lesser_x = 0;
    int greater_y = 0;
    int lesser_y = 0;
    
    if (s_x > e_x){
        greater_x = s_x + sparse_expansion;
        lesser_x = e_x - sparse_expansion;
    } else {
        greater_x = e_x + sparse_expansion;
        lesser_x = s_x - sparse_expansion;
    }
    
    if (s_y > e_y){
        greater_y = s_y + sparse_expansion;
        lesser_y = e_y - sparse_expansion;
    } else {
        greater_y = e_y + sparse_expansion;
        lesser_y = s_y - sparse_expansion;
    }
    
    // in legal_moves, we are checking for the legal moves allowed 2 moves into the future (the next additional move is only our immediate move)
    // Immediate next move addition
    // Check to see if expansion to the given node is legal...if not, tough luck
    while (!legal_expansions_for_child.empty()){
        bearing_t next_expansion = legal_expansions_for_child.front();
        legal_expansions_for_child.pop();
        
        // convert the parent bearing to the A-Star move setup (0 = EAST in A_ST, 0 = NORTH in MAP)
        int a_st_bearing = map_to_astar[parent_bearing];
        
        // the three legal expansions considered by A*
        int r[] = {(a_st_bearing-1), a_st_bearing%movegoals, (a_st_bearing+1)%movegoals};
        
        // Since C++ does not do MOD on negative numbers, we must manually wrap around
        if(r[0] < 0){ // wraps around from 0 to 7
            r[0] += movegoals;
        }
        
        // r[] positions to consider
        int range_start = 0;
        int range_end = 2;
        
        
        /**
         * If time_z = 1, then our parent is at t = 0
         * If our parent is at t = 0, then we only have one legal move in the future, and that is to continue our current course (it takes at least two moves to change course)
         */
        if (time_z == 1){
            range_start = 1;
            range_end = 1;
        }
        
        /**
         * This for loop looks at every possible child node the parent can have and determines if it is legal, if it is push onto A*'s set of nodes
         */
        for (int i = range_start; i <= range_end; i++){
            queue<bearing_t> child_expansions;
            int legal_moves = -1;
            if ((int)x + movex[r[i]] >= 0 && (int)x + movex[r[i]] < MAP_WIDTH &&
                (int)y + movey[r[i]] >= 0 && (int)y + movey[r[i]] < MAP_HEIGHT &&
                (int)x + movex[r[i]] >= lesser_x && (int)x + movex[r[i]] <= greater_x &&
                (int)y + movey[r[i]] >= lesser_y && (int)y + movey[r[i]] <= greater_y){
                
                child_expansions.push(astar_to_map[r[i]]);
                legal_moves = r[i];
                
            }
            MapSearchNode NewNode;
            // push each possible move except allowing the search to go backwards
            
            // left
            if( legal_moves == 4 )
            {
                NewNode = MapSearchNode( x-1, y, time_z, W, child_expansions, parentWentDiagonal);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // top
            if( legal_moves == 6 )
            {
                NewNode = MapSearchNode( x, y-1, time_z, N, child_expansions, parentWentDiagonal);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // right
            if( legal_moves == 0 )
            {
                NewNode = MapSearchNode( x+1, y, time_z, E, child_expansions, parentWentDiagonal);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // down
            if( legal_moves == 2 )
            {
                NewNode = MapSearchNode( x, y+1, time_z, S, child_expansions, parentWentDiagonal);
                astarsearch->AddSuccessor( NewNode );
            }
            
			// if the parent went diagonal and then you went diagonal, increase time by 1 since it takes roughly twice
			// as long to go along diagonal 
			bool parentWent = true;
			if (parentWentDiagonal) {
				parentWent = false;
				time_z++;
			}
            // down right
            if( legal_moves == 1 )
            {
                NewNode = MapSearchNode( x+1, y+1, time_z, SE,child_expansions, parentWent);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // top right
            if( legal_moves == 7 )
            {
                //cout << "TR " << x+1 << ", " << y-1 << endl;
                NewNode = MapSearchNode( x+1, y-1, time_z, NE, child_expansions, parentWent);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // down left
            if( legal_moves == 3 )
            {
                NewNode = MapSearchNode( x-1, y+1,time_z, SW, child_expansions, parentWent);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // top left
            if( legal_moves == 5 )
            {
                NewNode = MapSearchNode( x-1, y-1, time_z, NW, child_expansions, parentWent);
                astarsearch->AddSuccessor( NewNode );
            }
        }
    }
    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
    
    return (float) GetMap( x, y );
    
}

// place code in here to make main for UAV Sim friendly -- Andrew Kaizer
/**
 * This code contains the A* search calls, it is apart from astar_point to try and separate our concerns
 */
int other_main(){
    AStarSearch<MapSearchNode> astarsearch;
    
    int SearchCount = 0;
    
    const int NumSearches = 1;
    
    /**
     0: move right
     1: move bottom right
     2: move down
     3: move bottom left
     4: move left
     5: move top left
     6: move top
     7: move top right
     8: solution found...no more moves needed.....
     */
    int moves = 8;
    while(SearchCount < NumSearches)
    {
        // Create a start state
        MapSearchNode nodeStart;
        nodeStart.x = s_x;
        nodeStart.y = s_y;
        
        // Define the goal state
        MapSearchNode nodeEnd;
        nodeEnd.x = e_x;
        nodeEnd.y = e_y;
        
        // Set Start and goal states
        astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );
        
        int SearchState;
        int SearchSteps = 0;
        do // Loop until we find a goal or fail
        {
            SearchState = astarsearch.SearchStep();
            
            SearchSteps++;
        }
        while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
        
        // A* has found our goal from our start
        if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
        {
            MapSearchNode *node = astarsearch.GetSolutionStart();
            
            int steps = 0;
            
            // Loop until we no more nodes to look at
            for( ;; )
            {
                node = astarsearch.GetSolutionNext();
                
                if( !node )
                {
                    break;
                }
                
                /**
                 * Push A*'s recommendation onto our a_path queue
                 */
                if (!(node->getY()*MAP_WIDTH+node->getX() == nodeStart.y*MAP_WIDTH+nodeStart.x ||
                      node->getY()*MAP_WIDTH+node->getX() == nodeEnd.y*MAP_WIDTH+nodeEnd.x)){
                    // Route that astar has selected
                    point p;
                    p.x = node->getX();
                    p.y = node->getY();
                    p.t = node->getT();
                    p.b = node->getB();
                    a_path.push(p);
                }
                
                steps ++;
            };
            
            // if steps < 1 (i.e. steps == 0), then we are currently on our goal
            if (steps < 1){
                moves = 8; // moves should be a 8, if we have no move
            }
            
            // Once you're done with the solution you can free the nodes up in memory
            astarsearch.FreeSolutionNodes();
        }
        else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED )
        {
            // This code could be removed, however it is usually instructive to know when A* fails
            if (ROUTE_ABANDONED > 0)
                cout << "Search terminated. Did not find goal state\n";
            
            // moves = -1, we have failed to find any legal moves from start to goal (try checking how you are expanding your nodes and your sparseness)
            moves = -1;
        }
        
        SearchCount ++;
        
        astarsearch.EnsureMemoryFreed();
		std::cout << "Memory Freed\n";
    }
    
    return moves;
}

// Returns the point of divergence between provable optimal path and a-star path
/**
 * This function is what is called by collisionAvoidance, as well as what coordinates all the parts of A* (Check Sparse, setup Search, run Search, Analyze Search)
 *
 * @param bc the Best Cost/Danger Grid created by our heuristics
 * @param sx the starting position of a plane; the decimal value is where in the grid it is located (useful for future work)
 * @param sy same as sx
 * @param endx, endy the goal position
 * @param planeid the current plane we are operating on, very useful for debugging purposes
 * @param current_bear the current planes bearing as given by collision avoidance
 * @param planey_the_plane_map a list of all living planes in our world, as known by collision avoidance
 */
queue<point> astar_point(DangerGrid *bc, double sx, double sy, int endx, int endy, int planeid, bearing_t current_bear, std::map<int, DiscretizedPlane> *planey_the_plane_map, map_tools::gridValues gridVals)
{
    // set our global pointer bc_grd to point to the pointer bc which points to a Best Cost/Danger Grid
    bc_grid = bc;
	bc_grid->calculateDistances(endx, endy);
    
    // Construct atar_to_map std::map (since map is 0=N and A_ST is 0=E)
    astar_to_map[0] = E;
    astar_to_map[1] = SE;
    astar_to_map[2] = S;
    astar_to_map[3] = SW;
    astar_to_map[4] = W;
    astar_to_map[5] = NW;
    astar_to_map[6] = N;
    astar_to_map[7] = NE;
    
    map_to_astar[E] = 0;
    map_to_astar[SE] = 1;
    map_to_astar[S] = 2;
    map_to_astar[SW] = 3;
    map_to_astar[W] = 4;
    map_to_astar[NW] = 5;
    map_to_astar[N] = 6;
    map_to_astar[NE] = 7;
    
    // Get our maps parameters
    MAP_WIDTH = bc->get_width_in_squares();
    MAP_HEIGHT = bc->get_height_in_squares();
    
    // Since A* is allocated memory once, we need to make sure any traces of previous executions is removed to avoid conflicts
    while(!a_path.empty()){
        a_path.pop();
    }
    
    // DETERMINE ROUGH QUADRANT (note: currently not in use) and assign values
    s_x = floor(sx);
    s_y = floor(sy);
    e_x = endx;
    e_y = endy;
    
    // Set our initial bearing to our planes current bearing (note: this is important for A* node expansion and collision avoidance maneuvers)
    initial_bearing = current_bear;
    
    // move is the next move we recommend to Collision Avoidance
    point move;
    
    // grab the bearing to the goal from A* bearing, convert it to our goal bearing in Map (N, S, W, ...) and then grab the A* bearing of our initial_bearing
    int goal_orientation = getGridBearing(s_x, s_y, e_x, e_y);
    bearing_t goal_bearing = astar_to_map[goal_orientation];
    int astar_bearing = map_to_astar[initial_bearing];
    
    /**
     * We can only run is_sparse() (clear path checking) if we are already facing the goal's bearing, otherwise it is not accurate
     */
	//if (planeid == 0 || planeid == 1 || planeid == 2 || planeid == 3) {
//    if (initial_bearing == goal_bearing && is_sparse()){
//        move.x = endx;
//        move.y = endy;
//        if (move.x < 0)
//            move.x = 0;
//        if (move.x >= MAP_WIDTH)
//            move.x = MAP_WIDTH - 1;
//        
//        if (move.y < 0)
//            move.y = 0;
//        
//        if (move.y >= MAP_HEIGHT)
//            move.y = MAP_HEIGHT - 1;
//	
//		//TATE changed because of function returning whole queue now
//		a_path.push(move);
//        return a_path;
//    }
    
    // A queue of optimal moves that we will use to compare results with
    queue<point> opt_path;
    bool at_goal = false;
    
    int current_x = s_x;
    int current_y = s_y;
    int time = 0;
    
    // If our goal and initial bearing are similar, we can force our opt_path to follow A* slightly more closely (this way we expand our first step according to our initial bearing)
    if (similar_bearing(initial_bearing, goal_bearing)){
        current_x += movex[astar_bearing];
        current_y += movey[astar_bearing];
        time += 1;
        point pr;
        pr.x = current_x;
        pr.y = current_y;
        pr.t = time;
        opt_path.push(pr);
    }
    
    // Construct our optimal, euclidean distance based path
//    while (!at_goal){
//        double min_cost = numeric_limits<double>::infinity();
//        int best_move = -1;
//        for (int i = 0; i < 8; i++){
//            int new_x = current_x + movex[i];
//            int new_y = current_y + movey[i];
//            
//            if (sqrt(pow(new_x-endx, 2) + pow(new_y-endy, 2)) < min_cost){
//                min_cost = sqrt(pow(new_x-endx, 2) + pow(new_y-endy, 2));
//                best_move = i;
//            }
//        }
//        
//        current_x = current_x + movex[best_move];
//        current_y = current_y + movey[best_move];
//        point pr;
//        pr.x = current_x;
//        pr.y = current_y;
//        pr.t = time+1 < 20 ? time + 1 : 20;
//        opt_path.push(pr);
//        time += 1;
//        if (current_x == e_x && current_y == e_y)
//            at_goal = true;
//    }
    
    
    // third, run the algorithm and get our actual set of points
    move.x = e_x;
    move.y = e_y;
    
    // if result is -1...something bad has happened
    int result = other_main();
    
    // Outside Zero means: our plane is trying to move away from our goal (probably due to bearing facing a different direction)
    // However, if this is true, we must do a new type of bounds checking to bring it back in.
    // Our new type of checking asks: if plane is still out of bounds AND the position is safe, continue ELSE return spot
   // bool outside_zero = false;
   // 
   // // These lessers values are used to determine where our grid exists...the greater_x is the larger x between the start and end positions (same for y)
   // // These are used in outside_zero for our bounds checking
   // int lesser_x = -1;
   // int greater_x = -1;
   // int lesser_y = -1;
   // int greater_y = -1;
   // 
   // if (s_x > e_x){
   //     greater_x = s_x;
   //     lesser_x = e_x;
   // } else {
   //     greater_x = e_x;
   //     lesser_x = s_x;
   // }
   // 
   // if (s_y > e_y){
   //     greater_y = s_y;
   //     lesser_y = e_y;
   // } else {
   //     greater_y = e_y;
   //     lesser_y = s_y;
   // }
   // 
   // 
   //// point previous;
   //// previous.x = s_x;
   //// previous.y = s_y;
   //// previous.t = 0;
   // 
   // if (!a_path.empty()){
   //     point a_st = a_path.front();
   //     // based on orientation?
   //     // initial bearing and goal bearing
   //     if (!(similar_bearing(initial_bearing, goal_bearing)))
   //         outside_zero = true;
   // }
    
   // bool straight = true;
   // if (s_x != e_x && s_y != e_y)
   //     straight = false;
    
	// go 10 spots away for kicks
//	if (planeid == 0) {
//		int index = 0;
//		point at;
//		std::cout << "---------------------" << timey << "------------------------\n";
//		while (!a_path.empty()) {
//			at = a_path.front();
//			std::cout << at.x << " " << at.y << " " << at.t << "\n";
//			a_path.pop();
//		}
//		std::cout << "---------------------------------------------------------\n\n";
//		timey++;
//		return at;
//	}
//	if (planeid == 1 || planeid == 7) {
//		printPathAndDanger(a_path, planeid, sx, sy, endx, endy,bc);
//	}
	return a_path;
//	int index = 0;
//	while (!a_path.empty()) {
//		if (index < 5) {
//			a_path.pop();
//			index++;
//		}	
//		else {
//			return a_path.front();
//		}
//	}

    // THESE ARE TWO DIFFERENT CASES (read the research paper to find out more :P)
//    if (outside_zero){
//        point double_previous;
//        double_previous.x = -1;
//        while (!a_path.empty()){
//            point a_st = a_path.front();
//            move = a_st;
//            
//            // since we are using euclidean grid math, we must AVOID on the MY_PIVOT (we do not have straight diagonals)
//            // follow the star    
//            
//            double predicted_val = sqrt(pow(a_st.x-e_x, 2) + pow(a_st.y-e_y, 2));
//            // if, for some reason, our turn is dangerous -- break from it      
//            if ((bc->get_pos(a_st.x, a_st.y, a_st.t) > predicted_val) && a_st.t > 1){
//                move = immediate_avoidance_point(*planey_the_plane_map, a_st, previous);
//                break;
//            }
//            
//            // if we have managed to turn out plane to face our goal again, return that spot
//            if (a_st.x > greater_x || a_st.x < lesser_x || a_st.y > greater_y || a_st.y < lesser_y){
//                outside_zero = true;
//            } else {
//                move = immediate_avoidance_point(*planey_the_plane_map, a_st, previous);
//                break;
//            } 
//            
//            previous = a_st;
//            a_path.pop();
//        }
//    } else {
//        while (!a_path.empty()  && !opt_path.empty()){
//            point a_st = a_path.front();
//            point opt = opt_path.front();
//            
//            // eh, A-star might recommend this, but we should control it (danger)
//            if(bc->get_pos(a_st.x, a_st.y, a_st.t) > sqrt(pow(a_st.x-endx, 2) + pow(a_st.y-endy, 2)) && a_st.t > 1){
//                move = a_st;
//                // Immediate term avoidance (3,4,5, and 6 seconds)
//                if (a_st.t > 3 && a_st.t <= 7){
//                    // I am assuming control of this vessel
//                    
//                    // avoidance maneuver must follow the "I wanna live" logic, not A* path which is more like a "Hey, Google, I am feeling lucky; please don't rick roll me"
//                    // DETERMINE EMMY_PIRICALLY the best place to go; behind the hazardous plane or in front of it.
//                    // At times 4, 5, and 6 this is reasonable; at times 2 and 3 it may not be as good, but you should already be out of the way by then :)
//                    //TATE
//                    //move = immediate_avoidance_point(*planey_the_plane_map, a_st, previous);
//                } else {	  
//                    move = a_st;
//                }
//                break;
//            }
//            
//            // If the optimal and A* paths diverge, just return the point of divergence
//            if ((opt.x != a_st.x || opt.y != a_st.y) && a_st.t > 1){	  
//                move = a_st;
//                break;
//                
//            }
//            
//            // Pivot is an odd issue...
//            // This occurs when we start with one bearing but we encount a turn that must be made
//            if ((a_st.x == endx || a_st.y == endy) && !straight && a_st.t > 1){
//                move = a_st; //used to a_st
//                break;
//            }
//            
//            previous = a_st;
//            opt_path.pop();
//            a_path.pop();
//        } 
//    }
//    
//    
//    // Bounds check our move -- if it is moving off the field bring it back in
//    if (move.x < 0)
//        move.x = 0;
//    
//    if (move.y < 0)
//        move.y = 0;
//    
//    if (move.x >= MAP_WIDTH)
//        move.x = MAP_WIDTH - 1;
//    
//    if (move.y >= MAP_HEIGHT)
//        move.y = MAP_HEIGHT - 1;
//    
//    return move;
}
