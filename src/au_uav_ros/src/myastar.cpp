#include "stlastar.h" // See header for copyright and usage information
#include "DiscretizedPlane.h"
#include "dangerGrid.h"

#include <iostream>
using namespace std;

// This is our global pointer to the Best Cost/Danger Grid  so that all functions have access to it, as well as MapNode methods
DangerGrid *bc_grid;

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
};

/**
 * The heavily modified MapSearchNode.
 * This is the code that is used to expand A*, calculate the weight/cost of a given node, and check various conditions (legal move, end state, etc.)
 * It currently has a maximum expansion of 200,000 (as set in stlastar.h)
 */
class MapSearchNode
{
public:
    unsigned int x;	 // the (x,y) positions of the node
    unsigned int y;
    unsigned int timestep; // AK: the timestep that the node is being considered under (equivalent to the t value in our point struct)
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
    MapSearchNode( unsigned int px, unsigned int py, unsigned int t_step, bearing_t parent, queue<bearing_t> my_maneuver)
    {
        x=px;
        y=py;
        parent_bearing = parent;
        legal_expansions_for_child=my_maneuver;
        
        // Depending on how much you expand the Best Cost/Danger Grid determines what value is here; in this case 19
        if (t_step > 19)
            timestep = 19;
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
    time_z = timestep+1>19 ? 19 : 1+timestep;
    
    
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
                NewNode = MapSearchNode( x-1, y, time_z, W, child_expansions);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // top
            if( legal_moves == 6 )
            {
                NewNode = MapSearchNode( x, y-1, time_z, N, child_expansions);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // right
            if( legal_moves == 0 )
            {
                NewNode = MapSearchNode( x+1, y, time_z, E, child_expansions);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // down
            if( legal_moves == 2 )
            {
                NewNode = MapSearchNode( x, y+1, time_z, S, child_expansions);
                astarsearch->AddSuccessor( NewNode );
            }
            
            
            // Andrew Kaizer is adding Diagonals below here...original code did not allow for such nonsense!
            // down right
            if( legal_moves == 1 )
            {
                NewNode = MapSearchNode( x+1, y+1, time_z, SE,child_expansions);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // top right
            if( legal_moves == 7 )
            {
                //cout << "TR " << x+1 << ", " << y-1 << endl;
                NewNode = MapSearchNode( x+1, y-1, time_z, NE, child_expansions);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // down left
            if( legal_moves == 3 )
            {
                NewNode = MapSearchNode( x-1, y+1,time_z, SW, child_expansions);
                astarsearch->AddSuccessor( NewNode );
            }
            
            // top left
            if( legal_moves == 5 )
            {
                NewNode = MapSearchNode( x-1, y-1, time_z, NW, child_expansions);
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


queue<point> astar_point(DangerGrid *bc, double sx, double sy, int endx, int endy, int planeid, bearing_t current_bear, std::map<int, DiscretizedPlane> *planey_the_plane_map)
{
    // set our global pointer bc_grd to point to the pointer bc which points to a Best Cost/Danger Grid
    bc_grid = bc;
	bc_grid->calculateDistances(endx, endy);

    int result = other_main();
}

