#include "au_uav_ros/mapSearchNode"
using namespace au_uav_ros;

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
