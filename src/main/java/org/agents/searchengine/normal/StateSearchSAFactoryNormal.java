package org.agents.searchengine.normal;

import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;

import java.util.Arrays;
import java.util.HashMap;

final class StateSearchSAFactoryNormal {
    public static final int G_COST = 0;
    public static final int F_COST = 1;
    public static final int IN_HEAP = 2;

    private static int [][] state;
    private static int [][][] cost_so_far;
    private static int [][][] closed_states;

    SearchSAStateNormal searchState;

    public static void createCostSoFar() {
        //third index : one for g_cost   and second one for f_cost
        cost_so_far = new int[MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL][3];
        //cost_so_far[coord_y][coord_x]=new int[]{g_cost,f_cost,is_in_heap};
    }

    /*
        public static void updateCost(HashMap<Integer, Integer> cost_so_far) {
            cost_so_far.put(Arrays.hashCode(state[SearchState.ARRAYPOS]), state[ARRAYCOSTS][Costs.COST_G.ordinal()]);
        }
    */
    public static void putCostSoFar(int[] next_pos, int g_cost, int f_cost, boolean is_in_heap) {
        assert g_cost > 0;
        int y = Coordinates.getRow(0, next_pos);
        int x = Coordinates.getCol(0, next_pos);

        cost_so_far[y][x][G_COST] = g_cost ;
        cost_so_far[y][x][F_COST] = f_cost ;
        if (is_in_heap)
            cost_so_far[y][x][IN_HEAP] = 1;
     }

     public static void putCostSoFar(int[][] state) {
        assert SearchSAStateNormal.getGCost(state) >= 0;

        int[] pos_state = SearchSAStateNormal.getStateCoordinates(state);
        int time_state = SearchSAStateNormal.getTimeStep(state);
        int y = SearchSAStateNormal.getYCoordinate(state);
        int x = SearchSAStateNormal.getXCoordinate(state);

        cost_so_far[y][x][G_COST] = SearchSAStateNormal.getGCost(state);
        cost_so_far[y][x][F_COST] = SearchSAStateNormal.getFCost(state);

    }

    //can mark state as in heap by writting a value true for is_in_heap
    public static void mark_state_inqueue(int[][] state, boolean is_in_heap) {
        assert SearchSAStateNormal.getGCost(state) >= 0;

        int[] pos_state = SearchSAStateNormal.getStateCoordinates(state);
        int time_state = SearchSAStateNormal.getTimeStep(state);
        int y = SearchSAStateNormal.getYCoordinate(state);
        int x = SearchSAStateNormal.getXCoordinate(state);

        if (is_in_heap){
            cost_so_far[y][x][IN_HEAP] = 1;
        }
        else {
            cost_so_far[y][x][IN_HEAP] = 0;
        }
    }

    public static boolean isInCostSoFar(int[] next_pos) {
        int time_state = Coordinates.getTime(0, next_pos);
        int y = Coordinates.getRow(0, next_pos);
        int x = Coordinates.getCol(0, next_pos);

        return cost_so_far[y][x][G_COST] > 0 ;
    }

    public static int[] getCostSoFar(int[] next_) {
        int time_state = Coordinates.getTime(0, next_);
        int y = Coordinates.getRow(0, next_);
        int x = Coordinates.getCol(0, next_);

        //int prev_g_step = cost_so_far[y][x][G_COST];
        //int prev_f_step = cost_so_far[y][x][F_COST];

        return cost_so_far[y][x];
    }

    public static boolean isInHeap(int[][] next_) {
        int y = SearchSAStateNormal.getYCoordinate(next_);
        int x = SearchSAStateNormal.getXCoordinate(next_);

        return cost_so_far[y][x][IN_HEAP] > 0 ;
    }

    public static int[][] createDummyState() {
        return SearchSAStateNormal.createDummyState();
    }

    //is used many times to create a state for the a_star
    public static int[][] createState(int[] cell_coordinates, int total_gcost, int[] goal_coordinates){
        assert (goal_coordinates.length == 3);
        assert (cell_coordinates.length == 3);

        int heuristc_value = SearchEngineSANormal.getHeuristic(cell_coordinates, goal_coordinates);
        //int heuristc_value2 = SearchEngineSA.getConsistentHeuristic(cost_time,cell_coordinates, goal_coordinates);
        state  = SearchSAStateNormal.createNew(cell_coordinates, total_gcost, heuristc_value + total_gcost);

        return state;
    }

    public static int[][] createState(int[] cell_neighbour, int neighbour_gcost, int f_value) {
        assert (cell_neighbour.length == 3);
        state  = SearchSAStateNormal.createNew(cell_neighbour, neighbour_gcost, f_value);

        return state;
    }

    public static int getGCost(int[][] state) {
        return SearchSAStateNormal.getGCost(state);
    }

    public static void updateCameFromPrevCell(HashMap<int[],int[]> came_from, int[][] state, int[][] previouse_coordinates) {
        assert state != null;
        assert previouse_coordinates != null;
        came_from.put(getCellCoordinates(state), getCellCoordinates(previouse_coordinates));
    }

    public static int[] getCellCoordinates(int[][] state){
        return SearchSAStateNormal.getStateCoordinates(state);
    }

    private static int  getYCoordinate(int[][] state){
        return SearchSAStateNormal.getYCoordinate(state);
    }

    private static int  getXCoordinate(int[][] state){
        return SearchSAStateNormal.getXCoordinate(state);
    }

    private static int  getTimeStep(int[][] state){
       return SearchSAStateNormal.getTimeStep( state);
    }

    public static boolean isGoal(int[] state_coordinates, int[] goal_coordinates){
        boolean is_row_equal = Coordinates.getRow(state_coordinates) == Coordinates.getRow(goal_coordinates);
        boolean is_column_equal = Coordinates.getCol(state_coordinates) == Coordinates.getCol(goal_coordinates);

        return is_row_equal && is_column_equal;
    }

    public static void createClosedSet() {
        int g_costs_counter = 1;
        closed_states = new int[MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL][g_costs_counter];
        for (int[][] row:closed_states){
            for (int i = 0; i < row.length; i++) {
                Arrays.fill(row[i],Integer.MAX_VALUE);
            }
        }
    }
    //to closed state is added the coordinates with the time step
    //we add to closed state the latest time step : if the coordinates are added at a previouse
    //time step we aupdate that coordinates with the bigger time step
    //it will be ok if the states in prority quees sorts after time deadline
    //the remaining of the prority quee could be checked for  existing time steps not polled????
    public static void addToClosedSet(int[][] state) {
        int[] pos_state = SearchSAStateNormal.getStateCoordinates(state);
        int g_cost = SearchSAStateNormal.getGCost(state);
        int time_state = SearchSAStateNormal.getTimeStep(state);
        int y = SearchSAStateNormal.getYCoordinate(state);
        int x = SearchSAStateNormal.getXCoordinate(state);

        int prev_g_cost = closed_states[y][x][0];
/*
        if (prev_g_cost == 0 ) {
            closed_states[y][x][0] = g_cost ;
            return;
        }
*/
        if (g_cost < prev_g_cost) {
            closed_states[y][x][0] = g_cost;
            return;
        }else {
            //System.out.println("#StateSearchFactory: prev_time_step > time_state ");
        }
    }
    public static boolean isInClosedSet(int[] coordinate, int new_g_cost) {
        int y = Coordinates.getRow(coordinate);
        int x = Coordinates.getCol(coordinate);


        int prev_g_cost = closed_states[y][x][0];
        if(new_g_cost < prev_g_cost)
            return false;

        if(new_g_cost == prev_g_cost){
            int time_state = Coordinates.getTime(coordinate);
            if(Coordinates.getTime(coordinate) > SearchEngineSANormal.getCostCoordinate(coordinate, new_g_cost)){
                return true;
            }else{
                return false;
            }


        }

        return  true;
    }

    public static boolean isInClosedSetWithDeadline(int[] coordinates) {
        /*

        int time_state = Coordinates.getTime(coordinates);
        int y = Coordinates.getRow(coordinates);
        int x = Coordinates.getCol(coordinates);

        boolean is_the_first_value = !(0 < closed_states[y][x][0]);
        boolean is_previouse_explored = !(closed_states[y][x][0] <= time_state);

        //the time_state can not decrease
        return  is_the_first_value && is_previouse_explored;

         */



        //if deadline_enabled : if closed_states[y][x][0] < time_state then
        /*  if (closed_states[y][x][time_step][g_cost1][f_cost1] = [time_step1][g_cost1][f_cost1])
        *              return is_closed_true
        *
        *   if (closed_states[y][x][time_step][g_cost1][f_cost1] = [time_step1][g_cost1][f_cost2])
        *                 if(f_cost1>f_cost2)  return is_closed_false  //because every waiting step decreases the heuristic value
        *
        * */
        return false;
    }

}
