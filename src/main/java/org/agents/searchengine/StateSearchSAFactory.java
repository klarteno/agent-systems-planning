package org.agents.searchengine;

import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.planning.SearchState;

import java.util.HashMap;

final class StateSearchFactory {
    public static final int G_COST = 0;
    public static final int F_COST = 1;
    public static final int IN_HEAP = 2;


    private static int [][] state;
    private static int [][][] cost_so_far;
    private static int [][][] closed_states;
    private static int[][] deadline_constraint;

    SearchState searchState;

    //this methos is once when the prority queue is created
    //avoid recomputing the formula for heuristic
    public static void createCostSoFar() {
        //check later if should be encapsulated in a staic class like SearchState.java
        //third index : one for g_cost   and second one for f_cost
        cost_so_far = new int[MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL][3];
        //cost_so_far[coord_y][coord_x]=new int[]{g_cost,f_cost,is_in_heap};
    }

    /*
        public static void updateCost(HashMap<Integer, Integer> cost_so_far) {
            cost_so_far.put(Arrays.hashCode(state[SearchState.ARRAYPOS]), state[ARRAYCOSTS][Costs.COST_G.ordinal()]);
        }
    */
    public static void putCostSoFar(int[] next_, int g_cost, int f_cost, boolean is_in_heap) {
        assert g_cost > 0;
        //int[] pos_state = SearchState.getCellCoordinates(state);
        int time_state = Coordinates.getTime(next_);
        //int time_state = SearchState.getTimeStep(state);
        int y = Coordinates.getRow(next_);
        //int y = SearchState.getYCoordinate(state);
        int x = Coordinates.getCol(next_);
        //int x = SearchState.getXCoordinate(state);

        cost_so_far[y][x][G_COST] = g_cost ;
        cost_so_far[y][x][F_COST] = f_cost ;
        if (is_in_heap)
            cost_so_far[y][x][IN_HEAP] = 1;
     }

     public static void putCostSoFar(int[][] state) {
        assert SearchState.getGCost(state) > 0;

        int[] pos_state = SearchState.getCellCoordinates(state);
        int time_state = SearchState.getTimeStep(state);
        int y = SearchState.getYCoordinate(state);
        int x = SearchState.getXCoordinate(state);

        cost_so_far[y][x][G_COST] = SearchState.getGCost(state);
        cost_so_far[y][x][F_COST] = SearchState.getFCost(state);

    }

    //can mark state as in heap by writting a value true for is_in_heap
    public static void mark_state_inqueue(int[][] state, boolean is_in_heap) {
        assert SearchState.getGCost(state) > 0;

        int[] pos_state = SearchState.getCellCoordinates(state);
        int time_state = SearchState.getTimeStep(state);
        int y = SearchState.getYCoordinate(state);
        int x = SearchState.getXCoordinate(state);

        if (is_in_heap){
            cost_so_far[y][x][IN_HEAP] = 1;
        }
        else {
            cost_so_far[y][x][IN_HEAP] = 0;
        }
    }

    public static boolean isInCostSoFar(int[] next_) {
        //int[] pos_state = SearchState.getCellCoordinates(state);
        int time_state = Coordinates.getTime(next_);
        //int time_state = SearchState.getTimeStep(state);
        int y = Coordinates.getRow(next_);
        //int y = SearchState.getYCoordinate(state);
        int x = Coordinates.getCol(next_);
        //int x = SearchState.getXCoordinate(state);

        return cost_so_far[y][x][G_COST] > 0 ;
    }

    public static int[] getCostSoFar(int[] next_) {
        int time_state = Coordinates.getTime(next_);
        int y = Coordinates.getRow(next_);
        int x = Coordinates.getCol(next_);

        int prev_g_step = cost_so_far[y][x][G_COST];
        int prev_f_step = cost_so_far[y][x][F_COST];

        return cost_so_far[y][x];
    }

    public static boolean isInHeap(int[][] next_) {
        //int[] pos_state = SearchState.getCellCoordinates(state);
        //int time_state = Coordinates.getTime(next_);
        //int time_state = SearchState.getTimeStep(state);
        //int y = Coordinates.getRow(next_);
        int y = SearchState.getYCoordinate(next_);
        //int x = Coordinates.getCol(next_);
        int x = SearchState.getXCoordinate(next_);

        return cost_so_far[y][x][IN_HEAP] > 0 ;
    }

    public static int[][] createDummyState() {
        return SearchState.createDummyState();
    }

    //is used many times to create a state for the a_star
    public static int[][] createState(int[] cell_coordinates, int total_gcost, int[] goal_coordinates){
        assert (goal_coordinates.length == 3);
        assert (cell_coordinates.length == 3);

        int heuristc_value = SearchEngine.getHeuristic(cell_coordinates, goal_coordinates);
        //int heuristc_value2 = SearchEngine.getConsistentHeuristic(cost_time,cell_coordinates, goal_coordinates);
        state  = SearchState.createNew(cell_coordinates, total_gcost, heuristc_value + total_gcost);

        return state;
    }

    public static int[][] createState(int[] cell_neighbour, int neighbour_gcost, int f_value, int[] goal_coordinates) {
        assert (goal_coordinates.length == 3);
        assert (cell_neighbour.length == 3);
        state  = SearchState.createNew(cell_neighbour, neighbour_gcost, f_value);

        return state;
    }


    public static int getGCost(int[][] state) {
        return SearchState.getGCost(state);
    }

    public static void updateCameFromPrevCell(HashMap<int[],int[]> came_from, int[][] state, int[][] previouse_coordinates) {
        came_from.put(getCellCoordinates(state), getCellCoordinates(previouse_coordinates));
    }

    public static int[] getCellCoordinates(int[][] state){
        return SearchState.getCellCoordinates(state);
    }

    private static int  getYCoordinate(int[][] state){
        return SearchState.getYCoordinate(state);
    }

    private static int  getXCoordinate(int[][] state){
        return SearchState.getXCoordinate(state);
    }

    private static int  getTimeStep(int[][] state){
       return SearchState.getTimeStep( state);
    }

    public static boolean isGoal(int[] state_coordinates, int[] goal_coordinates){
        return (state_coordinates[0] == goal_coordinates[0]) && (state_coordinates[1] == goal_coordinates[1]);
    }

    public static void createClosedSet() {
        int time_steps_counter = 1;
        closed_states = new int[MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL][time_steps_counter];
    }
    //to closed state is added the coordinates with the time step
    //we add to closed state the latest time step : if the coordinates are added at a previouse
    //time step we aupdate that coordinates with the bigger time step
    //the remaining of the prority quee could be checked for  existing time steps not polled????
    //it will be ok if the states in prority quees sorts after time deadline
    public static void addToClosedSet(int[][] state) {
        int[] pos_state = SearchState.getCellCoordinates(state);
        //int time_state = Coordinates.getTime(pos_state);
        int time_state = SearchState.getTimeStep(state);
        //int y = Coordinates.getRow(pos_state);
        int y = SearchState.getYCoordinate(state);
        //int x = Coordinates.getCol(pos_state);
        int x = SearchState.getXCoordinate(state);

        int prev_time_step = closed_states[y][x][0];

        if (!(prev_time_step > 0 )) {
            cost_so_far[y][x][0] = time_state ;
        }

        if (prev_time_step < time_state ) {
            cost_so_far[y][x][0] = time_state;
        }else {
            System.out.println("#StateSearchFactory: prev_time_step > time_state ");
        }
    }
    public static boolean isInClosedSet(int[] coordinates) {
        int time_state = Coordinates.getTime(coordinates);
        int y = Coordinates.getRow(coordinates);
        int x = Coordinates.getCol(coordinates);

        //the time_state can not decrease
        return cost_so_far[y][x][0] >= time_state ;
    }

    public static int getDeadlineTimeConstraint() {
        return SearchState.getTimeStep(deadline_constraint);
    }

    public static int[] getDeadlineCoord() {
        return SearchState.getCellCoordinates(deadline_constraint) ;
    }
    public static void setDeadlineConstraint(int[] coordinate,int total_gcost,int heuristc_value) {
          deadline_constraint = SearchState.createNew(coordinate, total_gcost, heuristc_value + total_gcost);
    }

}
