package org.agents.searchengine;

import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;

import java.util.*;

final class StateSearchMAFactory {
    public static final int G_COST = 0;
    public static final int F_COST = 1;
    public static final int IN_HEAP = 2;

    final static int STATE_STANDARD = 0;  //starndard node or intermediate node as in stanleys paper
    final static int STATE_INTERMEDIATE = 1; //intermediate state is when not all agents in the position advanced a time step

    private static int [][] state;
    private static int [][][][] cost_so_far;
    //switch to bits shifting
    private static int [][][][] closed_states;
    private static int[] deadline_constraint;
    private static int number_of_movables;
    private static int[] goals_coordinates;
    private static int[][][] conflicting_paths;

    public static void setGoals(int[] goals__coordinates) {
        goals_coordinates =  goals__coordinates;
        number_of_movables = goals_coordinates.length/Coordinates.getLenght();//to do divide goals__coordinates by the length of the coordinates
    }

    public static void setConflictingPaths(int[][][] conflicting__paths) {
        conflicting_paths = conflicting__paths;

    }

    //this methos is once when the prority queue is created
    //avoid recomputing the formula for heuristic
    public static void createCostSoFar() {
        //check later if should be encapsulated in a staic class like SearchState.java
        //third index : one for g_cost   and second one for f_cost
        //
        cost_so_far = new int[number_of_movables][MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL][3];
        //cost_so_far[agent_index][coord_y][coord_x] = new int[]{g_cost,f_cost,is_in_heap};
    }

    /*
        public static void updateCost(HashMap<Integer, Integer> cost_so_far) {
            cost_so_far.put(Arrays.hashCode(state[SearchState.ARRAYPOS]), state[ARRAYCOSTS][Costs.COST_G.ordinal()]);
        }
    */
    public static void putCostSoFar(int[] next_pos, int g_cost, int f_cost, boolean is_in_heap) {
        assert g_cost > 0;
        //int time_state = Coordinates.getTime(0, next_pos);
        int y;
        int x;
        for (int coordinate = 0; coordinate < next_pos.length; coordinate = coordinate + Coordinates.getLenght()) {
            y = Coordinates.getRow(coordinate, next_pos);
            x = Coordinates.getCol(coordinate, next_pos);

            cost_so_far[coordinate][y][x][G_COST] = g_cost;
            cost_so_far[coordinate][y][x][F_COST] = f_cost;
            if (is_in_heap)
                cost_so_far[coordinate][y][x][IN_HEAP] = 1;
         }
    }


    public static void putCostSoFar(int[][] state) {
        //assert SearchMAState.getGCost(state) > 0;

        int y;
        int x;
        int[] coordinates = SearchMAState.getStateCoordinates(state);
        for (int coordinate = 0; coordinate < coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
            y = Coordinates.getRow(coordinate, coordinates);
            x = Coordinates.getCol(coordinate, coordinates);

            cost_so_far[coordinate][y][x][G_COST] = SearchMAState.getGCost(state);
            cost_so_far[coordinate][y][x][F_COST] = SearchMAState.getFCost(state);
        }
    }

    //can mark state as in heap by writting a value true for is_in_heap
    public static void mark_state_inqueue(int[][] state, boolean is_in_heap) {
       // assert SearchMAState.getGCost(state) > 0;

        int y;
        int x;
        int[] coordinates = SearchMAState.getStateCoordinates(state);
        for (int coordinate = 0; coordinate < coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
            y = Coordinates.getRow(coordinate, coordinates);
            x = Coordinates.getCol(coordinate, coordinates);

            if (is_in_heap){
                cost_so_far[coordinate][y][x][IN_HEAP] = 1;
            }
            else {
                cost_so_far[coordinate][y][x][IN_HEAP] = 0;
            }
        }
    }

    public static boolean isInCostSoFar(int[] next_pos) {
        int y;
        int x;
        boolean is_present = false;
        for (int coordinate = 0; coordinate < next_pos.length; coordinate = coordinate + Coordinates.getLenght()) {
            y = Coordinates.getRow(coordinate, next_pos);
            x = Coordinates.getCol(coordinate, next_pos);

            if (cost_so_far[coordinate][y][x][G_COST] > 0){
                is_present = true;
            }
            else {
                return false;
            }
        }

        return is_present;
    }
//returns the sum of paths for all agents
    public static int[] getCostSoFar(int[] next_pos) {
        int y;
        int x;
        int g_cost = 0;
        int f_cost = 0;
        int[] cost = cost_so_far[0][Coordinates.getRow(0, next_pos)][Coordinates.getCol(0, next_pos)];
        for (int coordinate = 1; coordinate < next_pos.length; coordinate = coordinate + Coordinates.getLenght()) {
            y = Coordinates.getRow(coordinate, next_pos);
            x = Coordinates.getCol(coordinate, next_pos);

            cost[G_COST] += cost_so_far[coordinate][y][x][G_COST];
            cost[F_COST] += cost_so_far[coordinate][y][x][F_COST];
        }

        return cost;
    }

    public static boolean isInHeap(int[][] state) {
        int y;
        int x;
        boolean is_present = false;
        int[] coordinates = SearchMAState.getStateCoordinates(state);
        for (int coordinate = 0; coordinate < coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
            y = Coordinates.getRow(coordinate, coordinates);
            x = Coordinates.getCol(coordinate, coordinates);

            if (cost_so_far[coordinate][y][x][IN_HEAP] >= 1){
                 is_present = true;
            }
            else {
                return false;
            }
        }
        return is_present;
    }

    public static int[][] createDummyState() {
        return SearchMAState.createDummyState(number_of_movables);
    }

    public static int getHeuristic(int[] cell_coordinates, int[] goal_coordinates){
        return Math.abs(Coordinates.getRow(cell_coordinates) - Coordinates.getRow(goal_coordinates)) + Math.abs(Coordinates.getCol(cell_coordinates) - Coordinates.getCol(goal_coordinates))  ;
    }

    public static int getHeuristic(int y, int x, int y_goal, int x_goal) {
        return Math.abs(y - y_goal) + Math.abs(x - x_goal);
    }

    public static int getConsistentHeuristic(int cost_time, int y, int x , int y_goal, int x_goal){
        int time_left = StateSearchMAFactory.getDeadlineTimeConstraint() - cost_time;
        int y_sub_goal = Coordinates.getRow(deadline_constraint);
        int x_sub_goal = Coordinates.getCol(deadline_constraint);

        if (time_left <= 0){
            return getHeuristic(y, x, y_goal, x_goal);
        } else{
            return time_left + getHeuristic(y_sub_goal, x_sub_goal, y_goal, x_goal);
        }
    }

    public static void setDeadlineConstraint() {
        int[][] path;
        int time_deadline = -1;
        int row_deadline = -1;
        int column_deadline = -1;
        for (int i = 0; i < conflicting_paths.length ; i++) {
            path = conflicting_paths[i];
            for (int row = 0; row < path.length; row++) {
                for (int column = 0; column < path[row].length; column++) {
                    if(path[row][column] > time_deadline){
                        time_deadline = path[row][column];
                        row_deadline = row;
                        column_deadline = column;
                    }
                }
            }
        }

        deadline_constraint = Coordinates.createCoordinates(time_deadline, row_deadline, column_deadline);
    }

    private static int getDeadlineTimeConstraint() {
        return Coordinates.getTime(deadline_constraint);
    }

    private static int getHeuristcOf(int[] cell_coordinates){
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;

        for (int coordinate = 0; coordinate < cell_coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
            int time_state = Coordinates.getTime(coordinate, cell_coordinates);

            y = Coordinates.getRow(coordinate, cell_coordinates);
            x = Coordinates.getCol(coordinate, cell_coordinates);

            y_goal = Coordinates.getRow(coordinate, goals_coordinates);
            x_goal = Coordinates.getCol(coordinate, goals_coordinates);

            heuristc_value += getConsistentHeuristic(time_state, y, x , y_goal, x_goal);
        }
        return heuristc_value;
    }

    private static int getStateHeuristcManhatten(int[] cell_coordinates) {
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;
        for (int coordinate = 0; coordinate < cell_coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
            y = Coordinates.getRow(coordinate, cell_coordinates);
            x = Coordinates.getCol(coordinate, cell_coordinates);

            y_goal = Coordinates.getRow(coordinate, goals_coordinates);
            x_goal = Coordinates.getCol(coordinate, goals_coordinates);

            heuristc_value += getHeuristic(y, x , y_goal, x_goal);
        }
        return heuristc_value;
    }

    //is used many times to create a state for the a_star
    public static int[][] createStandardState(int[] cell_coordinates, int total_gcost){
        assert ( ( cell_coordinates.length/Coordinates.getLenght() ) == number_of_movables);

        //int heuristc_value = getStateHeuristcManhatten(cell_coordinates);
        int heuristc_value = getHeuristcOf(cell_coordinates);
        state  = SearchMAState.createNew(cell_coordinates, total_gcost, heuristc_value + total_gcost);

        return state;
    }

    //is used many times to create a state for the a_star
    public static int[][] createIntermediatePositiom(int[] cell_coordinates, int total_gcost){
        assert (cell_coordinates.length/Coordinates.getLenght() == number_of_movables);

        //int heuristc_value = getStateHeuristcManhatten(cell_coordinates);
        int heuristc_value = getHeuristcOf(cell_coordinates);
        state  = SearchMAState.createNew(cell_coordinates,total_gcost, heuristc_value + total_gcost);

        return state;
    }

    public static int[][] createStandardState(int[] cell_neighbour, int neighbour_gcost, int f_value) {
        assert (cell_neighbour.length == number_of_movables);
        state  = SearchMAState.createNew(cell_neighbour, neighbour_gcost, f_value);

        return state;
    }

    public static int[][] createIntermediatePositiom(int[] cell_neighbour, int neighbour_gcost, int f_value) {
        assert (cell_neighbour.length == number_of_movables);
        state  = SearchMAState.createNew(cell_neighbour, neighbour_gcost, f_value);

        return state;
    }

    public static int getGCost(int[][] state) {
        return SearchMAState.getGCost(state);
    }

    public static void updateCameFromPrevCell(HashMap<int[],int[]> came_from, int[][] state, int[][] previouse_coordinates) {
        came_from.put(getCellCoordinates(state), getCellCoordinates(previouse_coordinates));
    }

    public static int[] getCellCoordinates(int[][] state){
        return SearchMAState.getStateCoordinates(state);
    }

    private static int  getYCoordinate(int movable_index, int[][] state){
        return SearchMAState.getYCoordinate(movable_index, state);
    }

    private static int  getXCoordinate(int movable_index, int[][] state){
        return SearchMAState.getXCoordinate(movable_index, state);
    }

    private static int  getTimeStep(int movable_index, int[][] state){
        return SearchMAState.getTimeStep(movable_index, state);
    }

    public static boolean isGoal(int[] state_coordinates){
        int y;
        int x;
        int y_goal;
        int x_goal;

        boolean isGoal = false;
        for (int coordinate = 0; coordinate < state_coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
            y = Coordinates.getRow(coordinate, state_coordinates);
            x = Coordinates.getCol(coordinate, state_coordinates);

            y_goal = Coordinates.getRow(coordinate, goals_coordinates);
            x_goal = Coordinates.getCol(coordinate, goals_coordinates);

            isGoal = (y == y_goal) && (x == x_goal);
            if (!isGoal)
                return false;
        }

        return isGoal;
    }

    public static void createClosedSet() {
        int time_steps_counter = 1;
        closed_states = new int[number_of_movables][MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL][time_steps_counter];
    }
    //to closed state is added the coordinates with the time step
    //we add to closed state the latest time step : if the coordinates are added at a previouse
    //time step we update that coordinates with the bigger time step
    //it will be ok if the states in prority quees sorts after time deadline
    //the remaining of the prority quee could be checked for  existing time steps not polled????
    public static void addToClosedSet(int[][] state) {
        int[] pos_coordinates = SearchMAState.getStateCoordinates(state);
        int time_state = SearchMAState.getTimeStep(0, state);//arbitarily 0 chosen

        int prev_time_step;
        int y;
        int x;
        for (int coordinate = 0; coordinate < pos_coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
            y = Coordinates.getRow(coordinate, pos_coordinates);
            x = Coordinates.getCol(coordinate, pos_coordinates);

            for (int i = 0; i < number_of_movables; i++) {
                prev_time_step = closed_states[i][y][x][0];

                if (!(prev_time_step > 0 )) {
                    closed_states[i][y][x][0] = time_state ;
                }

                if (prev_time_step < time_state ) {
                    closed_states[i][y][x][0] = time_state;
                }else {
                    System.out.println("#StateSearchFactory: prev_time_step > time_state ");
                }
            }
        }
    }

    public static boolean isInClosedSet(int[] coordinates) {
        int[] pos_coordinates = SearchMAState.getStateCoordinates(state);
        int time_state = Coordinates.getTime(0, coordinates);
        int y;
        int x;
        boolean is_present = false;
        for (int coordinate = 0; coordinate < pos_coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
            y = Coordinates.getRow(coordinate, pos_coordinates);
            x = Coordinates.getCol(coordinate, pos_coordinates);

            for (int i = 0; i < number_of_movables; i++) {
                //the time_state can not decrease
                if (closed_states[i][y][x][0] >= time_state) {
                    is_present = true;
                }else {
                    return false;
                }
            }
        }
        return is_present;
    }

    public static boolean isStandardNode(int[] pos_coordinates) {
        int prev_time = Coordinates.getTime(0, pos_coordinates);
        int next_time;

        if(pos_coordinates.length > Coordinates.getLenght()){
            for (int coordinate = 1; coordinate < pos_coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
                next_time = Coordinates.getTime(coordinate, pos_coordinates);
                if (prev_time != next_time)
                    return false;
            }
        }

        return true;
    }

    public static boolean isIntermediateNode(int[] pos_coordinates) {
        return !isStandardNode(pos_coordinates);
    }

    private static void discardConflictsMA(LinkedList<int[]> dirs, ArrayDeque<int[]> conflicts_avoidance) {
        int index_last;
        int [] temp;
        while (!conflicts_avoidance.isEmpty()){
            int[] cell = conflicts_avoidance.pop();
            for (int i = 0; i < dirs.size(); i++) {
                if (Arrays.equals(cell, dirs.get(i))){
                    int[] res = dirs.remove(i);
                }
            }
        }
        /*
        ArrayDeque<int[]> neighbours_indexes = new ArrayDeque<>();
        for (int[] dir : dirs) {
            if (dir != null) {
                neighbours_indexes.add(dir);
            }
        }
        */
    }

    private static void discardConflictingPaths(LinkedList<int[]> directions){
        //int[][] new_directions = directions;
        for (int i = 0; i < directions.size(); i++){
            int[] dir = directions.get(i);
            int row = Coordinates.getRow(0, dir);
            int col = Coordinates.getCol(0, dir);
            int time_step = Coordinates.getTime(0, dir);

            for (int[][] path : conflicting_paths){
                if(path[row][col] == time_step){
                    directions.remove(i);
                    break;
                }
            }
        }
    }

    public static ArrayDeque<int[][]> expandStandardState(int[] pos_coordinates, int g_cost, int f_cost) {
        Random random = new Random();
        int index_to_expand = random.nextInt(pos_coordinates.length/Coordinates.getLenght());//or other heuristic to use instead of random

        int prev_time = Coordinates.getTime(index_to_expand, pos_coordinates);
        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

        int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);
        LinkedList<int[]> neighbours = MapFixedObjects.getNeighboursMA(position_to_expand, Coordinates.getTime(deadline_constraint));
        discardConflictingPaths(neighbours);//last step , now the neighbours are valid

        int[] next_state_node;
         for(int [] cell_pos : neighbours){
             next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
             Coordinates.setCoordinateAtIndex(index_to_expand, next_state_node, cell_pos);
             next_state_nodes.add(SearchMAState.createNew(next_state_node, g_cost, f_cost));
         }

        return next_state_nodes;
    }

    public static ArrayDeque<int [][]> expandIntermediateState(int[] pos_coordinates, int g_cost, int f_cost) {
         int prev_time = Coordinates.getTime(0, pos_coordinates);
        int next_time;
        int previouse_coordinate_number;
        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();
        for (int coordinate = 1; coordinate < pos_coordinates.length; coordinate = coordinate + Coordinates.getLenght()) {
            next_time = Coordinates.getTime(coordinate, pos_coordinates);
            if (prev_time < next_time){
                previouse_coordinate_number = coordinate - 1;
                int[] arr = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                Coordinates.setTime(previouse_coordinate_number, arr,prev_time+1);
                ArrayDeque<int []> conflicts_avoidance = new ArrayDeque<>();
                for (int index = 0; index < pos_coordinates.length; index = index + Coordinates.getLenght()){
                    next_time = Coordinates.getTime(index, pos_coordinates);
                    if (prev_time < next_time){
                        conflicts_avoidance.add(Coordinates.getCoordinatesAt(index, pos_coordinates));
                    }
                }
                int[] to_expand = Coordinates.getCoordinatesAt(previouse_coordinate_number, pos_coordinates);

                LinkedList<int[]> neighbours = MapFixedObjects.getNeighboursMA(to_expand, Coordinates.getTime(deadline_constraint));
                discardConflictingPaths(neighbours);

                discardConflictsMA(neighbours, conflicts_avoidance);
                assert conflicts_avoidance.size() == 0;

                int[] next_state_node;
                for(int [] cell_pos : neighbours){
                    next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                    Coordinates.setCoordinateAtIndex(previouse_coordinate_number, next_state_node, cell_pos);
                    next_state_nodes.add(SearchMAState.createNew(next_state_node, g_cost, f_cost));
                }
            }
            prev_time = next_time;
        }
            return next_state_nodes;
    }


}



