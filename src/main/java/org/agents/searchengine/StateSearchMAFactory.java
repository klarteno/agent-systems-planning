package org.agents.searchengine;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;

import java.io.Serializable;
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
    private static HashMap<Integer,Integer> closed_states_MA;
    private static int[][] index_map_cells;
    private static int number_of_movables;

    private static int[] start_coordinates;
    private static int[] goals_coordinates;
    private static  int[] group_marks_ids;

    private static ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;

    public static void setAvoidanceCheckingRules(ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules) {
        conflict_avoidance_checking_rules = conflictAvoidanceCheckingRules;
    }


    public static void setStartGroup(int[] start_group) {
        group_marks_ids = start_group;
        start_coordinates = new int[start_group.length * Coordinates.getLenght()];
        goals_coordinates = new int[start_group.length * Coordinates.getLenght()];

        int coordinate_index = 0;
        ///Arrays.stream(start_group).parallel().
        Agent agent;
        Box box;
        int[] movable_coordinate;
        int[] goal_cordinate;
        for (int movable_id : start_group) {
            Serializable next_movable = MapFixedObjects.getByMarkNo(movable_id);
            if (next_movable instanceof Agent) {
                agent = (Agent) next_movable;
                movable_coordinate = agent.getCoordinates();
                goal_cordinate = agent.getGoalPosition();

                for (int j = 0; j < Coordinates.getLenght(); j++) {
                    start_coordinates[coordinate_index] = movable_coordinate[j];
                    goals_coordinates[coordinate_index++] = goal_cordinate[j];
                }

            } else if (next_movable instanceof Box) {
                box = (Box) next_movable;
                movable_coordinate = box.getCoordinates();
                goal_cordinate = box.getGoalPosition();

                for (int j = 0; j < Coordinates.getLenght(); j++) {
                    start_coordinates[coordinate_index] = movable_coordinate[j];
                    goals_coordinates[coordinate_index++] = goal_cordinate[j];
                }
            }
            //coordinate_index += Coordinates.getLenght();
        }
        number_of_movables = goals_coordinates.length/Coordinates.getLenght();//to do divide goals__coordinates by the length of the coordinates

    }

    public static int[] getStartCoordinatesGroup() {
            return start_coordinates;
    }

    public static int[] getGoalsCoordinatesGroup() {
        return goals_coordinates;
    }

    //this methos is once when the prority queue is created
    //avoid recomputing the formula for heuristic
    public static void createTimeCostSoFarUnused() {
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

    public static void createClosedSet() {
        closed_states_MA = new HashMap<>();
        //example of usage:
        /*
        int hash_key = Arrays.hashCode(new int[]{1, 2, 3});
        closed_states_MA.put(hash_key,4);
        */

        //store the index of every cell
        index_map_cells = new int[MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL];
        int counter_value = 0;
        int rows_lengh = index_map_cells.length;
        int coll_lengh = index_map_cells[0].length;
        for (int row_index = 0; row_index < rows_lengh ; row_index++) {
            for (int col_index = 0; col_index < coll_lengh ; col_index++) {
                index_map_cells[row_index][col_index] = counter_value++;
            }
        }
    }

    public static void putCostSoFarUnused(int[] next_pos, int g_cost, int f_cost, boolean is_in_heap) {
        assert g_cost > 0;
        //int time_state = Coordinates.getTime(0, next_pos);
        int y;
        int x;
        for (int coordinate = 0; coordinate < next_pos.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
            y = Coordinates.getRow(coordinate, next_pos);
            x = Coordinates.getCol(coordinate, next_pos);

            cost_so_far[coordinate][y][x][G_COST] = g_cost;
            cost_so_far[coordinate][y][x][F_COST] = f_cost;
            if (is_in_heap)
                cost_so_far[coordinate][y][x][IN_HEAP] = 1;
         }
    }

    public static void putCostSoFarUnused(int[][] state) {
        //assert SearchMAState.getGCost(state) > 0;
        int y;
        int x;
        int[] coordinates = SearchMAState.getStateCoordinates(state);
        for (int coordinate = 0; coordinate < coordinates.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
            y = Coordinates.getRow(coordinate, coordinates);
            x = Coordinates.getCol(coordinate, coordinates);

            cost_so_far[coordinate][y][x][G_COST] = SearchMAState.getGCost(state);
            cost_so_far[coordinate][y][x][F_COST] = SearchMAState.getFCost(state);
        }
    }

    //can mark state as in heap by writting a value true for is_in_heap
    public static void mark_state_inqueueUnused(int[][] state, boolean is_in_heap) {
       // assert SearchMAState.getGCost(state) > 0;
        int y;
        int x;
        int[] coordinates = SearchMAState.getStateCoordinates(state);
         for (int coordinate = 0; coordinate < coordinates.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
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

    public static boolean isInCostSoFarUnused(int[] next_pos) {
        int y;
        int x;
        boolean is_present = false;
        for (int coordinate = 0; coordinate < next_pos.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
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
    public static int[] getCostSoFarUnused(int[] next_pos) {
        int y;
        int x;
        int g_cost = 0;
        int f_cost = 0;
        int[] cost = cost_so_far[0][Coordinates.getRow(0, next_pos)][Coordinates.getCol(0, next_pos)];
        for (int coordinate = 1; coordinate < next_pos.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
            y = Coordinates.getRow(coordinate, next_pos);
            x = Coordinates.getCol(coordinate, next_pos);

            cost[G_COST] += cost_so_far[coordinate][y][x][G_COST];
            cost[F_COST] += cost_so_far[coordinate][y][x][F_COST];
        }
        return cost;
    }

    public static boolean isInHeapUnused(int[][] state) {
        int y;
        int x;
        boolean is_present = false;
        int[] coordinates = SearchMAState.getStateCoordinates(state);
        for (int coordinate = 0; coordinate < coordinates.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
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

    public static int getHeuristcOf(int[] cell_coordinates){
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;
        for (int coordinate = 0; coordinate < cell_coordinates.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
            int time_state = Coordinates.getTime(coordinate, cell_coordinates);
            y = Coordinates.getRow(coordinate, cell_coordinates);
            x = Coordinates.getCol(coordinate, cell_coordinates);
            y_goal = Coordinates.getRow(coordinate, goals_coordinates);
            x_goal = Coordinates.getCol(coordinate, goals_coordinates);

            heuristc_value += conflict_avoidance_checking_rules.getHeuristicOf(group_marks_ids[coordinate], time_state, y, x , y_goal, x_goal);
        }
        return heuristc_value;
    }

    private static int getStateHeuristcManhatten(int[] cell_coordinates) {
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;
        for (int coordinate = 0; coordinate < cell_coordinates.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
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

    public static void updateCameFromPrevCell(HashMap<int[],int[]> came_from, int[][] state, int[] previouse_coordinates) {
        came_from.put(getCellCoordinates(state), previouse_coordinates);
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
        for (int coordinate = 0; coordinate < state_coordinates.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
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

    public static void createClosedTimeSet() {
        int time_steps_counter = 1;
        closed_states = new int[number_of_movables][MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL][time_steps_counter];
    }
    //to closed state is added the coordinates with the time step
    //we add to closed state the latest time step : if the coordinates are added at a previouse
    //time step we update that coordinates with the bigger time step
    //it will be ok if the states in prority quees sorts after time deadline
    //the remaining of the prority quee could be checked for  existing time steps not polled????
    public static void addToClosedTimeSet(int[][] state) {
        int[] pos_coordinates = SearchMAState.getStateCoordinates(state);
        int coordinate_index = 0;
        int time_state = SearchMAState.getTimeStep(coordinate_index, state);//arbitarily 0 chosen

        int prev_time_step;
        int y;
        int x;
        int count_of_movables = pos_coordinates.length/Coordinates.getLenght();
        for (int coordinate = 0; coordinate < count_of_movables; coordinate = coordinate + 1) {
            y = Coordinates.getRow(coordinate, pos_coordinates);
            x = Coordinates.getCol(coordinate, pos_coordinates);

            prev_time_step = closed_states[coordinate][y][x][0];

            if (!(prev_time_step > 0 )) {
                closed_states[coordinate][y][x][0] = time_state ;
            }

            if (prev_time_step < time_state ) {
                closed_states[coordinate][y][x][0] = time_state;
            }else {
                System.out.println("#StateSearchFactory: prev_time_step > time_state ");
            }
        }
    }


    public static void addToClosedSet(int[][] state) {
        int[] pos_coordinates = SearchMAState.getStateCoordinates(state);
        int g_cost_state = SearchMAState.getGCost(state);

        int y;
        int x;
        int count_of_movables = pos_coordinates.length/Coordinates.getLenght();
        int[] state_to_close = new int[count_of_movables];
        int __index = 0;
        int g_cost_acumulated = 0;
        for (int coordinate = 0; coordinate < count_of_movables; coordinate = coordinate + 1) {
            y = Coordinates.getRow(coordinate, pos_coordinates);
            x = Coordinates.getCol(coordinate, pos_coordinates);

            int index_counter = index_map_cells[y][x];
            state_to_close[__index++] = index_counter;
        }

        int hash_key = Arrays.hashCode(state_to_close);
        if(!closed_states_MA.containsKey(hash_key)){
            closed_states_MA.put(hash_key, g_cost_state);
        }else {
            int __cost = closed_states_MA.get(hash_key);
            if(closed_states_MA.get(hash_key) > g_cost_state)
                    closed_states_MA.replace(hash_key, g_cost_state);
        }
    }

    public static boolean isInClosedSet(int[][] state) {
        int g_cost_state = SearchMAState.getGCost(state);
        int[] pos_coordinates = SearchMAState.getStateCoordinates(state);
        int y;
        int x;
        int count_of_movables = pos_coordinates.length/Coordinates.getLenght();
        int[] state_to_check = new int[count_of_movables];
        int __index = 0;
        int index_counter;
        for (int coordinate = 0; coordinate < count_of_movables; coordinate = coordinate + 1) {
            y = Coordinates.getRow(coordinate, pos_coordinates);
            x = Coordinates.getCol(coordinate, pos_coordinates);

            index_counter = index_map_cells[y][x];
            state_to_check[__index++] = index_counter;
        }

        int hash_key = Arrays.hashCode(state_to_check);
        if(!closed_states_MA.containsKey(hash_key)){
            return false;
         }else {
            int value__ = closed_states_MA.get(hash_key);
            return closed_states_MA.get(hash_key) <= g_cost_state;
        }
    }

    public static boolean isInClosedTimeSet(int[] coordinates) {
        int[] pos_coordinates = SearchMAState.getStateCoordinates(state);
        int coordinate_index = 0;
        int time_state = Coordinates.getTime(coordinate_index, coordinates);
        int y;
        int x;
        boolean is_present = false;

        int count_of_movables = pos_coordinates.length/Coordinates.getLenght();
        for (int coordinate = 0; coordinate < count_of_movables; coordinate = coordinate + 1) {
            y = Coordinates.getRow(coordinate, pos_coordinates);
            x = Coordinates.getCol(coordinate, pos_coordinates);

            //the time_state can not decrease
            if (closed_states[coordinate][y][x][0] >= time_state) {
                is_present = true;
            }else {
                return false;
            }
        }
        return is_present;
    }


    public static boolean isStandardNode(int[] pos_coordinates) {
        int prev_time = Coordinates.getTime(0, pos_coordinates);
        int next_time;

        if(pos_coordinates.length > Coordinates.getLenght()){
            for (int coordinate = 0; coordinate < pos_coordinates.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
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
                                                                                            //, set_edge_conflict
    public static ArrayDeque<int[][]> expandStandardState(int[] pos_coordinates, int g_cost, int f_cost) {
        Random random = new Random();
        int index_to_expand = random.nextInt(pos_coordinates.length/Coordinates.getLenght());//or other heuristic to use instead of random

        int prev_time = Coordinates.getTime(index_to_expand, pos_coordinates);
        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

        int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);

        int mark_id = group_marks_ids[index_to_expand];
        LinkedList<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand, new ArrayDeque<int[]>());

        //if neighbours contains one of the other mark_ids from standard node :
          //          add it to the set_edge_conflict

        int[] next_state_node;
         for(int [] cell_pos : neighbours){
             next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
             Coordinates.setCoordinateAtIndex(index_to_expand, next_state_node, cell_pos);
             next_state_nodes.add(SearchMAState.createNew(next_state_node, g_cost, f_cost));
         }



        return next_state_nodes;
    }


    public static ArrayDeque<int [][]> expandIntermediateState(int[] pos_coordinates, int g_cost, int f_cost) {
        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

        int min_time = Integer.MAX_VALUE;
        int coord_to_expand = -1;
        int __time ;
        for (int coordinate = 0; coordinate < pos_coordinates.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
            __time = Coordinates.getTime(coordinate, pos_coordinates);
            if(__time < min_time){
                min_time = __time;
                coord_to_expand = coordinate;
            }
        }



        if(coord_to_expand > -1) {
            int[] arr = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
            Coordinates.setTime(coord_to_expand, arr,min_time+1);

            ArrayDeque<int []> conflicts_avoidance = new ArrayDeque<>(); //transform it to verstex conflixts and edge conflicts
            for (int index = 0; index < pos_coordinates.length/Coordinates.getLenght(); index = index + 1){
                int next_time = Coordinates.getTime(index, pos_coordinates);
                if (min_time < next_time){
                    int[] __coord = Coordinates.getCoordinatesAt(index, pos_coordinates);
                    conflicts_avoidance.add(__coord);
                }
            }
            int[] to_expand = Coordinates.getCoordinatesAt(coord_to_expand, pos_coordinates);

            int mark_id = group_marks_ids[coord_to_expand];
            LinkedList<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, to_expand, conflicts_avoidance);

           // if neighbours contains a cell at time_step + 1  another cell at time_step
              //  add edge conflict


            int[] next_state_node;
            for(int [] cell_pos : neighbours){
                next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                Coordinates.setCoordinateAtIndex(coord_to_expand, next_state_node, cell_pos);
                next_state_nodes.add(SearchMAState.createNew(next_state_node, g_cost, f_cost));
            }
        }
        return next_state_nodes;
    }

    public static ArrayDeque<int [][]> expandIntermediateStatePrevProbablyNotGoodToDel(int[] pos_coordinates, int g_cost, int f_cost) {
        int prev_time = Coordinates.getTime(0, pos_coordinates);
        int next_time;
        int previouse_coordinate_number;
        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

        for (int coordinate = 1; coordinate < pos_coordinates.length/Coordinates.getLenght(); coordinate = coordinate + 1) {
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

                int mark_id = group_marks_ids[previouse_coordinate_number];
                LinkedList<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, to_expand, conflicts_avoidance);


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



