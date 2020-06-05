package org.agents.searchengine;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.dto.EdgeConflict;
import org.agents.planning.conflicts.dto.SimulationConflict;
import org.agents.planning.conflicts.dto.VertexConflict;
import org.agents.planning.constraints.PullConstraint;

import java.io.Serializable;
import java.util.*;

public final class StateSearchMAFactory {
    public static final int G_COST = 0;
    public static final int F_COST = 1;
    public static final int IN_HEAP = 2;

    final static int STATE_STANDARD = 0;  //starndard node or intermediate node as in stanleys paper
    final static int STATE_INTERMEDIATE = 1; //intermediate state is when not all agents in the position advanced a time step

    static final int COST_NEXT_CELL = 1;

    private static int [][] state;
    private static int [][][][] cost_so_far;

    private static int [][][][] closed_states;
    private static HashMap<Integer,Integer> closed_states_MA;
    private static int[][] index_map_cells;
    static int number_of_movables;

    private static int[] start_coordinates;
    private static int[] goals_coordinates;
    private static  int[] group_marks_ids;


    private static HashMap<Integer,Integer> all_agents_indexes;
    private static HashMap<Integer,Integer> all_boxes_indexes;
    private static HashMap<Integer, Set<Integer>> all_agents_to_boxes;
    private static HashMap<Integer, Set<Integer>> all_boxes_to_agents;

    private static HashMap<Integer, Set<Integer>> agent_boxes_to_avoid;// //avoid boxes not of the agent color
    private static int[] index_positions_to_agent_boxes_to_avoid;
    private static ArrayList<HashMap<Integer,int[]>> list_boxes_coord_to_avoid;


    private static ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;

    static SearchState searchMultiAgentState = SearchState.AGENTS_ONLY;


    private static int[] heuristic_standard_coordinates_output;
    private static int[][]  standard_node_costs;
    private static int[] heuristic_intermediate_coordinates_output;
    private static int[][]  intermediate_node_costs;


    public enum SearchState {
        AGENTS_ONLY,
        AGENTS_AND_BOXES;
    };

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

        setUpAgentsWithBoxesFromGroup();
        initStandardNodeCosts();
        initIntermediateNodeCosts();

    }

    //groups the agents and boxes by color
    private static void setUpAgentsWithBoxesFromGroup(){
        all_agents_indexes = new HashMap<>();//first is for position index , second is for color
        all_boxes_indexes = new HashMap<>();//first is for position index , second is for color

        for (int movable_index = 0; movable_index < number_of_movables; movable_index++) {
            int mark_id = group_marks_ids[movable_index];
            Serializable movable_to_exapand = MapFixedObjects.getByMarkNo(mark_id);
            if (movable_to_exapand instanceof Agent){
                Agent agent = (Agent) movable_to_exapand;
                int agent_color = agent.getColor();
                all_agents_indexes.put(movable_index, agent_color);
            }else if (movable_to_exapand instanceof Box){
                Box box = (Box) movable_to_exapand;
                int box_color = box.getColor();
                all_boxes_indexes.put(movable_index, box_color);
            }
            else {
                throw new UnsupportedOperationException("method :setConflictsStandardStateExpansionForAgentsAndBoxes");
            }
        }

        all_agents_to_boxes = new HashMap<>();//first is for position index of agent, second is for set of indexes of box
        for (Integer agent_key: all_agents_indexes.keySet()){
            for (Integer box_key: all_boxes_indexes.keySet()){
                if(all_agents_indexes.get(agent_key).equals(all_boxes_indexes.get(box_key))){
                    if (all_agents_to_boxes.containsKey(agent_key)){
                        all_agents_to_boxes.get(agent_key).add(box_key);
                    }else{
                        Set<Integer> boxes_indexes = new HashSet<>();
                        boxes_indexes.add(box_key);
                        all_agents_to_boxes.put(agent_key,boxes_indexes);
                    }
                }
            }
        }


        all_boxes_to_agents = new HashMap<>();//first is for position index of agent, second is for set of indexes of box
        for (Integer box_key: all_boxes_indexes.keySet()){
            for (Integer agent_key: all_agents_indexes.keySet()){
                if(all_boxes_indexes.get(box_key).equals(all_agents_indexes.get(agent_key))){
                    if (all_boxes_to_agents.containsKey(box_key)){
                        all_boxes_to_agents.get(box_key).add(agent_key);
                    }else{
                        Set<Integer> agents_indexes = new HashSet<>();
                        agents_indexes.add(agent_key);
                        all_boxes_to_agents.put(box_key, agents_indexes);
                    }
                }
            }
        }


        //avoid boxes not of the agent color
        agent_boxes_to_avoid = new HashMap<>();
        //avoid boxes not of the agent color
        Set<Integer> boxes_indexes_to_avoid2;
        for (Integer agent_key1: all_agents_to_boxes.keySet()){
            boxes_indexes_to_avoid2 = new HashSet<>();
            for (Integer agent_key2: all_agents_to_boxes.keySet()){
                if(!agent_key1.equals(agent_key2)){
                    boxes_indexes_to_avoid2.addAll(all_agents_to_boxes.get(agent_key2));

                }
            }
            agent_boxes_to_avoid.put(agent_key1, boxes_indexes_to_avoid2);
        }

        setUpBoxesCoordToAvoid();
    }

    private static void setUpBoxesCoordToAvoid() {
        index_positions_to_agent_boxes_to_avoid = new int[agent_boxes_to_avoid.size()];
        list_boxes_coord_to_avoid = new ArrayList<>();

        int i = 0;
        for (Integer index_to_expand : agent_boxes_to_avoid.keySet()) {
            index_positions_to_agent_boxes_to_avoid[i++] = index_to_expand;
            Set<Integer> boxes_indexes_to_avoid = agent_boxes_to_avoid.get(index_to_expand);
            HashMap<Integer,int[]> boxes_coord_to_avoid = new HashMap<>(boxes_indexes_to_avoid.size());

            for(Integer key : boxes_indexes_to_avoid){
                int[] __coord = Coordinates.getCoordinatesAt(key, start_coordinates);
                //boxes_to_avoid.add(__coord);
                boxes_coord_to_avoid.put(key,__coord);
            }
            list_boxes_coord_to_avoid.add(boxes_coord_to_avoid);
         }
    }

    private static HashMap<Integer,int[]> getBoxesCoordToAvoid(int index_to_expand) {
        HashMap<Integer,int[]> boxes_coord_to_avoid = new HashMap<>();
        for (int i = 0; i < index_positions_to_agent_boxes_to_avoid.length; i++) {
            if (index_positions_to_agent_boxes_to_avoid[i] == index_to_expand){
                boxes_coord_to_avoid = list_boxes_coord_to_avoid.get(i);
            }
        }
        return boxes_coord_to_avoid;
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
        //third index : one for g_cost   and second one for f_cost
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
        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
            int time_state = Coordinates.getTime(coordinate, cell_coordinates);
            y = Coordinates.getRow(coordinate, cell_coordinates);
            x = Coordinates.getCol(coordinate, cell_coordinates);
            y_goal = Coordinates.getRow(coordinate, goals_coordinates);
            x_goal = Coordinates.getCol(coordinate, goals_coordinates);

            heuristc_value += conflict_avoidance_checking_rules.getHeuristicOf(group_marks_ids[coordinate], time_state, y, x , y_goal, x_goal);
        }
        return heuristc_value;
    }

    public static int getHeuristcsMovablesOf(int[] cell_coordinates , int[] heuristic_coordinates_output){
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;
        int i = 0;

         for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
            int time_state = Coordinates.getTime(coordinate, cell_coordinates);
            y = Coordinates.getRow(coordinate, cell_coordinates);
            x = Coordinates.getCol(coordinate, cell_coordinates);
            y_goal = Coordinates.getRow(coordinate, goals_coordinates);
            x_goal = Coordinates.getCol(coordinate, goals_coordinates);

            heuristic_coordinates_output[i] = conflict_avoidance_checking_rules.getHeuristicOf(group_marks_ids[coordinate], time_state, y, x , y_goal, x_goal);
            heuristc_value += heuristic_coordinates_output[i];
            i++;
        }
        return heuristc_value;
    }

    public static void initStandardNodeCosts() {
        heuristic_standard_coordinates_output = new int[StateSearchMAFactory.number_of_movables];

        final int G_COST = 0;
        final int H_COST = 1;
        final int F_COST = 2;
        final int H_COSTS = 3;

        standard_node_costs = new int[4][];
        standard_node_costs[G_COST] = new int[1];
        standard_node_costs[H_COST] = new int[1];
        standard_node_costs[F_COST] = new int[1];
        standard_node_costs[H_COSTS] = heuristic_standard_coordinates_output;
    }

    public static void initIntermediateNodeCosts() {
        heuristic_intermediate_coordinates_output = new int[StateSearchMAFactory.number_of_movables];

        final int G_COST = 0;
        final int H_COST = 1;
        final int F_COST = 2;
        final int H_COSTS = 3;

        intermediate_node_costs = new int[4][];
        intermediate_node_costs[G_COST] = new int[1];
        intermediate_node_costs[H_COST] = new int[1];
        intermediate_node_costs[F_COST] = new int[1];
        intermediate_node_costs[H_COSTS] = heuristic_intermediate_coordinates_output;
    }

    public static int[][] updateStandardNodeCosts(int[][] state) {
        final int G_COST = 0;
        final int H_COST = 1;
        final int F_COST = 2;
        final int H_COSTS = 3;

        //int f_cost = SearchMAState.getFCost(current_state) - COST_NEXT_CELL;
        //int h_cost = StateSearchMAFactory.getHeuristcOf(SearchMAState.getStateCoordinates(current_state));
        int g_cost = SearchMAState.getGCost(state) + COST_NEXT_CELL;
        int h_cost = getHeuristcsMovablesOf(SearchMAState.getStateCoordinates(state), heuristic_standard_coordinates_output);
        int f_cost = h_cost + g_cost;

        standard_node_costs[G_COST][0] = g_cost;
        standard_node_costs[H_COST][0] = h_cost;
        standard_node_costs[F_COST][0] = f_cost;
        //standard_node_costs[H_COSTS] = heuristic_standard_coordinates_output;

        return standard_node_costs;
    }


    public static int[][] updateIntermediateNodeCosts(int[][] state) {
        final int G_COST = 0;
        final int H_COST = 1;
        final int F_COST = 2;
        final int H_COSTS = 3;

        int g_cost = SearchMAState.getGCost(state) + COST_NEXT_CELL;
        int h_cost = getHeuristcsMovablesOf(SearchMAState.getStateCoordinates(state), heuristic_intermediate_coordinates_output);
        int f_cost = h_cost + g_cost;

        intermediate_node_costs[G_COST][0] = g_cost;
        intermediate_node_costs[H_COST][0] = h_cost;
        intermediate_node_costs[F_COST][0] = f_cost;
        //intermediate_node_costs[H_COSTS] = heuristic_intermediate_coordinates_output;

        return intermediate_node_costs;
    }


    private static int getStateHeuristcManhatten(int[] cell_coordinates) {
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;
        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
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
        //assert ( ( cell_coordinates.length/Coordinates.getLenght() ) == number_of_movables);

        //int heuristc_value = getStateHeuristcManhatten(cell_coordinates);
        int heuristc_value = getHeuristcOf(cell_coordinates);
        state  = SearchMAState.createNew(cell_coordinates, total_gcost, heuristc_value + total_gcost);

        return state;
    }

    //is used many times to create a state for the a_star
    public static int[][] createIntermediatePositiom(int[] cell_coordinates, int total_gcost){
        //assert (cell_coordinates.length/Coordinates.getLenght() == number_of_movables);

        //int heuristc_value = getStateHeuristcManhatten(cell_coordinates);
        int heuristc_value = getHeuristcOf(cell_coordinates);
        state  = SearchMAState.createNew(cell_coordinates,total_gcost, heuristc_value + total_gcost);

        return state;
    }

    public static int[][] createStandardState(int[] cell_neighbour, int neighbour_gcost, int f_value) {
        //assert (cell_neighbour.length == number_of_movables);
        state  = SearchMAState.createNew(cell_neighbour, neighbour_gcost, f_value);

        return state;
    }

    public static int[][] createIntermediatePositiom(int[] cell_neighbour, int neighbour_gcost, int f_value) {
        //assert (cell_neighbour.length == number_of_movables);
        state  = SearchMAState.createNew(cell_neighbour, neighbour_gcost, f_value);

        return state;
    }

    public static int getGCost(int[][] state) {
        return SearchMAState.getGCost(state);
    }

    public static void updateCameFromPrevCell2(HashMap<int[],int[]> came_from, int[][] state, int[][] previouse_coordinates) {
        came_from.put(getCellCoordinates(state), getCellCoordinates(previouse_coordinates));
    }

    public static void updateCameFromPrevCell(HashMap<Integer,int[]> came_from, int[][] state, int[][] previouse_coordinates) {
        came_from.put( Arrays.hashCode(getCellCoordinates(state)) , getCellCoordinates(previouse_coordinates));
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
        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
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
        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
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
        int[] state_to_close = new int[number_of_movables];
        int __index = 0;
        int g_cost_acumulated = 0;
        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
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
        int[] state_to_check = new int[number_of_movables];
        int __index = 0;
        int index_counter;
        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
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

        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
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
            for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
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


     //if neighbours contains one of the other mark_ids from standard node :
    //add it to the set of SimulationConflicts
    //edge conflict only for overstepped movable object
    //vertex conflicts for all others movable objects
    public static void setConflictsStandardStateExpansion(int index_to_expand, int[] pos_coordinates, int[] cell_pos_neighbour, ArrayList<SimulationConflict> standard_to_conflicts){
        int row_neigbour = Coordinates.getRow(cell_pos_neighbour);
        int col_neigbour = Coordinates.getCol(cell_pos_neighbour);
        int time_neigbour = Coordinates.getTime(cell_pos_neighbour);


        boolean found_e = false;
        boolean found_v = false;
        for (int i = 0; i < number_of_movables; i++) {
            if(i == index_to_expand) continue;

            int row_coord = Coordinates.getRow(i, pos_coordinates);
            int col_coord = Coordinates.getCol(i, pos_coordinates);
            int time_coord = Coordinates.getTime(i, pos_coordinates);

            //do not impose constraints on the previouse expanded cell_positions
            if(time_coord == time_neigbour) continue;

            if ( row_coord == row_neigbour && col_coord == col_neigbour) {
                int[] cell_pos_neighbour1;
                int[] position_to_expand1;
                if(standard_to_conflicts.size() > 0){
                for (SimulationConflict simulationConflict : standard_to_conflicts) {
                    if (simulationConflict instanceof EdgeConflict) {
                        int mark_id_conflicted = simulationConflict.getMarkedId();
                        if (mark_id_conflicted == group_marks_ids[i]) {
                            found_e = true;
                            cell_pos_neighbour1 = Coordinates.createCoordinates(time_coord, Coordinates.getRow(cell_pos_neighbour), Coordinates.getCol(cell_pos_neighbour));
                            int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);
                            Coordinates.setTime(position_to_expand, time_coord +1);
                            //position_to_expand1 = Coordinates.createCoordinates( time_coord +1, Coordinates.getRow(position_to_expand), Coordinates.getCol(position_to_expand));
                            ((EdgeConflict) simulationConflict).addConflictedEdge(group_marks_ids[index_to_expand], cell_pos_neighbour1, position_to_expand);
                        }
                    }
                }
            }
                if (!found_e) {
                    EdgeConflict edge_conflict_found = new EdgeConflict(group_marks_ids[i]);
                    cell_pos_neighbour1 = Coordinates.createCoordinates(time_coord, Coordinates.getRow(cell_pos_neighbour), Coordinates.getCol(cell_pos_neighbour));
                    int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);
                    Coordinates.setTime(position_to_expand, time_coord +1);
                    //position_to_expand1 = Coordinates.createCoordinates( time_coord +1, Coordinates.getRow(position_to_expand), Coordinates.getCol(position_to_expand));
                    ((EdgeConflict) edge_conflict_found).addConflictedEdge(group_marks_ids[index_to_expand], cell_pos_neighbour1, position_to_expand);//the time steps should be reversed

                    standard_to_conflicts.add(edge_conflict_found);
                }
            found_e = false;
            }

            if(standard_to_conflicts.size() > 0){
                for (SimulationConflict simulationConflict : standard_to_conflicts){
                    if ( simulationConflict instanceof VertexConflict ){
                        int mark_id_conflicted = simulationConflict.getMarkedId();
                        if(mark_id_conflicted == group_marks_ids[i]){
                            found_v = true;
                            ((VertexConflict)simulationConflict).addConflictedCell(group_marks_ids[index_to_expand], cell_pos_neighbour);////the time steps should be increased
                        }
                    }
                }
            }if (!found_v) {
                VertexConflict vertex_conflict_found = new VertexConflict(group_marks_ids[i]);
                vertex_conflict_found.addConflictedCell(group_marks_ids[index_to_expand], cell_pos_neighbour);////the time steps should be increased
                standard_to_conflicts.add(vertex_conflict_found);
            }
            found_v = false;
        }
    }


    //entry point of the operator decomposition algorithm
    public static ArrayDeque<int[][]> expandStandardState(int[][] state, ArrayList<SimulationConflict> standard_to_conflicts) {
        //standard_node_costs = updateStandardNodeCosts(state);
         updateStandardNodeCosts(state);
        int[] pos_coordinates = SearchMAState.getStateCoordinates(state);
        //int[] pos_coordinates = Arrays.copyOf(SearchMAState.getStateCoordinates(state),SearchMAState.getStateCoordinates(state).length);
        ArrayDeque<int[][]> __result = new ArrayDeque<>();

        switch (searchMultiAgentState)
        {
            case AGENTS_ONLY: __result.addAll(expandStandardStateWithAgents(pos_coordinates, standard_node_costs, standard_to_conflicts));
                                break;
            case AGENTS_AND_BOXES: ArrayDeque<int[][]> __result__ = expandStandardStateWithAgentsAndBoxes(pos_coordinates, standard_node_costs, standard_to_conflicts);
                                    __result.addAll(__result__);
                                    break;
        }
        return __result;
    }

    //entry point of the operator decomposition algorithm
    public static ArrayDeque<int[][]> expandIntermediateState(int[][] state, ArrayList<SimulationConflict> standard_to_conflicts) {
        updateIntermediateNodeCosts(state);
        int[] pos_coordinates = SearchMAState.getStateCoordinates(state);
        //int[] pos_coordinates = Arrays.copyOf(SearchMAState.getStateCoordinates(state),SearchMAState.getStateCoordinates(state).length);
        ArrayDeque<int[][]> __result = new ArrayDeque<>();

        switch (searchMultiAgentState)
        {
            case AGENTS_ONLY: __result.addAll(expandIntermediateStateWithAgents(pos_coordinates, intermediate_node_costs, standard_to_conflicts));
                                break;
            case AGENTS_AND_BOXES:  ArrayDeque<int[][]> __result__ = expandIntermediateStateWithAgentsAndBoxes(pos_coordinates, intermediate_node_costs, standard_to_conflicts);
                                    __result.addAll(__result__);
                                    break;
        }
        return __result;
    }

    public static ArrayDeque<int[][]> expandStandardStateWithAgents(int[] pos_coordinates, int[][] standard_node_costs, ArrayList<SimulationConflict> standard_to_conflicts) {
        final int G_COST = 0;
        final int H_COST = 1;
        final int F_COST = 2;
        final int H_COSTS = 3;


        Random random = new Random();
        int index_to_expand = random.nextInt(number_of_movables);//or other heuristic to use instead of random
        /*
        int minimum_index_of_h = Utils.minIndexOf(standard_node_costs[H_COSTS]);
        //int maximum_index_of_h = Utils.maxIndexOf(standard_node_costs[H_COSTS]);
        int index_to_expand = minimum_index_of_h;
        */
        //int prev_time = Coordinates.getTime(index_to_expand, pos_coordinates);
        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

        int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);

        int mark_id = group_marks_ids[index_to_expand];
        LinkedList<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand, new ArrayDeque<int[]>());

        for(int [] cell_pos_neighbour : neighbours){
            setConflictsStandardStateExpansion(index_to_expand, pos_coordinates, cell_pos_neighbour, standard_to_conflicts);

            int[] next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
            Coordinates.setCoordinateAtIndex(index_to_expand, next_state_node, cell_pos_neighbour);
            next_state_nodes.add(SearchMAState.createNew(next_state_node, standard_node_costs[G_COST][0], standard_node_costs[F_COST][0]));
         }

        return next_state_nodes;
    }


    public static ArrayDeque<int [][]> expandIntermediateStateWithAgents(int[] pos_coordinates, int[][] intermediate_node_costs, ArrayList<SimulationConflict> standard_to_conflicts) {
        final int G_COST = 0;
        final int H_COST = 1;
        final int F_COST = 2;
        final int H_COSTS = 3;

        int min_time = Integer.MAX_VALUE;
        ArrayList<Integer> coord_candidates = new ArrayList<>();
        //int number__of__movables = pos_coordinates.length / Coordinates.getLenght();

        int __time ;
        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
            __time = Coordinates.getTime(coordinate, pos_coordinates);
            if(__time < min_time){
                min_time = __time;
            }
        }

        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
            __time = Coordinates.getTime(coordinate, pos_coordinates);
            if(__time == min_time){
                coord_candidates.add(coordinate);
            }
        }

        Random random = new Random();
        int coord_to_expand = -1;
        if (coord_candidates.size() > 0){
            int __index = random.nextInt(coord_candidates.size());//or other heuristic to use instead of random
            coord_to_expand = coord_candidates.get(__index);
        }

        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

        if(coord_to_expand > -1) {
            ArrayDeque<int []> conflicts_avoidance = new ArrayDeque<>(); //transform it to verstex conflixts and edge conflicts

            int[] to_expand = Coordinates.getCoordinatesAt(coord_to_expand, pos_coordinates);
            int mark_id = group_marks_ids[coord_to_expand];

            for ( SimulationConflict simulationConflict  : standard_to_conflicts ){
                if(simulationConflict.getMarkedId() == mark_id ){
                    ArrayList<int[]> coord = simulationConflict.getCoordinatesToAvoid();
                    conflicts_avoidance.addAll(coord);
                }
            }

        LinkedList<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, to_expand, conflicts_avoidance);

        for(int[] cell_pos_neighbour : neighbours){
            //it imposes the same constraints but not on the already expanded positions
            setConflictsStandardStateExpansion(coord_to_expand, pos_coordinates, cell_pos_neighbour, standard_to_conflicts);
        }
            int[] next_state_node;
            for(int [] cell_pos : neighbours){
                next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                Coordinates.setCoordinateAtIndex(coord_to_expand, next_state_node, cell_pos);
                next_state_nodes.add(SearchMAState.createNew(next_state_node, intermediate_node_costs[G_COST][0], intermediate_node_costs[F_COST][0]));
            }
        }
        return next_state_nodes;
    }


    public HashMap<Integer, int[]> getBoxesToAvoid(int index_to_expand, int[] position_to_expand, int[] pos_coordinates){
        Set<Integer> boxes_indexes_to_avoid = agent_boxes_to_avoid.get(index_to_expand);
        //ArrayDeque<int[]> boxes_to_avoid = new ArrayDeque<int[]>();//prune the positions where the box of different colour is
        HashMap<Integer,int[]> boxes_coord_to_avoid = new HashMap<>();
        for(Integer key : boxes_indexes_to_avoid){
            int[] __coord = Coordinates.getCoordinatesAt(key, pos_coordinates);
            //boxes_to_avoid.add(__coord);
            boxes_coord_to_avoid.put(key,__coord);
        }

        return boxes_coord_to_avoid ;
    }

    private static void addPullConstraint(Integer box_index, int[] pull_move_cell, ArrayList<SimulationConflict> standard_to_conflicts ){
        //PullConstraint gets the cell set up expanded when the box index gets expanded in another node expansion
        //if a box has multiple PullConstraints then the node expansion expands all the PullConstraints making a new node for each of them
        boolean constraint_found = false;
        for (SimulationConflict simulationConflict : standard_to_conflicts){
            if(simulationConflict.getMarkedId() == group_marks_ids[box_index] && simulationConflict instanceof PullConstraint){
                ((PullConstraint) simulationConflict).addNextMoveCell(pull_move_cell);
                constraint_found = true;
            }
        }
        if(!constraint_found){
            PullConstraint pullConstraint = new PullConstraint(box_index, group_marks_ids[box_index]);
            pullConstraint.addNextMoveCell(pull_move_cell);
            standard_to_conflicts.add(pullConstraint);
        }
    }


    //TO USE
    public static ArrayDeque<int[][]> expandStandardStateWithAgentsAndBoxes(int[] pos_coordinates, int[][] standard_node_costs, ArrayList<SimulationConflict> standard_to_conflicts) {
        //start with  expanding only agents
       Random random = new Random();
        int index_to_expand = random.nextInt(all_agents_indexes.size());//or other heuristic to use instead of random

        //heuristic: choose the agent to move that is next to the closest box to the goal
        //heuristic: choose the agent to move that is closest to the closest box to the goal
       /* final int H_COSTS = 3;
        int minimum_index_of_h = Utils.minIndexOf(standard_node_costs[H_COSTS]);
        //int maximum_index_of_h = Utils.maxIndexOf(standard_node_costs[H_COSTS]);
        int index_to_expand = minimum_index_of_h;
        */

        return expandStateStartingWithAgents(index_to_expand, pos_coordinates, standard_node_costs, standard_to_conflicts);
    }

    //TO USE
    public static ArrayDeque<int [][]> expandIntermediateStateWithAgentsAndBoxes(int[] pos_coordinates, int[][] intermediate_node_costs, ArrayList<SimulationConflict> standard_to_conflicts) {
        final int G_COST = 0;
        final int H_COST = 1;
        final int F_COST = 2;
        final int H_COSTS = 3;

        int min_time = Integer.MAX_VALUE;

        int __time ;
        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
            __time = Coordinates.getTime(coordinate, pos_coordinates);
            if(__time < min_time){
                min_time = __time;
            }
        }

        Set<Integer> coord_candidates1 = new HashSet<>();
        Set<Integer> waiting_candidates1 = new HashSet<>();
        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
            __time = Coordinates.getTime(coordinate, pos_coordinates);
            if(__time == min_time){
                if(all_boxes_indexes.containsKey(coordinate)) {
                    //if coordinate time does not corespond to a box with constraint : PullConstraint, VertexConflict, EdgeConflict then do not add candidates for exapansion
                    int mark_id = group_marks_ids[coordinate];
                    for (SimulationConflict simulationConflict : standard_to_conflicts ){
                        if( (simulationConflict instanceof PullConstraint || simulationConflict instanceof EdgeConflict) && simulationConflict.getMarkedId() == mark_id ){
                            coord_candidates1.add(coordinate);
                        }else {
                            waiting_candidates1.add(coordinate);
                        }
                    }
                }else if(all_agents_indexes.containsKey(coordinate)){
                    coord_candidates1.add(coordinate);
                }
            }
        }

        ArrayList<Integer> waiting_candidates = new ArrayList<>(waiting_candidates1);
        //ArrayList<Integer> coord_candidates = new ArrayList<>(coord_candidates1);
        Integer[] coord_candidates = coord_candidates1.toArray(new Integer[0]);

        Random random = new Random();

        int coord_to_expand = -1;
        if (coord_candidates.length > 0){
            //minimum of heuristics by indexes from  coord_candidates
            //int minimum_index_of_h = Utils.minIndexOf(intermediate_node_costs[H_COSTS],coord_candidates);
            //coord_to_expand = minimum_index_of_h;

            int __index = random.nextInt(coord_candidates.length);//or other heuristic to use instead of random
            coord_to_expand = coord_candidates[__index];

        }

        return  expandStateStartingWithBoxes(coord_to_expand, pos_coordinates, waiting_candidates, intermediate_node_costs, standard_to_conflicts);
    }

    /*
    * 2 kind of constraints : one for boxes one for agents
      agent moves from next cell of box than : constraint pull on the box
    * */
    /*if there is no box constrained to expand: expand only agents
     *if no more agents to expand left expand boxes and do not move them just increase time step
     * */
    private static ArrayDeque<int[][]> expandStateStartingWithAgents(int index_to_expand, int[] pos_coordinates, int[][] standard_node_costs, ArrayList<SimulationConflict> standard_to_conflicts) {
        final int G_COST = 0;
        final int H_COST = 1;
        final int F_COST = 2;
        final int H_COSTS = 3;

        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();
        int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);
        int mark_id = group_marks_ids[index_to_expand];

        HashMap<Integer, int[]> boxes_coord_to_avoid = getBoxesCoordToAvoid(index_to_expand);

        for (Integer key: boxes_coord_to_avoid.keySet()){
             Coordinates.setTime(boxes_coord_to_avoid.get(key), Coordinates.getTime(key ,pos_coordinates));
        }

        ArrayDeque<int []> conflicts_avoidance = new ArrayDeque<>(); //transform it to verstex conflixts and edge conflicts
        for ( SimulationConflict simulationConflict : standard_to_conflicts ){
            if(simulationConflict.getMarkedId() == mark_id ){
                ArrayList<int[]> coord = simulationConflict.getCoordinatesToAvoid();
                conflicts_avoidance.addAll(coord);
            }
        }

        conflicts_avoidance.addAll(boxes_coord_to_avoid.values());

        LinkedList<int[]> neighbours_agent = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand, conflicts_avoidance);
        conflicts_avoidance.clear();

        ArrayDeque<int[]> neighbours_to_remove = new ArrayDeque<>();

        //check if the boxes to avoid were overstepped in the new step time
        for(int[] neighbour_found : neighbours_agent){
            Set<Integer> keys = boxes_coord_to_avoid.keySet();
            for(Integer key_avoid : boxes_coord_to_avoid.keySet()){
                int[] box_to_avoid = boxes_coord_to_avoid.get(key_avoid);
                if( Coordinates.getRow(neighbour_found) == Coordinates.getRow(box_to_avoid) && Coordinates.getCol(neighbour_found) == Coordinates.getCol(box_to_avoid) ){
                    //check for PULL constraints
                    if(all_boxes_to_agents.containsKey(key_avoid)){
                        for (Integer key_agent : all_boxes_to_agents.get(key_avoid)){
                            //make  PULL constraints
                            int agent_time = Coordinates.getTime(key_agent, pos_coordinates);
                            int box_time_step = Coordinates.getTime(box_to_avoid);
                            //if the agent is next to box : add  PULL as state neighbours
                            //check if the agent has time steps left to pull the box and if yes add PULL constraint on box
                            int agent_row = Coordinates.getRow(key_agent, pos_coordinates);
                            int agent_col = Coordinates.getCol(key_agent, pos_coordinates);
                            if(agent_time < Coordinates.getTime(neighbour_found) && Coordinates.areNeighbours(box_to_avoid, agent_row, agent_col)) {
                                //make a pull constraint for the neighbour box
                                int[] pull_move_cell = new int[]{agent_time + 1, agent_row, agent_col };
                                addPullConstraint(key_avoid, pull_move_cell, standard_to_conflicts);
                                //the PullConstraint imposes VertexConflicts for the other movables
                                setConflictsStandardStateExpansion(key_avoid, pos_coordinates, pull_move_cell, standard_to_conflicts);
                            }else{
                                neighbours_to_remove.add(neighbour_found);
                            }
                        }
                    }
                }
            }
        }

        if(neighbours_to_remove.size() > 0){
            while(!neighbours_to_remove.isEmpty())
                neighbours_agent.remove(neighbours_to_remove.pop());
        }

        if(neighbours_agent.size() > 0){//if the size bigger then zero the agent moved
            Set<Integer> boxes_indexes = all_agents_to_boxes.get(index_to_expand);//get the box with the same color as the agent
            if(boxes_indexes == null)  boxes_indexes = new HashSet<>();

            for (Integer box_index : boxes_indexes ) {
                int box_time_step = Coordinates.getTime(box_index, pos_coordinates);
                int box_row = Coordinates.getRow(box_index, pos_coordinates);
                int box_column = Coordinates.getCol(box_index, pos_coordinates);

                //if the agent is next to box : add  PULL as state neighbours
                if(Coordinates.getTime(position_to_expand) == box_time_step && Coordinates.areNeighbours(position_to_expand, box_row, box_column) ) {
                    //make a pull constraint for the neighbour box
                    int[] pull_move_cell = new int[]{Coordinates.getTime(position_to_expand) + 1, Coordinates.getRow(position_to_expand), Coordinates.getCol(position_to_expand) };
                    addPullConstraint(box_index, pull_move_cell, standard_to_conflicts);
                    //the PullConstraint imposes VertexConflicts for the other movables
                    setConflictsStandardStateExpansion(box_index, pos_coordinates, pull_move_cell, standard_to_conflicts);
                }
            }
            for(int [] cell_pos_neighbour : neighbours_agent){
                setConflictsStandardStateExpansion(index_to_expand, pos_coordinates, cell_pos_neighbour, standard_to_conflicts);

                int[] next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                Coordinates.setCoordinateAtIndex(index_to_expand, next_state_node, cell_pos_neighbour);
                next_state_nodes.add(SearchMAState.createNew(next_state_node, standard_node_costs[G_COST][0], standard_node_costs[F_COST][0]));
            }
        }
        return next_state_nodes;
    }


    /*if there is no box constrained to expand: expand only agents
                    if no more agents to expand left expand boxes and do not move them just increase time step
    * */
    public static ArrayDeque<int [][]> expandStateStartingWithBoxes(int coord_to_expand, int[] pos_coordinates, ArrayList<Integer> waiting_candidates, int[][] intermediate_node_costs, ArrayList<SimulationConflict> standard_to_conflicts) {

        if(coord_to_expand > -1 && all_agents_indexes.containsKey(coord_to_expand)) {
            return expandStateStartingWithAgents(coord_to_expand, pos_coordinates, intermediate_node_costs, standard_to_conflicts);
        }

        //expand only for boxes
        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

        if(coord_to_expand > -1 && all_boxes_indexes.containsKey(coord_to_expand)) {
            ArrayDeque<int []> conflicts_avoidance = new ArrayDeque<>();
            int[] to_expand = Coordinates.getCoordinatesAt(coord_to_expand, pos_coordinates);
            int mark_id = group_marks_ids[coord_to_expand];

            ArrayDeque<int[]> constraint_moving = new ArrayDeque<>();
            for ( SimulationConflict simulationConflict  : standard_to_conflicts ){
                if(simulationConflict.getMarkedId() == mark_id ){
                    if (simulationConflict instanceof PullConstraint){
                        constraint_moving.addAll(((PullConstraint) simulationConflict).getConstraint_cells() );
                    }else{
                        ArrayList<int[]> coord = simulationConflict.getCoordinatesToAvoid();
                        conflicts_avoidance.addAll(coord);
                    }
                }
            }

            LinkedList<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, to_expand, conflicts_avoidance);
            conflicts_avoidance.clear();

            while (!constraint_moving.isEmpty())
                neighbours.add(constraint_moving.pop());


            for(int[]  cell_pos_neighbour : neighbours){
                //it imposes the same constraints but not on the already expanded positions
                setConflictsStandardStateExpansion(coord_to_expand, pos_coordinates, cell_pos_neighbour, standard_to_conflicts);
            }

            if(neighbours.size()==0){
                //box blocked can not move
                Coordinates.setTime(to_expand,Coordinates.getTime(to_expand) + 1 );
                neighbours.add(to_expand);
            }

            int[] next_state_node;
            for(int [] cell_pos : neighbours){
                next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                Coordinates.setCoordinateAtIndex(coord_to_expand, next_state_node, cell_pos);
                next_state_nodes.add(SearchMAState.createNew(next_state_node, intermediate_node_costs[G_COST][0], intermediate_node_costs[F_COST][0]));
            }
        }

        //waiting constraint states aplies for the rest of  boxes
        if (next_state_nodes.size() == 0){
            if (waiting_candidates.size() > 0){
                int[]  next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                for (Integer coord : waiting_candidates ){
                    Coordinates.setTime(coord, next_state_node, Coordinates.getTime(coord, pos_coordinates) + 1);
               }
                next_state_nodes.add(SearchMAState.createNew(next_state_node, intermediate_node_costs[G_COST][0], intermediate_node_costs[F_COST][0]));
            }
        }
        return next_state_nodes;
    }










    //expands all the pushes , pulls with box and the agent moves at the same time
    //it looks like it does make pull and push at the same time
    private static ArrayDeque<int[][]> expandStandardStateForAgentsAndBoxesMultiple(int[] pos_coordinates, int g_cost, int f_cost, ArrayList<SimulationConflict> standard_to_conflicts) {
        // if agent gets pulls or pushes expand two states: agent plus box
        //otherwise if it only moves expand only the agent
        //otherwise the box gets never expanded alone
        Random random = new Random();
        int number_of_coordinates = pos_coordinates.length / Coordinates.getLenght();

        ///to cache
        HashMap<Integer,Integer> all_agents_indexes = new HashMap<>();//first is for position index , second is for color
        HashMap<Integer,Integer> all_boxes_indexes = new HashMap<>();//first is for position index , second is for color

        for (int movable_index = 0; movable_index < number_of_coordinates; movable_index++) {
            int mark_id = group_marks_ids[movable_index];
            Serializable movable_to_exapand = MapFixedObjects.getByMarkNo(mark_id);
            if (movable_to_exapand instanceof Agent){
                Agent agent = (Agent) movable_to_exapand;
                int agent_color = agent.getColor();
                all_agents_indexes.put(movable_index, agent_color);
            }else if (movable_to_exapand instanceof Box){
                Box box = (Box) movable_to_exapand;
                int box_color = box.getColor();
                all_boxes_indexes.put(movable_index, box_color);
            }
            else {
                throw new UnsupportedOperationException("method :setConflictsStandardStateExpansionForAgentsAndBoxes");
            }
        }

        HashMap<Integer, Set<Integer>> all_agents_to_boxes = new HashMap<>();//first is for position index of agent, second is for position index of box

        for (Integer agent_key: all_agents_indexes.keySet()){
            for (Integer box_key: all_boxes_indexes.keySet()){
                if(all_agents_indexes.get(agent_key).equals(all_boxes_indexes.get(box_key))){
                    if (all_agents_to_boxes.containsKey(agent_key)){
                        all_agents_to_boxes.get(agent_key).add(box_key);
                    }else{
                        Set<Integer> boxes_indexes = new HashSet<>();
                        boxes_indexes.add(box_key);
                        all_agents_to_boxes.put(agent_key,boxes_indexes);
                    }
                }
            }
        }

        ////end cache

        //start with  expanding only agents
        int index_to_expand = random.nextInt(all_agents_indexes.size());//or other heuristic to use instead of random
        //heuristic: choose the agent to move that is next to the closest box to the goal
        //heuristic: choose the agent to move that is closest to the closest box to the goal

        int prev_time = Coordinates.getTime(index_to_expand, pos_coordinates);
        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();
        int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);
        int mark_id = group_marks_ids[index_to_expand];

        //avoid boxes not of the agent color
        Set<Integer> boxes_indexes_to_avoid = new HashSet<>();
        for (Integer agent_key: all_agents_to_boxes.keySet()){
            if(index_to_expand != agent_key)
                boxes_indexes_to_avoid.addAll(all_agents_to_boxes.get(agent_key));
        }

        //avoid all boxes and make the agent expand a push and pull at the same time
        boxes_indexes_to_avoid = new HashSet<>();
        for (Integer agent_key: all_agents_to_boxes.keySet()){
            boxes_indexes_to_avoid.addAll(all_agents_to_boxes.get(agent_key));
        }

        ArrayDeque<int[]> boxes_to_avoid = new ArrayDeque<int[]>();//prune the positions where the box of different colour is
        for(Integer key : boxes_indexes_to_avoid)
            boxes_to_avoid.add(Coordinates.getCoordinatesAt(key, pos_coordinates));

        LinkedList<int[]> neighbours_agent = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand, boxes_to_avoid);

        int __time_pos = Coordinates.getTime(position_to_expand);
        int __row = Coordinates.getRow(position_to_expand);
        int __column = Coordinates.getCol(position_to_expand);
        ArrayList<int[][]> neighbours_ops = new ArrayList<>();//neighbours for when the agent moves a box of his colour

        //this will hold the neighbours for states expanded for both movables
        //LinkedList<int[]> neighbours_agent_with_boxes = new LinkedList<>();
        if(neighbours_agent.size() > 0){//if the size bigger then zero the agent moved
            Set<Integer> boxes_indexes = all_agents_to_boxes.get(index_to_expand);
            for (Integer box_index : boxes_indexes ) {
                int box_row = Coordinates.getRow(box_index, pos_coordinates);
                int box_column = Coordinates.getCol(box_index, pos_coordinates);

                //if the agent is next to box : add PUSH and PULL as state neighbours
                if(Coordinates.areNeighbours(position_to_expand, box_row, box_column) ){
                    //make PUSH
                    int _mark_id = group_marks_ids[box_index];
                    int[] position_box_to_expand = Coordinates.getCoordinatesAt(box_index, pos_coordinates);

                    ArrayDeque<int[]> box__conflicted = new ArrayDeque<>();
                    box__conflicted.add(position_to_expand);//add the agent location to __conflicts to avoid this position for PUSH movements
                    box__conflicted.addAll(boxes_to_avoid);//add the boxes of different colour to __conflicts in order to avoid those positions
                    LinkedList<int[]> neighbours_box = conflict_avoidance_checking_rules.getFreeNeighboursMA(_mark_id, position_box_to_expand, box__conflicted);

                    for (int[] neighbour : neighbours_box ){
                        int[][] __indexed = new int[3][];
                        __indexed[0] =  Coordinates.createCoordinates(__time_pos + 1, box_row, box_column); //index position for where the agent is
                        __indexed[1] = new int[]{box_index};//the index position from the box moved
                        __indexed[2] = neighbour;//the neighbour position where the box will move
                        neighbours_ops.add(__indexed);
                    }
/*
                if(!(box_row + 1 == __row && box_column == __column  ))
                    neighbours_ops.add(new int[]{__time_pos + 1, box_row + 1, box_column});
                if(!(box_row - 1 == __row && box_column == __column  ))
                    neighbours_ops.add(new int[]{__time_pos + 1, box_row - 1 , box_column});
                if(!(box_row == __row && box_column + 1 == __column  ))
                    neighbours_ops.add( new int[]{__time_pos + 1, box_row, box_column + 1}  );
                if(!(box_row == __row && box_column - 1 == __column  ))
                    neighbours_ops.add(  new int[]{__time_pos + 1, box_row, box_column - 1 }  );
*/

                    //make PULL
                    //get the free cells the agent finds and make PULL actions expansions
                    int[] pull_move_cell = new int[]{__time_pos + 1, Coordinates.getRow(position_to_expand), Coordinates.getCol(position_to_expand) };

                    for (int[] agent  : neighbours_agent){
                        int[][] __indexed = new int[2][];
                        ///new copy???
                        __indexed[0] = agent; //index position for where the agent is
                        __indexed[1] = new int[]{box_index};
                        __indexed[2] = pull_move_cell;
                        neighbours_ops.add(__indexed);
                    }
                }
            }
        }

/*
        neighbour for agent
                neighbour for box moves
        expand ??
                1.neighbour for agent same g_cost as a new state
                 2.
and then from one standard node we get two standard nodes

      when expanding agent avoid all boxes cells and add push for boxes of the same colour

      constraints:
          if agent moves from a neighbour cell to a box of the same colour :impose constraints on that box :push and pulls

            one intetmediate node : moves without boxes
        one intetmediate node : moves with box and both advance to same time step

final solution : choose random what to move : agent or box : if agent moves first it moves without box and imposes a push constraint on the boxes of same color
                                                            if box is chose to move it can only move if an agent of the same color is in the next cell
                      because in this way : we get one standard node and not two like above


final solution expand only agents , do not allow agents to overstep boxes , and then make PUSHES for neigbour boxes of the same colour
                                                                                         make PUULS   for neigbour boxes of the same colour

*/

        for(int[] cell_pos_neighbour : neighbours_agent){
            setConflictsStandardStateExpansion(index_to_expand, pos_coordinates, cell_pos_neighbour, standard_to_conflicts);

            int[] next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
            Coordinates.setCoordinateAtIndex(index_to_expand, next_state_node, cell_pos_neighbour);
            next_state_nodes.add(SearchMAState.createNew(next_state_node, g_cost, f_cost));
        }

        ArrayList<Integer> indexes_to_expand = new ArrayList<>();//only grows if agent and box moves , otherwise is only for agent
        indexes_to_expand.add(index_to_expand);
        ArrayList<int []> cell_pos_neighbours = new ArrayList<>();
        cell_pos_neighbours.add(position_to_expand);

        for(int[][] cell_pos_neigh : neighbours_ops){ //TO DO : to make the agent make a pull and push at the same time
            int[] cell_to_expand_agent = cell_pos_neigh[0];
            int cell_pos_neighbour_index = cell_pos_neigh[1][0];
            indexes_to_expand.add(cell_pos_neighbour_index);
            int[] cell_pos_neighbour = cell_pos_neigh[2];
            cell_pos_neighbours.add(cell_pos_neighbour);

            setConflictsStandardStateExpansionForAgentsAndBoxesMultiple(indexes_to_expand,  pos_coordinates, cell_pos_neighbours, standard_to_conflicts);
            indexes_to_expand.remove(cell_pos_neighbour_index);
            cell_pos_neighbours.remove(cell_pos_neighbour);

            int[] next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
            Coordinates.setCoordinateAtIndex(index_to_expand, next_state_node, cell_to_expand_agent);
            Coordinates.setCoordinateAtIndex(cell_pos_neighbour_index, next_state_node, cell_pos_neighbour);

            next_state_nodes.add(SearchMAState.createNew(next_state_node, g_cost, f_cost));//how to update g_cost and f_cost
        }
        return next_state_nodes;
    }

    //always index_to_expand[0] is agent and index_to_expand[1] is box (if the box position gets expanded alongside)
    //always cell_pos_neighbours[0] is agent next position and cell_pos_neighbours[1] is box next position (if the box position gets expanded alongside)
    private static void setConflictsStandardStateExpansionForAgentsAndBoxesMultiple(ArrayList<Integer> indexes_to_expand, int[] pos_coordinates, ArrayList<int []> cell_pos_neighbours, ArrayList<SimulationConflict> standard_to_conflicts){

        boolean agent_to_expand = false;
        boolean box_to_expand = false;
        if (indexes_to_expand.size() == 2){
            agent_to_expand = true;
            box_to_expand = true;
        }else  if (indexes_to_expand.size()==1){
            agent_to_expand = true;
        }else {
            throw new UnsupportedOperationException("method :setConflictsStandardStateExpansionForAgentsAndBoxes");
        }

        if(agent_to_expand){
            //Serializable movable_for_id_to_exapand = MapFixedObjects.getByMarkNo(agent_mark_id_to_expand);
            //Agent agent = (Agent) movable_for_id_to_exapand;
            //int[] movable_coordinate = agent.getCoordinates();
        }
        if(box_to_expand){
            //Serializable movable_for_id_to_exapand = MapFixedObjects.getByMarkNo(box_mark_id_to_expand);
            //Box box = (Box) movable_for_id_to_exapand;
            //int[] movable_coordinate = box.getCoordinates();
        }

        ArrayList<int[]> positions_to_expand = new ArrayList<>();
        for (Integer index_to_expand  : indexes_to_expand){
            int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);
            positions_to_expand.add(position_to_expand);
        }

        boolean found_e = false;
        boolean found_v = false;

        int number_of_coordinates = pos_coordinates.length / Coordinates.getLenght();

        for (int i = 0; i < number_of_coordinates; i++) {
            if(indexes_to_expand.contains(i)) continue;

            int row_coord = Coordinates.getRow(i, pos_coordinates);
            int col_coord = Coordinates.getCol(i, pos_coordinates);
            int time_coord = Coordinates.getTime(i, pos_coordinates);

            //do not impose constraints on the previouse expanded cell_positions
            if(time_coord == Coordinates.getTime(cell_pos_neighbours.get(0))) continue;

            ArrayList<int[][]> edge_conflicted_neighbours = new ArrayList<>();
            boolean edge_conflict_to_check = false;
            final int POSITION_INDEX_TO_EXPAND = 0;
            final int NEIGHBOUR_CONFLICT_INDEX = 1;
            for (int j = 0; j < cell_pos_neighbours.size(); j++) {
                int[] neighbour = cell_pos_neighbours.get(j);
                if (row_coord == Coordinates.getRow(neighbour) && col_coord == Coordinates.getCol(neighbour)) {
                    edge_conflict_to_check = true;
                    Integer index_to_expand = indexes_to_expand.get(j);

                    //index_to_conflict is pair of the: index from the state to be expanded and the conflict cause by the neighbour expanded from this index
                    int[][] index_to_conflict = new int[2][];
                    index_to_conflict[POSITION_INDEX_TO_EXPAND] = new int[]{index_to_expand};
                    index_to_conflict[NEIGHBOUR_CONFLICT_INDEX] = neighbour;

                    edge_conflicted_neighbours.add(index_to_conflict);
                }
            }

            if ( edge_conflict_to_check) {
                if(standard_to_conflicts.size() > 0){
                    for (SimulationConflict simulationConflict : standard_to_conflicts) {
                        if (simulationConflict instanceof EdgeConflict) {
                            int mark_id_conflicted = simulationConflict.getMarkedId();
                            if (mark_id_conflicted == group_marks_ids[i]) {
                                found_e = true;

                                for (int[][] conflict : edge_conflicted_neighbours ){
                                    int[] cell_pos_neighbour = Coordinates.createCoordinates(time_coord, Coordinates.getRow(conflict[NEIGHBOUR_CONFLICT_INDEX]), Coordinates.getCol(conflict[NEIGHBOUR_CONFLICT_INDEX]));
                                    int index_to_expand = conflict[POSITION_INDEX_TO_EXPAND][0];
                                    int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);
                                    Coordinates.setTime(position_to_expand, time_coord +1);
                                    ((EdgeConflict) simulationConflict).addConflictedEdge(group_marks_ids[index_to_expand], cell_pos_neighbour, position_to_expand);
                                }
                            }
                        }
                    }
                }

                if (!found_e) {
                    EdgeConflict edge_conflict_found = new EdgeConflict(group_marks_ids[i]);
                    for (int[][] conflict : edge_conflicted_neighbours ){
                        int[] cell_pos_neighbour = Coordinates.createCoordinates(time_coord, Coordinates.getRow(conflict[NEIGHBOUR_CONFLICT_INDEX]), Coordinates.getCol(conflict[NEIGHBOUR_CONFLICT_INDEX]));
                        int index_to_expand = conflict[POSITION_INDEX_TO_EXPAND][0];
                        int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);
                        Coordinates.setTime(position_to_expand, time_coord +1);

                        ((EdgeConflict) edge_conflict_found).addConflictedEdge(group_marks_ids[index_to_expand], cell_pos_neighbour, position_to_expand);
                    }

                    standard_to_conflicts.add(edge_conflict_found);
                }
                found_e = false;
            }

            if(standard_to_conflicts.size() > 0){
                for (SimulationConflict simulationConflict : standard_to_conflicts){
                    if ( simulationConflict instanceof VertexConflict ){
                        int mark_id_conflicted = simulationConflict.getMarkedId();
                        if(mark_id_conflicted == group_marks_ids[i]){
                            found_v = true;

                            for (int j = 0; j < cell_pos_neighbours.size(); j++) {
                                int[] cell_pos_neighbour = cell_pos_neighbours.get(j);
                                Integer index_to_expand = indexes_to_expand.get(j);
                                ((VertexConflict)simulationConflict).addConflictedCell(group_marks_ids[index_to_expand], cell_pos_neighbour);
                            }
                        }
                    }
                }
            }if (!found_v) {
                VertexConflict vertex_conflict_found = new VertexConflict(group_marks_ids[i]);

                for (int j = 0; j < cell_pos_neighbours.size(); j++) {
                    int[] cell_pos_neighbour = cell_pos_neighbours.get(j);
                    Integer index_to_expand = indexes_to_expand.get(j);
                    vertex_conflict_found.addConflictedCell(group_marks_ids[index_to_expand], cell_pos_neighbour);
                }
                standard_to_conflicts.add(vertex_conflict_found);
            }

            found_v = false;
        }
    }

}



