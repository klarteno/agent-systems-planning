package org.agents.searchengine;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.IllegalPathsStore;
import org.agents.planning.conflicts.dto.EdgeConflict;
import org.agents.planning.conflicts.dto.SimulationConflict;
import org.agents.planning.conflicts.dto.VertexConflict;
import org.agents.searchengine.conflicts.ODConflicts;

import java.io.Serializable;
import java.util.*;

public final class StateSearchMAFactory {
    private static int [][] start_state;
    private static int [][][][] cost_so_far;

    private static int [][][][] closed_states;
    private static HashMap<Integer,Integer> closed_states_MA;
    private static HashMap<int[],int[]> came_from;

    private static int[][] index_map_cells;

    public static void setNumber_of_movables(int number_of_movables) {
        StateSearchMAFactory.number_of_movables = number_of_movables;
    }

    static int number_of_movables;

    private static int[] start_coordinates;
    private static int[] goals_coordinates;
    private static HashMap<Integer, ArrayList<int[]>> goals_neighbours;

    public static void setGroup_marks_ids(int[] group_marks_ids) {
        StateSearchMAFactory.group_marks_ids = group_marks_ids;
    }

    private static  int[] group_marks_ids;

    private static HashMap<Integer,Integer> all_agents_indexes;
    private static HashMap<Integer,Integer> all_boxes_indexes;
    private static HashMap<Integer, Set<Integer>> all_agents_to_boxes;
    private static HashMap<Integer, Set<Integer>> all_boxes_to_agents;
    private static int[] index_boxes;

    private static HashMap<Integer, Set<Integer>> agent_boxes_to_avoid;// //avoid boxes not of the agent color
    private static int[] index_positions_to_agent_boxes_to_avoid;
    private static ArrayList<HashMap<Integer,int[]>> list_boxes_coord_to_avoid;
    private static ODConflicts od_conflicts;

    private static ODNodeStructure od_node_structure;

    private static ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;

    static SearchState searchMultiAgentState = SearchState.AGENTS_ONLY;

    public static HeuristicMetricsSearch heuristicMetricsSearch;

    public static void createStatesCameFrom() {
        came_from = new HashMap<>();
    }

    public static int[] getCameFrom(int[] stateCoordinates) {
       return came_from.get(stateCoordinates);
    }

    public static boolean removeCameFrom(int[] next_key2, int[] next_key) {
        return came_from.remove(next_key2, next_key);
    }

    public static void setIndexBoxes(int[] indexBoxes) {
        index_boxes = indexBoxes;
    }

    public static ODNodeStructure getODNodeStructure() {
        return od_node_structure;
    }

    public enum SearchState {
        AGENTS_ONLY,
        AGENTS_AND_BOXES,
        AVOIDING_GOALS
    };

    public static void setAvoidanceCheckingRules(ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules) {
        conflict_avoidance_checking_rules = conflictAvoidanceCheckingRules;
    }

    public static void setStartGroup(int[] start_group, int[] startCoordinates, int[] goalsCoordinates, HashMap<Integer, ArrayList<int[]>> goalsNeighbours) {
        group_marks_ids = start_group;
        start_coordinates = startCoordinates;
        goals_coordinates = goalsCoordinates;
        goals_neighbours = goalsNeighbours;

        number_of_movables = start_coordinates.length/Coordinates.getLenght();

        setUpAgentsWithBoxesFromGroup();

        heuristicMetricsSearch = new HeuristicMetricsSearch(goals_coordinates);
        heuristicMetricsSearch.initStandardNodeCosts();
        heuristicMetricsSearch.initIntermediateNodeCosts();

        od_node_structure = new ODNodeStructure();
        od_conflicts = new ODConflicts(group_marks_ids, od_node_structure);
    }

    public static int[] getStartGroup() {
        return group_marks_ids;
    }

    public static ODConflicts getConflictsManager() {
        return od_conflicts;
    }
    //groups the agents and boxes by color
    public static void setUpAgentsWithBoxesFromGroup(){
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

    public static int[][] createDummyState() {
        return SearchMAState.createDummyState(number_of_movables);
    }

    public static int[][] getDummyState() {
        return SearchMAState.getDummyState(number_of_movables);
    }

    public static int[][] createStartState(int[] cell_coordinates, int total_gcost){
        //assert ( ( cell_coordinates.length/Coordinates.getLenght() ) == number_of_movables);
        int heuristc_value = 0;
        ConflictAvoidanceCheckingRules.SearchState search_state = conflict_avoidance_checking_rules.getSearchState();

        switch (searchMultiAgentState)
            {
            case AGENTS_ONLY:
                switch (search_state){
                    case CHECK_TIME_DEADLINE :
                        IllegalPathsStore illegal_paths_store = conflict_avoidance_checking_rules.getIllegalPathsStore();
                        heuristc_value = heuristicMetricsSearch.getStatePosHeuristc(illegal_paths_store, group_marks_ids, cell_coordinates);
                        break;

                    case NO_CHECK_CONFLICTS :
                    case AVOID_PATH : heuristc_value = heuristicMetricsSearch.getStatePosHeuristc(cell_coordinates);
                        break;
                }

                break;
            case AGENTS_AND_BOXES:

                switch (search_state){
                    case CHECK_TIME_DEADLINE :
                        IllegalPathsStore illegal_paths_store = conflict_avoidance_checking_rules.getIllegalPathsStore();
                        heuristc_value =  heuristicMetricsSearch.getHeuristcOfAGENTS_AND_BOXES(illegal_paths_store, group_marks_ids, cell_coordinates, index_boxes);
                        break;

                    case NO_CHECK_CONFLICTS :
                    case AVOID_PATH : heuristc_value = heuristicMetricsSearch.getHeuristcOfAGENTS_AND_BOXES(index_boxes, cell_coordinates);
                        break;
                }

                break;
        }
        start_state = SearchMAState.createNew(cell_coordinates, total_gcost, heuristc_value + total_gcost);

        return start_state;
    }

    public static int getGCost(int[][] state) {
        return SearchMAState.getGCost(state);
    }

    public static void updateCameFromPrevCell2(int[][] state, int[][] previouse_coordinates) {
        came_from.put(getCellCoordinates(state), getCellCoordinates(previouse_coordinates));
    }

    public static void updateCameFromPrevCell(HashMap<Integer,int[]> came_from, int[][] state, int[][] previouse_coordinates) {
        came_from.put( Arrays.hashCode(getCellCoordinates(state)) , getCellCoordinates(previouse_coordinates));
    }

    public static void updateCameFromPrevCell(int[][] state, int[] previouse_coordinates) {
        came_from.put(getCellCoordinates(state), previouse_coordinates);
    }

    public static int[] getCellCoordinates(int[][] state){
        return SearchMAState.getStateCoordinates(state);
    }

    public static boolean isGoal(int[] state_coordinates){
        boolean isGoal = true;
        int y_goal;
        int x_goal;
        int y;
        int x;

        switch (searchMultiAgentState){
            case AGENTS_ONLY:
                for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
                    y = Coordinates.getRow(coordinate, state_coordinates);
                    x = Coordinates.getCol(coordinate, state_coordinates);

                    y_goal = Coordinates.getRow(coordinate, goals_coordinates);
                    x_goal = Coordinates.getCol(coordinate, goals_coordinates);

                    isGoal = isGoal &&  (y == y_goal) && (x == x_goal);

                    if (!isGoal) break;
                }
                return isGoal;

            case AGENTS_AND_BOXES:
                int number_of_goals = goals_coordinates.length/Coordinates.getLenght();
                for (int coordinate = 0; coordinate < number_of_goals; coordinate = coordinate + 1) {
                    //get row of boxes
                    int idx = index_boxes[coordinate];
                    y = Coordinates.getRow(idx, state_coordinates);
                    x = Coordinates.getCol(idx, state_coordinates);

                    y_goal = Coordinates.getRow(coordinate, goals_coordinates);
                    x_goal = Coordinates.getCol(coordinate, goals_coordinates);

                    isGoal = isGoal &&  (y == y_goal) && (x == x_goal);

                    if (!isGoal) break;
                }

                return isGoal;

            case AVOIDING_GOALS:
                for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
                    y = Coordinates.getRow(coordinate, state_coordinates);
                    x = Coordinates.getCol(coordinate, state_coordinates);

                    ArrayList<int[]> __goals = goals_neighbours.get(coordinate);
                    for(int[] __goal : __goals){
                        y_goal = Coordinates.getRow(__goal);
                        x_goal = Coordinates.getCol(__goal);

                        isGoal = isGoal &&  (y == y_goal) && (x == x_goal);
                        if (isGoal) break;
                    }
                    if (!isGoal) break;
                }

                return isGoal;
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
        int coordinate_index = 0;
        int time_state = Coordinates.getTime(coordinate_index, coordinates);
        int y;
        int x;
        boolean is_present = false;

        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
            y = Coordinates.getRow(coordinate, coordinates);
            x = Coordinates.getCol(coordinate, coordinates);

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
       return od_node_structure.isStandardNode(pos_coordinates);
    }

    public static boolean isIntermediateNode(int[] pos_coordinates) {
        return !isStandardNode(pos_coordinates);
    }



     //if neighbours contains one of the other mark_ids from standard node :
    //add it to the set of SimulationConflicts
    //edge conflict only for overstepped movable object
    //vertex conflicts for all others movable objects
    public static void setConflictsNodeExpansion(int index_to_expand, int[] pos_coordinates, int[] cell_pos_neighbour){
        int row_neigbour = Coordinates.getRow(cell_pos_neighbour);
        int col_neigbour = Coordinates.getCol(cell_pos_neighbour);
        int time_neigbour = Coordinates.getTime(cell_pos_neighbour);

        for (int i = 0; i < number_of_movables; i++) {
            if(i == index_to_expand) continue;

            int row_coord = Coordinates.getRow(i, pos_coordinates);
            int col_coord = Coordinates.getCol(i, pos_coordinates);
            int time_coord = Coordinates.getTime(i, pos_coordinates);

            //do not impose constraints on the previouse expanded cell_positions
            if(time_coord == time_neigbour) continue;
            if ( row_coord == row_neigbour && col_coord == col_neigbour) {
                od_conflicts.setConflictsEdgeConflict(i, time_coord, index_to_expand, pos_coordinates, cell_pos_neighbour);

            od_conflicts.setConflictsVertexConflict(i, index_to_expand, pos_coordinates, cell_pos_neighbour);
            }
        }
    }

    //entry point of the operator decomposition algorithm
    public static ArrayDeque<int[][]> expandStandardState(int[][] state) {
        int[] pos_coordinates = SearchMAState.getStateCoordinates(state);
        //int[] pos_coordinates = Arrays.copyOf(SearchMAState.getStateCoordinates(state),SearchMAState.getStateCoordinates(state).length);
        ArrayDeque<int[][]> __result = new ArrayDeque<>();
        od_conflicts.setNodeConflicts(pos_coordinates);

        switch (searchMultiAgentState)
        {
            case AGENTS_ONLY:
                heuristicMetricsSearch.setStandardNodeAGENTS_ONLY(state);
                ArrayDeque<int[][]> __result__1 = expandStandardStateWithAgents(pos_coordinates);
                                    __result.addAll(__result__1);
                                    break;
            case AGENTS_AND_BOXES:
                heuristicMetricsSearch.setsetStandardNodeAGENTS_AND_BOXES(state);

                ArrayDeque<int[][]> __result__2 = expandStandardStateWithAgentsAndBoxes(pos_coordinates);
                                    __result.addAll(__result__2);
                                    break;
        }
        return __result;
    }

    //entry point of the operator decomposition algorithm
    public static ArrayDeque<int[][]> expandIntermediateState(int[][] state) {
        int[] pos_coordinates = SearchMAState.getStateCoordinates(state);
        ArrayDeque<int[][]> __result = new ArrayDeque<>();

        switch (searchMultiAgentState)
        {
            case AGENTS_ONLY:
                heuristicMetricsSearch.setStandardNodeAGENTS_ONLY(state);

                ArrayDeque<int[][]> _res = expandIntermediateStateWithAgents(pos_coordinates);
                __result.addAll(_res);
                break;
            case AGENTS_AND_BOXES:
                heuristicMetricsSearch.setsetStandardNodeAGENTS_AND_BOXES(state);
                int g_cost = SearchMAState.getGCost(state);
                ArrayDeque<int[][]> __result__ = expandIntermediateStateWithAgentsAndBoxes(pos_coordinates);
                                    __result.addAll(__result__);
                break;
        }
        return __result;
    }

    public static ArrayDeque<int[][]> expandStandardStateWithAgents(int[] pos_coordinates) {
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
        LinkedList<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand);

        ////////////to test
        //validateNeighbourOfGoal(pos_coordinates, index_to_expand, position_to_expand, neighbours);
        //////////////

        for(int [] cell_pos_neighbour : neighbours){
            setConflictsNodeExpansion(index_to_expand, pos_coordinates, cell_pos_neighbour);
            it should not be the above because the chlidren nodes will share the conflicts
            it should be :
            setConflictsNodeExpansion(index_to_expand, next_state_node, cell_pos_neighbour);


            int[] next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
            Coordinates.setCoordinateAtIndex(index_to_expand, next_state_node, cell_pos_neighbour);
            int[][] standard_node_costs = heuristicMetricsSearch.updateIntermediateNodeAGENTS_ONLY(next_state_node);
            next_state_nodes.add(SearchMAState.createNew(next_state_node, standard_node_costs[HeuristicMetricsSearch.G_COST][0], standard_node_costs[HeuristicMetricsSearch.F_COST][0]));
         }

        return next_state_nodes;
    }

    private static void validateNeighbourOfGoal(int[] pos_coordinates, int index_to_expand, int[] position_to_expand, LinkedList<int[]> neighbours) {
        ArrayDeque<Integer> indexes= new ArrayDeque<>();

        if (Coordinates.getRow(position_to_expand) == Coordinates.getRow(index_to_expand, goals_coordinates) &&
                Coordinates.getCol(position_to_expand) == Coordinates.getCol(index_to_expand, goals_coordinates)){
            int[] prev_pos  = came_from.get(pos_coordinates);

            if(prev_pos == null)
                System.out.println("#prev_pos is null at line 887 in StateSearchMAFactory");

            for (int i = 0; i < neighbours.size(); i++){
                int[] cell_pos_neighbour = neighbours.get(i);
                //if the cell that the agent came from when occupying the box cell
                //is still valid added to neighbours and remove the rest
                if (Coordinates.getRow(cell_pos_neighbour) == Coordinates.getRow(index_to_expand, prev_pos)  &&
                        Coordinates.getCol(cell_pos_neighbour) == Coordinates.getCol(index_to_expand, prev_pos)){
                    int[] __cell = neighbours.get(i);
                }else {
                    indexes.add(i);
                }
            }
        }

        while (!indexes.isEmpty()){
            int idx = indexes.pop();
            int[] cell = neighbours.remove(idx);
        }
    }


    public static ArrayDeque<int [][]> expandIntermediateStateWithAgents(int[] pos_coordinates) {

        int min_time = Integer.MAX_VALUE;
        ArrayList<Integer> coord_candidates = new ArrayList<>();

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
        int index_to_expand = -1;
        if (coord_candidates.size() > 0){
            int __index = random.nextInt(coord_candidates.size());//or other heuristic to use instead of random
            index_to_expand = coord_candidates.get(__index);
        }

        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

        if(index_to_expand > -1) {
            int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);
            int mark_id = group_marks_ids[index_to_expand];
            ArrayDeque<int []> conflicts_avoidance = od_conflicts.getConflictsAvoidance( index_to_expand, pos_coordinates);
            LinkedList<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand, conflicts_avoidance);

        ////////////to test
        //validateNeighbourOfGoal(pos_coordinates, index_to_expand, position_to_expand, neighbours);
        //////////////
        for(int[] cell_pos_neighbour : neighbours){
            //it imposes the same constraints but not on the already expanded positions
            setConflictsNodeExpansion(index_to_expand, pos_coordinates, cell_pos_neighbour);
        }

        int[] next_state_node;
        for(int [] cell_pos : neighbours){
            next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
            Coordinates.setCoordinateAtIndex(index_to_expand, next_state_node, cell_pos);
            int[][] intermediate_node_costs = heuristicMetricsSearch.updateIntermediateNodeAGENTS_ONLY(next_state_node);
            next_state_nodes.add(SearchMAState.createNew(next_state_node, intermediate_node_costs[HeuristicMetricsSearch.G_COST][0],
                    intermediate_node_costs[HeuristicMetricsSearch.F_COST][0]));
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

    //TO USE
    public static ArrayDeque<int[][]> expandStandardStateWithAgentsAndBoxes(int[] pos_coordinates) {
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
        return expandStateStartingWithAgents(index_to_expand, pos_coordinates);
    }

    //TO USE
    public static ArrayDeque<int [][]> expandIntermediateStateWithAgentsAndBoxes(int[] pos_coordinates) {

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
                    coord_candidates1 = od_conflicts.getCoordCandidates(coordinate, pos_coordinates);
                    boolean non_waiting_found = true;

                    if(coord_candidates1.size() > 0) non_waiting_found = false;
                    if (non_waiting_found) waiting_candidates1.add(coordinate);
                    
                }else if(all_agents_indexes.containsKey(coordinate)){
                    coord_candidates1.add(coordinate);
                }
            }
        }

        ArrayList<Integer> waiting_candidates = new ArrayList<>(waiting_candidates1);
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

    /*if there is no box constrained to expand: expand only agents
                    if no more agents to expand left expand boxes and do not move them just increase time step
    * */
        if(coord_to_expand > -1 && all_agents_indexes.containsKey(coord_to_expand)) {


            return expandStateStartingWithAgents(coord_to_expand, pos_coordinates);
        }else{
            return expandStateStartingWithBoxes(coord_to_expand, pos_coordinates, waiting_candidates);
        }
    }


    private static ArrayDeque<int[][]> expandStateStartingWithAgents(int index_to_expand, int[] pos_coordinates) {

        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();
        int[] position_to_expand = Coordinates.getCoordinatesAt(index_to_expand, pos_coordinates);
        int mark_id = group_marks_ids[index_to_expand];

        //get the boxes of different colour and add pull constraints if these are neighbours
        HashMap<Integer, int[]> boxes_coord_to_avoid = getBoxesCoordToAvoid(index_to_expand);
        for (Integer key: boxes_coord_to_avoid.keySet()){
            Coordinates.setTime(boxes_coord_to_avoid.get(key), Coordinates.getTime(key ,pos_coordinates));
        }
        //ArrayDeque<int []> conflicts_avoidance = od_conflicts.getConflictsAvoidance(index_to_expand, pos_coordinates);
        //conflicts_avoidance.addAll(boxes_coord_to_avoid.values());

        ArrayDeque<int []> conflicts_avoidance = od_conflicts.getConflictsAvoidance(index_to_expand, pos_coordinates);

        LinkedList<int[]> neighbours_agent = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand, conflicts_avoidance);
        //////////to test
        //validateNeighbourOfGoal(pos_coordinates, index_to_expand, position_to_expand, neighbours_agent);
        ///////////
       // conflicts_avoidance.clear();

        //for boxes to avoid add pull constraint if or not other agents are neighbours
        ArrayDeque<int[]> neighbours_to_remove = new ArrayDeque<>();

        //check if the boxes to avoid were overstepped in the new step time
        for(int[] neighbour_found : neighbours_agent){
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
                                od_conflicts.addPullConstraint(index_to_expand, key_avoid, pos_coordinates, pull_move_cell);
                                //the PullConstraint imposes VertexConflicts for the other movables
                                setConflictsNodeExpansion(key_avoid, pos_coordinates, pull_move_cell);
                            }else{
                                neighbours_to_remove.add(neighbour_found);
                            }
                        }
                    }
                }
            }
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
                    int agent_index = index_to_expand;
                    od_conflicts.addPullConstraint(agent_index, box_index, pos_coordinates, pull_move_cell);
                }
            }

            for(int [] cell_pos_neighbour : neighbours_agent){
                setConflictsNodeExpansion(index_to_expand, pos_coordinates, cell_pos_neighbour);

                int[] next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                Coordinates.setCoordinateAtIndex(index_to_expand, next_state_node, cell_pos_neighbour);
                //pos_coordinates get g cost  pos_coordinates get f cost
                int[][] standard_node_costs = heuristicMetricsSearch.updateIntermediateNodeAGENTS_AND_BOXES(index_boxes, next_state_node);
                next_state_nodes.add(SearchMAState.createNew(next_state_node,
                        standard_node_costs[HeuristicMetricsSearch.G_COST][0], standard_node_costs[HeuristicMetricsSearch.F_COST][0]));
            }
        }

        return next_state_nodes;
    }

    //expand only for boxes
    public static ArrayDeque<int [][]> expandStateStartingWithBoxes(int coord_to_expand, int[] pos_coordinates, ArrayList<Integer> waiting_candidates) {
        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();


        if(coord_to_expand > -1 && all_boxes_indexes.containsKey(coord_to_expand)) {
            ArrayDeque<int []> conflicts_avoidance = new ArrayDeque<>();
            int[] to_expand = Coordinates.getCoordinatesAt(coord_to_expand, pos_coordinates);
            int mark_id = group_marks_ids[coord_to_expand];

            conflicts_avoidance = od_conflicts.getConflictsAvoidance(coord_to_expand, pos_coordinates);
            ArrayDeque<int[]> constraint_moving = od_conflicts.getConstraintMoving(coord_to_expand, pos_coordinates);

            LinkedList<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, to_expand, conflicts_avoidance);
            conflicts_avoidance.clear();

            while (!constraint_moving.isEmpty())
                neighbours.add(constraint_moving.pop());

            for(int[]  cell_pos_neighbour : neighbours){
                //it imposes the same constraints but not on the already expanded positions
                setConflictsNodeExpansion(coord_to_expand, pos_coordinates, cell_pos_neighbour);
            }

            if(neighbours.size() == 0){
                //box blocked can not move
                Coordinates.setTime(to_expand,Coordinates.getTime(to_expand) + 1 );
                neighbours.add(to_expand);
            }

            int[] next_state_node;
            for(int [] cell_pos : neighbours){
                next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                Coordinates.setCoordinateAtIndex(coord_to_expand, next_state_node, cell_pos);
                int[][] intermediate_node_costs = heuristicMetricsSearch.getBoxesCosts(index_boxes, next_state_node);
                next_state_nodes.add(SearchMAState.createNew(next_state_node, intermediate_node_costs[HeuristicMetricsSearch.G_COST][0], intermediate_node_costs[HeuristicMetricsSearch.F_COST][0]));
            }
        }

        //waiting constraint states aplies for the rest of  boxes
        if (next_state_nodes.size() == 0){
            if (waiting_candidates.size() > 0){
                int[]  next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                for (Integer coord : waiting_candidates ){
                    Coordinates.setTime(coord, next_state_node, Coordinates.getTime(coord, pos_coordinates) + 1);
               }
                int[][] intermediate_node_costs = heuristicMetricsSearch.getBoxesCosts(index_boxes, next_state_node);
                next_state_nodes.add(SearchMAState.createNew(next_state_node, intermediate_node_costs[HeuristicMetricsSearch.G_COST][0], intermediate_node_costs[HeuristicMetricsSearch.F_COST][0]));
            }
        }
        return next_state_nodes;
    }


    public void avoidCoordinate(StateSearchMAFactory.SearchState searchMultiAgentState, LinkedList<int[]> neighbours, int[] coordinates_to_avoid){
        int number_of_movables = coordinates_to_avoid.length/Coordinates.getLenght();//to do divide goals__coordinates by the length of the coordinates

        //avoid box goal when is just a goal
        switch (searchMultiAgentState) {
            case AGENTS_ONLY:
                int index = 0;
                ArrayList<Integer> to_remove = new ArrayList<>();

                for (int[] neighbour : neighbours) {
                    for (int j = 0; j < number_of_movables; j++) {
                        int row_goal = Coordinates.getRow(j, coordinates_to_avoid);
                        int col_goal = Coordinates.getCol(j, coordinates_to_avoid);
                        if (Coordinates.getRow(neighbour) == row_goal
                                && Coordinates.getCol(neighbour) == col_goal){
                            to_remove.add(index);
                        }
                    }
                    index++;
                }
                for (int __index : to_remove) {
                    neighbours.remove(__index);
                }

                break;
            case AGENTS_AND_BOXES: break;
        }
    }

    //expands all the pushes , pulls with box and the agent moves at the same time
    //it looks like it does make pull and push at the same time
    private static ArrayDeque<int[][]> expandStandardStateForAgentsAndBoxesMultiple(int[] pos_coordinates, int g_cost, int f_cost, ArrayList<SimulationConflict> standard_to_conflicts) {
        // if agent gets pulls or pushes expand two states: agent plus box
        //otherwise if it only moves expand only the agent
        //otherwise the box gets never expanded alone
        Random random = new Random();
        int number_of_coordinates = pos_coordinates.length / Coordinates.getLenght();

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

        for(int[] cell_pos_neighbour : neighbours_agent){
            setConflictsNodeExpansion(index_to_expand, pos_coordinates, cell_pos_neighbour);

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



