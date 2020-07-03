package org.agents.searchengine;

import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.IllegalPathsStore;

public class HeuristicMetricsSearch {
    static final int COST_NEXT_CELL = 1;

    final static int G_COST = 0;
    final static int H_COST = 1;
    final static int F_COST = 2;
    final static int H_COSTS = 3;

    int[] goals_coordinates;

    private int[][] standard_node_root_costs;
    private int[][] standard_node_costs;
    private int[][] intermediate_node_costs;
    private int[][] intermediate_roote_node;
    private int[][] agent_node_costs;

    final static int STATE_STANDARD = 0;  //starndard node or intermediate node as in stanleys paper
    final static int STATE_INTERMEDIATE = 1; //intermediate state is when not all agents in the position advanced a time step

    public HeuristicMetricsSearch(int[] goalsCoordinates) {
        goals_coordinates = goalsCoordinates;
    }

    public void initStandardNodeCosts() {
        int[] heuristic_standard_coordinates_output = new int[goals_coordinates.length / Coordinates.getLenght()];

        standard_node_root_costs = new int[4][];
        standard_node_root_costs[G_COST] = new int[1];
        standard_node_root_costs[H_COST] = new int[1];
        standard_node_root_costs[F_COST] = new int[1];
        standard_node_root_costs[H_COSTS] = heuristic_standard_coordinates_output;


        standard_node_costs = new int[4][];
        standard_node_costs[G_COST] = new int[1];
        standard_node_costs[H_COST] = new int[1];
        standard_node_costs[F_COST] = new int[1];
        standard_node_costs[H_COSTS] = heuristic_standard_coordinates_output;

        agent_node_costs = new int[4][];
        agent_node_costs[G_COST] = new int[1];
        agent_node_costs[H_COST] = new int[1];
        agent_node_costs[F_COST] = new int[1];
        agent_node_costs[H_COSTS] = heuristic_standard_coordinates_output;
    }

    public void initIntermediateNodeCosts() {
        int[] heuristic_intermediate_coordinates_output = new int[StateSearchMAFactory.number_of_movables];

        intermediate_node_costs = new int[4][];
        intermediate_node_costs[G_COST] = new int[1];
        intermediate_node_costs[H_COST] = new int[1];
        intermediate_node_costs[F_COST] = new int[1];
        intermediate_node_costs[H_COSTS] = heuristic_intermediate_coordinates_output;

        intermediate_roote_node = new int[4][];
        intermediate_roote_node[G_COST] = new int[1];
        intermediate_roote_node[H_COST] = new int[1];
        intermediate_roote_node[F_COST] = new int[1];
        intermediate_roote_node[H_COSTS] = heuristic_intermediate_coordinates_output;
    }

    public void setStandardNodeCosts(int[][] state) {
        standard_node_root_costs[G_COST][0] = SearchMAState.getGCost(state);
        standard_node_root_costs[H_COST][0] = SearchMAState.getFCost(state) - SearchMAState.getGCost(state);
        standard_node_root_costs[F_COST][0] = SearchMAState.getFCost(state);
    }

    public void setStandardNodeAGENTS_ONLY(int[][] state) {
        setStandardNodeCosts(state);
    }


    public int[][] updateIntermediateNodeAGENTS_ONLY(int[] next_state_node) {
        int g_cost = standard_node_root_costs[G_COST][0] + COST_NEXT_CELL;
        int h_cost = getStatePosHeuristc(next_state_node);
        int f_cost = h_cost + g_cost;

        intermediate_node_costs[G_COST][0] = g_cost;
        intermediate_node_costs[H_COST][0] = h_cost;
        intermediate_node_costs[F_COST][0] = f_cost;
        //standard_node_costs[H_COSTS] = heuristic_standard_coordinates_output;

        return intermediate_node_costs;
    }

    public int[][] updateIntermediateNodeAGENTS_AND_BOXES(int[] index_boxes, int[] next_state_node) {
        int g_cost = intermediate_roote_node[G_COST][0] + COST_NEXT_CELL;
        int h_cost =  getHeuristcOfAGENTS_AND_BOXES(index_boxes, next_state_node);
        int f_cost = h_cost + g_cost;

        intermediate_node_costs[G_COST][0] = g_cost;
        intermediate_node_costs[H_COST][0] = h_cost;
        intermediate_node_costs[F_COST][0] = f_cost;
        //standard_node_costs[H_COSTS] = heuristic_standard_coordinates_output;

        return intermediate_node_costs;
    }

    //agent costs when the  searching combines the states of agents and boxes
    public void setsetStandardNodeAGENTS_AND_BOXES(int[][] state) {
        intermediate_roote_node[G_COST][0] = SearchMAState.getGCost(state);
        intermediate_roote_node[H_COST][0] = SearchMAState.getFCost(state) - SearchMAState.getGCost(state);
        intermediate_roote_node[F_COST][0] = SearchMAState.getFCost(state);
    }

    public int[][] getBoxesCosts(int[] index_boxes, int[] positiion_coordinates) {
        //int g_cost = SearchMAState.getGCost(state) + COST_NEXT_CELL;
        int g_cost = intermediate_roote_node[G_COST][0] + COST_NEXT_CELL;;
        int h_cost = getHeuristcOfAGENTS_AND_BOXES(index_boxes, positiion_coordinates);
        int f_cost = h_cost + g_cost;

        intermediate_roote_node[G_COST][0] = g_cost;
        intermediate_roote_node[H_COST][0] = h_cost;
        intermediate_roote_node[F_COST][0] = f_cost;
        //intermediate_node_costs[H_COSTS] = heuristic_intermediate_coordinates_output;

        return intermediate_roote_node;
    }

    public int[][] createIntermediateNodeCosts() {
        int[] heuristic_intermediate_coordinates_output2 = new int[StateSearchMAFactory.number_of_movables];

        int[][] intermediate_node_costs = new int[4][];
        intermediate_node_costs[G_COST] = new int[1];
        intermediate_node_costs[H_COST] = new int[1];
        intermediate_node_costs[F_COST] = new int[1];
        intermediate_node_costs[H_COSTS] = heuristic_intermediate_coordinates_output2;
        //int g_cost = SearchMAState.getGCost(state) + COST_NEXT_CELL;
        //int h_cost = getHeuristcsMovablesOf(SearchMAState.getStateCoordinates(state), heuristic_intermediate_coordinates_output2);
        //int f_cost = h_cost + g_cost;

        intermediate_node_costs[G_COST][0] = 1;
        intermediate_node_costs[H_COST][0] = 0;
        intermediate_node_costs[F_COST][0] = 1;
        //intermediate_node_costs[H_COSTS] = heuristic_intermediate_coordinates_output;

        return intermediate_node_costs;
    }

    public int[][] updateIntermediateNodeAGENTS_ONLY(int[][] state) {
        int g_cost = SearchMAState.getGCost(state) + COST_NEXT_CELL;
        int h_cost = getStatePosHeuristc(SearchMAState.getStateCoordinates(state));
        int f_cost = h_cost + g_cost;

        intermediate_node_costs[G_COST][0] = g_cost;
        intermediate_node_costs[H_COST][0] = h_cost;
        intermediate_node_costs[F_COST][0] = f_cost;
        //intermediate_node_costs[H_COSTS] = heuristic_intermediate_coordinates_output;

        return intermediate_node_costs;
    }



    public  int getStateHeuristcManhatten(int[] cell_coordinates) {
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;

        int number_of_movables = cell_coordinates.length/Coordinates.getLenght();
        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
            y = Coordinates.getRow(coordinate, cell_coordinates);
            x = Coordinates.getCol(coordinate, cell_coordinates);
            y_goal = Coordinates.getRow(coordinate, goals_coordinates);
            x_goal = Coordinates.getCol(coordinate, goals_coordinates);

            heuristc_value += getManhattenHeuristic(y, x , y_goal, x_goal);
        }
        return heuristc_value;
    }

    public int getStatePosHeuristc(int[] cell_coordinates){
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;

        int number_of_movables = cell_coordinates.length/Coordinates.getLenght();

        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
            int time_state = Coordinates.getTime(coordinate, cell_coordinates);
            y = Coordinates.getRow(coordinate, cell_coordinates);
            x = Coordinates.getCol(coordinate, cell_coordinates);
            y_goal = Coordinates.getRow(coordinate, goals_coordinates);
            x_goal = Coordinates.getCol(coordinate, goals_coordinates);

            heuristc_value += getManhattenHeuristic(y, x, y_goal, x_goal);
        }

        return heuristc_value;
    }

    public int getStatePosHeuristc(IllegalPathsStore illegal_paths_store, int[] group_marks_ids, int[] cell_coordinates){
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;

        int number_of_movables = cell_coordinates.length/Coordinates.getLenght();

        for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
            int time_state = Coordinates.getTime(coordinate, cell_coordinates);
            y = Coordinates.getRow(coordinate, cell_coordinates);
            x = Coordinates.getCol(coordinate, cell_coordinates);
            y_goal = Coordinates.getRow(coordinate, goals_coordinates);
            x_goal = Coordinates.getCol(coordinate, goals_coordinates);

            int mark_id = group_marks_ids[coordinate];
            int cost_time = time_state;
            heuristc_value += getDeadLineHeuristic(illegal_paths_store, mark_id, cost_time, y, x, y_goal, x_goal);
        }

        return heuristc_value;
    }

    public int getHeuristcOfAGENTS_AND_BOXES(int[] index_boxes, int[] cell_coordinates){
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;

        int index = 0;
        for (int coordinate : index_boxes) {
            y = Coordinates.getRow(coordinate, cell_coordinates);
            x = Coordinates.getCol(coordinate, cell_coordinates);

            y_goal = Coordinates.getRow(index, goals_coordinates);
            x_goal = Coordinates.getCol(index++, goals_coordinates);

            heuristc_value += getManhattenHeuristic(y, x, y_goal, x_goal);
        }

        return heuristc_value;
    }





    public int getHeuristcOfAGENTS_AND_BOXES(IllegalPathsStore illegal_paths_store, int[] group_marks_ids, int[] cell_coordinates, int[] index_boxes){
        int heuristc_value = 0;
        int y;
        int x;
        int y_goal;
        int x_goal;

        int index = 0;
        for (int coordinate : index_boxes) {
            int time_state = Coordinates.getTime(coordinate, cell_coordinates);
            y = Coordinates.getRow(coordinate, cell_coordinates);
            x = Coordinates.getCol(coordinate, cell_coordinates);

            y_goal = Coordinates.getRow(index, goals_coordinates);
            x_goal = Coordinates.getCol(index++, goals_coordinates);

            int mark_id = group_marks_ids[coordinate];
            int cost_time = time_state;
            heuristc_value += getDeadLineHeuristic(illegal_paths_store, mark_id, cost_time, y, x, y_goal, x_goal);
        }

        return heuristc_value;
    }

    public static int getDeadLineHeuristic(IllegalPathsStore illegal_paths_store, int mark_id, int cost_time, int y, int x, int y_goal, int x_goal) {
        int[] deadline_state = illegal_paths_store.getIllegalPath(mark_id).getDeadlineConstraint();
        int time_deadline_constraint = Coordinates.getTime(deadline_state);

        int time_left = time_deadline_constraint - cost_time;
        int row_deadline_state = Coordinates.getRow(deadline_state);
        int col_deadline_state = Coordinates.getCol(deadline_state);

        if (time_left <= 0){
            return getManhattenHeuristic(y, x, y_goal,x_goal);
        } else{
            return time_left + getManhattenHeuristic(row_deadline_state, col_deadline_state, y_goal, x_goal);
        }

    }

    //cost_time(s) is total cost of the path until now
    public  int getDeadLineHeuristic(IllegalPathsStore illegal_paths_store, int mark_id, int cost_time, int[] cell_coordinates, int[] goal_coordinates){
      return getDeadLineHeuristic(illegal_paths_store,  mark_id,
              cost_time,Coordinates.getRow(cell_coordinates), Coordinates.getCol(cell_coordinates),
              Coordinates.getRow(goal_coordinates), Coordinates.getCol(goal_coordinates));
    }


    public static int getManhattenHeuristic(int y, int x, int y_goal, int x_goal) {
        return Math.abs(y - y_goal) + Math.abs(x - x_goal);
    }

    public static int getManhattenHeuristic(int[] cell_coordinates, int[] goal_coordinates){
        return Math.abs(Coordinates.getRow(cell_coordinates) - Coordinates.getRow(goal_coordinates)) + Math.abs(Coordinates.getCol(cell_coordinates) - Coordinates.getCol(goal_coordinates))  ;
    }



}
