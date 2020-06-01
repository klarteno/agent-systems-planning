package org.agents.searchengine;

 import org.agents.Agent;
 import org.agents.Box;
 import org.agents.markings.Coordinates;
 import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;

 import java.util.*;
 import java.util.logging.MemoryHandler;

public class SearchEngineOD {
    private static final int COST_NEXT_CELL = 1;

    private ArrayDeque<int[]> path;
    private static PriorityQueue<int[][]> frontier;
    public static HashMap<int[],int[]> came_from;
    ArrayDeque<int[]> path_normal;

    public SearchEngineOD(int[] start_group, ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules){
        StateSearchMAFactory.setStartGroup(start_group);
        StateSearchMAFactory.setAvoidanceCheckingRules(conflictAvoidanceCheckingRules);
        //move to parent class and subclass??
        //make second option for comparator
        frontier = new PriorityQueue<int[][]>(5, Comparator.comparingInt(SearchMAState::getFCost));
        came_from = new HashMap<>();
        path_normal = new ArrayDeque<>();

    }

    public int[] getStartCoordinatesOfGroup(){
        return StateSearchMAFactory.getStartCoordinatesGroup();
    }

    public int[] getGoalsCoordinatesOfGroup(){
        return StateSearchMAFactory.getGoalsCoordinatesGroup();
    }

    public ArrayDeque<int[]> getPath(){
        assert this.path.size() > 0;
        return path;
    }

    public int getPathCost(){
        assert this.path.size() > 0;
        return path.size();
    }

    public boolean isPathFound() {
        return this.path.size() > 0;
    }

    public static int getHeuristic(int[] cell_coordinates, int[] goal_coordinates){
        return Math.abs(Coordinates.getRow(cell_coordinates) - Coordinates.getRow(goal_coordinates)) + Math.abs(Coordinates.getCol(cell_coordinates) - Coordinates.getCol(goal_coordinates))  ;
    }

    public void runOperatorDecomposition(Agent agent){
        this.runOperatorDecomposition(agent.getCoordinates(), agent.getGoalPosition(), new int[0][][]);
    }

    public void runOperatorDecomposition(Box box){
        int y_pos = box.getRowPosition();
        int x_pos = box.getColumnPosition();
        int time_pos = box.getTimeStep();
        this.runOperatorDecomposition(box.getCoordinates(), box.getGoalPosition(), new int[0][][]);
    }

    public void runOperatorDecomposition(int[] start_coordinates, int[] goal_coordinates){
        runOperatorDecomposition(start_coordinates, goal_coordinates, new  int[0][][]);
    }

    public void runOperatorDecomposition(int[] start_coordinates, int[] goal_coordinates, int[][][] conflicting_paths){
        assert start_coordinates.length == goal_coordinates.length;
        assert ( start_coordinates.length % Coordinates.getLenght() )== 0;
        assert start_coordinates.length/Coordinates.getLenght() > 1;

        HashMap<int[], int[]> intermediate_came_from_standard = new HashMap<>();
        HashMap<int[], int[]> standard_came_from_intermediate = new HashMap<>();
        HashMap<int[], int[]> intermediate_came_from_intermediate = new HashMap<>();

        ArrayDeque<int[][] > path_to_test = new ArrayDeque<>();
        ArrayDeque<int[]> path = new ArrayDeque<>();




        frontier.clear();
        //StateSearchMAFactory.createCostSoFar();
        StateSearchMAFactory.createClosedSet();

        int time_step = 0;
        //unused for output from algorithm, delete it when make bench mark

        int[][] next_state = StateSearchMAFactory.createStandardState(start_coordinates, 0);
        int[] prev_standard_state = StateSearchMAFactory.getCellCoordinates(next_state);
        frontier.add(next_state);
        //StateSearchMAFactory.putCostSoFar(next_state);
        //StateSearchMAFactory.mark_state_inqueue(next_state,true);

        StateSearchMAFactory.updateCameFromPrevCell(came_from, next_state, next_state);

        //init state with dummy variables
        int[][] current_state = null;
        int[][] prev_state = StateSearchMAFactory.createDummyState(); //StateSearchMAFactory.createDummyState()

        while(!frontier.isEmpty()){
            current_state = frontier.poll();

            if (StateSearchMAFactory.isStandardNode(SearchMAState.getStateCoordinates(current_state))){
                int prev_time = SearchMAState.getTime(0, SearchMAState.getStateCoordinates(prev_state));
                int current_time = SearchMAState.getTime(0, SearchMAState.getStateCoordinates(current_state));
                if(current_time - prev_time != 1  ){
                    SearchMAState.setTimeStep(current_state, prev_time + 1);
                }

                prev_state = current_state;

                path.push(SearchMAState.getStateCoordinates(current_state));
                path_to_test.push(current_state);   

                if (StateSearchMAFactory.isGoal(SearchMAState.getStateCoordinates(current_state))){
                    this.path = path;

                    int[] next_key;

                    path_normal.add(SearchMAState.getStateCoordinates(current_state));
                    next_key = came_from.get(SearchMAState.getStateCoordinates(current_state));

                    /*
                    path_normal.add(next_key);
                    while (came_from.get(next_key) != null){ different from start position start_coordinates
                        next_key = came_from.get(next_key);
                        path_normal.add(next_key);
                    }
                    */

                    path_normal.add(next_key);
                    int[] next_key2;
                    boolean removed;
                    while (came_from.get(next_key) != start_coordinates){
                        next_key2 = next_key;
                        next_key = came_from.get(next_key);
                        path_normal.add(next_key);
                        removed = came_from.remove(next_key2, next_key);
                    }



                    return;
                    //break;
                }

                StateSearchMAFactory.addToClosedSet(current_state);
                int g_cost = SearchMAState.getGCost(current_state) + COST_NEXT_CELL;
                //int f_cost = SearchMAState.getFCost(current_state) - COST_NEXT_CELL;
                int h_cost = StateSearchMAFactory.getHeuristcOf(SearchMAState.getStateCoordinates(current_state));
                int f_cost = h_cost + g_cost;

                int[] pos_coordinates = Arrays.copyOf(SearchMAState.getStateCoordinates(current_state), SearchMAState.getStateCoordinates(current_state).length);
                ArrayDeque<int[][]> next_intermediate_nodes = StateSearchMAFactory.expandStandardState(pos_coordinates, g_cost, f_cost);
                 //assert StateSearchMAFactory.isIntermediateNode(next_intermediate_nodes);
                while(!next_intermediate_nodes.isEmpty()){
                    int[][] state = next_intermediate_nodes.pop();
                    StateSearchMAFactory.updateCameFromPrevCell(intermediate_came_from_standard, state, current_state);
                    frontier.add(state);
                    //StateSearchMAFactory.putCostSoFar(state);
                    //StateSearchMAFactory.mark_state_inqueue(state,true);
                }
            }else  if (StateSearchMAFactory.isIntermediateNode(SearchMAState.getStateCoordinates(current_state))){
                int g_cost = SearchMAState.getGCost(current_state) + COST_NEXT_CELL;
                int h_cost = StateSearchMAFactory.getHeuristcOf(SearchMAState.getStateCoordinates(current_state));
                int f_cost = h_cost + g_cost;

                int[] pos_coordinates = Arrays.copyOf(SearchMAState.getStateCoordinates(current_state), SearchMAState.getStateCoordinates(current_state).length);
                ArrayDeque<int[][]> next_nodes = StateSearchMAFactory.expandIntermediateState(pos_coordinates, g_cost, f_cost);

                for (int[][] state : next_nodes) {
                    int[] pos = SearchMAState.getStateCoordinates(state);
                    if (StateSearchMAFactory.isStandardNode(pos)) {
                        int neighbour_gcost = SearchMAState.getGCost(state);
                        int[] next_time_step_state = SearchMAState.getStateCoordinates(state);
                        if (!StateSearchMAFactory.isInClosedSet(state)){
                            StateSearchMAFactory.updateCameFromPrevCell(standard_came_from_intermediate, state, current_state);
                            int[] intermadiate_state = standard_came_from_intermediate.get(pos);
                            int[] intermadiate_state2 = intermadiate_state;

                            while(intermediate_came_from_intermediate.containsKey(intermadiate_state)){
                                intermadiate_state2 = intermadiate_state;
                                intermadiate_state = intermediate_came_from_intermediate.get(intermadiate_state);
                                //intermediate_came_from_intermediate.remove(intermadiate_state2,intermadiate_state);
                            }
                            prev_standard_state = intermediate_came_from_standard.get(intermadiate_state);
                            //intermediate_came_from_standard.remove(intermadiate_state, prev_standard_state);
                            StateSearchMAFactory.updateCameFromPrevCell(came_from, state, prev_standard_state);
                                 //how to prune
                            frontier.add(state);
                                //StateSearchMAFactory.putCostSoFar(state);
                                //StateSearchMAFactory.mark_state_inqueue(state,true);
                        }
                    }else {
                         StateSearchMAFactory.updateCameFromPrevCell(intermediate_came_from_intermediate, state, current_state);
                        //how to store moves
                        // and heuristics
                        frontier.add(state);
                    }
                }
            }
        }
        //if the goal is not found return the empty path
        path.clear();
        this.path = path;
    }
}





