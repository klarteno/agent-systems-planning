package org.agents.searchengine;

 import org.agents.Agent;
 import org.agents.Box;
 import org.agents.markings.Coordinates;

 import java.util.*;

public class SearchEngineOD {
    private static final int COST_NEXT_CELL = 1;

    private ArrayDeque<int[]> path;
    private static PriorityQueue<int[][]> frontier;

    public SearchEngineOD(){
        //move to parent class and subclass??
        //make second option for comparator
        frontier = new PriorityQueue<int[][]>(5, Comparator.comparingInt(SearchMAState::getFCost));
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
        return Math.abs(Coordinates.getRow(0,cell_coordinates) - Coordinates.getRow(0,goal_coordinates)) + Math.abs(Coordinates.getCol(0,cell_coordinates) - Coordinates.getCol(0,goal_coordinates))  ;
    }

    public static int getHeuristic(int y, int x, int y_goal, int x_goal) {
        return Math.abs(y - y_goal) + Math.abs(x - x_goal)  ;
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
        frontier.clear();
        StateSearchMAFactory.setUpGoals(goal_coordinates);
        StateSearchMAFactory.createCostSoFar();
        StateSearchMAFactory.createClosedSet();
        StateSearchMAFactory.setUpConflictingPaths(conflicting_paths);

        ArrayDeque<int[]> path = new ArrayDeque<int[]>();
        int time_step = 0;
        //unused for output from algorithm, delete it when make bench mark
        HashMap<int[],int[]> came_from = new HashMap<>();

        int[][] next_state = StateSearchMAFactory.createStandardState(start_coordinates, 0);

        frontier.add(next_state);
        StateSearchMAFactory.putCostSoFar(next_state);
        StateSearchMAFactory.mark_state_inqueue(next_state,true);

        StateSearchMAFactory.updateCameFromPrevCell(came_from,next_state, null);

        //init state with dummy variables
        int[][] current_state = null; //StateSearchMAFactory.createDummyState()

        while(!frontier.isEmpty()){
            current_state = frontier.poll();
            assert current_state.length == 3;
            if (StateSearchMAFactory.isInHeap(current_state)){
                StateSearchMAFactory.mark_state_inqueue(current_state,false);

            if (StateSearchMAFactory.isStandardNode(SearchMAState.getStateCoordinates(current_state))){
                path.push(SearchMAState.getStateCoordinates(current_state));

                if (StateSearchMAFactory.isGoal(SearchMAState.getStateCoordinates(current_state))){
                    this.path = path;
                    return;
                    //break;
                }
                StateSearchMAFactory.addToClosedSet(current_state);
                int g_cost = SearchMAState.getGCost(current_state) + COST_NEXT_CELL;
                int f_cost = SearchMAState.getFCost(current_state) - COST_NEXT_CELL;
                int[] pos_coordinates = Arrays.copyOf(SearchMAState.getStateCoordinates(current_state), SearchMAState.getStateCoordinates(current_state).length);
                ArrayDeque<int[][]> next_intermediate_nodes = StateSearchMAFactory.expandStandardState(pos_coordinates, g_cost, f_cost);
                 //assert StateSearchMAFactory.isIntermediateNode(next_intermediate_nodes);
                while(!next_intermediate_nodes.isEmpty())
                    frontier.add(next_intermediate_nodes.pop());

            }else  if (StateSearchMAFactory.isIntermediateNode(SearchMAState.getStateCoordinates(current_state))){
                int g_cost = SearchMAState.getGCost(current_state) + COST_NEXT_CELL;
                int f_cost = SearchMAState.getFCost(current_state) - COST_NEXT_CELL;
                int[] pos_coordinates = Arrays.copyOf(SearchMAState.getStateCoordinates(current_state), SearchMAState.getStateCoordinates(current_state).length);
                ArrayDeque<int[][]> next_nodes = StateSearchMAFactory.expandIntermediateState(pos_coordinates, g_cost, f_cost);

                for (int[][] state : next_nodes) {
                    int[] pos = SearchMAState.getStateCoordinates(state);
                    if (StateSearchMAFactory.isStandardNode(pos)) {
                        int[][] next_node;
                        int neighbour_gcost = SearchMAState.getGCost(state);
                        int[] next_time_step_state = SearchMAState.getStateCoordinates(state);
                        if (!StateSearchMAFactory.isInClosedSet(next_time_step_state)){
                            if(!StateSearchMAFactory.isInCostSoFar(next_time_step_state)){
                                //how to prune
                                frontier.add(state);
                                StateSearchMAFactory.putCostSoFar(state);
                                StateSearchMAFactory.mark_state_inqueue(state,true);
                            }else{
                                frontier.add(state);
                                StateSearchMAFactory.putCostSoFar(state);
                                StateSearchMAFactory.mark_state_inqueue(state,true);
                            }
                        }
                    }
                    //how to store moves and heuristics
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





