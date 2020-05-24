package org.agents.searchengine;

import org.agents.Agent;
import org.agents.Box;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;

import java.util.*;

public class SearchEngineSA {
    private static final int COST_NEXT_CELL = 1;

    private ArrayDeque<int[]> path;
    private static PriorityQueue<int[][]> frontier;
    private final ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;

    public SearchEngineSA(ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules){
         this.conflict_avoidance_checking_rules =  conflictAvoidanceCheckingRules;
        //move to parent class and subclass??
        //make second option for comparator
        frontier = new PriorityQueue<int[][]>(5, Comparator.comparingInt(SearchSAState::getFCost));
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

    //cost_time(s) is total cost of the path until now
    public static int getConsistentHeuristic(int cost_time, int[] cell_coordinates, int[] goal_coordinates){
        int time_left = StateSearchSAFactory.getDeadlineTimeConstraint() - cost_time;
        int[] state_last_deadline = StateSearchSAFactory.getDeadlineCoord();
        if (time_left <= 0){
            return getHeuristic(cell_coordinates, goal_coordinates);
        } else{
            return time_left + getHeuristic(state_last_deadline, goal_coordinates);
        }
    }

    public void runAstar(Agent agent){
        assert agent != null;

        this.runAstar(agent.getColor(), agent.getCoordinates(), agent.getGoalPosition());
    }

    public void runAstar(Box box){
        int y_pos = box.getRowPosition();
        int x_pos = box.getColumnPosition();
        int time_pos = box.getTimeStep();

        this.runAstar(box.getColor(),  box.getCoordinates(), box.getGoalPosition());
    }

     private void runAstar(int color_movable, int[] start_coordinates, int[] goal_coordinates){
        frontier.clear();
        StateSearchSAFactory.createCostSoFar();
        StateSearchSAFactory.createClosedSet();

        int total_gcost = getHeuristic(start_coordinates, goal_coordinates);
        int heur2 = getHeuristic(start_coordinates, goal_coordinates);
        StateSearchSAFactory.setDeadlineConstraint(goal_coordinates, total_gcost, heur2);
        int total_gcost3 = 0;
        int heur4 = 0;
        // StateSearchSAFactory.setDeadlineConstraint(start_coordinates, total_gcost3, heur4); it looks no good to be earlier tnan goal
        ArrayDeque<int[]> path = new ArrayDeque<int[]>();
        int path_index = 0;
        int time_step = 0;
        //unused for output from algorithm, delete it when make bench mark
        HashMap<int[],int[]> came_from = new HashMap<>();
        HashMap<Integer, Stack<int[]>> paths = new HashMap<>();

        int[][] next_state = StateSearchSAFactory.createState(start_coordinates, 0, goal_coordinates);
        frontier.add(next_state);
        StateSearchSAFactory.putCostSoFar(next_state);
        StateSearchSAFactory.mark_state_inqueue(next_state,true);


        //init state with dummy variables
        int[][] current_state = StateSearchSAFactory.createDummyState();
        int[][] previouse_state = null;
        StateSearchSAFactory.updateCameFromPrevCell(came_from, next_state, next_state);

         int cost_time = 0;
        //unused for output from algorithm, delete it when make bench mark
        ArrayList<int[]> prev_cell_neighbours = new ArrayList<>();
        prev_cell_neighbours.add(new int[]{Integer.MAX_VALUE,Integer.MAX_VALUE});

        while(!frontier.isEmpty()){
            previouse_state = current_state;

            current_state = frontier.poll();
            assert current_state.length == 2;
            if (StateSearchSAFactory.isInHeap(current_state)){
            StateSearchSAFactory.mark_state_inqueue(current_state,false);
            path.push(SearchSAState.getStateCoordinates(current_state));

            if (StateSearchSAFactory.isGoal(SearchSAState.getStateCoordinates(current_state), goal_coordinates)){
                this.path = path;
                return;
            }
            StateSearchSAFactory.addToClosedSet(current_state);

            time_step = SearchSAState.getTimeStep(current_state);
            ArrayDeque<int[]> neighbours =   this.conflict_avoidance_checking_rules.getFreeNeighbours(SearchSAState.getStateCoordinates(current_state), color_movable, time_step, StateSearchSAFactory.getDeadlineTimeConstraint());
            prev_cell_neighbours.clear();//needed to clear it because this how this data structure works

            int neighbour_gcost = SearchSAState.getGCost(current_state) + COST_NEXT_CELL;

            boolean isFound = false;
            for(int[] cell_neighbour: neighbours){
                //assert cell_neighbour.length;
                if (!Arrays.equals(SearchSAState.getStateCoordinates(previouse_state), cell_neighbour)){
                    prev_cell_neighbours.add(cell_neighbour);
                }
                if (!StateSearchSAFactory.isInClosedSet(cell_neighbour, neighbour_gcost)){

                if(!StateSearchSAFactory.isInCostSoFar(cell_neighbour)){
                    next_state = StateSearchSAFactory.createState(cell_neighbour, neighbour_gcost, goal_coordinates);
                    frontier.add(next_state);
                    StateSearchSAFactory.putCostSoFar(next_state);
                    StateSearchSAFactory.mark_state_inqueue(next_state,true);
                }else {                       //this is an old node, uniform cost applies now
                    int[] next_state_costs = StateSearchSAFactory.getCostSoFar(cell_neighbour);
                    if (neighbour_gcost <= next_state_costs[StateSearchSAFactory.G_COST]){
                        int cost_difference = next_state_costs[StateSearchSAFactory.G_COST] - neighbour_gcost;
                        int f_value = next_state_costs[StateSearchSAFactory.F_COST] - cost_difference;
                        next_state = StateSearchSAFactory.createState(cell_neighbour, neighbour_gcost, f_value);
                        frontier.add(next_state);
                        StateSearchSAFactory.putCostSoFar(next_state);
                        StateSearchSAFactory.mark_state_inqueue(next_state,true);
                        StateSearchSAFactory.updateCameFromPrevCell(came_from, current_state, previouse_state);

                    }else {
                        neighbour_gcost = next_state_costs[StateSearchSAFactory.G_COST];
                    }
                }
                }
                cost_time = neighbour_gcost;
            }
        }
    }
        //if the goal is not found return the empty path
        path.clear();
        this.path = path;
    }


}





