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
    private static ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;
    static int searched_mark_id = 0;

    public SearchEngineSA(ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules){
         conflict_avoidance_checking_rules = conflictAvoidanceCheckingRules;
        //move to parent class and subclass??
        //make second option for comparator
        frontier = new PriorityQueue<int[][]>(5, Comparator.comparingInt(SearchSAState::getFCost));
    }

    public ArrayDeque<int[]> getPath(){
        /*  remove the cells positions  that do not lead to goal

            by keeping previouse time step for previouse state
            then pop out the cells that do not have a diffrence of g of 1 when the time step decresses
            accumulate the time from poped cells and added to the previouse branching cells
        */
        ArrayDeque<int[]> path_valid = new ArrayDeque<>();
        int path_explored_size = this.path.size();
        if (path_explored_size > 0){

            int[] prev__ = this.path.pop();
            int time_prev ;
            path_valid.add(prev__);

            int[] next__ ;
            int time_next;
            int time_steps_skipped = 0;
            while (!this.path.isEmpty())  {
                next__ = this.path.pop();
                time_prev = Coordinates.getTime(prev__);
                time_next = Coordinates.getTime(next__);
                if( time_prev - time_next == 1 && Coordinates.areNeighbours(prev__, next__) ){
                    path_valid.add(next__);
                    prev__ = next__;
                }else if( Coordinates.areNeighbours(prev__, next__) ){
                    if(time_steps_skipped > 0){
                        Coordinates.setTime(next__, Coordinates.getTime(next__) + time_steps_skipped);
                        path_valid.add(next__);

                        prev__ = next__;
                        time_steps_skipped = 0;
                    }
                }else{
                    time_steps_skipped +=1;
                }
            }

            assert path_valid.size() > 0;
        }

        return path_valid;
    }

    public int getPathCost(){
        assert this.path.size() > 0;
        return path.size();
    }

    public boolean isPathFound() {
        return this.path.size() > 0;
    }

    static int getHeuristic(int[] cell_coordinate, int[] goal_coordinate) {
        return conflict_avoidance_checking_rules.getHeuristicOf(searched_mark_id, cell_coordinate, goal_coordinate);
    }

    //gets an aproximation for a cell when expands in a time step that oversteps the time_deadline_constraint
    static int getCostCoordinate(int[] cell_coordinate, int new_g_cost) {
        return conflict_avoidance_checking_rules.getCostTimeCoordinate(searched_mark_id, cell_coordinate);
    }

    public void runAstar(Agent agent){
        assert agent != null;

        searched_mark_id = agent.getNumberMark();
        this.runAstar(agent.getNumberMark(), agent.getCoordinates(), agent.getGoalPosition());
    }

    public void runAstar(Box box){
        int y_pos = box.getRowPosition();
        int x_pos = box.getColumnPosition();
        int time_pos = box.getTimeStep();

        searched_mark_id = box.getLetterMark();
        this.runAstar(box.getLetterMark(),  box.getCoordinates(), box.getGoalPosition());
    }

     private void runAstar(int mark_id, int[] start_coordinates, int[] goal_coordinates){
        frontier.clear();
        StateSearchSAFactory.createCostSoFar();
        StateSearchSAFactory.createClosedSet();

        int total_gcost = getHeuristic(start_coordinates, goal_coordinates);
        int heur2 = getHeuristic(start_coordinates, goal_coordinates);
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

            int time_step_path = Coordinates.getTime(SearchSAState.getStateCoordinates(previouse_state));
            int time_step_exapanded = Coordinates.getTime(SearchSAState.getStateCoordinates(current_state));
            if (time_step_exapanded - time_step_path != 1)
                Coordinates.setTime(SearchSAState.getStateCoordinates(current_state), time_step_path + 1);

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
            ArrayDeque<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursSA(SearchSAState.getStateCoordinates(current_state), mark_id);
            prev_cell_neighbours.clear();//needed to clear it because this how this data structure works

            int neighbour_gcost;

            boolean isFound = false;
            for(int[] cell_neighbour: neighbours){
                //assert cell_neighbour.length;
                if (!Arrays.equals(SearchSAState.getStateCoordinates(previouse_state), cell_neighbour)){
                    prev_cell_neighbours.add(cell_neighbour);
                }

                int[] curr = SearchSAState.getStateCoordinates(current_state);
                if(Coordinates.getRow(curr) == Coordinates.getRow(cell_neighbour) &&
                        Coordinates.getCol(curr) == Coordinates.getCol(cell_neighbour )&&
                        Coordinates.getTime(curr) <= Coordinates.getTime(cell_neighbour)){
                    neighbour_gcost = SearchSAState.getGCost(current_state);

                }else {
                     neighbour_gcost = SearchSAState.getGCost(current_state) + COST_NEXT_CELL;
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





