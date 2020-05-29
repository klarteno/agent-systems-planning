package org.agents.searchengine.normal;

import org.agents.Agent;
import org.agents.Box;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;

import java.util.*;

public class SearchEngineSANormal {
    private static final int COST_NEXT_CELL = 1;

    private ArrayDeque<int[]> path;
    private static PriorityQueue<int[][]> frontier;
    private static ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;
    static int searched_mark_id = 0;

    public SearchEngineSANormal(ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules){
         conflict_avoidance_checking_rules = conflictAvoidanceCheckingRules;
        //move to parent class and subclass??
        //make second option for comparator
        frontier = new PriorityQueue<int[][]>(5, Comparator.comparingInt(SearchSAStateNormal::getFCost));
    }

    public ArrayDeque<int[]> getPath(){
        //assert this.path.size() > 0;
        return path;
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
        StateSearchSAFactoryNormal.createCostSoFar();
        StateSearchSAFactoryNormal.createClosedSet();

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

        int[][] next_state = StateSearchSAFactoryNormal.createState(start_coordinates, 0, goal_coordinates);
        frontier.add(next_state);
        StateSearchSAFactoryNormal.putCostSoFar(next_state);
        StateSearchSAFactoryNormal.mark_state_inqueue(next_state,true);


        //init state with dummy variables
        int[][] current_state = StateSearchSAFactoryNormal.createDummyState();
        int[][] previouse_state = null;
        StateSearchSAFactoryNormal.updateCameFromPrevCell(came_from, next_state, next_state);

         int cost_time = 0;
        //unused for output from algorithm, delete it when make bench mark
        ArrayList<int[]> prev_cell_neighbours = new ArrayList<>();
        prev_cell_neighbours.add(new int[]{Integer.MAX_VALUE,Integer.MAX_VALUE});

        while(!frontier.isEmpty()){
            previouse_state = current_state;

            current_state = frontier.poll();
            assert current_state.length == 2;
            if (StateSearchSAFactoryNormal.isInHeap(current_state)){
            StateSearchSAFactoryNormal.mark_state_inqueue(current_state,false);
            path.push(SearchSAStateNormal.getStateCoordinates(current_state));

            if (StateSearchSAFactoryNormal.isGoal(SearchSAStateNormal.getStateCoordinates(current_state), goal_coordinates)){
                this.path = path;
                return;
            }
            StateSearchSAFactoryNormal.addToClosedSet(current_state);

            time_step = SearchSAStateNormal.getTimeStep(current_state);
            ArrayDeque<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursSA(SearchSAStateNormal.getStateCoordinates(current_state), mark_id);
            prev_cell_neighbours.clear();//needed to clear it because this how this data structure works

            int neighbour_gcost = SearchSAStateNormal.getGCost(current_state) + COST_NEXT_CELL;;

            boolean isFound = false;
            for(int[] cell_neighbour: neighbours){
                //assert cell_neighbour.length;
                if (!Arrays.equals(SearchSAStateNormal.getStateCoordinates(previouse_state), cell_neighbour)){
                    prev_cell_neighbours.add(cell_neighbour);
                }

                if (!StateSearchSAFactoryNormal.isInClosedSet(cell_neighbour, neighbour_gcost)){

                if(!StateSearchSAFactoryNormal.isInCostSoFar(cell_neighbour)){
                    next_state = StateSearchSAFactoryNormal.createState(cell_neighbour, neighbour_gcost, goal_coordinates);
                    frontier.add(next_state);
                    StateSearchSAFactoryNormal.putCostSoFar(next_state);
                    StateSearchSAFactoryNormal.mark_state_inqueue(next_state,true);
                }else {                       //this is an old node, uniform cost applies now
                    int[] next_state_costs = StateSearchSAFactoryNormal.getCostSoFar(cell_neighbour);
                    if (neighbour_gcost <= next_state_costs[StateSearchSAFactoryNormal.G_COST]){
                        int cost_difference = next_state_costs[StateSearchSAFactoryNormal.G_COST] - neighbour_gcost;
                        int f_value = next_state_costs[StateSearchSAFactoryNormal.F_COST] - cost_difference;
                        next_state = StateSearchSAFactoryNormal.createState(cell_neighbour, neighbour_gcost, f_value);
                        frontier.add(next_state);
                        StateSearchSAFactoryNormal.putCostSoFar(next_state);
                        StateSearchSAFactoryNormal.mark_state_inqueue(next_state,true);
                        StateSearchSAFactoryNormal.updateCameFromPrevCell(came_from, current_state, previouse_state);

                    }else {
                        neighbour_gcost = next_state_costs[StateSearchSAFactoryNormal.G_COST];
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





