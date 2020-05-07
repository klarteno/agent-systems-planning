package org.agents.searchengine;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.planning.SearchState;

import java.io.Serializable;
import java.util.*;

public class SearchEngine {
    private static final int COST_NEXT_CELL = 1;
    private ArrayDeque<int[]> path;
    private static PriorityQueue<int[][]> frontier;
    public SearchEngine(){
        //move to parent class and subclass??
        //make second option for comparator
        frontier = new PriorityQueue<int[][]>(5, Comparator.comparingInt(SearchState::getFCost));
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
        int time_left = StateSearchFactory.getDeadlineTimeConstraint() - cost_time;
        int[] state_last_deadline = StateSearchFactory.getDeadlineCoord();
        if (time_left <= 0){
            return getHeuristic(cell_coordinates, goal_coordinates);
        } else{
            return time_left + getHeuristic(state_last_deadline, goal_coordinates);
        }
    }
    public void runAstar(MapFixedObjects mapFixedObjects, int movable_id){
        Serializable obj = MapFixedObjects.getByMarkNo(movable_id);

    }

    public void runAstar(MapFixedObjects mapFixedObjects, Agent agent){
        this.runAstar(mapFixedObjects, agent.getNumberMark(), agent.getColor(),agent.getCoordinates(), agent.getGoalPosition());
    }

    public void runAstar(MapFixedObjects mapFixedObjects, Box box){
        int y_pos = box.getRowPosition();
        int x_pos = box.getColumnPosition();
        int time_pos = box.getTimeStep();

        this.runAstar(mapFixedObjects, box.getLetterMark(), box.getColor(),  box.getCoordinates(), box.getGoalPosition());
    }

    private void runAstar(MapFixedObjects mapFixedObjects, int number_mark, int color_movable, int[] start_coordinates, int[] goal_coordinates){
        frontier.clear();
        StateSearchFactory.createCostSoFar();
        StateSearchFactory.createClosedSet();

        ArrayDeque<int[]> path = new ArrayDeque<int[]>();
        int path_index = 0;
        int time_step = 0;
        //unused for output from algorithm, delete it when make bench mark
        HashMap<int[],int[]> came_from = new HashMap<>();
        HashMap<Integer, Stack<int[]>> paths = new HashMap<>();

        int[][] next_state = StateSearchFactory.createState(start_coordinates, 0, goal_coordinates);
        frontier.add(next_state);
        StateSearchFactory.putCostSoFar(next_state);
        StateSearchFactory.mark_state_inqueue(next_state,true);

        StateSearchFactory.updateCameFromPrevCell(came_from,next_state, null);

        //init state with dummy variables
        int[][] current_state = StateSearchFactory.createDummyState();
        int[][] previouse_state = null;
        int cost_time = 0;
        //unused for output from algorithm, delete it when make bench mark
        ArrayList<int[]> prev_cell_neighbours = new ArrayList<>();
        prev_cell_neighbours.add(new int[]{Integer.MAX_VALUE,Integer.MAX_VALUE});

        while(!frontier.isEmpty()){
            previouse_state = current_state;

            current_state = frontier.poll();
            assert current_state.length == 3;
            if (StateSearchFactory.isInHeap(current_state)){
            StateSearchFactory.mark_state_inqueue(current_state,false);
            path.push(SearchState.getCellCoordinates(current_state));

            if (StateSearchFactory.isGoal(SearchState.getCellCoordinates(current_state), goal_coordinates)){
                this.path = path;
                return;
                //break;
            }
            StateSearchFactory.addToClosedSet(current_state);

            time_step = SearchState.getTimeStep(current_state);
            ArrayDeque<int[]> neighbours = mapFixedObjects.getFreeNeighbours(SearchState.getCellCoordinates(current_state), number_mark, color_movable, time_step, StateSearchFactory.getDeadlineTimeConstraint());
            prev_cell_neighbours.clear();//needed to clear it because this how this data structure works

            int neighbour_gcost =  SearchState.getGCost(current_state) + COST_NEXT_CELL;

            boolean isFound = false;
            for(int[] cell_neighbour: neighbours){
                //assert cell_neighbour.length;
                if (!Arrays.equals(SearchState.getCellCoordinates(previouse_state), cell_neighbour)){
                    prev_cell_neighbours.add(cell_neighbour);
                }
                if (!StateSearchFactory.isInClosedSet(cell_neighbour)){

                if(!StateSearchFactory.isInCostSoFar(cell_neighbour)){
                    next_state = StateSearchFactory.createState(cell_neighbour, neighbour_gcost, goal_coordinates);
                    frontier.add(next_state);
                    StateSearchFactory.putCostSoFar(next_state);
                    StateSearchFactory.mark_state_inqueue(current_state,true);
                }else {                       //this is an old node, uniform cost applies now
                    int[] next_state_costs = StateSearchFactory.getCostSoFar(cell_neighbour);
                    if (neighbour_gcost <= next_state_costs[StateSearchFactory.G_COST]){
                        int cost_difference = next_state_costs[StateSearchFactory.G_COST] - neighbour_gcost;
                        int f_value = next_state_costs[StateSearchFactory.F_COST] - cost_difference;
                        next_state = StateSearchFactory.createState(cell_neighbour, neighbour_gcost, f_value, goal_coordinates);
                        frontier.add(next_state);
                        StateSearchFactory.putCostSoFar(next_state);
                        StateSearchFactory.mark_state_inqueue(current_state,true);
                        StateSearchFactory.updateCameFromPrevCell(came_from,current_state,previouse_state);

                    }else {
                        neighbour_gcost = next_state_costs[StateSearchFactory.G_COST];
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





