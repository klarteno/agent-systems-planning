package org.agents.searchengine;

import org.agents.MapFixedObjects;

import java.util.*;

public class SearchEngine {
    private static final int COST_NEXT_CELL = 1;
    private ArrayDeque<int[]> path;

    public SearchEngine( )
    {
    }

    public ArrayDeque<int[]> getPath(){
        assert this.path.size() > 0;
        return path;
    }

    public int getPathCost(){
        assert this.path.size() > 0;
        return path.size();
    }

    public static int getHeuristic(int[] cell_coordinates1, int[] cell_coordinates2){
        return Math.abs(cell_coordinates1[0] - cell_coordinates2[0]) + Math.abs(cell_coordinates1[1] - cell_coordinates2[1]);
    }

    public void runAstar(MapFixedObjects mapFixedObjects, int number_mark, int color_movable, int[] start_coordinates_movable, int[] goal_coordinates){
        //move to parent class and subclass??
        PriorityQueue<int[][]> frontier = new PriorityQueue<int[][]>(5, Comparator.comparingInt(a -> a[SearchState.ARRAYCOSTS][SearchState.Costs.COST_F.ordinal()]));

        //HashMap<Integer, Integer> cost_so_far = new HashMap<>();
        //check later if should be encapsulated in a staic class like SearchState.java
        int [][] cost_so_far = new int [MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL];
        ArrayDeque<int[]> path = new ArrayDeque<int[]>();
        int path_index = 0;
        int time_step;
        //unused for output from algorithm, delete it when make bench mark
        HashMap<int[],int[]> came_from = new HashMap<>();
        HashMap<Integer, Stack<int[]>> paths = new HashMap<>();

        //int[][] next_state = State.createState(start_coordinates_movable, 0, goal_coordinates);
        frontier.add( SearchState.createState(start_coordinates_movable, 0, goal_coordinates) );
        SearchState.updateCost(cost_so_far);

        SearchState.updateCameFromPrevCell(came_from,null);

        //init state with dummy variables
        int[][] current_state = SearchState.createDummyState();
        int[][] previouse_state = null;

        //unused for output from algorithm, delete it when make bench mark
        ArrayList<int[]> prev_cell_neighbours = new ArrayList<>();
        prev_cell_neighbours.add(new int[]{Integer.MAX_VALUE,Integer.MAX_VALUE});

        while(!frontier.isEmpty()){
            previouse_state = current_state;
            current_state = frontier.poll();
            assert current_state.length == 2;
            path.push(current_state[SearchState.ARRAYPOS]);

            if (SearchState.isGoal(current_state[SearchState.ARRAYPOS], goal_coordinates)){
                this.path = path;
                return;
                //break;
            }

            time_step = SearchState.getGCost();//assumed that g_cost increases as the time_step , otherwise make a new field for time_step
            ArrayDeque<int[]> neighbours = mapFixedObjects.getFreeNeighbours( current_state[SearchState.ARRAYPOS], number_mark, color_movable, time_step);
            prev_cell_neighbours.clear();//needed to clear it because this how this data structure works
            // int new_gcost =  cost_so_far.get(Arrays.hashCode(current_state[SearchState.ARRAYPOS])) + this.COST_NEXT_CELL;
            int new_gcost =  cost_so_far[SearchState.getYCoordinate()][SearchState.getXCoordinate()] + COST_NEXT_CELL;
            boolean isFound = false;
            for(int[] cell_neighbour: neighbours){
                if (!Arrays.equals(previouse_state[SearchState.ARRAYPOS], cell_neighbour)){
                    prev_cell_neighbours.add(cell_neighbour);

                isFound = false;
               // if (cost_so_far.containsKey(Arrays.hashCode(cell_neighbour)) && new_gcost > cost_so_far.get(key))
                int cost_stored = cost_so_far[cell_neighbour[0]][cell_neighbour[1]];
                if (cost_stored > 0 && new_gcost > cost_stored) {
                    isFound = true;
                }
                if (!isFound){
                   // cost_so_far.put(key, new_gcost);
                    cost_so_far[cell_neighbour[0]][cell_neighbour[1]] = new_gcost;
                    frontier.add(SearchState.createState(cell_neighbour, new_gcost, goal_coordinates));
                    SearchState.updateCost(cost_so_far);
                    SearchState.updateCameFromPrevCell(came_from,current_state[SearchState.ARRAYPOS]);
                }
                }
            }
        }
        //if the goal is not found return the empty path
        path.clear();

         this.path = path;
    }

}





