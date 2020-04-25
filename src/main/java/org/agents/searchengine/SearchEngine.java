package org.agents.searchengine;

import org.agents.MapFixedObjects;

import java.util.*;

public class SearchEngine {
    private final int COST_NEXT_CELL = 1;

    public SearchEngine( )
    {
    }

    public static int getHeuristic(int[] cell_coordinates1, int[] cell_coordinates2){
        return Math.abs(cell_coordinates1[0] - cell_coordinates2[0]) + Math.abs(cell_coordinates1[1] - cell_coordinates2[1]);
    }

    public Stack<int[]> runAstar(MapFixedObjects mapFixedObjects, int color_movable, int[] start_coordinates_movable, int[] goal_coordinates){
        PriorityQueue<int[][]> frontier = new PriorityQueue<int[][]>(5, Comparator.comparingInt(a -> a[State.ARRAYCOSTS][State.Costs.COST_F.ordinal()]));

        //int[][] next_state = State.createState(start_coordinates_movable, 0, goal_coordinates);
        frontier.add( State.createState(start_coordinates_movable, 0, goal_coordinates) );

        HashMap<int[],int[]> came_from = new HashMap<>();
        HashMap<Integer, Stack<int[]>> paths = new HashMap<>();

        Stack<int[]> path = new Stack<int[]>();
        int path_index = 0;
        HashMap<Integer, Integer> cost_so_far = new HashMap<>();

        State.updateCameFromPrevCell(came_from,null);
        State.updateCost(cost_so_far);

        //init state with dummy variables
        int[][] current_state = State.createDummyState();
        int[][] previouse_state = null;

        ArrayList<int[]> prev_cell_neighbours = new ArrayList<>();
        prev_cell_neighbours.add(new int[]{Integer.MAX_VALUE,Integer.MAX_VALUE});

        while(!frontier.isEmpty()){
            previouse_state = current_state;
            current_state = frontier.poll();
            assert current_state.length == 2;

            if (State.isGoal(current_state[State.ARRAYPOS], goal_coordinates)){
                path.push(current_state[State.ARRAYPOS]);
                break;
            }

            path.push(current_state[State.ARRAYPOS]);
            Stack<int[]> neighbours = mapFixedObjects.getNeighbours(current_state[State.ARRAYPOS], color_movable);

            prev_cell_neighbours.clear();//needed because this how this data structure works
            int new_gcost;
            boolean isFound = false;
            for(int[] cell_neighbour: neighbours){
                if (!Arrays.equals(previouse_state[State.ARRAYPOS], cell_neighbour)){
                    prev_cell_neighbours.add(cell_neighbour);

                new_gcost =  cost_so_far.get(Arrays.hashCode(current_state[State.ARRAYPOS])) + this.COST_NEXT_CELL;
                isFound = false;
                int key = Arrays.hashCode(cell_neighbour);
                if (cost_so_far.containsKey(key) && new_gcost > cost_so_far.get(key)){
                    isFound = true;
                }
                if (!isFound){
                    cost_so_far.put(key, new_gcost);
                    frontier.add(State.createState(cell_neighbour, new_gcost, goal_coordinates));
                    State.updateCameFromPrevCell(came_from,current_state[State.ARRAYPOS]);
                }
                }
            }
        }
        return path;
    }

}





