package org.agents.searchengine;

import java.util.Arrays;
import java.util.HashMap;

class  State{
    final static int ARRAYPOS = 0;
    final static int ARRAYCOSTS = 1;

    public static int[][] createDummyState() {
        int[][] dummy_state = new int[2][2];
        dummy_state[0][0]=Integer.MAX_VALUE;
        dummy_state[0][1]=Integer.MAX_VALUE;
        dummy_state[1][0]=Integer.MAX_VALUE;
        dummy_state[1][1]=Integer.MAX_VALUE;

        return dummy_state;
    }

    enum Coordinates {
        POS_Y,
        POS_X;
    };

    enum Costs {
        COST_G,
        COST_F;
    };

    private static int [][] state ;

    public static int[][] createState(int[] cell_coordinates, int total_gcost, int[] goal_coordinates){
        assert (goal_coordinates.length == 2);

        //an array of 2 ,each one for: costs and coordiantes
        state = new int[2][Costs.values().length];

        state[State.ARRAYPOS][Coordinates.POS_Y.ordinal()] = cell_coordinates[0];
        state[State.ARRAYPOS][Coordinates.POS_X.ordinal()] = cell_coordinates[1];

        state[State.ARRAYCOSTS][Costs.COST_G.ordinal()] = total_gcost;
        state[State.ARRAYCOSTS][Costs.COST_F.ordinal()] = SearchEngine.getHeuristic(cell_coordinates, goal_coordinates) + total_gcost;

        return state;
    }

    public static int getPositionHashed(){
        return Arrays.hashCode(state[State.ARRAYPOS]);
    }

    public static void updateCost(HashMap<Integer, Integer> cost_so_far) {
        cost_so_far.put(Arrays.hashCode(state[State.ARRAYPOS]), state[ARRAYCOSTS][Costs.COST_G.ordinal()]);
    }

    public static void updateCameFromPrevCell(HashMap<int[],int[]> came_from,int[] previouse_coordinates) {
        came_from.put(State.getCellCoordinates(), previouse_coordinates);
    }

    public static int[] getCellCoordinates(){
        return state[State.ARRAYPOS];
    }

    public static boolean isGoal(int[] cell_coordinates, int[] goal_coordinates){
        return Arrays.equals(cell_coordinates, goal_coordinates);
    }
}
