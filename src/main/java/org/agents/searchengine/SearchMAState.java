package org.agents.searchengine;

import org.agents.markings.Coordinates;

import java.util.ArrayDeque;
import java.util.Arrays;

public enum SearchMAState {
    ARRAYPOS,
    ARRAYCOSTS;


    enum Costs {
        COST_G,
        COST_F;
    };
    private static int[][] dummy_state ;

    static int[][] createDummyState(int number_movables) {
        //int[] dummy_state123 = new int[number_movables * Coordinates.getLenght() + SearchMAState.Costs.values().length];
        dummy_state = new int[SearchMAState.values().length][];

        dummy_state[getLocationIndex()] = new int[number_movables * Coordinates.getLenght()];
        dummy_state[getCostsIndex()] = new int[Costs.values().length];

        for (int i = 0; i < number_movables; i++) {
            Coordinates.setTime(i, dummy_state[getLocationIndex()], -1);
            Coordinates.setRow(i, dummy_state[getLocationIndex()], Integer.MAX_VALUE);
            Coordinates.setCol(i, dummy_state[getLocationIndex()], Integer.MAX_VALUE);
        }

        return dummy_state;
    }

    public static int[][] getDummyState(int number_of_movables) {
        return dummy_state;
    }


    static int[][] createStartState(ArrayDeque<int []> movables) {
        //int[] dummy_state123 = new int[number_movables * Coordinates.getLenght() + SearchMAState.Costs.values().length];
        int[][] dummy_state = new int[movables.size()][];

        dummy_state[getLocationIndex()] = new int[movables.size() * Coordinates.getLenght()];
        dummy_state[getCostsIndex()] = new int[Costs.values().length];
        int[] next;
        for (int i = 0; i < movables.size(); i++) {
            next = movables.poll();
            Coordinates.setTime(i, dummy_state[getLocationIndex()], 0);
            Coordinates.setRow(i, dummy_state[getLocationIndex()], Coordinates.getRow(0, next));
            Coordinates.setCol(i, dummy_state[getLocationIndex()], Coordinates.getCol(0, next));
        }

        return dummy_state;
    }


    public static int getTime(int index_pos, int[] pos){
        return Coordinates.getTime(index_pos, pos );
    }

    public static int getRow(int index_pos, int[] pos){
        return Coordinates.getRow(index_pos, pos );
    }

    public static int getColumn(int index_pos, int[] pos){ return Coordinates.getCol(index_pos, pos ); }

    //cell_coordinates contains the position for all the movables
    public static int[][] createNew(int[] cell_coordinates, int total_gcost, int f_value){
        //assert (cell_coordinates.length == 3);
        assert total_gcost > (-1);

        //an array concatanation of 2 arrays ,each one for: costs and one for all the movables coordiantes
        int [][] state = new int[SearchMAState.getPropsLenth()][];
        state[getLocationIndex()] = cell_coordinates; //will represent a tuple of the form :(time,y_coord,x_coord)

        state[getCostsIndex()] = new int[SearchMAState.Costs.values().length];
        setCostG(state, total_gcost);
        setCostF(state, f_value);

        return state;
    }

    public static int[] getStateUpdatedOf(int[][] prev_state, int coordinates_index, int[] cell_coordinates, int time_step, int total_gcost, int f_value){
        assert (cell_coordinates.length == Coordinates.getLenght());

        int[] copy_coordinates = Arrays.copyOf(prev_state[ARRAYPOS.ordinal()], prev_state[ARRAYPOS.ordinal()].length);
        for (int i = 0; i < prev_state[ARRAYPOS.ordinal()].length; i = i + Coordinates.getLenght()) {
            Coordinates.setTime(coordinates_index, copy_coordinates, Coordinates.getTime(i, cell_coordinates));
        }

        Coordinates.setRow(coordinates_index, copy_coordinates, Coordinates.getRow(0, cell_coordinates));
        Coordinates.setCol(coordinates_index, copy_coordinates, Coordinates.getCol(0, cell_coordinates));

        return copy_coordinates;
    }

    private static int getPropsLenth(){
        return 2;//SearchMAState.values().length
    }

    private static int getLocationIndex(){
        return SearchMAState.ARRAYPOS.ordinal();
    }

    private static int getCostsIndex(){
        return SearchMAState.ARRAYCOSTS.ordinal();
    }

    private static void setCostG(int[][] state, int total_gcost){
        state[getCostsIndex()][SearchMAState.Costs.COST_G.ordinal()] = total_gcost;
    }

    private static void setCostF(int[][] state, int f_value){
        state[getCostsIndex()][SearchMAState.Costs.COST_F.ordinal()] = f_value;
    }

    public static int[] getStateCoordinates(int[][] state){
        return state[ARRAYPOS.ordinal()];
    }

    public static int  getYCoordinate(int movable_index, int[][] state){
        return Coordinates.getRow(movable_index, state[ARRAYPOS.ordinal()]);
    }

    public static int  getXCoordinate(int movable_index, int[][] state){
        return Coordinates.getCol(movable_index, (state[ARRAYPOS.ordinal()]));
    }

    public static int  getTimeStep(int movable_index, int[][] state){
        return Coordinates.getTime(movable_index,state[ARRAYPOS.ordinal()]);
    }

    public static void setTimeStep(int[][] state, int time_step){
        int[] state__ = getStateCoordinates(state);
        int number_movables = state__.length / Coordinates.getLenght();
        for (int coordinate = 0; coordinate < number_movables ; coordinate++) {
            Coordinates.setTime(coordinate, state__, time_step);
        }
     }

    public static int getGCost(int[][] state) {
        return state[getCostsIndex()][SearchMAState.Costs.COST_G.ordinal()];
    }

    public static int getFCost(int[][] state) {
        return state[getCostsIndex()][SearchMAState.Costs.COST_F.ordinal()];
    }

    public static int getPositionHashed(int[][] state){
        return Arrays.hashCode(getStateCoordinates(state));
    }


}
