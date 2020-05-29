package org.agents.searchengine.normal;

import org.agents.markings.Coordinates;

import java.util.Arrays;

public enum SearchSAStateNormal {
   // final static int ARRAYPOS = 0;
    //final static int ARRAYCOSTS = 1;
    ARRAYPOS,
    ARRAYCOSTS;

    enum Costs {
        COST_G,
        COST_F;
    };

    public static int[][] createDummyState() {
        int[][] dummy_state = new int[SearchSAStateNormal.values().length][];
        dummy_state[getLocationIndex()] = new  int[Coordinates.getLenght()];
        dummy_state[getCostsIndex()] = new  int[Costs.values().length];

        dummy_state[getLocationIndex()][0]=Integer.MAX_VALUE; //time
        dummy_state[getLocationIndex()][1]=Integer.MAX_VALUE; //row
        dummy_state[getLocationIndex()][2]=Integer.MAX_VALUE; //column

        dummy_state[getCostsIndex()][0]=Integer.MAX_VALUE;
        dummy_state[getCostsIndex()][1]=Integer.MAX_VALUE;

        return dummy_state;
    }

    public static int[][] createNew(int[] cell_coordinates, int total_gcost, int f_value){
        assert (cell_coordinates.length == Coordinates.getLenght());
        assert total_gcost > (-1);

        //an array of 2 arrays ,each one for: costs and coordiantes
        int [][] state = new int[SearchSAStateNormal.getPropsLenth()][];
        //state[getLocationIndex()] = new int[Coordinates.values().length];
        state[getLocationIndex()] = cell_coordinates; //will represent a tuple of the form :(time,y_coord,x_coord)

        state[getCostsIndex()] = new int[Costs.values().length];
        setCostG(state, total_gcost);
        setCostF(state, f_value);

        return state;
    }

    private static int getPropsLenth(){
        return 2;//SearchState.values().length
    }

    private static int getLocationIndex(){
        return SearchSAStateNormal.ARRAYPOS.ordinal();
    }

    private static int getCostsIndex(){
        return SearchSAStateNormal.ARRAYCOSTS.ordinal();
    }

    private static void setCostG(int[][] state, int total_gcost){
        state[getCostsIndex()][Costs.COST_G.ordinal()] = total_gcost;
    }

    private static void setCostF(int[][] state, int f_value){
        state[getCostsIndex()][Costs.COST_F.ordinal()] = f_value;
    }

    public static int[] getStateCoordinates(int[][] state){
        return state[ARRAYPOS.ordinal()];
    }

    public static int  getYCoordinate(int[][] state){
        return Coordinates.getRow(state[ARRAYPOS.ordinal()]);
    }

    public static int  getXCoordinate(int[][] state){
        return Coordinates.getCol(state[ARRAYPOS.ordinal()]);
    }

    public static int  getTimeStep(int[][] state){
        return Coordinates.getTime(state[ARRAYPOS.ordinal()]);
    }

    public static int getGCost(int[][] state) {
        return state[getCostsIndex()][Costs.COST_G.ordinal()];
    }

    public static int getFCost(int[][] state) {
        return state[getCostsIndex()][Costs.COST_F.ordinal()];
    }

    public static int getPositionHashed(int[][] state){
        return Arrays.hashCode(getStateCoordinates(state));
    }

}
