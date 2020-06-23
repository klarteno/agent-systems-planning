package org.agents;

import org.agents.markings.Coordinates;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

public final class Utils {
    public static void logStartForFile(String fileName)throws IOException {
        String str = "Hello runOperatorDecomposition";
        BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
        writer.write(str);
        writer.newLine();

        writer.close();
    }


    public static void logAppendToFile(String fileName, int[][] current_state , int size) throws IOException {
        BufferedWriter writer = new BufferedWriter(new FileWriter(fileName, true));
        writer.append(' ');
        writer.newLine();

        String current_state_0 = Arrays.toString(current_state[0]);
        String current_state_1 = Arrays.toString(current_state[1]);
        writer.append("current_state[0]: ");
        writer.append(current_state_0);
        writer.append(' ');
        writer.append("current_state[1]: ");
        writer.append(current_state_1);
        writer.append(" frontier_size: ");
        writer.append(Integer.toString(size));
        writer.newLine();

        writer.close();
    }

    ////////////////example
    /*boolean start_to_add_to_del2 = Utils.isTheRightCoordinatesToLog(pos_coordinates, Coordinates.getTime(0,pos_coordinates),1,7,3,10);
                if (start_to_add_to_del2){
        boolean temp = start_to_add_to_del2;
    }*/
    ////////////////////
    public static boolean isTheRightCoordinatesToLog(int[] pos_coordinates, int time0, int row_0, int col_0, int row_1, int col_1) {
        boolean start_to_add_to_del = false;
        if (        Coordinates.getTime(0, pos_coordinates) == time0
                &&  Coordinates.getRow(0, pos_coordinates) == row_0
                &&  Coordinates.getCol(0, pos_coordinates) == col_0
                &&  Coordinates.getRow(1, pos_coordinates) == row_1
                &&  Coordinates.getCol(1, pos_coordinates) == col_1) {
            start_to_add_to_del = true;
            //standard_states_expanded_to_del.add(pos_coordinates);
        }
        return start_to_add_to_del;
    }


    public static void logAppendToFile(String logFileName, String str_message) throws IOException {
        BufferedWriter writer = new BufferedWriter(new FileWriter(logFileName, true));
        writer.append(' ');
        writer.newLine();

        writer.append(str_message);
        writer.newLine();

        writer.close();

    }

    public static int min(int[] array) {
        int min = array[0];

        for (int value : array) {
            if (value < min) {
                min = value;
            }
        }
        return min;
    }

    public static int min2(int[] array) {
        Arrays.sort(array);
        int min = array[0];
        int max = array[array.length-1];

        return min;
    }

    public static int minIndexOf(int[] array) {
        int min = array[0];
        int index = 0;

        for(int i = 0; i < array.length; i++ ) {
            if(array[i] < min) {
                min = array[i];
                index = i;
            }
        }
        return index;
    }

    public static int minIndexOf(int[] array, Integer[] from_idexes) {
        int min = array[from_idexes[0]];
        int index = 0;

        for (Integer from_inx : from_idexes) {
            if (array[from_inx] < min) {
                min = array[from_inx];
                index = from_inx;
            }
        }
        return index;
    }

    public static int maxIndexOf(int[] array) {
        int max = array[0];
        int index = 0;

        for(int i = 0; i < array.length; i++ ) {
            if(array[i] > max) {
                max = array[i];
                index = i;
            }
        }
        return index;
    }

    public static int[] getOrderedColectionOf(ArrayList<Integer> array){
        int[] array_ordered = new int[array.size()];

        int index = 0;
        for (Integer ag_id:array)
            array_ordered[index++] = ag_id;

        return array_ordered;
    }



    public static int[] concatanateArr(int[] agts_state, int[] boxes_pos) {
        int[] group_pos = Arrays.copyOf(agts_state,agts_state.length + boxes_pos.length);
        int _index = 0;
        for (int i = agts_state.length; i < group_pos.length; i++) {
            group_pos[i] = boxes_pos[_index++];
        }
        return group_pos;
    }


}
