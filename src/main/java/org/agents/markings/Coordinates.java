package org.agents.markings;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

public enum Coordinates {
    TIME,//includes waiting time in a cell location
    ROW,// coordinate Y
    COLUMN;//corrdinate X

    public static int[] createCoordinates() {
        return new int[Coordinates.getLenght()];
    }

    public static int[] createCoordinates(int time_step, int row_index, int col_index) {
        return new int[]{time_step, row_index, col_index};
    }

    public static int getTime(int[] pos) {
        return getTime(0, pos);
    }

    public static int getTime(int index_pos, int[] pos) {
        assert pos[index_pos * Coordinates.getLenght() + 2] >= 0;
        return pos[Coordinates.getLenght() * index_pos + Coordinates.TIME.ordinal()];
    }

    public static int getRow(int[] pos) {
        return getRow(0, pos);
    }

    public static int getRow(int index_pos, int[] pos) {
        return pos[Coordinates.values().length * index_pos + Coordinates.ROW.ordinal()];
    }

    public static int getCol(int[] pos) {
        return getCol(0, pos);
    }

    public static int getCol(int index_pos, int[] pos) {
        assert pos[index_pos * Coordinates.getLenght() + 2] >= 0;
        return pos[Coordinates.values().length * index_pos + Coordinates.COLUMN.ordinal()];
    }

    public static int getLenght() {
        return Coordinates.values().length;
    }


    public static void setTime(int[] pos, int value) {
        setTime(0, pos, value);
    }

    public static void setTime(int index_pos, int[] pos, int value) {
        assert pos[index_pos * Coordinates.getLenght() + 2] >= 0;
        pos[Coordinates.values().length * index_pos + Coordinates.TIME.ordinal()] = value;
    }

    public static void setRow(int[] pos, int value) {
        setRow(0, pos, value);
    }

    public static void setRow(int index_pos, int[] pos, int value) {
        assert pos[index_pos * Coordinates.getLenght() + 2] >= -1;
        pos[Coordinates.values().length * index_pos + Coordinates.ROW.ordinal()] = value;
    }

    public static void setCol(int[] pos, int value) {
        setCol(0, pos, value);
    }

    public static void setCol(int index_pos, int[] pos, int value) {
        assert pos[index_pos * Coordinates.getLenght()] >= -1;
        pos[Coordinates.values().length * index_pos + Coordinates.COLUMN.ordinal()] = value;
    }

    public static int[] getCoordinatesAt(int index, int[] pos_coordinates) {
        int[] coord_pos = new int[0];
        for (int coordinate = 0; coordinate < pos_coordinates.length / Coordinates.getLenght(); coordinate = coordinate + 1) {
            if (index == coordinate) {
                coord_pos = new int[Coordinates.getLenght()];
                Coordinates.setTime(coord_pos, Coordinates.getTime(coordinate, pos_coordinates));
                Coordinates.setRow(coord_pos, Coordinates.getRow(coordinate, pos_coordinates));
                Coordinates.setCol(coord_pos, Coordinates.getCol(coordinate, pos_coordinates));
            }
        }
        return coord_pos;
    }

    public static void setCoordinateAtIndex(int index, int[] coordinates, int[] coordinate) {
        if (!isValid(coordinates) || !isValid(coordinate)) {
            throw new IndexOutOfBoundsException("coordinate not valid");
        }
        for (int i = 0; i < coordinates.length / Coordinates.getLenght(); i = i + 1) {
            if (i == index) {
                Coordinates.setTime(index, coordinates, Coordinates.getTime(coordinate));
                Coordinates.setRow(index, coordinates, Coordinates.getRow(coordinate));
                Coordinates.setCol(index, coordinates, Coordinates.getCol(coordinate));
            }
        }
    }

    public static boolean isValid(int[] pos_coordinates) {
        //if it has a lenght multiple of of the coordinates lenght is valid
        return (pos_coordinates.length % Coordinates.getLenght() == 0);
    }

    //it compares by coordinates and does not consider time steps
    public static boolean areNeighbours(int[] prev__, int[] next__) {
        int prev__row = Coordinates.getRow(prev__);
        int prev__col = Coordinates.getCol(prev__);

        int next__row = Coordinates.getRow(next__);
        int next__col = Coordinates.getCol(next__);

        if (prev__row == next__row && Math.abs(prev__col - next__col) == 1)
            return true;

        return prev__col == next__col && Math.abs(prev__row - next__row) == 1;
    }

    //it compares by coordinates and does not consider time steps
    public static boolean areNeighbours(int[] prev__, int next__row, int next__col) {
        int prev__row = Coordinates.getRow(prev__);
        int prev__col = Coordinates.getCol(prev__);

        if (prev__row == next__row && Math.abs(prev__col - next__col) == 1)
            return true;

        return prev__col == next__col && Math.abs(prev__row - next__row) == 1;
    }
    //it compares by coordinates and does not consider time steps
    public static boolean areNeighbours(int prev_row,int prev_col, int next_row, int next_col) {
        if (prev_row == next_row && Math.abs(prev_col - next_col) == 1)
            return true;

        return prev_col == next_col && Math.abs(prev_row - next_row) == 1;
    }

    public static boolean getNeighboursByIndexRanges(int[] cell_pos, int[] index_agents, HashMap<Integer,int[]> agents_idx_to_boxes_idx, ArrayList<int[]> output_pairs) {
        boolean neighbours_found = false;
        for (int index_agent : index_agents) {
            int row_ag = Coordinates.getRow(index_agent, cell_pos);
            int col_ag = Coordinates.getCol(index_agent, cell_pos);

            int[] boxes_idx = agents_idx_to_boxes_idx.get(index_agent);
            for (int boxesIdx : boxes_idx) {
                int row_box = Coordinates.getRow(boxesIdx, cell_pos);
                int col_box = Coordinates.getCol(boxesIdx, cell_pos);
                boolean are_neighbours = Coordinates.areNeighbours(row_ag, col_ag, row_box, col_box);
                if (are_neighbours) {
                    int[] pair = new int[]{index_agent, boxesIdx};
                    output_pairs.add(pair);
                    neighbours_found = true;
                }
            }
        }
        return neighbours_found;
    }

    public static boolean areEqual(int[] coordinates_one, int[] coordinates_two) {
        return Arrays.equals(coordinates_one,coordinates_two);
    }

    public static int getNumberOfCoordinates(int[] goal_coordinates) {
        return goal_coordinates.length/Coordinates.getLenght();
    }
}

