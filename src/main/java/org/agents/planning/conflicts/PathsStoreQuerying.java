package org.agents.planning.conflicts;

import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.planning.schedulling.TrackedGroups;

import java.io.Serializable;
import java.util.*;

class PathsStoreQuerying implements Serializable {
    //make ia a matrix of aarays of java bitset , one bitset holding the time for each id
    private int[][][] table_for_paths;
    private int[] path_lenghs; //indexed by table_ids which is in other class
    private final TrackedGroups tracked_groups;
    private int paths_rows;
    private int paths_columns;

    public PathsStoreQuerying(TrackedGroups trackedGroups) {
        this.tracked_groups = trackedGroups;
        this.setUpTracked(trackedGroups);
    }

    public void setUpTracked(TrackedGroups trackedGroups){
        int group_size = trackedGroups.getGroupSize();
        this.paths_rows = MapFixedObjects.MAX_ROW;
        this.paths_columns = MapFixedObjects.MAX_COL;

        this.table_for_paths = new int[group_size][paths_rows][paths_columns];
        this.path_lenghs = new int[group_size];
    }

    public  int getPathsRowsNo(){
        return this.paths_rows;
    }

    public int getPathsColumnsNo(){
        return this.paths_columns;
    }

    public void setCellLocationOf(int mark_id, ArrayDeque<int[]> path){
        this.removePath(mark_id);
        for (int[] cell_location : path) {
            this.setCellLocationOf(mark_id, cell_location);
        }
        this.path_lenghs[this.getIndexFor(mark_id)] = path.size();
    }

    void setCellLocationOf(int mark_id, int[] cell_location){
        int y_loc =   Coordinates.getRow(cell_location);
        int x_loc =   Coordinates.getCol(cell_location);
        int start_time_step = Coordinates.getTime(cell_location);

        int id_index = this.getIndexFor(mark_id);
        this.table_for_paths[id_index][y_loc][x_loc] = start_time_step;
    }

    private int getIndexFor(int mark_id){
        return this.tracked_groups.getIndexFor(mark_id);
    }

    public void setCellLocationOf(int[][] groups_marks, ArrayDeque<int[]> paths){
        int[] group = getMergedGroupOfTwo(groups_marks);
        for(int mark_id : group){
            int id_index = this.getIndexFor(mark_id);
            this.path_lenghs[id_index] = paths.size();
        }

        while (!paths.isEmpty())
            setCellLocationOf(group, paths.pop());
    }

    //merges two groups
    private int[] getMergedGroupOfTwo(int[][] groups_marks) {
        int[] group = Arrays.copyOf(groups_marks[0], groups_marks[0].length + groups_marks[1].length);
        System.arraycopy(groups_marks[1], 0, group, group[groups_marks[0].length], groups_marks[1].length);
        return group;
    }

    //TO DO: instead of array copying : iterate through the matrix groups_marks
    private void setCellLocationOf(int[][] groups_marks, int[] cell_locations){
        int[] group = getMergedGroupOfTwo(groups_marks);
        setCellLocationOf(group, cell_locations);
    }

    public void setCellLocationOf(int[] group_marks, int[] cell_locations){
        int time_step;
        int row;
        int column;
        int mark_id;
        for (int start_position = 0; start_position < cell_locations.length; start_position += Coordinates.getLenght()) {
            time_step = start_position;
            row = start_position + 1;
            column = start_position + 2;
            mark_id = group_marks[start_position/Coordinates.getLenght()];
            int id_index = this.getIndexFor(mark_id);

            this.table_for_paths[id_index][cell_locations[row]][cell_locations[column]] = cell_locations[time_step];
        }
    }

    public boolean removePath(int mark_id){
        int id_index = this.getIndexFor(mark_id);

        if(this.table_for_paths[id_index].length > 0 ){
            ///int[][] ggggg = this.table_for_paths[id_index];
            for (int i = 0; i < this.paths_rows; i++) {
                for (int j = 0; j < this.paths_columns; j++) {
                    this.table_for_paths[id_index][i][j] = -1;
                }
            }
            this.path_lenghs[id_index] = 0;

            return true;
        }else{
            return false;
        }
    }

    public boolean removePath(int[] group_marks){
        for (int mark_id : group_marks){
            int id_index = this.getIndexFor(mark_id);
            this.table_for_paths[id_index] = new int[0][0];
            this.path_lenghs[id_index] = 0;
        }
        return true;
    }

    public int getTimeStep(int mark_id, int[] cell_location){
        int y_loc = cell_location[0];
        int x_loc = cell_location[1];
        int id_index = this.getIndexFor(mark_id);

        return this.table_for_paths[id_index][y_loc][x_loc];
     }

    public static boolean isOverlap(int[][] first_path, int[][] second_path){
        for (int row_index = 0; row_index < first_path.length; row_index++) {
            if(Arrays.equals(first_path[row_index], second_path[row_index]))
                return true;
        }
        return false;
    }

    public static int[] getFirstOverlapFor(int[][] first_path, int[][] second_path){
        int row_index = 0;

        for (int[] row : first_path) {
            for (int col_index : row) {
                if (first_path[row_index][col_index] == second_path[row_index][col_index]){
                    int time_step = first_path[row_index][col_index];
                    return Coordinates.createCoordinates(time_step, row_index, col_index);
                }
            }
            row_index++;
        }
        return Coordinates.createCoordinates();
    }

    public static ArrayDeque<int[]> getAllOverlapsFor(int[][] first_path, int[][] second_path){
        int row_index = 0;
        ArrayDeque<int[]> overlaps = new ArrayDeque<>();
        int time_step;

        for (int[] row : first_path) {
            for (int col_index : row) {
                if (first_path[row_index][col_index] == second_path[row_index][col_index]){
                    time_step = first_path[row_index][col_index];
                    overlaps.add(Coordinates.createCoordinates(time_step, row_index, col_index));
                }
            }
            row_index++;
        }
        return overlaps;
    }

    public int[][] getPathCloneFor(int mark_id){
        int[][] clone_path = new int[getPathsRowsNo()][];
        int id_index = this.getIndexFor(mark_id);
        int[][] path = this.table_for_paths[id_index];

        for (int row = 0; row < getPathsRowsNo(); row++) {
            clone_path[row] = Arrays.copyOf(path[row], path[row].length);
        }

        return clone_path;
    }

    private int[][] getPathFor(int mark_id){
        int id_index = this.getIndexFor(mark_id);
        return this.table_for_paths[id_index];
    }

    public int[][][] getPathsCloneForGroup(int[] group_marks) {
        assert Objects.requireNonNull(group_marks).length > 0;

        int rows = getPathsRowsNo();
        int[][][] clone_paths = new int[group_marks.length][rows][];
        int[][] next_path;
        int index = 0;

        for (int i = 0; i < group_marks.length; i++) {
            next_path = getPathFor(group_marks[i]);
            for (int row = 0; row < rows; row++) {
                clone_paths[i][row] = Arrays.copyOf(next_path[row], next_path[row].length);
            }

        }

        return clone_paths;
    }


    public  int getPathLenght(int mark_id) {
        int index = this.getIndexFor(mark_id);

        return this.path_lenghs[index];
    }

    public int getNumberOfPaths(){
        return this.path_lenghs.length;
    }
}