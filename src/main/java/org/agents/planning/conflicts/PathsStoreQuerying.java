package org.agents.planning.conflicts;

import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;

import java.io.Serializable;
import java.util.*;

class PathsStoreQuerying implements Serializable {
    //make ia a matrix of aarays of java bitset , one bitset holding the time for each id
    private int[][][] table_for_paths;
    private int[] path_lenghs; //indexed by table_ids which is in other class
    //this table stores for every number mark of the movable objectss a index that is used to index in the table_for_paths
    private Collection<? extends Integer> movables_ids;
    private HashMap<Integer,Integer> ids_indexes;//should be declared final


    public PathsStoreQuerying() {
        int number_of_movables = MapFixedObjects.getNumerOfAgents() + MapFixedObjects.getNumerOfBoxes();
        //table_for_paths = new int[number_of_movables][][];
        this.table_for_paths = new int[number_of_movables][MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL];

        this.movables_ids = MapFixedObjects.getAllIdsMarks();
        initIdsIndexes();//indexes the mark_ids and stores a index
        this.path_lenghs = new int[MapFixedObjects.getAllIdsMarks().size()];

    }

      private void initIdsIndexes(){
          this.ids_indexes = new HashMap<>(this.movables_ids.size());
          int index = 0;
          for (Integer next : movables_ids){
              ids_indexes.put(next, index++);
          }
      }

    public void setUpTracked(Collection<? extends Integer> movables_ids){
        //table_for_paths = new int[number_of_movables][][];
        this.table_for_paths = new int[movables_ids.size()][MapFixedObjects.MAX_ROW][MapFixedObjects.MAX_COL];

        this.movables_ids = movables_ids;
        initIdsIndexes();
        this.path_lenghs = new int[movables_ids.size()];


    }

    public static int getPathsRowsNo(){
        return MapFixedObjects.MAX_ROW;
    }

    public static int getPathsColumnsNo(){
        return MapFixedObjects.MAX_COL;
    }

    public void setCellLocationOf(int mark_id, int[] cell_location){
        int y_loc =   Coordinates.getRow(cell_location);
        int x_loc =   Coordinates.getCol(cell_location);
        int start_time_step = Coordinates.getTime(cell_location);
        this.table_for_paths[this.ids_indexes.get(mark_id)][y_loc][x_loc] = start_time_step;
    }

    public void setCellLocationOf(int[][] groups_marks, ArrayDeque<int[]> paths){
        int[] group = getSingleGroupOf(groups_marks);
        for(int mark_id : group)
            this.path_lenghs[ids_indexes.get(mark_id)] = paths.size();

        while (!paths.isEmpty())
            setCellLocationOf(group, paths.pop());
    }

    //merges two groups
    private int[] getSingleGroupOf(int[][] groups_marks) {
        int[] group = Arrays.copyOf(groups_marks[0], groups_marks[0].length + groups_marks[1].length);
        System.arraycopy(groups_marks[1], 0, group, group[groups_marks[0].length], groups_marks[1].length);
        return group;
    }

    //TO DO: instead of array copying : iterate through the matrix groups_marks
    private void setCellLocationOf(int[][] groups_marks, int[] cell_locations){
        int[] group = getSingleGroupOf(groups_marks);
        setCellLocationOf(group, cell_locations);
    }

    public void setCellLocationOf(int[] group_marks, int[] cell_locations){
        int time_step;
        int row;
        int column;
        int mark_id;
        for (int start_position = 0; start_position < cell_locations.length; start_position += Coordinates.getLenght()) {
            time_step = start_position;
            row = time_step + 1;
            column = row + 1;
            mark_id = group_marks[start_position/Coordinates.getLenght()];

            this.table_for_paths[this.ids_indexes.get(mark_id)][cell_locations[row]][cell_locations[column]] = cell_locations[time_step];
        }
    }

    public boolean removePath(int mark_id){
        if(this.table_for_paths[this.ids_indexes.get(mark_id)].length > 0 ){
            this.table_for_paths[this.ids_indexes.get(mark_id)] = new int[0][0];
            this.path_lenghs[ids_indexes.get(mark_id)] = 0;

            return true;
        }else{
            return false;
        }
    }

    public boolean removePath(int[] group_marks){
        for (int mark_id : group_marks){
            this.table_for_paths[this.ids_indexes.get(mark_id)] = new int[0][0];
            this.path_lenghs[ids_indexes.get(mark_id)] = 0;
        }
        return true;
    }

    public int getTimeStep(int mark_id, int[] cell_location){
        int y_loc = cell_location[0];
        int x_loc = cell_location[1];
        return this.table_for_paths[this.ids_indexes.get(mark_id)][y_loc][x_loc];
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
        int[][] path = this.table_for_paths[this.ids_indexes.get(mark_id)];

        for (int row = 0; row < getPathsRowsNo(); row++) {
            clone_path[row] = Arrays.copyOf(path[row], path[row].length);
        }

        return clone_path;
    }

    private int[][] getPathFor(int mark_id){
        return this.table_for_paths[this.ids_indexes.get(mark_id)];
    }

    public int[][][] getPathsCloneForGroup(int[] group_marks) {
        assert Objects.requireNonNull(group_marks).length > 0;

        int rows = getPathsRowsNo();
        int[][][] clone_paths = new int[group_marks.length][rows][];
        int[][] nex_path;
        int index = 0;
        for (int mark_id : group_marks){
            nex_path = getPathFor(mark_id);
            for (int row = 0; row < rows; row++) {
                clone_paths[index++][row] = Arrays.copyOf(nex_path[row], nex_path[row].length);
            }
        }
        return clone_paths;
    }


    public  int getPathLenght(int mark_id) {
        return this.path_lenghs[ids_indexes.get(mark_id)];
    }

    public int getNumberOfPaths(){
        return this.path_lenghs.length;
    }

    public Collection<? extends Integer> getTrackedIds() {
        return  this.movables_ids;
    }

    public int getIndexFor(int mark_id) {
        return this.ids_indexes.get(mark_id);
    }
}