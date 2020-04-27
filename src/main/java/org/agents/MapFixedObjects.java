package org.agents;

import org.agents.planning.ConflictAvoidanceTable;

import java.io.Serializable;
import java.util.*;

//replace with composition one MapFixedObjects and one with opearations
public final class MapFixedObjects implements Serializable {
        private  ConflictAvoidanceTable conflictAvoidanceTable;

        enum SearchState {
                NO_CHECK_CONFLICTS,
                CHECK_CONFLICTS;
        };

        //change this when the searching algorithm requires to check for conflicts
        public SearchState searchState = SearchState.NO_CHECK_CONFLICTS;

        public static int MAX_ROW = 0;
        public static int MAX_COL = 0;

        public boolean[][] walls;

        //goals is not intended to use in the algorithms instead we store the goals in the box or agent object
        public HashMap<Character, int[]> goals = new HashMap<>();

        public static Box[] boxes;
        public static Agent[] agents;

        public MapFixedObjects(){
        }

        public MapFixedObjects(ConflictAvoidanceTable conflictAvoidanceTable){
                this.conflictAvoidanceTable = conflictAvoidanceTable;
        }


        public static int getNumerOfAgents(){
                return  agents.length;
        }

        public static int getNumerOfBoxes() {
                return  boxes.length;
        }

        //this opearation is used the most in searching :TO DO replace with matrix look up
        private boolean isFreeCell(int[] cell, int movable_color) {
                boolean isFreeCell = true;
                int y = cell[0];
                int x = cell[1];
                isFreeCell = (y > 0 && y < MAX_ROW  && x > 0 && x < MAX_COL && !this.walls[y][x]);
                if(!isFreeCell)
                        return isFreeCell;

                for(Agent next_agent: agents){
                        if(Arrays.equals(next_agent.getCoordinates(),cell)){
                                isFreeCell = false;
                                break;
                        }
                }

                for(Box next_box: boxes){
                        if(Arrays.equals(next_box.getCoordinates(),cell) && !(next_box.getColor() == movable_color)) {
                                isFreeCell = false;
                                break;
                        }
                }
                return isFreeCell;
        }

        //gets the neighbours of the cell  and removes those that conflicts at the start_time_step
        public ArrayDeque<int[]> getFreeNeighbours(int[] coordinates, int number_mark, int color_movable, int start_time_step){
                ArrayDeque<int[]> next_cells = new ArrayDeque<>();
                if(searchState == SearchState.NO_CHECK_CONFLICTS){
                        return this.getNeighbours(coordinates, color_movable);
                } 
                 else if(searchState == SearchState.CHECK_CONFLICTS){
                        next_cells = this.getNeighbours(coordinates, color_movable);
                        this.getCheckConflictAvoidanceTable(coordinates, start_time_step, next_cells);
                        return next_cells;
                }
                 return next_cells;
        }

        private void getCheckConflictAvoidanceTable(int[] coordinates, int time_step, ArrayDeque<int[]> next_cells) {
                this.conflictAvoidanceTable.removeCellConflicts(coordinates,time_step, next_cells);
        }

        private ArrayDeque<int[]> getNeighbours(int[] coordinates, Color color_movable){
                return this.getNeighbours(coordinates, Color.getColorCoded(color_movable));
        }

        public ArrayDeque<int[]>  getNeighbours(int[] coordinates, int color_movable){
                int[] dir1 = new int[]{coordinates[0]+1,coordinates[1] };
                int[] dir2 = new int[]{coordinates[0]-1,coordinates[1]};
                int[] dir3 = new int[]{coordinates[0],coordinates[1]+1};
                int[] dir4= new int[]{coordinates[0],coordinates[1]-1};

                ArrayList<int[]> dirs = new ArrayList<>();
                dirs.add(dir1);
                dirs.add(dir2);
                dirs.add(dir3);
                dirs.add(dir4);

                ArrayDeque<int[]> neighbours_indexes = new ArrayDeque<>();
                for(int[] cell:dirs){
                        if (this.isFreeCell(cell, color_movable))
                                neighbours_indexes.add(cell);
                }

                return neighbours_indexes;
        }

        public Optional<Box> getNextBoxBy(int color) {
                for(Box next_box : boxes){
                        if(next_box.getColor() == color) {
                                return Optional.of(next_box);
                        }
                }

                return Optional.ofNullable(boxes[0]);
        }

}
