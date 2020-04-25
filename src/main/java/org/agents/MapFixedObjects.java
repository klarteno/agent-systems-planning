package org.agents;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import java.util.Stack;

public class MapFixedObjects implements Serializable {
        public int MAX_ROW = 0;
        public int MAX_COL = 0;

        public boolean[][] walls;

        //goals is not intended to use in the algorithms instead we store the goals in the box or agent object
        public HashMap<Character, int[]> goals = new HashMap<>();

        public Box[] boxes;
        public Agent[] agents;

        private boolean isFreeCell(int[] cell, int movable_color) {
                boolean isFreeCell = true;
                int y = cell[0];
                int x = cell[1];
                isFreeCell = (y > 0 && y < this.MAX_ROW  && x > 0 && x < this.MAX_COL && !this.walls[y][x]);
                if(!isFreeCell)
                        return isFreeCell;

                for(Agent next_agent:this.agents){
                        if(next_agent.getRowPosition() == y && next_agent.getColumnPosition() == x){
                                isFreeCell = false;
                                break;
                        }
                }

                for(Box next_box:this.boxes){
                        if(next_box.getRowPosition() == y && next_box.getColumnPosition() == x && !(next_box.getColor() == movable_color)) {
                                isFreeCell = false;
                                break;
                        }
                }
                return isFreeCell;
        }

        public Stack<int[]> getNeighbours(int[] coordinates, Color color_movable){
                return this.getNeighbours(coordinates, Color.getColorCoded(color_movable));
        }

        public  Stack<int[]>  getNeighbours(int[] coordinates, int color_movable){
                int[] dir1 = new int[]{coordinates[0]+1,coordinates[1] };
                int[] dir2 = new int[]{coordinates[0]-1,coordinates[1]};
                int[] dir3 = new int[]{coordinates[0],coordinates[1]+1};
                int[] dir4= new int[]{coordinates[0],coordinates[1]-1};

                ArrayList<int[]> dirs = new ArrayList<>();
                dirs.add(dir1);
                dirs.add(dir2);
                dirs.add(dir3);
                dirs.add(dir4);

                Stack<int[]> neighbours_indexes = new Stack<>();
                for(int[] cell:dirs){
                        if (this.isFreeCell(cell, color_movable))
                                neighbours_indexes.add(cell);
                }

                return neighbours_indexes;
        }

        public Optional<Box> getNextBoxBy(int color) {
                for(Box next_box : this.boxes){
                        if(next_box.getColor() == color) {
                                return Optional.of(next_box);
                        }
                }

                final Optional<Box> option = Optional.ofNullable(boxes[0]);
                return option;
        }

}
