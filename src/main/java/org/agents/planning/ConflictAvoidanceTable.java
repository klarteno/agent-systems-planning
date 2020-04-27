package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;

import java.util.*;

public final class ConflictAvoidanceTable {
    private final int[][][] table_for_paths;
    //this table stores for every number mark of the movable objectss a index that is used to index in the table_for_paths
    private final HashMap<Integer,Integer> table_ids;

    //the conflict avoidance is used only for the agents and the boxes used as input
    public ConflictAvoidanceTable(Agent[] agents,Box[] boxes) {
        int number_of_movables = agents.length + boxes.length;
        table_for_paths = new int[number_of_movables][][];
        table_ids = new HashMap<>(number_of_movables);

        int index = 0;
        for (Agent agent : agents) {
            table_ids.put(agent.getNumberMark(), index++);
        }
        for (int i = 0; i < boxes.length; i++) {
            table_ids.put(agents[i].getNumberMark(), index++);
         }
    }

    public HashMap<Integer, Integer> getTrackedIDs(){
        return this.table_ids;
    }



    //adds the path and removes the path
    public void addMarkedPathAndPopped(ArrayDeque<int[]> path, int number_mark,int start_time_step){

        int[] cell;
        int y_loc;
        int x_loc;
        start_time_step = start_time_step + path.size()-1;
        while (!path.isEmpty()){
            cell = path.pop();
            y_loc=cell[0];
            x_loc=cell[1];
            this.table_for_paths[this.table_ids.get(number_mark)][y_loc][x_loc] = start_time_step--;

        }
    }

    public void addMarkedPath(ArrayDeque<int[]> path, int number_mark, int start_time_step){

        int[] cell;
        int y_loc;
        int x_loc;
        for (int[] ints : path) {
            cell = ints;
            y_loc = cell[0];
            x_loc = cell[1];
            this.table_for_paths[this.table_ids.get(number_mark)][y_loc][x_loc] = start_time_step++;

        }
    }

        public boolean removeMarkedPath(int number_mark){
        if(this.table_for_paths[this.table_ids.get(number_mark)].length > 0 ){
            this.table_for_paths[this.table_ids.get(number_mark)] = new int[0][0];
            return true;
        }else{
            return false;
        }
    }

    /*
    public int removePathConflictsToDel(int start_time_step, Vector<int[]> path, ArrayList<int[]> next_conflicts){
        next_conflicts.clear();
        int[] next_cell_location;
        int y_loc;
        int x_loc;
        int time;
        int next_id;
        for (int id = 0; id < table_ids.keySet().size(); id++) {
            next_id = table_ids.get(id);
            time = start_time_step;
            for (int j = 0; j < path.size(); j++) {
                next_cell_location = path.get(j);
                y_loc = next_cell_location[0];
                x_loc = next_cell_location[1];
                if(table_for_paths[next_id][y_loc][x_loc] == time){
                    next_conflicts.add(path.get(j));
                    path.remove(j);
                }
                time++;
            }
            if (next_conflicts.size() > 0){
                //movable_conflicted[0] = id;
                return id;
            }
        }
        return -1;
    }*/
    //removes the elements from the input path that are in conflict with the conflict table avoidance
    //next_conflicts is the elemnts that are in conflict conform CAT
    public int removePathConflicts(int[] prev_cell_location, int start_time_step, ArrayDeque<int[]> path, ArrayList<int[]> next_conflicts){
        next_conflicts.clear();
        int[] next_cell_location;
        int time;
        int next_id;
        Iterator<int[]> path_iterator;
        for (int id = 0; id < table_ids.keySet().size(); id++) {
            next_id = table_ids.get(id);

            time = start_time_step;
            path_iterator = path.iterator();
            while (path_iterator.hasNext()) {
                next_cell_location = path_iterator.next();
                if (isCellOrEdgeConflicts(next_cell_location, prev_cell_location, time, next_id))
                    next_conflicts.add(next_cell_location);

                time++;
                prev_cell_location = next_cell_location;
            }
            if (next_conflicts.size() > 0){
                //movable_conflicted[0] = id;
                return id;//return the conflicted id
            }
        }
        return -1;
    }

//removes edge conflicts and cell conflicts
    //TO DO : add more rules depending on the colors between boxes and agents
    public void removeCellConflicts(int[] prev_cell_location, int fixed_time_step, ArrayDeque<int[]> path){
        int[] cell_location;
        int next_id;
        Iterator<int[]> path_iterator;

        for (int id = 0; id < table_ids.keySet().size(); id++) {
            next_id = table_ids.get(id);

            path_iterator = path.iterator();
            while (path_iterator.hasNext()){
                cell_location = path_iterator.next();
                if (isCellOrEdgeConflicts(cell_location, prev_cell_location, fixed_time_step, next_id))
                    path_iterator.remove();//is it removing the present element??
            }

            if (path.size() == 0){
                //movable_conflicted[0] = id;
                return;
            }
        }
    }

    private boolean isCellOrEdgeConflicts(int[] cell_location, int[] prev_cell_location, int fixed_time_step, int next_id) {
        int y_loc = cell_location[0];
        int x_loc = cell_location[1];
        //check cell location conflicts
        if(table_for_paths[next_id][y_loc][x_loc] == fixed_time_step){
            return true;
        }
        return getEdgeConflicts(prev_cell_location, cell_location, next_id, fixed_time_step);
    }

    //check edge between cell locations for conflicts
    public boolean getEdgeConflicts(int[] prev_cell_location, int[] cell_location, int movable_id, int fixed_time_step){

        int y_loc_prev = prev_cell_location[0];
        int x_loc_prev = prev_cell_location[1];
        int y_loc = cell_location[0];
        int x_loc = cell_location[1];
        return (table_for_paths[movable_id][y_loc_prev][x_loc_prev] == fixed_time_step) && (table_for_paths[movable_id][y_loc][x_loc] == fixed_time_step-1);
        //throw new UnsupportedOperationException("to be implemented");
    }
}
