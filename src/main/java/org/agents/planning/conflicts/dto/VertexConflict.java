package org.agents.planning.conflicts.dto;

import org.agents.markings.Coordinates;

import java.util.ArrayList;
import java.util.HashMap;

public final class VertexConflict extends SimulationConflict{
    HashMap<Integer, ArrayList<int[]>> conflicted_ids_to_cells;

    public VertexConflict(int movable_mark_id) {
        super(movable_mark_id);

        conflicted_ids_to_cells = new HashMap<>();
    }

    public void addConflictedCell(int mark_id_conflicted, int[] coordinate_conflicted){
        movable_mark_id_to_conflicted_ids.add(mark_id_conflicted);

        if (conflicted_ids_to_cells.containsKey(mark_id_conflicted)){
            conflicted_ids_to_cells.get(mark_id_conflicted).add(coordinate_conflicted);
        }else{
            ArrayList<int[]> vertices = new ArrayList<>();
            vertices.add(coordinate_conflicted);
            conflicted_ids_to_cells.put(mark_id_conflicted, vertices);
        }
    }

    @Override
    public int[] getMaxTimeDeadline(){
        ArrayList<int[]> vertexConflict;
        int time_step ;
        //Coordinates coordinates;

        for (Integer key : this.conflicted_ids_to_cells.keySet()){
            vertexConflict = conflicted_ids_to_cells.get(key);

            for (int[] coordinate : vertexConflict){
                time_step = Coordinates.getTime(coordinate);

                max_t_deadline = Coordinates.getTime(max_coordinate_deadline);

                if(max_t_deadline < time_step){
                    max_coordinate_deadline = coordinate;
                }
            }
        }

        return max_coordinate_deadline;
    }

    @Override
    public ArrayList<int[]> getCoordinatesToAvoid(){
        ArrayList<int[]> list_to_avoid = new ArrayList<>();
        for (Integer key : conflicted_ids_to_cells.keySet()){
            ArrayList<int[]> conflicted_vertices = conflicted_ids_to_cells.get(key);
            list_to_avoid.addAll(conflicted_vertices);
        }

        return list_to_avoid;
    }

}
