package org.agents.planning.conflicts.dto;

import org.agents.markings.Coordinates;

import java.util.ArrayList;
import java.util.HashMap;

public final class EdgeConflict extends SimulationConflict {
   public final static int COORDINATE_START = 0;
   public final static int COORDINATE_END = 1;
   public final static int COORDINATES_NUMBER = 2;

    HashMap<Integer, ArrayList<int[][]>> conflicted_ids_to_edges;

    //edge is the connection betweeen two cell location
    //coordinate_edge_start is the start of the edge conflicted
    //coordinates_edge_end is the end of the edge conflicted
    //mark_id_start is the movable (agent or box) id at the start of the edge conflicted
    //mark_id_end is the movable (agent or box) id at the end of the edge conflicted
    public EdgeConflict(int movable_mark_id) {
        super(movable_mark_id);

        conflicted_ids_to_edges = new HashMap<>();
    }

    public void addConflictedEdge(int mark_id_conflicted, int[] coordinate_edge_start, int[] coordinates_edge_end){
        int[][] edge = new int[COORDINATES_NUMBER][];
        edge[COORDINATE_START] = coordinate_edge_start;
        edge[COORDINATE_END] = coordinates_edge_end;

        movable_mark_id_to_conflicted_ids.add(mark_id_conflicted);

        if (conflicted_ids_to_edges.containsKey(mark_id_conflicted)){
            conflicted_ids_to_edges.get(mark_id_conflicted).add(edge);
        }else{
            ArrayList<int[][]> edges = new ArrayList<>();
            edges.add(edge);
            conflicted_ids_to_edges.put(mark_id_conflicted, edges);
        }
    }

    @Override
    public int[] getMaxTimeDeadline(){
        ArrayList<int[][]> edgeConflict;
        int time_step ;
        //Coordinates coordinates;

        for (Integer key : this.conflicted_ids_to_edges.keySet()){
            edgeConflict = conflicted_ids_to_edges.get(key);

            for (int[][] edge : edgeConflict){
                int time_step_start = getTimeEdgeStart(edge);
                time_step = getTimeEdgeEnd(edge);
                max_t_deadline = Coordinates.getTime(max_coordinate_deadline);
                if(max_t_deadline < time_step){
                    max_coordinate_deadline = edge[COORDINATE_END];
                }
            }
        }
        return max_coordinate_deadline;
    }

    private static int getTimeEdgeStart(int[][] edge){
        return  Coordinates.getTime(edge[COORDINATE_START]);
    }

    private static int getTimeEdgeEnd(int[][] edge){
        return  Coordinates.getTime(edge[COORDINATE_END]);
    }
}

