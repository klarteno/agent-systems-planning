package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.searchengine.SearchEngineOD;

import java.io.Serializable;
import java.util.ArrayDeque;
import java.util.Arrays;

public class GroupSearch {
    //concatanates group 1 with group 2 , works only for two groups
    public ArrayDeque<int[]> runGroupSearchMA(int[][] start_group) {
        int[] groups = Arrays.copyOf(start_group[0], start_group[0].length + start_group[1].length);
        System.arraycopy(start_group[1], 0, groups, start_group[0].length, start_group[1].length);

        return runGroupSearchMA(groups, new  int[0][][]);
    }


    //gets the path for start_group and avoids colisions in group path; conflict_path
    //the path retturned is indexed in the same order as start_group
    public ArrayDeque<int[]> runGroupSearchMA(int[] start_group, int[][][] conflicting_paths) {
        SearchEngineOD searchEngineOD = new SearchEngineOD();
        int[] start_coordinates = new int[start_group.length * Coordinates.getLenght()];
        int[]  goal_coordinates = new int[start_group.length * Coordinates.getLenght()];
        int coordinate_index = 0;
        ///Arrays.stream(start_group).parallel().

        Agent agent;
        Box box;
        int[] movable_coordinate;
        int[] goal_cordinate;
        for (int movable_id : start_group) {
            Serializable next_movable = MapFixedObjects.getByMarkNo(movable_id);
            if (next_movable instanceof Agent) {
                agent = (Agent) next_movable;
                movable_coordinate = agent.getCoordinates();
                goal_cordinate = agent.getGoalPosition();

                for (int j = 0; j < Coordinates.getLenght(); j++) {
                    start_coordinates[coordinate_index] = movable_coordinate[j];
                    goal_coordinates[coordinate_index++] = goal_cordinate[j];
                }

            } else if (next_movable instanceof Box) {
                box = (Box) next_movable;
                movable_coordinate = box.getCoordinates();
                goal_cordinate = box.getGoalPosition();

                for (int j = 0; j < Coordinates.getLenght(); j++) {
                    start_coordinates[coordinate_index] = movable_coordinate[j];
                    goal_coordinates[coordinate_index++] = goal_cordinate[j];
                }
            }
            //coordinate_index += Coordinates.getLenght();
        }

        searchEngineOD.runOperatorDecomposition(start_coordinates, goal_coordinates, conflicting_paths);

        return searchEngineOD.getPath();
    }


    public void runGroupSearchMA2(int movable1_id, int movable2_id) {
        throw new UnsupportedOperationException("group sarch not implemented");
    }



}
