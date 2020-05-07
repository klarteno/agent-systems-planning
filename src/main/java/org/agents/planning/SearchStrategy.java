package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.ConflictAvoidanceTable;
import org.agents.searchengine.PathProcessing;
import org.agents.searchengine.SearchEngineOD;
import org.agents.searchengine.SearchEngineSA;

import java.io.Serializable;
import java.util.*;

public class SearchStrategy {
    private final SearchEngineSA search_engine;
    private static ConflictAvoidanceTable conflict_avoidance_table;

    public SearchStrategy(SearchEngineSA searchEngine) {
        this.search_engine = searchEngine;
        conflict_avoidance_table = this.search_engine.getConflictAvoidanceCheckingRules().getConflictsTable();
    }

    public ArrayDeque<ListIterator<String>> getPathsSequencial(SearchEngineSA searchEngine) {
       PathProcessing pathProcessing = new PathProcessing();
        ListIterator<String> path_iter;
        ArrayDeque<ListIterator<String>> paths_iterations = new ArrayDeque<>();
        ArrayList<String> path;

        Set<Integer> keys = MapFixedObjects.getAgentsMarks();
        for (Integer key:keys) {
            path = pathProcessing.get_moves_agent_goal(MapFixedObjects.getByAgentMarkId(key), searchEngine);
            path_iter = path.listIterator(path.size());
            paths_iterations.add(path_iter);

        }
        return paths_iterations;

/*

        Agent agent1 = mapFixedObjects.agents[0];

        //Stack<int[]> path = searchEngine.runAstar(mapFixedObjects, agent1.getColor(), agent1.getCoordinates(), box1.get().getGoalPosition());
        PathProcessing pathProcessing = new PathProcessing();
        ArrayList<String> path1 = pathProcessing.get_moves_agent_goal(agent1, searchEngine, mapFixedObjects);
        Agent agent2 = mapFixedObjects.agents[1];
        ArrayList<String> path2 = pathProcessing.get_moves_agent_goal(agent2, searchEngine, mapFixedObjects);

        ListIterator<String> path_iter1 = path1.listIterator(path1.size());
        ListIterator<String> path_iter2 = path2.listIterator(path1.size());

        Stack<ListIterator<String>> paths_iterations = new Stack<>();
        paths_iterations.add(path_iter2);
        paths_iterations.add(path_iter1);
*/
    }




    //the agents has to have goals for the boxes set up
    //TO DO decouple to agregation or commands together with conflict_avoidance_table
    public void runDescenteralizedSearch(Agent[] agents, Box[] boxes) {
        assert conflict_avoidance_table != null;
        //make a parallel thread for each agen and box , put the id of the thread to the agent id
        Integer agent_id;
        for (Agent agent : agents) {
            this.search_engine.runAstar(agent);
            if(this.search_engine.isPathFound()){
                ArrayDeque<int[]> agent_path = this.search_engine.getPath();
                int agent_mark = agent.getNumberMark();
                conflict_avoidance_table.replaceMarkedPathFor(agent_mark, agent_path);
            }
        }

        for (Box box : boxes) {
            this.search_engine.runAstar(box);
            if(this.search_engine.isPathFound()){
                ArrayDeque<int[]> box_path = this.search_engine.getPath();
                int box_mark = box.getLetterMark();
                conflict_avoidance_table.replaceMarkedPathFor(box_mark, box_path);
            }
        }
    }


    public ArrayDeque<int[]> runSearch(int movable_id) {
        Serializable obj = MapFixedObjects.getByMarkNo(movable_id);

        if (obj instanceof Box){
            Box box = (Box)obj;
            this.search_engine.runAstar(box );
            if(this.search_engine.isPathFound()){
                int box_mark = box.getLetterMark();

                return  this.search_engine.getPath();
            }

        }else if (obj instanceof Agent){
            Agent agent = (Agent)obj;
            this.search_engine.runAstar(agent);
            if(this.search_engine.isPathFound()){
                int agent_mark = agent.getNumberMark();

                return this.search_engine.getPath();

            }

        }else{
            throw new UnsupportedOperationException("unknown movable id cast from Serializable");
        }


        return null;
    }

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
