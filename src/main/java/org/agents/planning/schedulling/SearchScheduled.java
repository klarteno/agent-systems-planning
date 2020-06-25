package org.agents.planning.schedulling;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.searchengine.StateSearchMAFactory;

import java.io.Serializable;
import java.util.*;

public class SearchScheduled {
    //final int INDEX_OF_BOXES = 2;
    public static final int INDEX_OF_AGENTS = 0;
    public static final int START_GROUP_AGENTS = 1;
    public static final int INDEX_OF_GROUP = 2;

    private int[][] total_group;
    private HashMap<Integer,int[]> agents_idxs_to_boxes_idxs;
    private UUID unique_id;

    public static final int NEXT_GOAL_TO_BOX = 0;
    private int[] index_boxes;

    public SearchScheduled() { }

    public void setTotalGroup(int[][] totalGroup) {
        this.total_group = totalGroup;
    }

    public int[][] getTotalGroup(int[][] totalGroup)  { return this.total_group; }

    private static  int[] group_marks_ids;

    private static int[] start_coordinates;
    private static int[] start_coordinates_agt_boxes;
    private static int[] goals_coordinates;
    private static HashMap<Integer, ArrayList<int[]>> goals_neighbours;

    void setAgentsIdxsToBoxesIdxs(HashMap<Integer,int[]> agentsIdxs_to_boxesIdxs) {
        this.agents_idxs_to_boxes_idxs = agentsIdxs_to_boxesIdxs;
    }

    public int[][] getTotalGroup() {
        //example how to use
        int[] ints = this.total_group[INDEX_OF_AGENTS];
        int[] ints2 = this.total_group[START_GROUP_AGENTS];
        int[] ints3 = this.total_group[INDEX_OF_GROUP];

        return this.total_group;
    }

    private int[] getIndexOfAgents(){
        return  this.total_group[INDEX_OF_AGENTS];
    }

    private int[] getStartGroupOfAgents(){
        return  this.total_group[START_GROUP_AGENTS];
    }

    public int[] getGroup(){
        return  this.total_group[INDEX_OF_GROUP];
    }

    //group independence detection to refactor
    public void setGroup(int[] group){
        this.total_group[INDEX_OF_GROUP] = group;
    }

    public int[] getIndexBoxes(){
        if(this.index_boxes == null){
            index_boxes = new int[this.total_group[INDEX_OF_GROUP].length - this.total_group[INDEX_OF_AGENTS].length];
        }else {
            return index_boxes;
        }

        int[] group = this.total_group[INDEX_OF_GROUP];
        int[] agents = this.total_group[INDEX_OF_AGENTS];
        Set<Integer> agts = new HashSet<Integer>();

        for (int agent : agents) {
            agts.add(agent);
        }


        int index = 0;
        for (int idx = 0; idx < group.length; idx++) {
            if( !agts.contains(group[idx]) ){
                index_boxes[index++] = idx;
            }
        }

        return this.index_boxes;
    }

    public HashMap<Integer,int[]> getAgentstIdxsToBoxesIdxs() {
        return this.agents_idxs_to_boxes_idxs;
    }

    public void setUUID(UUID uniqueID) { this.unique_id = uniqueID; }
    public UUID getUUID() { return this.unique_id; }

    //usage: sched_group.setState(SearchScheduled.NEXT_GOAL_TO_BOX);
    //updates goals to be other node cells to be closed to the box
    public void setState(int nextGoalToBox) {
        for(Integer key : this.agents_idxs_to_boxes_idxs.keySet()){
            int _idx = this.total_group[INDEX_OF_AGENTS][key];
            int agt_id  = this.total_group[INDEX_OF_GROUP][_idx];

            int[] boxes_idxs = agents_idxs_to_boxes_idxs.get(key);
            for (int __idx : boxes_idxs) {
                int box_id = this.total_group[INDEX_OF_GROUP][__idx];
                Box box = (Box) MapFixedObjects.getByMarkNo(box_id);

                setStateGoals(nextGoalToBox, agt_id, box_id);
            }
        }
    }

    private void setStateGoals(int nextGoalToBox, int agt_id, int box_id ) {
        int BOX_GOAL_STRATEGY = nextGoalToBox;

        switch (BOX_GOAL_STRATEGY){
            case NEXT_GOAL_TO_BOX:
                Agent agent   = (Agent)MapFixedObjects.getByMarkNo(agt_id);
                Box box   = (Box)MapFixedObjects.getByMarkNo(box_id);
                int[] boxgoal_cordinate = box.getCoordinates();

                //run bfs for next neighbours that are closer to agent scheduled
                //now it gets neighbours in a naive way :some neighbours are behind the box
                ArrayDeque<int[]> next_neigh_cells = MapFixedObjects.getNeighbours(boxgoal_cordinate, agt_id);
                int[] next_goal_cell;
                /*int min_h_value = Integer.MAX_VALUE ;
                while (!next_neigh_cells.isEmpty()){
                    int[] cell = next_neigh_cells.pop();
                    int heuristic_value = MapFixedObjects.getManhattenHeuristic(cell,  agent.getCoordinates());

                    if (min_h_value > heuristic_value){
                        min_h_value = heuristic_value;
                        next_goal_cell = cell;
                    }
                }*/
                while (!next_neigh_cells.isEmpty()){
                    next_goal_cell = next_neigh_cells.pop();
                    agent.addGoalPosition(next_goal_cell);
                    box.addNeighbourGoal( next_goal_cell);
                }
                break;
        }
    }

    public void setStartGroup( StateSearchMAFactory.SearchState searchMultiAgentState) {
        int[] start_group = this.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];
        group_marks_ids = start_group;
        int[] idx_agt = this.getTotalGroup()[SearchScheduled.INDEX_OF_AGENTS];
        int start_boxes_length = start_group.length - idx_agt.length;


        switch (searchMultiAgentState){
            case AGENTS_ONLY:
                this.agentsOnly(this.getTotalGroup()[SearchScheduled.INDEX_OF_AGENTS]);
                group_marks_ids = this.getTotalGroup()[SearchScheduled.INDEX_OF_AGENTS];
                break;
            case AGENTS_AND_BOXES:
                this.agentsAndBoxes(start_group, start_boxes_length);
                break;
        }
    }

    private void agentsOnly(int[] start_group) {
        start_coordinates = new int[start_group.length * Coordinates.getLenght()];
        goals_coordinates = new int[start_group.length * Coordinates.getLenght()];

        int coordinate_start = 0;
        ///Arrays.stream(start_group).parallel().
        Agent agent;
        Box box;
        int[] movable_coordinate;
        int movable_number = 0;
        int[] goal_cordinate;

        for (int movable_id : start_group) {

            Serializable next_movable = MapFixedObjects.getByMarkNo(movable_id);
            if (next_movable instanceof Agent) {
                agent = (Agent) next_movable;
                movable_coordinate = agent.getCoordinates();
                ArrayList<int[]> goals_list = new ArrayList<>();
                goal_cordinate = agent.getGoalStopPosition();
                    if(goals_neighbours == null )goals_neighbours = new HashMap<>();
                    int[] goal_cordinate_2 = agent.getNextGoal();

                    while( !Arrays.equals(goal_cordinate_2, Coordinates.getEmptyInstance())){
                        goals_list.add(goal_cordinate_2);
                        goal_cordinate_2 = agent.getNextGoal();
                    }
                    goals_neighbours.put(movable_number, goals_list);
                    movable_number++;
                    for (int j = 0; j < Coordinates.getLenght(); j++) {
                        start_coordinates[coordinate_start] = movable_coordinate[j];
                        goals_coordinates[coordinate_start++] = goal_cordinate[j];
                    }

            } else if (next_movable instanceof Box) {
                box = (Box) next_movable;
                movable_coordinate = box.getCoordinates();
                goal_cordinate = box.getGoalPosition();

                for (int j = 0; j < Coordinates.getLenght(); j++) {
                    start_coordinates[coordinate_start] = movable_coordinate[j];
                    goals_coordinates[coordinate_start++] = goal_cordinate[j];
                }
            }
            //coordinate_index += Coordinates.getLenght();
        }
    }

    private void agentsAndBoxes(int[] start_group, int start_boxes_length) {
        start_coordinates = new int[start_group.length * Coordinates.getLenght()];
        goals_coordinates = new int[start_boxes_length * Coordinates.getLenght()];

        int coordinate_start = 0;
        int coordinate_goal = 0;

        ///Arrays.stream(start_group).parallel().
        Agent agent;
        Box box;
        int[] movable_coordinate;
        int movable_number = 0;
        int[] goal_cordinate;

        for (int movable_id : start_group) {

            Serializable next_movable = MapFixedObjects.getByMarkNo(movable_id);
            if (next_movable instanceof Agent) {
                agent = (Agent) next_movable;
                movable_coordinate = agent.getCoordinates();
                ArrayList<int[]> goals_list = new ArrayList<>();
                if(goals_neighbours == null )goals_neighbours = new HashMap<>();
                int[] goal_cordinate_2 = agent.getNextGoal();

                while( !Arrays.equals(goal_cordinate_2, Coordinates.getEmptyInstance())){
                    goals_list.add(goal_cordinate_2);
                    goal_cordinate_2 = agent.getNextGoal();
                }
                goals_neighbours.put(movable_number, goals_list);
                movable_number++;
                for (int j = 0; j < Coordinates.getLenght(); j++) {
                    start_coordinates[coordinate_start++] = movable_coordinate[j];
                 }

            } else if (next_movable instanceof Box) {
                box = (Box) next_movable;
                movable_coordinate = box.getCoordinates();
                goal_cordinate = box.getGoalPosition();

                for (int j = 0; j < Coordinates.getLenght(); j++) {
                    start_coordinates[coordinate_start++] = movable_coordinate[j];
                 }
                for (int j = 0; j < Coordinates.getLenght(); j++) {
                     goals_coordinates[coordinate_goal++] = goal_cordinate[j];
                }
            }
            //coordinate_index += Coordinates.getLenght();
        }
    }

    public int[] getGroup_marks_ids() {
        return group_marks_ids;
    }

    public int[] getStart_coordinates() {
        return start_coordinates;
    }

    public int[] getStart_coordinates_agts_boxes() {
        return start_coordinates_agt_boxes;
    }

    public void setStart_coordinates_agts_boxes(int[] startCoordinates) {
        int __number_movables = startCoordinates.length/Coordinates.getLenght();
        for (int i = 0; i <__number_movables ; i++) {
            Coordinates.setTime(i,startCoordinates,0);
        }
        start_coordinates_agt_boxes = startCoordinates;
    }

    public int[] getGoals_coordinates() {
        return goals_coordinates;
    }

    public HashMap<Integer, ArrayList<int[]>> getGoals_neighbours() {
        return goals_neighbours;
    }
}
