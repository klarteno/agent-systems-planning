package org.agents.planning.schedulling;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.UUID;

public class SearchScheduled {
    //final int INDEX_OF_BOXES = 2;
    public static final int INDEX_OF_AGENTS = 0;
    public static final int START_GROUP_AGENTS = 1;
    public static final int INDEX_OF_GROUP = 2;

    private int[][] total_group;
    private HashMap<Integer,int[]> agents_idxs_to_boxes_idxs;
    private UUID unique_id;


    public static final int NEXT_GOAL_TO_BOX = 0;


    public SearchScheduled() { }

    public void setTotalGroup(int[][] totalGroup) {
        this.total_group = totalGroup;
    }

    public int[][] getTotalGroup(int[][] totalGroup)  { return this.total_group; }

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
}
