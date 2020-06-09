package org.agents.searchengine;

import org.agents.planning.schedulling.TrackedGroups;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.UUID;

public class SearchTaskResult {
    public static final int INDEX_OF_AGENTS = 0;
    public static final int START_GROUP_AGENTS = 1;
    public static final int INDEX_OF_GROUP = 2;

    private final ArrayDeque<int[]> path;
    private int[] start_coordinates_of_group;
    private int[] goals_coordinates_of_group;
    private int[] start_group;              //the marked ids that have the path resolved
    private ArrayList<int[]> conflicts;     //conflict found during the search of the path
    private int[][] total_group;
    private HashMap<Integer,int[]> agents_idxs_to_boxes_idxs; //agent indexes to bobes indexes from total group
    private UUID unique_id;
    private TrackedGroups tracked_groups;
    private int[] index_agents;                                 //agent indexes from total group

    public SearchTaskResult(ArrayDeque<int[]> path) {
        this.path = path;
        this.conflicts = new ArrayList<>();
    }

    public void addStartCoordinates(int[] startCoordinatesOfGroup) {
        this.start_coordinates_of_group = startCoordinatesOfGroup;
    }

    public void addGoalCoordinates(int[] goalsCoordinatesOfGroup) {
        this.goals_coordinates_of_group = goalsCoordinatesOfGroup;
    }

    public void setGroup(int[] startGroup) {
        this.start_group = startGroup;
    }

    public void addLastConflict(ArrayList<int[]> last_conflicts) {
        this.conflicts.addAll(last_conflicts);
    }

    public ArrayDeque<int[]> getPath() {
        return this.path;
    }

    public void setTotalGroup(int[][] totalGroup) {
        this.total_group = totalGroup;
    }

    public void setAgentstIdxsToBoxesIdxs(HashMap<Integer,int[]> agentsIdxsToBoxesIdxs) {
        this.agents_idxs_to_boxes_idxs = agentsIdxsToBoxesIdxs;
    }

    public void setUUID(UUID uniqueID) {
        this.unique_id = uniqueID;
    }

    public UUID getUUID(UUID uniqueID) { return this.unique_id; }

    public void setTrackedGroup(TrackedGroups trackedGroups) {
        this.tracked_groups = trackedGroups;
    }

    public int[] getStartGroupAgents() { return this.start_group; }

    public int[][] getTotalGroup() {
        //example how to use
        int[] ints = this.total_group[INDEX_OF_AGENTS];
        int[] ints2 = this.total_group[START_GROUP_AGENTS];
        int[] ints3 = this.total_group[INDEX_OF_GROUP];

        return this.total_group; }

    public HashMap<Integer,int[]> getAgentstIdxsToBoxesIdxs() { return this.agents_idxs_to_boxes_idxs; }

    public void setIndexAgents(int[] indexAgents) {
        this.index_agents = indexAgents;
    }
}
