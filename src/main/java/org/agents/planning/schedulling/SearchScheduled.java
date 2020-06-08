package org.agents.planning.schedulling;

import java.util.HashMap;

public class SearchScheduled {
    //final int INDEX_OF_BOXES = 2;
    public static final int INDEX_OF_AGENTS = 0;
    public static final int START_GROUP_AGENTS = 1;
    public static final int INDEX_OF_GROUP = 2;
    private int[][] total_group;
    private HashMap<Integer,int[]> agents_idxs_to_boxes_idxs;

    void setTotalGroup(int[][] totalGroup) {
        this.total_group = totalGroup;
    }

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

    public HashMap<Integer,int[]> getAgentstIdxsToBoxesIdxs() {
        return this.agents_idxs_to_boxes_idxs;
    }
}
