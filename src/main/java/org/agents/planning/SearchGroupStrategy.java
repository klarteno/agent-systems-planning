package org.agents.planning;

import datastructures.DisjointSet;
import org.agents.Agent;
import org.agents.Box;

import java.util.HashMap;

public class SearchGroupStrategy {
    private ConflictAvoidanceTable conflictAvoidanceTable;
    SearchStrategy searchStrategy;

    enum SearchingMode {
        SINGLE,
        GROUP,
        MERGING,
        CANCELLING,
        GOAL_REACHED;
    };

//conflictAvoidanceTable already contains agents and boxes
    public SearchGroupStrategy(ConflictAvoidanceTable conflictAvoidanceTable, Agent[] agents, Box[] boxes) {
        this.conflictAvoidanceTable = conflictAvoidanceTable;
        assert this.conflictAvoidanceTable.getTrackedIDs().keySet().size()>0;

        searchStrategy.runDescenteralizedSearch(conflictAvoidanceTable,agents,boxes);

    }

    public void runIndependenceDetection(){
        HashMap<Integer, Integer> ordered_ids = this.conflictAvoidanceTable.getTrackedIDs();
        DisjointSet group_set = new DisjointSet(ordered_ids.size());



        }
    }


