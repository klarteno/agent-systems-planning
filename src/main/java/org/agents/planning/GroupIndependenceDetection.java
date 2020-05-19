package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.ConflictAvoidanceTable;
import org.agents.searchengine.SearchEngineSA;

import java.util.*;
import java.util.stream.Collectors;

public class SearchGroupStrategy {
    private final ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;
    public ConflictAvoidanceTable conflict_avoidance_table;
    private   SearchStrategy search_strategy;

    private final static int first_collide = 0;
    private final static int second_collide = 1;
    private final int[] colided_ids = new int[]{-1,-1};

    private boolean isColide(){
        return colided_ids[first_collide] != -1;
    }

    private int getFirstColide(){
        return colided_ids[first_collide];
    }

    private int getSecondColide(){
        return colided_ids[second_collide];
    }


     //asssumed the agents,boxes have set up their goals corectly
    public SearchGroupStrategy(ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules) {
        this.conflict_avoidance_checking_rules = conflictAvoidanceCheckingRules;
        this.conflict_avoidance_table = this.conflict_avoidance_checking_rules.getConflictsTable();
     }

    public void runIndependenceDetection(){
        boolean isColided = this.conflict_avoidance_table.getNextConflictedMovables(colided_ids);

        while(isColide()){
            int movable_id_one = this.getFirstColide();
            int movable_id_two = this.getSecondColide();
/*
            if(this.conflict_avoidance_table.isUnGrouped(movable_id_one)&&this.conflict_avoidance_table.isUnGrouped(movable_id_two))
                startIDForSingleAgents(movable_id_one, movable_id_two);
*/
            ArrayDeque<Integer> groupone = this.conflict_avoidance_table.getGroupOf(movable_id_one);
            ArrayDeque<Integer> grouptwo = this.conflict_avoidance_table.getGroupOf(movable_id_two);
            //Integer[] group__one = group_one.toArray(new Integer[0]);
            int [] group_one = groupone.stream().mapToInt(Integer::intValue).toArray();
            int [] group_two = grouptwo.stream().mapToInt(Integer::intValue).toArray();

            int[][][] conflicting_path;
            if (this.conflict_avoidance_table.isNewConflict(group_one, group_two)){
                int path_lenght_one = this.conflict_avoidance_table.getPathLenght(group_one);
                conflicting_path = this.conflict_avoidance_table.getMarkedPaths(group_two);
                ArrayDeque<int[]> new_path_one = this.search_strategy.runGroupSearchMA(group_one, conflicting_path);
               //group_one and new_path_one have the same ordering of indexes
                assert new_path_one != null;

                if (path_lenght_one == new_path_one.size()){
                    //replace with new path optimal
                    this.conflict_avoidance_table.replaceMarkedPathFor(group_one, new_path_one);
                    //keep the  other path
                    isColided = this.conflict_avoidance_table.getNextConflictedMovables(colided_ids);//colided_ids registes conflicts
                    //continue;

                }else {
                    int path_lenght_two = this.conflict_avoidance_table.getPathLenght(group_two);
                    conflicting_path = this.conflict_avoidance_table.getMarkedPaths(group_one);
                    ArrayDeque<int[]> new_path_two = this.search_strategy.runGroupSearchMA(group_two, conflicting_path);
                    //group_two and new_path_two have the same ordering of indexes
                    assert new_path_two != null;
                    if (path_lenght_two == new_path_two.size()){
                        //replace with new path optimal
                        this.conflict_avoidance_table.replaceMarkedPathFor(group_two, new_path_two);
                        //keep the  other path
                        isColided = this.conflict_avoidance_table.getNextConflictedMovables(colided_ids);

                    }else {
                        //the paths groups are removed when grouped
                        int[][] group_marks_total = this.conflict_avoidance_table.groupIDs(group_one, group_two);
                        ArrayDeque<int[]> paths = this.search_strategy.runGroupSearchMA(group_marks_total);
                        this.conflict_avoidance_table.addMarkedPathsFor(group_marks_total, paths);
                        isColided = this.conflict_avoidance_table.getNextConflictedMovables(colided_ids);
                    }
                }
            }else{
                int[][] group_marks_total = this.conflict_avoidance_table.groupIDs(group_one, group_two);
                ArrayDeque<int[]> paths = this.search_strategy.runGroupSearchMA(group_marks_total);
                this.conflict_avoidance_table.addMarkedPathsFor(group_marks_total, paths);
                isColided = this.conflict_avoidance_table.getNextConflictedMovables(colided_ids);
            }
         }
    }
/*
    //it runs independece detection only for single agents and merges them if conflicted
    //to keep if used later
    private boolean startIDForSingleAgents(int movable_id_one, int movable_id_two) {
        boolean isColided;
        if ( this.conflict_avoidance_table.isUnGrouped(movable_id_one)&&this.conflict_avoidance_table.isUnGrouped(movable_id_two)){
            int path_lenght_one = this.conflict_avoidance_table.getPathLenght(movable_id_one);

            ArrayDeque<int[]> path_one = this.search_strategy.runSearchConstrained(movable_id_one, this.conflict_avoidance_table);
            assert path_one != null;

            if (path_lenght_one == path_one.size()){
                //replace with new path optimal
                 this.conflict_avoidance_table.replaceMarkedPathFor(movable_id_one, path_one);
                //keep the  other path
                isColided = this.conflict_avoidance_table.getNextConflictedMovables(colided_ids);
                return isColided;

            }else {
                int path_lenght_two = this.conflict_avoidance_table.getPathLenght(movable_id_two);
                ArrayDeque<int[]> path_two = this.search_strategy.runSearchConstrained(movable_id_two, this.conflict_avoidance_table);
                if (path_lenght_two == path_two.size()){
                    //replace with new path optimal
                    this.conflict_avoidance_table.replaceMarkedPathFor(movable_id_two, path_two);
                    //keep the  other path
                    isColided = this.conflict_avoidance_table.getNextConflictedMovables(colided_ids);
                    return isColided;
                }else {
                    this.conflict_avoidance_table.removeMarkedPath(movable_id_one);
                    this.conflict_avoidance_table.removeMarkedPath(movable_id_two);

                    this.search_strategy.runGroupSearchMA(movable_id_one, movable_id_two);
                    this.conflict_avoidance_table.groupIDs(movable_id_one, movable_id_two);
                    isColided = this.conflict_avoidance_table.getNextConflictedMovables(colided_ids);
                    return isColided;
                }
            }


        }else{
            this.conflict_avoidance_table.groupIDs(movable_id_one,movable_id_two);
            this.search_strategy.runGroupSearchMA(movable_id_one, movable_id_two);

            isColided = this.conflict_avoidance_table.getNextConflictedMovables(colided_ids);


        }
        return isColided;
    }
    */
}


