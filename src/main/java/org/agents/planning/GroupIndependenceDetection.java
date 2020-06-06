package org.agents.planning;

import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.ConflictAvoidanceTable;
import org.agents.planning.conflicts.IllegalPath;
import org.agents.searchengine.PathProcessing;


import java.io.IOException;
import java.util.*;

public final class GroupIndependenceDetection {
    private final ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;
    public ConflictAvoidanceTable conflict_avoidance_table;
    private final GroupSearch group_search_strategy;
    private final PathProcessing pathProcessing;

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
    public GroupIndependenceDetection(ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules) {
        this.conflict_avoidance_checking_rules = conflictAvoidanceCheckingRules;
        this.conflict_avoidance_table = this.conflict_avoidance_checking_rules.getConflictsTable();
        this.group_search_strategy = new GroupSearch(this.conflict_avoidance_checking_rules);

        pathProcessing = new PathProcessing();
    }

    public boolean runIndependenceDetection() throws IOException {
        /*IllegalPath illegalPath = conflict_avoidance_checking_rules.getIllegalPathsStore().pollNextIllegalPath();

        //het the illegal paths for single search movables
        assert illegalPath != null;
        int[] group_1 = illegalPath.getStartGroup();
        assert group_1.length == 1;
        int[] group_2 = illegalPath.getConflictingGroup();
        assert group_2.length == 1;
        colided_ids[0] = group_1[0];
        colided_ids[1] = group_2[0];*/
        //to remove :
        boolean isColided = false;
        isColided = this.conflict_avoidance_checking_rules.setNextConflictedMovables(colided_ids);

        while(isColide()){
            int movable_id_one = this.getFirstColide();
            int movable_id_two = this.getSecondColide();

            Set<Integer> groupone = this.conflict_avoidance_table.getGroupOf(movable_id_one);
            Set<Integer> grouptwo = this.conflict_avoidance_table.getGroupOf(movable_id_two);

            int [] group_one = groupone.stream().mapToInt(Integer::intValue).toArray();
            int [] group_two = grouptwo.stream().mapToInt(Integer::intValue).toArray();

            int[][][] conflicting_path;
            boolean is_removed;
            if (this.conflict_avoidance_table.isNewConflict(group_one, group_two)){
                conflicting_path = this.conflict_avoidance_table.getMarkedPaths(group_two);
                ArrayDeque<int[]> new_path_one = this.group_search_strategy.runGroupSearch(group_one, group_two, conflicting_path);
                assert new_path_one != null;
                int path_lenght_one = this.conflict_avoidance_table.getPathLenght(group_one);

                boolean is_replaced;
                if (path_lenght_one == new_path_one.size()){
                    //replace with new path optimal
                    pathProcessing.resetTimeSteps(new_path_one);
                    //this.conflict_avoidance_checking_rules.clearTaskScheduledList();
                    is_replaced = this.conflict_avoidance_checking_rules.replaceTaskScheduledFor(group_one, new_path_one);
                    this.conflict_avoidance_table.replaceMarkedPathFor(group_one, new_path_one);
                    //keep the  other path
                    isColided = this.conflict_avoidance_checking_rules.setNextConflictedMovables(colided_ids);//colided_ids registes conflicts
                    //continue;

                }else {
                    conflicting_path = this.conflict_avoidance_table.getMarkedPaths(group_one);
                    ArrayDeque<int[]> new_path_two = this.group_search_strategy.runGroupSearch(group_two, group_one, conflicting_path );
                    //group_two and new_path_two have the same ordering of indexes
                    assert new_path_two != null;
                    int path_lenght_two = this.conflict_avoidance_table.getPathLenght(group_two);

                    if (path_lenght_two == new_path_two.size()){
                        //replace with new path optimal
                        pathProcessing.resetTimeSteps(new_path_two);
                        is_replaced = this.conflict_avoidance_checking_rules.replaceTaskScheduledFor(group_two, new_path_two);
                        this.conflict_avoidance_table.replaceMarkedPathFor(group_two, new_path_two);
                        //keep the  other path
                        isColided = this.conflict_avoidance_checking_rules.setNextConflictedMovables(colided_ids);
                    }else {
                        //the paths groups are removed when grouped
                        int[] group_marks_total = this.conflict_avoidance_table.groupIDs(group_one, group_two);
                        is_removed = this.conflict_avoidance_checking_rules.removeTaskScheduledFor(group_marks_total);
                        ArrayDeque<int[]> paths = this.group_search_strategy.runGroupSearchMA(group_marks_total);
                        pathProcessing.resetTimeSteps(paths);
                        //remove later when conflicted
                        //this.conflict_avoidance_checking_rules.clearTaskScheduledList();
                        boolean is_added = this.conflict_avoidance_checking_rules.addPathsToTaskScheduledPahs(group_one, group_two, group_marks_total, paths);
                        this.conflict_avoidance_table.addMarkedPathsFor(group_marks_total, paths);
                        isColided = this.conflict_avoidance_checking_rules.setNextConflictedMovables(colided_ids);
                    }
                }
            }
            else{
                int[] group_marks_total = this.conflict_avoidance_table.groupIDs(group_one, group_two);
                is_removed = this.conflict_avoidance_checking_rules.removeTaskScheduledFor(group_marks_total);
                ArrayDeque<int[]> paths = this.group_search_strategy.runGroupSearchMA(group_marks_total);
                pathProcessing.resetTimeSteps(paths);
                //remove later when conflicted
                this.conflict_avoidance_checking_rules.addPathsToTaskScheduledPahs(group_one, group_two, group_marks_total, paths);
                this.conflict_avoidance_table.addMarkedPathsFor(group_marks_total, paths);
                isColided = this.conflict_avoidance_checking_rules.setNextConflictedMovables(colided_ids);
            }
         }

        return isColided;
    }
}


