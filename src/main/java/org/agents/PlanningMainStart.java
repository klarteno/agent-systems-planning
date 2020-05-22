package org.agents;

import org.agents.planning.GroupIndependenceDetection;
import org.agents.planning.MovablesScheduling;
import org.agents.planning.SearchStrategy;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.ConflictAvoidanceTable;
import org.agents.planning.schedulling.TaskScheduled;
import org.agents.planning.schedulling.TrackedGroups;
import org.agents.searchengine.SearchEngineSA;

import java.util.Set;

public class PlanningMainStart {

    private final GroupIndependenceDetection search_group_strategy;
    private ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;

    enum SearchingMode {
        SINGLE,
        GROUP,
        MERGING,
        CANCELLING,
        GOAL_REACHED;
    };
    public  PlanningMainStart(GroupIndependenceDetection searchGroupStrategy) {
        this.search_group_strategy = searchGroupStrategy;

        DivideAndScheduleMovables divideAndScheduleMovables = new DivideAndScheduleMovables();
      //  GroupingOfMovables groupingOfMovables = new  GroupingOfMovables();
       // groupingOfMovables.addGroupSearch(MapFixedObjects.getAgentsMarks());
       // groupingOfMovables.addGroupSearch(MapFixedObjects.getBoxesMarks());
    }

    public void start(Agent[] agents, Box[] boxes){
        Set<Box> boxes_unsolved = MapFixedObjects.getBoxes();

        MovablesScheduling movablesScheduling = new MovablesScheduling();
        SearchStrategy search_strategy = new SearchStrategy(movablesScheduling);

        TrackedGroups trackedGroups = null;
        ConflictAvoidanceTable conflictAvoidanceTable = new ConflictAvoidanceTable(trackedGroups);
        ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules = new ConflictAvoidanceCheckingRules(conflictAvoidanceTable);
        SearchEngineSA searchEngine = new SearchEngineSA(conflictAvoidanceCheckingRules);


        TaskScheduled result_plan = search_strategy.runDescenteralizedSearch(searchEngine);



    }


}
