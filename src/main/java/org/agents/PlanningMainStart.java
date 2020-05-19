package org.agents;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.planning.GroupIndependenceDetection;
import org.agents.planning.MovablesScheduling;
import org.agents.planning.SearchStrategy;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.searchengine.SearchEngineSA;

import java.util.Arrays;
import java.util.LinkedList;
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

      //  GroupingOfMovables groupingOfMovables = new  GroupingOfMovables();
       // groupingOfMovables.addGroupSearch(MapFixedObjects.getAgentsMarks());
       // groupingOfMovables.addGroupSearch(MapFixedObjects.getBoxesMarks());



    }

    public void start(Agent[] agents, Box[] boxes){
        Set<Box> boxes_unsolved = MapFixedObjects.getBoxes();

        MovablesScheduling movablesScheduling = new MovablesScheduling(new LinkedList<>(boxes_unsolved));
        SearchStrategy search_strategy = new SearchStrategy(this.conflict_avoidance_checking_rules, movablesScheduling);

        SearchEngineSA searchEngine = null;
        search_strategy.runDescenteralizedSearch(new LinkedList<>(Arrays.asList(agents)),searchEngine);

        GroupIndependenceDetection searchGroupStrategy = null;
        searchGroupStrategy.runIndependenceDetection();


    }


}
