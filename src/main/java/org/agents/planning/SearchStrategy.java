package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.SolvedStatus;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.schedulling.TaskScheduled;
import org.agents.searchengine.PathProcessing;
import org.agents.searchengine.SearchEngineSA;

import java.io.Serializable;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.ListIterator;

//TO DO send  TaskScheduled to SearchEngineSA instead of depeding on it on some publih subscribe
public class SearchStrategy {
    private final MovablesScheduling movablesScheduling;
    ConflictAvoidanceCheckingRules avoidanceCheckingRules;

    public SearchStrategy(MovablesScheduling movablesScheduling) {
        this.movablesScheduling = movablesScheduling;
        //TrackedGroups trackedGroups = movablesScheduling.getTrackedGroups();
        this.avoidanceCheckingRules = new ConflictAvoidanceCheckingRules(movablesScheduling.getTrackedGroups());
    }

    public ArrayDeque<ListIterator<String>> getPathsSequencial() {
        PathProcessing pathProcessing = new PathProcessing();
        ListIterator<String> path_iter;
        ArrayDeque<ListIterator<String>> paths_iterations = new ArrayDeque<>();
        ArrayList<String> path;

        SearchEngineSA searchEngine = new SearchEngineSA(this.avoidanceCheckingRules);

        for (Agent agent:this.movablesScheduling.getAgentsScheduled()) {
            path = pathProcessing.get_moves_agent_goal(agent, searchEngine);
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
    public TaskScheduled runDescenteralizedSearch() {
        SearchEngineSA searchEngine = new SearchEngineSA(avoidanceCheckingRules);

        ArrayList<Agent> agents = movablesScheduling.getAgentsScheduled();
        TaskScheduled taskScheduled = new TaskScheduled();

        for (Agent agent : agents){
            searchEngine.runAstar(agent);
            if(searchEngine.isPathFound()){
                ArrayDeque<int[]> agent_path = searchEngine.getPath();
                int agent_mark = agent.getNumberMark();
                agent.setSolvedStatus(SolvedStatus.GOAL_STEP_SOLVED);
                //conflict_avoidance_table.replaceMarkedPathFor(agent_mark, agent_path);//asyncrounouse

                taskScheduled.add(agent, agent_path);
            }
        }

       // get the agent path and status , get the boxes solved therir path add them
           //     to taskScheduled in the movables scduling , add to conflict checking rules

        LinkedList<Box> boxes = movablesScheduling.getBoxesScheduled();//by agent with goal satisfied???
        for (Box box : boxes){
            searchEngine.runAstar(box);
            if(searchEngine.isPathFound()){
                ArrayDeque<int[]> box_path = searchEngine.getPath();
                int box_mark = box.getLetterMark();
                box.setSolvedStatus(SolvedStatus.GOAL_FINAL_SOLVED);
                //conflict_avoidance_table.replaceMarkedPathFor(box_mark, box_path);

                taskScheduled.add(box, box_path);
            }
        }

        //scheduled tasks are pushed to be stored in ConflictAvoidanceCheckingRules
        //only ConflictAvoidanceCheckingRules can invalidate all or some TaskScheduled , create new out of mmore TaskScheduled
        TaskScheduled task_scheduled2 = movablesScheduling.getSearchResults();
        //this.conflict_avoidance_checking_rules.addTaskScheduled(task_scheduled2);

        return taskScheduled;
    }

    public TaskScheduled runCentralizedSearch(ArrayDeque<TaskScheduled> paths_found){
        while (!paths_found.isEmpty())
            this.avoidanceCheckingRules.addTaskScheduledPaths(paths_found.pop());

        GroupIndependenceDetection searchGroupStrategy = new GroupIndependenceDetection(this.avoidanceCheckingRules);
        searchGroupStrategy.runIndependenceDetection();

        TaskScheduled task_result = this.avoidanceCheckingRules.getNextValidTask();

        return task_result;
    }

    public ArrayDeque<int[]> runSearch(SearchEngineSA searchEngine, int movable_id) {
        Serializable obj = MapFixedObjects.getByMarkNo(movable_id);

        if (obj instanceof Box){
            Box box = (Box)obj;
            searchEngine.runAstar(box );
            if(searchEngine.isPathFound()){
                int box_mark = box.getLetterMark();

                return  searchEngine.getPath();
            }

        }else if (obj instanceof Agent){
            Agent agent = (Agent)obj;
            searchEngine.runAstar(agent);
            if(searchEngine.isPathFound()){
                int agent_mark = agent.getNumberMark();

                return searchEngine.getPath();
            }

        }else{
            throw new UnsupportedOperationException("unknown movable id cast from Serializable");
        }
        return null;
    }
}
