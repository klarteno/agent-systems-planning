package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.SolvedStatus;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.schedulling.MovablesScheduling;
import org.agents.planning.schedulling.Synchronization;
import org.agents.planning.schedulling.TaskScheduled;
import org.agents.searchengine.PathProcessing;
import org.agents.searchengine.SearchEngineSA;
import org.agents.searchengine.normal.SearchEngineSANormal;

import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.ListIterator;

public final class SearchStrategy {
    private final MovablesScheduling movablesScheduling;
    ConflictAvoidanceCheckingRules avoidanceCheckingRules;

    public SearchStrategy(MovablesScheduling movablesScheduling, Synchronization synchronised_time) {
        this.movablesScheduling = movablesScheduling;
        //TrackedGroups trackedGroups = movablesScheduling.getTrackedGroups();
        synchronised_time.resetCentralTime();
        this.avoidanceCheckingRules = new ConflictAvoidanceCheckingRules(movablesScheduling.getTrackedGroups(), synchronised_time);
    }

    public ArrayDeque<ListIterator<String>> getPathsSequencial() {
        PathProcessing pathProcessing = new PathProcessing();
        ListIterator<String> path_iter;
        ArrayDeque<ListIterator<String>> paths_iterations = new ArrayDeque<>();
        ArrayList<String> path;

        SearchEngineSANormal searchEngineSANormal = new SearchEngineSANormal(this.avoidanceCheckingRules);

        for (Agent agent:this.movablesScheduling.getAgentsScheduled()) {
            path = pathProcessing.get_moves_agent_goal(agent, searchEngineSANormal);
            path_iter = path.listIterator(path.size());
            paths_iterations.add(path_iter);
        }
        return paths_iterations;

    }

    //the agents has to have goals for the boxes set up
    //TO DO decouple to agregation or commands together with conflict_avoidance_table
    public TaskScheduled runDescenteralizedSearch() {
        SearchEngineSANormal searchEngineSANormal = new SearchEngineSANormal(this.avoidanceCheckingRules);

        ArrayList<Agent> agents = this.movablesScheduling.getAgentsScheduled();
        TaskScheduled taskScheduled = new TaskScheduled();

        for (Agent agent : agents){
            searchEngineSANormal.runAstar(agent);
            if(searchEngineSANormal.isPathFound()){
                ArrayDeque<int[]> agent_path = searchEngineSANormal.getPath();
                int agent_mark = agent.getNumberMark();
                agent.setSolvedStatus(SolvedStatus.GOAL_STEP_SOLVED);
                //conflict_avoidance_table.replaceMarkedPathFor(agent_mark, agent_path);//asyncrounouse

                taskScheduled.add(agent, agent_path);
            }
        }

       // get the agent path and status , get the boxes solved therir path add them
           //     to taskScheduled in the movables scheduling , add to conflict checking rules

        LinkedList<Box> boxes = this.movablesScheduling.getBoxesScheduled();
        for (Box box : boxes){
            searchEngineSANormal.runAstar(box);
            if(searchEngineSANormal.isPathFound()){
                ArrayDeque<int[]> box_path = searchEngineSANormal.getPath();
                int box_mark = box.getLetterMark();
                box.setSolvedStatus(SolvedStatus.GOAL_FINAL_SOLVED);
                //conflict_avoidance_table.replaceMarkedPathFor(box_mark, box_path);

                taskScheduled.add(box, box_path);
            }
        }

        //scheduled tasks are pushed to be stored in ConflictAvoidanceCheckingRules
        //only ConflictAvoidanceCheckingRules can invalidate all or some TaskScheduled , create new out of mmore TaskScheduled
        TaskScheduled task_scheduled2 = this.movablesScheduling.getSearchResults();

        return taskScheduled;
    }

    //this SearchStrategy will set the SynchronizedTime  in the ConflictAvoidanceCheckingRules , ConflictAvoidanceCheckingRules maintains the overall time clock
    //and gives a final TaskScheduled with paths from the centralized search
    public ArrayList<TaskScheduled> runCentralizedSearch(ArrayDeque<TaskScheduled> paths_found) throws IOException {
        while (!paths_found.isEmpty())
            this.avoidanceCheckingRules.addTaskScheduledPaths(paths_found.pop());


        GroupIndependenceDetection searchGroupStrategy = new GroupIndependenceDetection(this.avoidanceCheckingRules);
        boolean are_paths_found = searchGroupStrategy.runIndependenceDetection();

        ArrayList<TaskScheduled> task_result = new ArrayList<>();
        if (are_paths_found)
            task_result = this.avoidanceCheckingRules.getValidTasks();

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
