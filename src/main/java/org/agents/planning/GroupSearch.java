package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.IllegalPathsStore;
import org.agents.planning.schedulling.SearchScheduled;
import org.agents.planning.schedulling.TrackedGroups;
import org.agents.searchengine.SearchEngineOD;
import org.agents.searchengine.SearchEngineSA;
import org.agents.searchengine.SearchTaskResult;
import org.agents.searchengine.StateSearchMAFactory;

import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.Set;
import java.util.UUID;


public final class GroupSearch {
    private final TrackedGroups root_tracked_group;
    private final ArrayDeque<TrackedGroups> in_process_tracked_groups;

    private final ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;
    private StateSearchMAFactory.SearchState search_state = StateSearchMAFactory.SearchState.AGENTS_AND_BOXES;
    private SearchTaskResult search_task_independence_detection;

    public GroupSearch(TrackedGroups trackedGroups) {
        this.conflict_avoidance_checking_rules = new ConflictAvoidanceCheckingRules(trackedGroups);
        Set<Integer> agts = trackedGroups.getAgentsScheduledIds();
        Set<Integer> boxes = trackedGroups.getBoxesScheduledIds();
        //this.conflict_avoidance_checking_rules.setTrackedGroups(trackedGroups);
        this.root_tracked_group = trackedGroups;
        in_process_tracked_groups = new ArrayDeque<>();

        setUpIndependenceDetection();
    }

    public SearchTaskResult runAgentsSearchMA(SearchScheduled sched_group, int[] conflicting_group, int[][][] conflicting_paths) throws IOException {
        int[] start_group_agents = sched_group.getTotalGroup()[SearchScheduled.START_GROUP_AGENTS];
        setSearchState(StateSearchMAFactory.SearchState.AGENTS_ONLY);
        setNextTrackedAgents(start_group_agents);


        SearchTaskResult searchTaskResult = getSearchMATaskResult(start_group_agents, conflicting_group, conflicting_paths);
        searchTaskResult.setTotalGroup(sched_group.getTotalGroup());
        searchTaskResult.setAgentstIdxsToBoxesIdxs(sched_group.getAgentstIdxsToBoxesIdxs());

        UUID uniqueID = UUID.randomUUID();
        sched_group.setUUID(uniqueID);
        searchTaskResult.setUUID(uniqueID);

        return searchTaskResult;
    }

    public SearchTaskResult runAgentsBoxesSearchMA(SearchScheduled sched_group, int[] conflicting_group, int[][][] conflicting_paths) throws IOException {
        int[] start_group = sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];
        setSearchState(StateSearchMAFactory.SearchState.AGENTS_AND_BOXES);
        setNextTrackedAgents(start_group);

        SearchTaskResult searchTaskResult = getSearchMATaskResult(start_group, conflicting_group, conflicting_paths);
        searchTaskResult.setTotalGroup(sched_group.getTotalGroup());
        searchTaskResult.setAgentstIdxsToBoxesIdxs(sched_group.getAgentstIdxsToBoxesIdxs());

        UUID uniqueID = UUID.randomUUID();
        sched_group.setUUID(uniqueID);
        searchTaskResult.setUUID(uniqueID);

        return searchTaskResult;
    }

    private SearchTaskResult getSearchMATaskResult(int[] start_group, int[] conflicting_group, int[][][] conflicting_paths ) throws IOException {
        boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.AVOID_PATH);

        IllegalPathsStore illegalPathsStore = conflict_avoidance_checking_rules.getIllegalPathsStore();
        illegalPathsStore.removeAllIlegalPaths();
        if (!(conflicting_group.length == 0))
            conflict_avoidance_checking_rules.setIllegalPathsOfGroup(start_group, conflicting_group, conflicting_paths);

        SearchEngineOD searchEngineOD = new SearchEngineOD(start_group, this.conflict_avoidance_checking_rules, this.search_state);

        int[] start_coordinates = searchEngineOD.getStartCoordinatesOfGroup();
        int[] goal_coordinates = searchEngineOD.getGoalsCoordinatesOfGroup();
        searchEngineOD.runOperatorDecomposition();

        SearchTaskResult searchTaskResult = searchEngineOD.getPath();
        searchTaskResult.setTrackedGroup(this.in_process_tracked_groups.pop());

        return searchTaskResult;
    }


    private void setNextTrackedAgents(int[] start_group_agents) {
        Set<Integer> agents_to_schedule = new HashSet<>();
        for (int start_group_agent : start_group_agents) {
            agents_to_schedule.add(start_group_agent);
        }

        TrackedGroups trackedGroups;
        if (this.root_tracked_group.getAgentsScheduledIds().containsAll(agents_to_schedule)){
            trackedGroups = new TrackedGroups(agents_to_schedule, new HashSet<>());
            trackedGroups.initIdsIndexes(start_group_agents , new int[0]);
            this.conflict_avoidance_checking_rules.setTrackedGroups(trackedGroups);
            this.in_process_tracked_groups.add(trackedGroups);
        }else {
            throw new IllegalArgumentException("agent id unknown");
        }
    }
    //called by GroupIndependenceDetection
    public SearchTaskResult runGroupSearch(SearchScheduled sched_group, int[] conflicting_group, int[][][] conflicting_paths) throws IOException {
        //boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.CHECK_TIME_DEADLINE);
        boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.AVOID_PATH);

        ArrayDeque<int[]> new_path_one;

        int[] start_group =  sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];
        if(start_group.length == 1){
            IllegalPathsStore illegalPathsStore = conflict_avoidance_checking_rules.getIllegalPathsStore();
            conflict_avoidance_checking_rules.setIllegalPathsOfGroup(start_group, conflicting_group, conflicting_paths);

            SearchEngineSA searchEngineSA = new SearchEngineSA(conflict_avoidance_checking_rules);

            int mark_id = start_group[0];
            Serializable next_movable = MapFixedObjects.getByMarkNo(mark_id);

            if (next_movable instanceof Agent) {
                searchEngineSA.runAstar(  (Agent)next_movable );
            }else if (next_movable instanceof Box) {
                searchEngineSA.runAstar( (Box)next_movable );
            }
            this.search_task_independence_detection = searchEngineSA.getPath();

            illegalPathsStore.removeAllIlegalPaths();
        }else{
            this.search_task_independence_detection = this.runAgentsBoxesSearchMA(sched_group, conflicting_group, conflicting_paths);
         }

        return this.search_task_independence_detection;
    }

    public SearchTaskResult runGroupIndependenceDetection(SearchScheduled sched_group) throws IOException {
        int[] conflicting_group = new int[0];
        int[][][] conflicting_paths = new int[0][][];
        return this.runGroupIndependenceDetection(sched_group, conflicting_group, conflicting_paths);
    }

    private SearchTaskResult runGroupIndependenceDetection(SearchScheduled sched_group, int[] conflicting_group, int[][][] conflicting_paths) throws IOException {
    /*  * make coupled search from GroupIndependenceDetection to SearchEngineOD
        *   objects SearchTaskResult,SearchScheduled remain the same acroos the running of GroupIndependenceDetection
        *   only internals change
        * */

        int[] start_group = sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];//?????????
         //setNextTrackedAgents(start_group);//?????????

        boolean isChanged = this.conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.AVOID_PATH);//?????????

        IllegalPathsStore illegalPathsStore = conflict_avoidance_checking_rules.getIllegalPathsStore();
        illegalPathsStore.removeAllIlegalPaths();
        conflict_avoidance_checking_rules.setIllegalPathsOfGroup(start_group, conflicting_group, conflicting_paths);

        SearchEngineOD searchEngineOD = new SearchEngineOD(start_group, this.conflict_avoidance_checking_rules, StateSearchMAFactory.SearchState.AGENTS_AND_BOXES);//?????????

        int[] start_coordinates = searchEngineOD.getStartCoordinatesOfGroup();
        int[] goal_coordinates = searchEngineOD.getGoalsCoordinatesOfGroup();
        searchEngineOD.runOperatorDecomposition();

        //this.search_task_independence_detection.setTrackedGroup(this.in_process_tracked_groups.pop());
        this.search_task_independence_detection = searchEngineOD.getPath();
        this.search_task_independence_detection.setTotalGroup(sched_group.getTotalGroup());
        this.search_task_independence_detection.setAgentstIdxsToBoxesIdxs(sched_group.getAgentstIdxsToBoxesIdxs());

        return this.search_task_independence_detection;
    }


    //SearchTaskResult to be used only by the GroupIndependenceDetection algorithm
    private void setUpIndependenceDetection(){
        ArrayDeque<int[]> path_empty = new ArrayDeque<>();
        this.search_task_independence_detection = new SearchTaskResult(path_empty);
    }


    public ConflictAvoidanceCheckingRules getConflictAvoidanceCheckingRules(){ return  this.conflict_avoidance_checking_rules; }

    private void setSearchState(StateSearchMAFactory.SearchState searchState) {
        this.search_state = searchState;
    }

}
