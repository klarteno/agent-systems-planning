package sysmain;

import org.agents.*;
import org.agents.markings.SolvedStatus;
import org.agents.planning.GroupSearch;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.ConflictAvoidanceTable;
import org.agents.planning.schedulling.*;
import org.agents.searchengine.PathProcessing;
import org.agents.searchengine.StateSearchMAFactory;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.DecimalFormat;
import java.util.*;

/**
 * This class is used to lunch project
 *
 * @author autor
 *  {@link ConflictAvoidanceTable} interface
 *   to handle updates of path
 */
public final class Main{

    enum SearchingMode {
        SINGLE,
        GROUP,
        MERGING,
        CANCELLING,
        GOAL_REACHED;
    };

    private static final DecimalFormat df = new DecimalFormat("0.0000");

    static int[][] total_group_copy;

    /**
     * This is the main method of our application.
     * @param args {@link String} Input arguments.
     */
    public static void main(String[] args)
    {
        try
        {
            long startTime = System.nanoTime();

            BufferedReader serverMessages = new BufferedReader(new InputStreamReader(System.in));
            // Use stderr to print to console
            System.out.println("#SearchClient initializing. I am sending this using the error output stream.");

            // Read level and create the initial state of the problem
            SearchClient client = new SearchClient(serverMessages);
            client.parse();

            MapFixedObjects mapFixedObjects = client.initObjects();

            mapFixedObjects.setUpTrackedMovables(MapFixedObjects.getAgents().toArray(new Agent[0]), MapFixedObjects.getBoxes().toArray(new Box[0]));

            ////////        entry--point        ///////////////////
            //ArrayList<String[]> pathOperatorDecompositionSearch = getPathOperatorDecompositionSearch();
            ArrayList<String[]> pathOperatorDecompositionSearch = getPathOperatorDecompositionScheduling();
            ////////        entry--point        ///////////////////

             ListIterator<String[]> path_iter = pathOperatorDecompositionSearch.listIterator(pathOperatorDecompositionSearch.size());
            ArrayDeque<ListIterator<String[]>> paths_iterations = new ArrayDeque<>();
            paths_iterations.add(path_iter);

            String[] no_of_slots = pathOperatorDecompositionSearch.get(0);
            outputPathsForAgents(serverMessages, total_group_copy, no_of_slots.length,  paths_iterations.pop());

            long endTime = System.nanoTime();
            String duration = df.format( (endTime - startTime) / 1000000000.0 );
            System.out.println("#Result time duration : " + duration    );
        }
        catch (Throwable e) {
            e.printStackTrace();
        }
    }

    private static void outputPathsForAgents(BufferedReader serverMessages, int[][] total_group_copy, int slots_length, ListIterator<String[]> path_iter) throws IOException {
        int[] index_agents = total_group_copy[0];
        int[] start_group = total_group_copy[1];

        String[] msg1 = new String[slots_length + 1];
        Arrays.fill(msg1, "NoOp");
        msg1[slots_length]=  System.lineSeparator();

        String[] msg_commnand_list;
        int path_iter_index = 0;
        while (path_iter.hasPrevious()){
            msg_commnand_list = path_iter.previous();
            path_iter_index++;

            for (int i = 0; i < msg_commnand_list.length; i++) {
                if(msg_commnand_list[i] == null || msg_commnand_list[i].equals("null")){
                    msg1[i]= "NoOp";
                    System.out.print("#msg_commnand_list was null for agent: " + i +"at path_iter_index: " + path_iter_index );
                    System.out.println("# new message out of null:  " + msg1[i]);

                }else {
                    msg1[i]= msg_commnand_list[i];
                }
            }

            String joinedString =  String.join(";", msg1);
            System.out.print(joinedString);

            String response = serverMessages.readLine();
            //System.out.println("#response from srv:"+response);

            if (response.contains("false")) {
                System.err.println("#Server responsed with: "+ response + " to the inapplicable action: " + joinedString + " action number: " + path_iter_index + "\n");
                break;
            }
        }
    }

    public synchronized static ArrayList<String[]> getPathOperatorDecompositionScheduling() throws Exception {
        // mapFixedObjects.setUpTrackedMovables(MapFixedObjects.getAgents(), MapFixedObjects.getBoxes())
        LinkedList<Agent> agents_unsolved = MapFixedObjects.getAgents();

        DivideAndScheduleMovables divideAndScheduleMovables = new DivideAndScheduleMovables(MapFixedObjects.getAllBoxesIds());
        MovablesScheduling movables_scheduling = divideAndScheduleMovables.getAgentsScheduled(agents_unsolved);

        Set<Integer> agents_ids = movables_scheduling.getAgentsIds();
        Set<Integer> boxes_ids = movables_scheduling.getBoxesIds();
        SearchScheduled sched_group = movables_scheduling.getStartGroupAgentsBoxes_ToSearch();
        int[][] total_group = sched_group.getTotalGroup();

        int[] start_group_agents = total_group[SearchScheduled.START_GROUP_AGENTS];
        int[] index_agents = total_group[SearchScheduled.INDEX_OF_AGENTS];
        int[] start_group = total_group[SearchScheduled.INDEX_OF_GROUP];
        HashMap<Integer, int[]> agents_idx_to_boxes_idx = sched_group.getAgentstIdxsToBoxesIdxs();

        TrackedGroups trackedGroups = new TrackedGroups(agents_ids, boxes_ids);
        Synchronization synchronised_time = new Synchronization();
        ConflictAvoidanceCheckingRules avoidanceCheckingRules = new ConflictAvoidanceCheckingRules(trackedGroups, synchronised_time);
        GroupSearch groupSearch = new GroupSearch(avoidanceCheckingRules);

        int[] conflicting_group = new int[0];
        int[][][] conflicting_paths = new int[0][][];

        total_group_copy = Arrays.copyOf(total_group, total_group.length);
        for (int row = 0; row < total_group.length; row++) {
            total_group_copy[row] = Arrays.copyOf(total_group[row], total_group[row].length);
        }

        for (int i = 0; i < start_group_agents.length; i++) {
            System.out.println("#start_group_agents: " + i +" "+ start_group_agents[i] + " ");
        }

        StateSearchMAFactory.SearchState searchState_1 = StateSearchMAFactory.SearchState.AGENTS_ONLY;
        groupSearch.setSearchState(searchState_1);
        ArrayDeque<int[]> path_found_1 = groupSearch.runGroupSearchMA(start_group_agents, conflicting_group, conflicting_paths);
        PathProcessing pathProcessing = new PathProcessing();
        pathProcessing.resetTimeSteps(path_found_1);
        divideAndScheduleMovables.setAgentsGoalsFound(start_group_agents , path_found_1);


        int[] final_agents_position = pathProcessing.getValidAgentsGoalCoordinates(path_found_1);
        int[] box_pos = path_found_1.peek();
        if(path_found_1.size() > 0){
            //set start_group_agents solved status to finding boxes
            // // here the coordinates of agents became goal coordinates
            boolean is_solved_changed = movables_scheduling.setAgentsScheduledSolvedResults(start_group_agents, final_agents_position, SolvedStatus.GOAL_STEP_SOLVED);
            //test that the coordinates of agents became goal coordinates;
            Agent agent2 = MapFixedObjects.getByAgentMarkId(start_group_agents[1]);
            int[] g_pos2 = agent2.getGoalPosition();
            int[] pos2 = agent2.getCoordinates();
             Agent agent1 = MapFixedObjects.getByAgentMarkId(start_group_agents[0]);
            int[] g_pos1 = agent1.getGoalPosition();
            int[] pos1 = agent1.getCoordinates();
        }

        //ArrayDeque<int[]> path_found = groupSearch.runGroupSearchMA(start_group, conflicting_group, conflicting_paths);
         String[] path_not_found = new String[index_agents.length];
        for (int i = 0; i < path_not_found.length; i++) {
            path_not_found[i] = "PATH NOT FOUND";
        }

        ArrayList<String[]> path_output_found_1 = new ArrayList<>();
        if (path_found_1.size() > 0){
            //pathProcessing = new PathProcessing();
            //pathProcessing.resetTimeSteps(path_found);
            /////////path processing
            path_output_found_1 = pathProcessing.getMAAgentMoves(path_found_1, index_agents);
            System.out.println("#path_output_found_1 : ");

            for (String[] __msg : path_output_found_1){
                System.out.println("#__msg_1:  " + Arrays.toString(__msg));
            }

            String[] msg111 = path_output_found_1.get(path_output_found_1.size() - 1);
            System.out.println("#msg111  msg111 msg111:  " + Arrays.toString(msg111));


            for (int i = path_output_found_1.size() - 1; i > path_output_found_1.size() - 4 ; i--) {
                String msg_i = Arrays.toString(path_output_found_1.get(i));
                if (msg_i.equals("NoOp;NoOp;")){
                    System.out.println("#msg111 true true:  " + msg_i);

                }
            }



            //path_output_found = pathProcessing.getMAAgentBoxesMoves(path_found, index_agents, agents_idx_to_boxes_idx);
        }else {
            System.out.println("######################## PATH NOT FOUND######################");
        }
        System.out.println("#path_output_found: " + path_output_found_1.size());


        ///////////////////////////////next search /////////////////////////////////////
        System.out.println("######################## NEXT SEARCH ######################");

        TaskScheduled task_scheduled = divideAndScheduleMovables.getSearchResults();

        ArrayList<Integer> agents_solved = task_scheduled.getAgentsSolved(SolvedStatus.GOAL_STEP_SOLVED);
        Set<Map.Entry<Integer, ArrayDeque<Integer>>> agents_to_boxes_ = task_scheduled.getAgentsToBoxes();
        Set<Integer> res1 = task_scheduled.getGroupMarksSolved();
        int[] __agts = task_scheduled.getValidAgents();
        int[] __bxs = task_scheduled.getValidBoxes();
        int[] ___group_total = task_scheduled.getGroupsMarks();

        SearchScheduled search_sched = movables_scheduling.getMatchedAgentsBoxesIndexes(__agts, __bxs);
        total_group = search_sched.getTotalGroup();

        index_agents = total_group[SearchScheduled.INDEX_OF_AGENTS];
        start_group = total_group[SearchScheduled.INDEX_OF_GROUP];
        agents_idx_to_boxes_idx = search_sched.getAgentstIdxsToBoxesIdxs();

        for (int i = 0; i < start_group.length; i++) {
            System.out.println("#start_group agents with boxes: " + i +" "+ start_group[i] + " ");
        }

        //change the start goal coordinates in runGroupSearchMA :
        ///movables_scheduling.setTupStartCoordinatesAgents(agents_solved)
        StateSearchMAFactory.SearchState searchState_2 = StateSearchMAFactory.SearchState.AGENTS_AND_BOXES;
        groupSearch.setSearchState(searchState_2);
        ArrayDeque<int[]> path_found_2 = groupSearch.runGroupSearchMA(start_group, conflicting_group, conflicting_paths);

        ArrayList<String[]> path_output_found_2 = new ArrayList<>();
        if (path_found_2.size() > 0){
            pathProcessing.resetTimeSteps(path_found_2);
            /////////path processing
            //path_output_found = pathProcessing.getMAAgentMoves(path_found, index_agents);
            path_output_found_2 = pathProcessing.getMAAgentBoxesMoves(path_found_2, index_agents, agents_idx_to_boxes_idx);
        }else {
            System.out.println("########################PATH NOT FOUND######################");
        }

        System.out.println("#path_output_found_2:" + path_output_found_2.size());
        if (path_output_found_2.size() > 0){
            //System.out.println("#path_output_found_2:" + Arrays.toString(path_output_found_2.get(0)));
            for (String[] __msg : path_output_found_2){
                System.out.println("#__msg_2:  " + Arrays.toString(__msg));
            }
        }

        path_output_found_2.addAll(path_output_found_1);

        return path_output_found_2;
    }
}
