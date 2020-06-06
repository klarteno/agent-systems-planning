package sysmain;

import org.agents.*;
import org.agents.planning.GroupSearch;
import org.agents.planning.MovablesScheduling;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.ConflictAvoidanceTable;
import org.agents.planning.schedulling.Synchronization;
import org.agents.planning.schedulling.TrackedGroups;
import org.agents.searchengine.PathProcessing;

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
                if(msg_commnand_list[i] == null){
                    msg1[i]= "NoOp";
                    System.out.println("#msg_commnand_list was null for agent: " + i +"at path_iter_index: " + path_iter_index );
                }
                msg1[i]= msg_commnand_list[i];
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
        final int INDEX_OF_AGENTS = 0;
        final int START_GROUP_AGENTS = 1;
        final int INDEX_OF_GROUP = 2;
        // mapFixedObjects.setUpTrackedMovables(MapFixedObjects.getAgents(), MapFixedObjects.getBoxes())
        LinkedList<Agent> agents_unsolved = MapFixedObjects.getAgents();

        DivideAndScheduleMovables divideAndScheduleMovables = new DivideAndScheduleMovables(MapFixedObjects.getAllBoxesIds());
        MovablesScheduling movables_scheduling = divideAndScheduleMovables.getAgentsScheduled(agents_unsolved);
        ArrayList<Agent> agents_sched = movables_scheduling.getAgentsScheduled();

        Set<Integer> agents_ids = movables_scheduling.getAgentsIds();
        Set<Integer> boxes_ids = movables_scheduling.getBoxesIds();

        int[][] total_group = movables_scheduling.getStartGroupAgentsBoxes_ToSearch();

        int[] index_agents = total_group[INDEX_OF_AGENTS];

        total_group_copy = Arrays.copyOf(total_group, total_group.length);
        for (int row = 0; row < total_group.length; row++) {
            total_group_copy[row] = Arrays.copyOf(total_group[row], total_group[row].length);
        }
        int[] start_group_agents = total_group[START_GROUP_AGENTS];
        int[] start_group = total_group[INDEX_OF_GROUP];
        HashMap<Integer, int[]> agents_idx_to_boxes_idx = movables_scheduling.getAgentsIdxsToBoxesIdxs();

        TrackedGroups trackedGroups = new TrackedGroups(agents_ids, boxes_ids);
        Synchronization synchronised_time = new Synchronization();
        ConflictAvoidanceCheckingRules avoidanceCheckingRules = new ConflictAvoidanceCheckingRules(trackedGroups, synchronised_time);
        GroupSearch groupSearch = new GroupSearch(avoidanceCheckingRules);

        for (int i = 0; i < start_group.length; i++) {
            System.out.println("#start_group: " + i +" "+ start_group[i] + " ");
        }

        int[] conflicting_group = new int[0];
        int[][][] conflicting_paths = new int[0][][];

        ArrayDeque<int[]> path_found = groupSearch.runGroupSearchMA(start_group_agents, conflicting_group, conflicting_paths);

        //ArrayDeque<int[]> path_found = groupSearch.runGroupSearchMA(start_group, conflicting_group, conflicting_paths);
        ArrayList<String[]> path_output_found = new ArrayList<String[]>();
        String[] path_not_found = new String[index_agents.length];
        for (int i = 0; i < path_not_found.length; i++) {
            path_not_found[i] = "PATH NOT FOUND";
        }

        path_output_found.add(path_not_found);

        if (path_found.size() > 0){
            PathProcessing pathProcessing = new PathProcessing();
            pathProcessing.resetTimeSteps(path_found);
            /////////path processing
            path_output_found = pathProcessing.getMAAgentMoves(path_found, index_agents);
            //path_output_found = pathProcessing.getMAAgentBoxesMoves(path_found, index_agents, agents_idx_to_boxes_idx);

         }else {
            System.out.println("######################## PATH NOT FOUND######################");
        }

        ArrayList<String[]> path_output_found_2 = new ArrayList<>();

        {
            ///////////////////////////////next search /////////////////////////////////////
            System.out.println("######################## NEXT SEARCH ######################");

             divideAndScheduleMovables = new DivideAndScheduleMovables(MapFixedObjects.getAllBoxesIds());
             movables_scheduling = divideAndScheduleMovables.getAgentsScheduled(agents_unsolved);
             agents_sched = movables_scheduling.getAgentsScheduled();

            agents_ids = movables_scheduling.getAgentsIds();
             boxes_ids = movables_scheduling.getBoxesIds();

            total_group = movables_scheduling.getStartGroupAgentsBoxes_ToSearch();
            index_agents = total_group[INDEX_OF_AGENTS];
            start_group = total_group[INDEX_OF_GROUP];

            ArrayDeque<int[]> path_found_2 = groupSearch.runGroupSearchMA(start_group, conflicting_group, conflicting_paths);
            if (path_found.size() > 0){
                PathProcessing pathProcessing = new PathProcessing();
                pathProcessing.resetTimeSteps(path_found);
                /////////path processing
                //path_output_found = pathProcessing.getMAAgentMoves(path_found, index_agents);
                path_output_found_2 = pathProcessing.getMAAgentBoxesMoves(path_found_2, index_agents, agents_idx_to_boxes_idx);

            }else {
                System.out.println("########################PATH NOT FOUND######################");
            }
        }

        System.out.println("#path_output_found_2:" + path_output_found_2.size());
        if (path_output_found_2.size()>0){
            System.out.println("#path_output_found_2:" + Arrays.toString(path_output_found_2.get(0)));

        }

        path_output_found.addAll(path_output_found_2);

        return path_output_found;

    }






}
