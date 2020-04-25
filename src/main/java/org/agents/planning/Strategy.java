package org.agents.planning;

import org.agents.Agent;
import org.agents.MapFixedObjects;
import org.agents.searchengine.PathProcessing;
import org.agents.searchengine.SearchEngine;

import java.util.ArrayList;
import java.util.ListIterator;
import java.util.Stack;

public class Strategy {

    public static Stack<ListIterator<String>> getPathsSequencial(MapFixedObjects mapFixedObjects, SearchEngine searchEngine) {
       PathProcessing pathProcessing = new PathProcessing();
        ListIterator<String> path_iter;
        Stack<ListIterator<String>> paths_iterations = new Stack<>();
        ArrayList<String> path;

        for (int i = mapFixedObjects.agents.length-1; i >=0 ; i--) {
            path = pathProcessing.get_moves_agent_goal(mapFixedObjects.agents[i], searchEngine, mapFixedObjects);
            path_iter = path.listIterator(path.size());
            paths_iterations.add(path_iter);

        }


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


        return paths_iterations;
    }

}
