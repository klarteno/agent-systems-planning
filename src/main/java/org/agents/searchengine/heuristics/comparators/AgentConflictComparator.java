package org.agents.searchengine.heuristics.comparators;

import java.util.Comparator;

public class AgentConflictComparator   {


    public int compare( int getNumberAgentsConflicts1, int getNumberAgentsConflicts2) {

        if (getNumberAgentsConflicts1 < getNumberAgentsConflicts2){
            return -1;
        }
        else if(getNumberAgentsConflicts1 == getNumberAgentsConflicts2 ) {
            return 0;
        }
        return 1;
    }
}
