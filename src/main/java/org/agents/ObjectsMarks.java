package org.agents;

import java.io.Serializable;

public final class ObjectsMarks implements Serializable {
    enum BoxField {
        LETTER_MARK_INDEX,
        COLOR_MARK_INDEX,
        SOLVED_STATUS;
    };

    enum CoordinatesField {
        ROW_POS,
        COLUMN_POS;
    };

    enum AgentField {
        NUMBER_MARK_INDEX,
        COLOR_MARK_INDEX,
        SOLVED_STATUS;
    };
}
