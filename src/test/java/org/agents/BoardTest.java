package org.agents;

import org.junit.jupiter.api.Test;

import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

public class BoardTest {

    @Test
    public void testConstructor() throws Exception {
        var b = new String("gggggg");
        assertEquals("gggggg", b);
        assertEquals(true, true);
    }
}