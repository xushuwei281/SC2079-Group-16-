package com.sc2079.group16.controller;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

/**
 * Unit tests for ArenaView covering obstacle construction, ALG command
 * serialization, face rotation sequence, and arena bounds validation.
 */
public class ArenaViewTest {

    @Test
    public void testObstacleConstructionAndFaceInitialization() {
        ArenaView.Obstacle obstacle = new ArenaView.Obstacle(1, 40f, 60f, 'N');
        assertEquals(1, obstacle.id);
        assertEquals(40f, obstacle.xCm, 0.001f);
        assertEquals(60f, obstacle.yCm, 0.001f);
        assertEquals('N', obstacle.face);
        assertNull("Symbol should be unassigned upon creation", obstacle.recognizedSymbol);

        // Test with different face directions
        ArenaView.Obstacle obsEast = new ArenaView.Obstacle(2, 10f, 20f, 'E');
        assertEquals('E', obsEast.face);

        ArenaView.Obstacle obsSouth = new ArenaView.Obstacle(3, 80f, 90f, 'S');
        assertEquals('S', obsSouth.face);

        ArenaView.Obstacle obsWest = new ArenaView.Obstacle(4, 150f, 180f, 'W');
        assertEquals('W', obsWest.face);

        // Test mutating properties
        obsWest.xCm = 155.5f;
        obsWest.yCm = 175.5f;
        obsWest.face = 'N';
        obsWest.recognizedSymbol = 12;

        assertEquals(155.5f, obsWest.xCm, 0.001f);
        assertEquals(175.5f, obsWest.yCm, 0.001f);
        assertEquals('N', obsWest.face);
        assertEquals(Integer.valueOf(12), obsWest.recognizedSymbol);
    }

    @Test
    public void testBuildAlgCommandEmpty() {
        List<ArenaView.Obstacle> obstacles = new ArrayList<>();
        assertEquals("ALG", ArenaView.buildAlgCommand(obstacles));

        ArenaView arenaView = new ArenaView();
        assertEquals("ALG", arenaView.buildAlgCommand());
    }

    @Test
    public void testBuildAlgCommandSingleAndMultipleObstacles() {
        List<ArenaView.Obstacle> obstacles = new ArrayList<>();
        obstacles.add(new ArenaView.Obstacle(1, 100f, 100f, 'N'));
        assertEquals("ALG|1,100,100,N", ArenaView.buildAlgCommand(obstacles));

        // Test with coordinate rounding and multiple obstacles
        obstacles.add(new ArenaView.Obstacle(2, 25.4f, 40.6f, 'E'));
        obstacles.add(new ArenaView.Obstacle(3, 150.1f, 80.8f, 'S'));
        obstacles.add(new ArenaView.Obstacle(4, 70.0f, 180.0f, 'W'));

        String expected = "ALG|1,100,100,N|2,25,41,E|3,150,81,S|4,70,180,W";
        assertEquals(expected, ArenaView.buildAlgCommand(obstacles));
    }

    @Test
    public void testBuildAlgCommandViaArenaViewInstance() {
        ArenaView arenaView = new ArenaView();

        // Add first obstacle (default is centered at 100, 100, face N)
        arenaView.addObstacle();
        assertEquals(1, arenaView.getObstacleCount());
        assertEquals("ALG|1,100,100,N", arenaView.buildAlgCommand());

        // Add second obstacle
        arenaView.addObstacle();
        assertEquals(2, arenaView.getObstacleCount());
        arenaView.setObstacleFace(2, 'S');
        assertEquals("ALG|1,100,100,N|2,100,100,S", arenaView.buildAlgCommand());

        // Remove obstacle 1
        arenaView.removeObstacle(1);
        assertEquals(1, arenaView.getObstacleCount());
        assertEquals("ALG|2,100,100,S", arenaView.buildAlgCommand());

        // Clear all obstacles
        arenaView.clearObstacles();
        assertEquals(0, arenaView.getObstacleCount());
        assertEquals("ALG", arenaView.buildAlgCommand());
    }

    @Test
    public void testFaceCyclingLogic() {
        // Standard N -> E -> S -> W -> N cycle
        assertEquals('E', ArenaView.nextFace('N'));
        assertEquals('S', ArenaView.nextFace('E'));
        assertEquals('W', ArenaView.nextFace('S'));
        assertEquals('N', ArenaView.nextFace('W'));

        // Full 360 degree rotation returns to start
        char face = 'N';
        face = ArenaView.nextFace(face); // E
        face = ArenaView.nextFace(face); // S
        face = ArenaView.nextFace(face); // W
        face = ArenaView.nextFace(face); // N
        assertEquals('N', face);

        // Fallback for invalid or unexpected faces defaults to 'N'
        assertEquals('N', ArenaView.nextFace('?'));
        assertEquals('N', ArenaView.nextFace(' '));
        assertEquals('N', ArenaView.nextFace('\0'));
    }

    @Test
    public void testOutOfBoundsDetectionLogic() {
        // Interior points within [0, 200]
        assertFalse(ArenaView.isOutsideArena(100f, 100f));
        assertFalse(ArenaView.isOutsideArena(50f, 150f));
        assertFalse(ArenaView.isOutsideArena(10f, 10f));

        // Exact corner boundaries are within bounds
        assertFalse(ArenaView.isOutsideArena(0f, 0f));
        assertFalse(ArenaView.isOutsideArena(200f, 0f));
        assertFalse(ArenaView.isOutsideArena(0f, 200f));
        assertFalse(ArenaView.isOutsideArena(200f, 200f));

        // Edge boundaries are within bounds
        assertFalse(ArenaView.isOutsideArena(0f, 100f));
        assertFalse(ArenaView.isOutsideArena(200f, 100f));
        assertFalse(ArenaView.isOutsideArena(100f, 0f));
        assertFalse(ArenaView.isOutsideArena(100f, 200f));

        // Negative X coordinates (Left of arena)
        assertTrue(ArenaView.isOutsideArena(-0.1f, 100f));
        assertTrue(ArenaView.isOutsideArena(-1f, 100f));
        assertTrue(ArenaView.isOutsideArena(-50f, 100f));

        // X beyond arena limit (Right of arena)
        assertTrue(ArenaView.isOutsideArena(200.1f, 100f));
        assertTrue(ArenaView.isOutsideArena(201f, 100f));
        assertTrue(ArenaView.isOutsideArena(250f, 100f));

        // Negative Y coordinates (Below arena)
        assertTrue(ArenaView.isOutsideArena(100f, -0.1f));
        assertTrue(ArenaView.isOutsideArena(100f, -1f));
        assertTrue(ArenaView.isOutsideArena(100f, -50f));

        // Y beyond arena limit (Above arena)
        assertTrue(ArenaView.isOutsideArena(100f, 200.1f));
        assertTrue(ArenaView.isOutsideArena(100f, 201f));
        assertTrue(ArenaView.isOutsideArena(100f, 300f));

        // Diagonal out of bounds
        assertTrue(ArenaView.isOutsideArena(-5f, -5f));
        assertTrue(ArenaView.isOutsideArena(205f, 205f));
        assertTrue(ArenaView.isOutsideArena(-5f, 205f));
        assertTrue(ArenaView.isOutsideArena(205f, -5f));
    }

    @Test
    public void testObstacleDragBoundsAndClamping() {
        // Drag clamping reasonably bounds coordinates within arena bounds + delete margin (-15cm to 215cm)
        assertEquals(100f, ArenaView.clampDragCm(100f), 0.001f);
        assertEquals(0f, ArenaView.clampDragCm(0f), 0.001f);
        assertEquals(200f, ArenaView.clampDragCm(200f), 0.001f);

        // Outside arena drag clamping: stays within visual delete boundary
        assertEquals(-10f, ArenaView.clampDragCm(-10f), 0.001f); // within margin: not clamped
        assertEquals(-15f, ArenaView.clampDragCm(-500f), 0.001f); // clamped to -15cm
        assertEquals(210f, ArenaView.clampDragCm(210f), 0.001f); // within margin: not clamped
        assertEquals(215f, ArenaView.clampDragCm(500f), 0.001f); // clamped to 215cm

        // When dropped inside arena, obstacles clamp within [5cm, 195cm] so 10x10cm body fits in arena
        assertEquals(5f, ArenaView.clampInsideArena(0f), 0.001f);
        assertEquals(5f, ArenaView.clampInsideArena(3f), 0.001f);
        assertEquals(10f, ArenaView.clampInsideArena(10f), 0.001f);
        assertEquals(100f, ArenaView.clampInsideArena(100f), 0.001f);
        assertEquals(190f, ArenaView.clampInsideArena(190f), 0.001f);
        assertEquals(195f, ArenaView.clampInsideArena(198f), 0.001f);
        assertEquals(195f, ArenaView.clampInsideArena(200f), 0.001f);
    }
}
