package com.sc2079.group16.controller;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

/**
 * Unit tests for AstryxLayer covering placement modes and default offset metrics
 * adhering to Facebook Astryx core layer specification.
 */
public class AstryxLayerTest {

    @Test
    public void testPlacementEnumValues() {
        AstryxLayer.Placement[] placements = AstryxLayer.Placement.values();
        assertEquals("AstryxLayer.Placement should have exactly 4 values", 4, placements.length);

        assertEquals(AstryxLayer.Placement.ABOVE, AstryxLayer.Placement.valueOf("ABOVE"));
        assertEquals(AstryxLayer.Placement.BELOW, AstryxLayer.Placement.valueOf("BELOW"));
        assertEquals(AstryxLayer.Placement.START, AstryxLayer.Placement.valueOf("START"));
        assertEquals(AstryxLayer.Placement.END, AstryxLayer.Placement.valueOf("END"));

        // Verify ordinal order
        assertEquals(0, AstryxLayer.Placement.ABOVE.ordinal());
        assertEquals(1, AstryxLayer.Placement.BELOW.ordinal());
        assertEquals(2, AstryxLayer.Placement.START.ordinal());
        assertEquals(3, AstryxLayer.Placement.END.ordinal());
    }

    @Test
    public void testDefaultOffsetDistance() {
        // Astryx core layer default specification defines a 12dp clearance offset
        assertEquals(12.0f, AstryxLayer.DEFAULT_OFFSET_DP, 0.001f);

        AstryxLayer layer = new AstryxLayer();
        assertEquals("Default offset distance should be 12dp", 12.0f, layer.getOffsetDp(), 0.001f);

        // Custom offset configuration
        layer.setOffsetDp(16.0f);
        assertEquals(16.0f, layer.getOffsetDp(), 0.001f);

        layer.setOffsetDp(0.0f);
        assertEquals(0.0f, layer.getOffsetDp(), 0.001f);
    }

    @Test
    public void testDefaultPlacementAndCustomPlacement() {
        AstryxLayer layer = new AstryxLayer();
        assertEquals("Default placement should be ABOVE", AstryxLayer.Placement.ABOVE, layer.getPlacement());

        layer.setPlacement(AstryxLayer.Placement.BELOW);
        assertEquals(AstryxLayer.Placement.BELOW, layer.getPlacement());

        layer.setPlacement(AstryxLayer.Placement.START);
        assertEquals(AstryxLayer.Placement.START, layer.getPlacement());

        layer.setPlacement(AstryxLayer.Placement.END);
        assertEquals(AstryxLayer.Placement.END, layer.getPlacement());
    }
}
