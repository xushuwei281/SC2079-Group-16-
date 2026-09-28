import unittest

from mdp_perception.live_confirmation import LiveConfirmation


class TestLiveConfirmation(unittest.TestCase):
    def test_three_distinct_frames_then_suppress_duplicates(self):
        gate = LiveConfirmation()
        self.assertEqual([gate.observe(25, t) for t in range(6)],
                         [None, None, 25, None, None, None])

    def test_repeated_frame_time_cannot_confirm(self):
        gate = LiveConfirmation()
        self.assertEqual([gate.observe(11, 1.0) for _ in range(5)], [None] * 5)
        self.assertEqual(gate.count, 1)

    def test_miss_interrupts_confirmation(self):
        gate = LiveConfirmation()
        self.assertEqual([gate.observe(s, t) for t, s in enumerate([25, 25, None, 25, 25, 25])],
                         [None, None, None, None, None, 25])

    def test_alternating_symbols_do_not_confirm(self):
        gate = LiveConfirmation()
        self.assertTrue(all(gate.observe(s, t) is None for t, s in enumerate([11, 25] * 5)))

    def test_brief_occlusion_does_not_republish(self):
        gate = LiveConfirmation()
        for t in range(3): gate.observe(25, t)
        self.assertIsNone(gate.observe(None, 3))
        self.assertTrue(all(gate.observe(25, t) is None for t in range(4, 8)))

    def test_sustained_absence_rearms(self):
        gate = LiveConfirmation()
        for t in range(3): gate.observe(25, t)
        for t in range(3, 6): gate.observe(None, t)
        self.assertEqual([gate.observe(25, t) for t in range(6, 9)], [None, None, 25])

    def test_camera_gap_breaks_confirmation_but_not_duplicate_suppression(self):
        gate = LiveConfirmation()
        gate.observe(11, 0); gate.observe(11, 1)
        self.assertIsNone(gate.observe(11, 10))
        self.assertEqual(gate.count, 1)
        gate.observe(11, 11)
        self.assertEqual(gate.observe(11, 12), 11)
        gate.break_sequence()
        self.assertTrue(all(gate.observe(11, t) is None for t in range(20, 24)))

    def test_new_symbol_can_publish_after_confirmation(self):
        gate = LiveConfirmation()
        for t in range(3): gate.observe(11, t)
        self.assertEqual([gate.observe(25, t) for t in range(3, 6)], [None, None, 25])

    def test_obstacle_reset_requires_new_confirmation(self):
        gate = LiveConfirmation()
        for t in range(3): gate.observe(11, t)
        gate.reset()
        self.assertEqual([gate.observe(11, t) for t in range(3, 6)], [None, None, 11])

    def test_filled_circle_is_valid_and_invalid_ids_never_publish(self):
        gate = LiveConfirmation()
        self.assertEqual([gate.observe(40, t) for t in range(3)], [None, None, 40])
        for t, sid in enumerate([0, 10, 41, -1, True], start=3):
            self.assertIsNone(gate.observe(sid, t))


if __name__ == '__main__':
    unittest.main()
