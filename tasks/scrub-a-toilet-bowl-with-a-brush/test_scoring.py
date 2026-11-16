import unittest

import numpy as np

from task import EXPOSED, MAX_STEPS, State, demo


class HiddenTargetsTest(unittest.TestCase):
    def test_air_pass_leaves_every_mark(self):
        state = State()
        for index in range(20):
            state.scrub(np.array([index * .004, 0., 0.]), ())
        np.testing.assert_array_equal(state.dirt, 1.)

    def test_exposed_pass_cannot_clean_scored_under_rim_marks(self):
        state = State()
        exposed = state.patches[:len(EXPOSED)].reshape(-1, 3)
        for index in range(20):
            state.scrub(np.array([index * .004, 0., 0.]), exposed)
        self.assertTrue(np.all(state.dirt[:len(EXPOSED)] < 0.1))
        np.testing.assert_array_equal(state.dirt[len(EXPOSED):], 1.)

    def test_stationary_load_does_not_remove_dirt(self):
        state = State()
        for _ in range(20):
            state.scrub(np.zeros(3), state.patches.reshape(-1, 3))
        np.testing.assert_array_equal(state.dirt, 1.)

    def test_choreography_fits_the_shared_limit(self):
        self.assertLessEqual(max(sum(w.duration for w in p) for p in demo().values()) * 20,
                             MAX_STEPS)


if __name__ == '__main__':
    unittest.main()
