import unittest
from carton_seal.score import ReleaseHold


class ReleasedSealTests(unittest.TestCase):
    def measure(self, hold, t, **changes):
        data = dict(
            position=[0, 0, 0],
            angle=0.14,
            bonds=range(5),
            inside=True,
            clear=True,
            pads=[0] * 5,
        )
        data.update(changes)
        hold.update(t, **data)

    def test_retraction_interrupted_at_last_frame(self):
        for interrupted in (
            dict(clear=False),
            dict(inside=False),
            dict(angle=4),
            dict(bonds=range(4)),
            dict(position=[0.01, 0, 0]),
        ):
            with self.subTest(interrupted=interrupted):
                hold = ReleaseHold()
                for k in range(41):
                    self.measure(hold, k / 20)
                self.assertEqual(hold.score, 0)
                self.measure(hold, 2.05, **interrupted)
                self.assertEqual(hold.duration, 0)
                for k in range(42, 81):
                    self.measure(hold, k / 20)
                self.assertEqual(hold.score, 0)
                for k in range(81, 85):
                    self.measure(hold, k / 20)
                self.assertEqual(hold.score, 1)

    def test_missing_samples_and_duplicate_updates(self):
        hold = ReleaseHold()
        for k in range(40):
            self.measure(hold, k / 20)
        self.measure(hold, 2.10)
        self.assertEqual(hold.duration, 0)
        with self.assertRaises(ValueError):
            self.measure(hold, 2.10)

    def test_new_episode_has_independent_force_history(self):
        first, second = ReleaseHold(0.1), ReleaseHold(0.1)
        self.measure(first, 0, pads=[0, 0, 31, 0, 0])
        self.measure(second, 0)
        for k in range(1, 25):
            self.measure(first, k / 10)
            self.measure(second, k / 10)
        self.assertEqual((first.score, second.score), (0, 1))


if __name__ == "__main__":
    unittest.main()
