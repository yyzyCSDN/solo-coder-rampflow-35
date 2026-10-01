import unittest
from datetime import datetime, timedelta, timezone

from rampflow.apron_safety import ApronSafetyMonitor, VehicleTrack
from rampflow.models import Decision
from rampflow.surface_movement import SurfaceRouter, TaxiConflict, TaxiEdge
from rampflow.workflows.control import RampWorkflowControl, WorkflowRejected

UTC = timezone.utc
T0 = datetime(2026, 1, 1, 12, 0, 0, tzinfo=UTC)


class NearMissTimelineTests(unittest.TestCase):
    def setUp(self):
        self.monitor = ApronSafetyMonitor()

    def test_head_on_equal_speed_is_detected_on_shared_timeline(self):
        a = VehicleTrack("V1", T0, 0.0, 0.0, 36.0, 0.0)
        b = VehicleTrack("V2", T0, 0.0, 120.0, 36.0, 180.0)
        decision = self.monitor.near_miss(a, b)
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.code, "near-miss")
        self.assertAlmostEqual(decision.details["seconds"], 6.0)
        self.assertAlmostEqual(decision.details["separation_m"], 0.0)

    def test_parallel_same_speed_remains_clear(self):
        a = VehicleTrack("V1", T0, 0.0, 0.0, 36.0, 0.0)
        b = VehicleTrack("V2", T0, 50.0, 0.0, 36.0, 0.0)
        decision = self.monitor.near_miss(a, b)
        self.assertTrue(decision.accepted)
        self.assertAlmostEqual(decision.details["minimum_m"], 50.0)

    def test_tracks_too_far_apart_in_time_are_not_compared(self):
        a = VehicleTrack("V1", T0, 0.0, 0.0, 36.0, 0.0)
        b = VehicleTrack("V2", T0 + timedelta(seconds=10), 0.0, 120.0, 36.0, 180.0)
        decision = self.monitor.near_miss(a, b)
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.code, "asynchronous")


class StaleRouteRevalidationTests(unittest.TestCase):
    def test_stale_route_cannot_cross_occupied_intersection(self):
        router = SurfaceRouter([
            TaxiEdge("E1", "A", "B", 60, crossing_group="X"),
            TaxiEdge("E2", "C", "D", 60, crossing_group="X"),
        ])
        stale = router.plan("M2", "A", "B", T0, 30.0)
        router.commit(router.plan("M1", "C", "D", T0, 30.0))
        with self.assertRaises(TaxiConflict):
            router.commit(stale)
        with self.assertRaises(KeyError):
            router.route("M2")
        replanned = router.plan("M2b", "A", "B", T0, 30.0)
        self.assertGreaterEqual(replanned.start, T0 + timedelta(seconds=105))
        router.commit(replanned)

    def test_stale_route_rejected_against_opposite_direction_traffic(self):
        router = SurfaceRouter([
            TaxiEdge("E3", "P", "Q", 60, opposite_edge="E4"),
            TaxiEdge("E4", "Q", "P", 60, opposite_edge="E3"),
        ])
        stale = router.plan("M2", "P", "Q", T0, 30.0)
        router.commit(router.plan("M1", "Q", "P", T0, 30.0))
        with self.assertRaises(TaxiConflict):
            router.commit(stale)
        with self.assertRaises(KeyError):
            router.route("M2")


class SafetyPublicationTests(unittest.TestCase):
    def test_denied_safety_decision_publishes_nothing(self):
        control = RampWorkflowControl()
        calls = []
        denied = Decision(False, "near-miss", "predicted vehicle separation below threshold")
        with self.assertRaises(WorkflowRejected):
            control.publish_apron_movement_safety(
                "op-deny", "V1", T0, lambda: calls.append("committed"), requirements=(denied,))
        self.assertEqual(calls, [])
        self.assertEqual(control.stream.head, 0)
        self.assertEqual(control.receipts, {})

    def test_failed_safety_action_publishes_nothing(self):
        control = RampWorkflowControl()

        def rejected():
            raise RuntimeError("domain rejected")

        with self.assertRaises(RuntimeError):
            control.publish_apron_movement_safety("op-fail", "V1", T0, rejected)
        self.assertEqual(control.stream.head, 0)
        self.assertEqual(control.receipts, {})

    def test_accepted_safety_decision_publishes_once(self):
        control = RampWorkflowControl()
        clear = Decision(True, "clear", "vehicle trajectories remain separated")
        first = control.publish_apron_movement_safety(
            "op-clear", "V1", T0, lambda: "committed", requirements=(clear,))
        second = control.publish_apron_movement_safety(
            "op-clear", "V1", T0, lambda: "duplicate")
        self.assertIs(first, second)
        self.assertEqual(first.result, "committed")
        self.assertEqual(first.event.topic, "apron.movement.safety.decided")
        self.assertEqual(control.stream.head, 1)
        self.assertTrue(control.stream.verify_chain())


class LockdownScenarioTests(unittest.TestCase):
    def test_denied_passage_hides_stale_route_and_outward_status(self):
        monitor = ApronSafetyMonitor()
        decision = monitor.near_miss(
            VehicleTrack("V1", T0, 0.0, 0.0, 36.0, 0.0),
            VehicleTrack("V2", T0, 0.0, 120.0, 36.0, 180.0),
        )
        self.assertFalse(decision.accepted)

        control = RampWorkflowControl()
        with self.assertRaises(WorkflowRejected):
            control.publish_apron_movement_safety(
                "op-lockdown", "V1", T0, lambda: "normal", requirements=(decision,))
        self.assertEqual(control.stream.head, 0)
        self.assertEqual(control.receipts, {})

        router = SurfaceRouter([
            TaxiEdge("E1", "A", "B", 60, crossing_group="X"),
            TaxiEdge("E2", "C", "D", 60, crossing_group="X"),
        ])
        stale = router.plan("M2", "A", "B", T0, 30.0)
        router.commit(router.plan("M1", "C", "D", T0, 30.0))
        with self.assertRaises(TaxiConflict):
            router.commit(stale)
        with self.assertRaises(KeyError):
            router.route("M2")


if __name__ == "__main__":
    unittest.main()
