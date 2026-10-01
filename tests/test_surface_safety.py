import unittest
from datetime import datetime, timezone, timedelta

from rampflow.apron_safety import ApronSafetyMonitor, VehicleTrack
from rampflow.models import Decision
from rampflow.surface_movement import SurfaceRouter, TaxiEdge, TaxiConflict
from rampflow.timeutil import Window
from rampflow.workflows.control import RampWorkflowControl, WorkflowRejected

UTC = timezone.utc
T0 = datetime(2026, 1, 1, 10, 0, tzinfo=UTC)


def at(seconds):
    return T0 + timedelta(seconds=seconds)


def head_on_tracks():
    a = VehicleTrack("V1", at(120), 0.0, 0.0, 30.0, 90.0)
    b = VehicleTrack("V2", at(120), 120.0, 0.0, 30.0, 270.0)
    return a, b


def crossing_router():
    edges = [
        TaxiEdge("e1", "A", "X", 60),
        TaxiEdge("e2", "X", "B", 60, crossing_group="X1"),
        TaxiEdge("e3", "C", "D", 60, crossing_group="X1"),
        TaxiEdge("e4", "P", "Q", 60, opposite_edge="e5"),
        TaxiEdge("e5", "Q", "P", 60, opposite_edge="e4"),
    ]
    return SurfaceRouter(edges)


class NearMissTests(unittest.TestCase):
    def test_head_on_equal_speeds_is_detected(self):
        monitor = ApronSafetyMonitor()
        a, b = head_on_tracks()
        decision = monitor.near_miss(a, b)
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.code, "near-miss")
        self.assertAlmostEqual(decision.details["seconds"], 7.2, places=2)
        self.assertAlmostEqual(decision.details["separation_m"], 0.0, places=6)

    def test_same_direction_equal_speeds_stays_clear(self):
        monitor = ApronSafetyMonitor()
        a = VehicleTrack("V1", at(120), 0.0, 0.0, 30.0, 90.0)
        b = VehicleTrack("V2", at(120), 50.0, 0.0, 30.0, 90.0)
        decision = monitor.near_miss(a, b)
        self.assertTrue(decision.accepted)
        self.assertEqual(decision.code, "clear")


class StaleRouteTests(unittest.TestCase):
    def test_stale_route_through_occupied_crossing_is_rejected(self):
        router = crossing_router()
        old = router.plan("M1", "X", "B", at(300), 30.0)
        router.commit(router.plan("M2", "C", "D", at(300), 30.0))
        with self.assertRaises(TaxiConflict):
            router.commit(old)

    def test_stale_route_against_opposite_edge_is_rejected(self):
        router = crossing_router()
        old = router.plan("M3", "P", "Q", at(600), 30.0)
        router.commit(router.plan("M4", "Q", "P", at(600), 30.0))
        with self.assertRaises(TaxiConflict):
            router.commit(old)

    def test_fresh_route_still_commits(self):
        router = crossing_router()
        route = router.plan("M1", "X", "B", at(300), 30.0)
        router.commit(route)
        self.assertEqual(router.route("M1").movement_id, "M1")


class SafetyPublicationTests(unittest.TestCase):
    def test_denied_requirement_blocks_outward_event(self):
        control = RampWorkflowControl()
        calls = []
        denied = Decision(False, "near-miss", "predicted vehicle separation below threshold")
        with self.assertRaises(WorkflowRejected):
            control.publish_apron_movement_safety(
                "op-deny", "M1", at(180),
                lambda: calls.append("commit"),
                requirements=(denied,),
            )
        self.assertEqual(calls, [])
        self.assertEqual(control.stream.head, 0)
        self.assertEqual(control.receipts, {})

    def test_failed_action_publishes_nothing(self):
        control = RampWorkflowControl()

        def rejected():
            raise RuntimeError("domain rejected")

        with self.assertRaises(RuntimeError):
            control.publish_apron_movement_safety("op-fail", "M1", at(180), rejected)
        self.assertEqual(control.stream.head, 0)

    def test_accepted_decision_publishes_once_after_commit(self):
        control = RampWorkflowControl()
        calls = []
        ok = Decision(True, "clear", "vehicle trajectories remain separated")
        first = control.publish_apron_movement_safety(
            "op-ok", "M1", at(180),
            lambda: calls.append("commit") or "done",
            requirements=(ok,),
        )
        second = control.publish_apron_movement_safety(
            "op-ok", "M1", at(180),
            lambda: calls.append("duplicate"),
            requirements=(ok,),
        )
        self.assertIs(first, second)
        self.assertEqual(calls, ["commit"])
        self.assertEqual(control.stream.head, 1)
        event = control.stream.snapshot()[0]
        self.assertEqual(event.topic, "apron.movement.safety.decided")


class IncidentTimelineTests(unittest.TestCase):
    def test_timeline_drives_immediate_lockdown(self):
        monitor = ApronSafetyMonitor()
        router = crossing_router()
        control = RampWorkflowControl()

        # T0 10:00 - old route planned through crossing group X1 at graph generation 0.
        old_route = router.plan("M1", "X", "B", at(300), 30.0)
        # T1 10:01 - another movement commits and occupies the same crossing.
        router.commit(router.plan("M2", "C", "D", at(300), 30.0))
        # T2 10:02 - two vehicles close head-on; the monitor must deny.
        a, b = head_on_tracks()
        safety = monitor.near_miss(a, b)
        self.assertFalse(safety.accepted)
        # T3 10:03 - the stale route must not commit over the latest occupancy.
        with self.assertRaises(TaxiConflict):
            router.commit(old_route)
        # T4 10:04 - denied passage must not surface as a normal outward status.
        with self.assertRaises(WorkflowRejected):
            control.publish_apron_movement_safety(
                "op-lockdown", "M1", at(240),
                lambda: None,
                requirements=(safety,),
            )
        self.assertEqual(control.stream.head, 0)


if __name__ == "__main__":
    unittest.main()
