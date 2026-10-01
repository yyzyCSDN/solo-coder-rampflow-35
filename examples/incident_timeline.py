"""Reconstructs the incident timeline and derives the lockdown verdict.

Run with:  PYTHONPATH=src python3 examples/incident_timeline.py
"""
from datetime import datetime, timedelta, timezone

from rampflow.apron_safety import ApronSafetyMonitor, VehicleTrack
from rampflow.surface_movement import SurfaceRouter, TaxiEdge, TaxiConflict
from rampflow.workflows.control import RampWorkflowControl, WorkflowRejected

UTC = timezone.utc
T0 = datetime(2026, 1, 1, 10, 0, tzinfo=UTC)


def at(seconds):
    return T0 + timedelta(seconds=seconds)


def stamp(seconds):
    return at(seconds).strftime("%H:%M:%S")


def main():
    edges = [
        TaxiEdge("e1", "A", "X", 60),
        TaxiEdge("e2", "X", "B", 60, crossing_group="X1"),
        TaxiEdge("e3", "C", "D", 60, crossing_group="X1"),
    ]
    router = SurfaceRouter(edges)
    monitor = ApronSafetyMonitor()
    control = RampWorkflowControl()
    timeline = []

    # T0: old route planned through crossing group X1 at graph generation 0.
    old_route = router.plan("M1", "X", "B", at(300), 30.0)
    timeline.append(f"{stamp(0)}  M1 旧路线规划完成，经过交叉口 X1，窗口 {stamp(300)}-{stamp(360)}，图版本 0")

    # T1: another movement commits and occupies the crossing; graph moves on.
    router.commit(router.plan("M2", "C", "D", at(300), 30.0))
    timeline.append(f"{stamp(60)}  M2 提交，占用交叉口 X1，窗口 {stamp(300)}-{stamp(360)}，图版本升至 {router.generation}")

    # T2: two vehicles close head-on at 30 kph each, 120 m apart.
    a = VehicleTrack("V1", at(120), 0.0, 0.0, 30.0, 90.0)
    b = VehicleTrack("V2", at(120), 120.0, 0.0, 30.0, 270.0)
    safety = monitor.near_miss(a, b)
    if safety.accepted:
        timeline.append(f"{stamp(120)}  V1/V2 相向接近 120m：监视器报 clear（漏报！）")
    else:
        timeline.append(
            f"{stamp(120)}  V1/V2 相向接近 120m：预测 {safety.details['seconds']:.1f}s 后间隔 "
            f"{safety.details['separation_m']:.1f}m -> 拒绝通行（{safety.code}）"
        )

    # T3: the stale route must be denied against the latest crossing occupancy.
    try:
        router.commit(old_route)
        timeline.append(f"{stamp(180)}  M1 旧路线提交成功（危险：未重验交叉口最新占用！）")
        route_blocked = False
    except TaxiConflict as exc:
        timeline.append(f"{stamp(180)}  M1 旧路线提交被拒：{exc}（交叉口 X1 最新占用已重验）")
        route_blocked = True

    # T4: a denied movement must not surface as a normal outward status.
    try:
        control.publish_apron_movement_safety(
            "op-lockdown", "M1", at(240), lambda: None, requirements=(safety,)
        )
        timeline.append(f"{stamp(240)}  对外状态已发布 apron.movement.safety.decided（危险：拒绝后仍显示正常！）")
        outward_silent = False
    except WorkflowRejected:
        timeline.append(f"{stamp(240)}  拒绝通行 -> 对外不发布任何“正常”状态，事件流深度保持 {control.stream.head}")
        outward_silent = True

    print("=== 事故时间线 ===")
    for line in timeline:
        print(line)
    lockdown = (not safety.accepted) and route_blocked and outward_silent
    print("\n=== 判定 ===")
    print("立即封控" if lockdown else "仍存在漏报/误放行，需修复")


if __name__ == "__main__":
    main()
