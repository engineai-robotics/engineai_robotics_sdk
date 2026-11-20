"""Motion-plan runner. Local kOk only means the request was sent."""

import sys
import time

from engineai_sdkpy import JointMotionPlanRequest, JointMotionPlanRequestType, JointMotionPlanStatus

from _cli import check_ok, err_name, stop_requested


class RunCode:
    COMPLETED = "completed"
    DRY_RUN = "dry_run"
    SEND_FAILED = "send_failed"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"
    NOT_CONFIRMED = "not_confirmed"
    PLANNER_DISABLED = "planner_disabled"
    NOT_IDLE = "planner_not_idle"
    NO_STATE = "no_planner_state"


def plan_status_name(status) -> str:
    names = {
        JointMotionPlanStatus.kDisabled: "DISABLED",
        JointMotionPlanStatus.kIdle: "IDLE",
        JointMotionPlanStatus.kExecuting: "EXECUTING",
        JointMotionPlanStatus.kExiting: "EXITING",
    }
    return names.get(status, "UNKNOWN")


def _confirm_cancel(session, request_id, poll, poll_user):
    req = JointMotionPlanRequest()
    req.request_id = request_id
    req.request_type = JointMotionPlanRequestType.kCancel
    err = session.joint_motion_planner.execute(req)
    print(f"cancel send request_id={request_id} -> {err_name(err)}")
    if err_name(err) != "Ok":
        print("cancel was not sent; robot stop is not confirmed")
        return RunCode.NOT_CONFIRMED
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if poll is not None:
            poll(session, poll_user)
        time.sleep(0.05)
        latest = session.joint_motion_planner.latest_state()
        if latest is None or latest.request_id != request_id:
            continue
        print(
            f"planner request_id={latest.request_id} status={plan_status_name(latest.status)} "
            f"progress={latest.progress}"
        )
        if latest.status == JointMotionPlanStatus.kIdle:
            print("robot confirmed planner idle after cancel")
            return RunCode.CANCELLED
    print("cancel was sent; robot did not confirm idle")
    return RunCode.NOT_CONFIRMED


def run_motion_plan(session, plan, request, poll=None, poll_user=None):
    """request keys: speed_scale, timeout_s, cancel_after_ms, dry_run.

    poll(session, poll_user) returns True when the caller wants to cancel.
    """
    speed_scale = request["speed_scale"]
    if speed_scale <= 0.0:
        print("speed_scale must be > 0", file=sys.stderr)
        return RunCode.SEND_FAILED
    if request.get("dry_run"):
        for task in plan["tasks"]:
            seconds = 0.0 if task["reset"] else task["duration"] / speed_scale
            kind = "RESET" if task["reset"] else "PLAN_EXECUTE"
            print(f"dry_run step={task['name']} type={kind} duration_s={seconds:.10g}")
        return RunCode.DRY_RUN
    if session is None:
        return RunCode.NO_STATE

    discover_deadline = time.monotonic() + 10.0
    state = None
    while time.monotonic() < discover_deadline:
        cancel = stop_requested() or (poll(session, poll_user) if poll is not None else False)
        if cancel:
            return RunCode.CANCELLED
        latest = session.joint_motion_planner.latest_state()
        if latest is not None:
            state = latest
            break
        time.sleep(0.05)
    if state is None:
        print("timed out waiting for joint_motion_planner.latest_state", file=sys.stderr)
        return RunCode.NO_STATE
    if state.status != JointMotionPlanStatus.kIdle:
        print(f"planner not IDLE, status={plan_status_name(state.status)}", file=sys.stderr)
        return RunCode.NOT_IDLE

    run_start = time.monotonic()
    cancel_after_ms = request.get("cancel_after_ms", -1)
    timeout_s = request.get("timeout_s", 0.0)
    tasks = plan["tasks"]
    for index, task in enumerate(tasks):
        cancel = stop_requested() or (poll(session, poll_user) if poll is not None else False)
        if cancel_after_ms >= 0 and (time.monotonic() - run_start) * 1000.0 >= cancel_after_ms:
            cancel = True
        if cancel:
            print(f"cancel before step {task['name']}")
            return RunCode.CANCELLED

        latest = session.joint_motion_planner.latest_state()
        if latest is None:
            print(f"no planner state; cannot send {task['name']}", file=sys.stderr)
            return RunCode.NO_STATE
        req = JointMotionPlanRequest()
        req.request_id = latest.request_id + 1
        sent_request_id = req.request_id
        scaled = 0.0 if task["reset"] else task["duration"] / speed_scale
        wait_s = 15.0 if task["reset"] else max(10.0, scaled * 3.0 + 5.0)
        if timeout_s > 0.0:
            wait_s = timeout_s
        if task["reset"]:
            req.request_type = JointMotionPlanRequestType.kReset
            print(f"send RESET {task['name']} request_id={sent_request_id}")
        else:
            req.request_type = JointMotionPlanRequestType.kPlanExecute
            req.use_gravity_compensation = task["use_gravity_compensation"]
            req.joint_indices = list(plan["joint_indices"])
            req.target_positions = list(task["target_positions"])
            req.execution_time = scaled
            req.stiffness = list(task["stiffness"])
            req.damping = list(task["damping"])
            print(f"send PLAN_EXECUTE {task['name']} request_id={sent_request_id} duration_s={scaled}")
        sent_at = session.joint_motion_planner.last_receive_time()
        if not check_ok(session.joint_motion_planner.execute(req), "joint_motion_planner.execute"):
            return RunCode.SEND_FAILED

        task_deadline = time.monotonic() + wait_s
        step_done = False
        logged_request_id = None
        logged_status = None
        while time.monotonic() < task_deadline:
            stop_now = stop_requested() or (poll(session, poll_user) if poll is not None else False)
            if cancel_after_ms >= 0 and (time.monotonic() - run_start) * 1000.0 >= cancel_after_ms:
                stop_now = True
            if stop_now:
                return _confirm_cancel(session, sent_request_id, poll, poll_user)
            time.sleep(0.05)
            sample = session.joint_motion_planner.latest_state()
            if sample is None or sample.request_id != sent_request_id:
                continue
            if sample.request_id != logged_request_id or sample.status != logged_status:
                logged_request_id = sample.request_id
                logged_status = sample.status
                motion = session.motion.current_motion()
                motion_text = motion if motion is not None else "(no sample)"
                print(
                    f"planner request_id={sample.request_id} status={plan_status_name(sample.status)} "
                    f"progress={sample.progress} motion={motion_text}"
                )
            if sample.status == JointMotionPlanStatus.kDisabled:
                print(
                    f"planner DISABLED while waiting for {task['name']} request_id={sent_request_id}",
                    file=sys.stderr,
                )
                return RunCode.PLANNER_DISABLED
            received_at = session.joint_motion_planner.last_receive_time()
            newer = received_at is not None and (sent_at is None or received_at > sent_at)
            idle = sample.status == JointMotionPlanStatus.kIdle
            reset_settled = task["reset"] and sample.progress >= 1.0
            finished = idle and sample.progress >= 1.0
            if newer and (finished or reset_settled):
                print(f"completed {task['name']} ({index + 1}/{len(tasks)})")
                step_done = True
                break
        if not step_done:
            sample = session.joint_motion_planner.latest_state()
            message = f"timed out waiting for {task['name']} request_id={sent_request_id}"
            if sample is not None:
                message += (
                    f" planner request_id={sample.request_id} status={plan_status_name(sample.status)} "
                    f"progress={sample.progress}"
                )
            print(message, file=sys.stderr)
            return RunCode.TIMEOUT
    print("all motion plan tasks completed")
    return RunCode.COMPLETED
