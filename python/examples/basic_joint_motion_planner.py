#!/usr/bin/env python3
"""basic_joint_motion_planner: latest_state + optional execute(RESET).

  python3 examples/basic_joint_motion_planner.py --domain-id 0
  python3 examples/basic_joint_motion_planner.py --domain-id 0 --reset
"""

import sys

from engineai_sdkpy import JointMotionPlanRequest, JointMotionPlanRequestType

from _cli import (
    check_ok,
    create_session_or_exit,
    parse_session_args,
    pause_before_close,
    print_no_robot_data_hint,
    wait_for_motion_sample,
    wait_for_robot_sample,
    wants_help,
)


def print_plan(session) -> None:
    plan = session.joint_motion_planner.latest_state()
    if plan is None:
        print("plan (no sample)")
        return
    print(f"plan request_id={plan.request_id} status={int(plan.status)} progress={plan.progress}")


def main(argv) -> int:
    usage = f"Usage: {argv[0]} --domain-id N [--interface IF] [--network-address ADDR] [--reset]"
    if wants_help(argv):
        print(usage, file=sys.stderr)
        return 0
    options, rest = parse_session_args(argv)
    if options is None:
        print(usage, file=sys.stderr)
        return 2
    do_reset = False
    for arg in rest:
        if arg == "--reset":
            do_reset = True
        else:
            print(usage, file=sys.stderr)
            return 2

    session = create_session_or_exit(options)
    wait_for_motion_sample(session)
    print_plan(session)
    if do_reset:
        if not wait_for_robot_sample(session):
            print_no_robot_data_hint()
            session.close()
            return 1
        req = JointMotionPlanRequest()
        plan = session.joint_motion_planner.latest_state()
        req.request_id = (plan.request_id + 1) if plan is not None else 1
        req.request_type = JointMotionPlanRequestType.kReset
        if not check_ok(session.joint_motion_planner.execute(req), "joint_motion_planner.execute"):
            session.close()
            return 1
        print_plan(session)
        pause_before_close()
    session.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
