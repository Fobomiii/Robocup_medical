"""Select the task-specific Nav2 MPPI controller for a route."""

FOLLOW_PATH_NURSE = "FollowPathNurse"
FOLLOW_PATH_HEADING_HOLD = "FollowPathHeadingHold"
FOLLOW_PATH_LATERAL_HOLD = "FollowPathLateralHold"
BED_GOALS = frozenset(("bed1", "bed3"))


def _normalize_goal_name(goal_name):
    if goal_name is None:
        return None
    return str(goal_name).strip().lower()


def controller_for_route(source_goal_name, destination_goal_name: str) -> str:
    """Return the controller for a source-to-destination mission leg.

    Bed-to-bed travel needs the wider lateral MPPI exploration profile.  A
    return from either bed to Home uses the ordinary fixed-heading profile.
    Nurse approach and QR viewpoints retain MPPI yaw authority.
    """

    source = _normalize_goal_name(source_goal_name)
    destination = _normalize_goal_name(destination_goal_name)

    if destination == "nurse" or destination.startswith("nurse_"):
        return FOLLOW_PATH_NURSE
    if source in BED_GOALS and destination in BED_GOALS and source != destination:
        return FOLLOW_PATH_LATERAL_HOLD
    if destination in BED_GOALS or destination == "home":
        return FOLLOW_PATH_HEADING_HOLD
    raise ValueError(
        f"no MPPI controller profile for route "
        f"{source_goal_name!r} -> {destination_goal_name!r}"
    )
