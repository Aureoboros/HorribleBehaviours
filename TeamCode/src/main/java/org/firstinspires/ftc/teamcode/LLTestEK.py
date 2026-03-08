"""
Limelight 3A - AprilTag ID 24 Detector
Connects to the Limelight camera via NetworkTables and checks
whether AprilTag ID 24 is currently visible.
"""

import time

try:
    import ntcore
except ImportError:
    raise SystemExit(
        "ntcore is not installed. Run:  pip install robotpy-ntcore --break-system-packages"
    )

# ── Configuration ──────────────────────────────────────────────────────────────
LIMELIGHT_NAME = "limelight"          # Change if your Limelight has a custom hostname
TARGET_TAG_ID  = 24                   # The AprilTag ID we care about
POLL_INTERVAL  = 0.1                  # Seconds between checks (10 Hz)
# ──────────────────────────────────────────────────────────────────────────────


def connect_to_limelight(name: str) -> tuple:
    """Start NetworkTables client and return (inst, table)."""
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4(identity="AprilTag-Checker")

    # Try mDNS hostname first, then fallback IP
    inst.setServerTeam(0)                          # placeholder – overridden below
    inst.setServer(f"{name}.local")                # mDNS  (works on most networks)
    # If mDNS doesn't resolve on your network, replace the line above with:
    #   inst.setServer("10.TE.AM.11")              # static IP, e.g. 10.0.0.11

    table = inst.getTable(name)
    return inst, table


def get_visible_tag_ids(table) -> list[int]:
    """
    Return the list of currently-detected AprilTag IDs.

    Limelight publishes an array called 'tid' when running in AprilTag pipeline.
    When multiple tags are visible the full list is in 'rawfiducials' (LL3+),
    but 'tid' always reflects the *primary* (best) tag.  For a simple
    single-tag check, 'tid' is sufficient.
    """
    # Primary detected tag ID  (-1 when nothing is seen)
    tid = int(table.getEntry("tid").getDouble(-1))
    if tid == -1:
        return []

    # Optional: read all visible IDs from the JSON botpose array or rawfiducials
    # For simplicity we use tid here (detects one tag at a time in most pipelines)
    return [tid]


def main():
    print(f"Connecting to Limelight '{LIMELIGHT_NAME}' …")
    inst, table = connect_to_limelight(LIMELIGHT_NAME)

    # Give NetworkTables a moment to connect
    time.sleep(1.0)

    print(f"Watching for AprilTag ID {TARGET_TAG_ID}. Press Ctrl-C to stop.\n")

    last_state = None  # Track state so we only print on change

    try:
        while True:
            visible_ids = get_visible_tag_ids(table)
            tag_found   = TARGET_TAG_ID in visible_ids

            if tag_found != last_state:          # Print only when state changes
                if tag_found:
                    print(f":)  — AprilTag {TARGET_TAG_ID} is VISIBLE  (ids seen: {visible_ids})")
                else:
                    print(f":(  — AprilTag {TARGET_TAG_ID} is NOT visible (ids seen: {visible_ids})")
                last_state = tag_found

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        inst.stopClient()


if __name__ == "__main__":
    main()
