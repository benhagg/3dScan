import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from live_stream import replay_session

if __name__ == "__main__":
    if len(sys.argv) > 1:
        target_dir = sys.argv[1]
    else:
        # Pick the most recent session in captures/
        captures_root = os.path.join(os.path.dirname(__file__), "..", "captures")
        if os.path.exists(captures_root):
            sessions = sorted(
                [os.path.join(captures_root, d) for d in os.listdir(captures_root) if d.startswith("session_")],
                key=os.path.getmtime,
                reverse=True
            )
            target_dir = sessions[0] if sessions else None
        else:
            target_dir = None

    if target_dir:
        replay_session(target_dir)
    else:
        print("Usage: python viewer/visualize.py captures/session_<timestamp>")
