#!/usr/bin/env python3
"""Create an auditable site yaw/map-frame calibration record.

This tool deliberately never edits a SLAM configuration or map manifest.
"""

import argparse
import hashlib
import json
import math
import os
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yaw-deg", type=float, required=True)
    parser.add_argument("--map-frame-convention-id", required=True)
    parser.add_argument("--description", required=True)
    parser.add_argument("--site-id", required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--evidence", action="append", default=[])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.yaw_deg):
        parser.error("--yaw-deg must be finite")
    payload = {
        "schema_version": 1,
        "record_type": "SITE_YAW_MAP_CONVENTION_CALIBRATION",
        "site_id": args.site_id,
        "operator": args.operator,
        "created_at_unix_sec": time.time(),
        "configured_base_yaw_in_map_deg": args.yaw_deg,
        "map_frame_convention_id": args.map_frame_convention_id,
        "map_frame_convention_description": args.description,
        "evidence": args.evidence,
        "production_config_modified": False,
        "approval_status": "PENDING_SITE_APPROVAL",
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    payload["record_sha256"] = hashlib.sha256(canonical.encode()).hexdigest()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(args.output))
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
