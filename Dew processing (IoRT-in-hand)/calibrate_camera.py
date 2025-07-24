#!/usr/bin/env python3
"""
Camera calibration tool (OpenCV) for checkerboard patterns.
Author  : Juan Bravo‑Arrabal
License : MIT
"""

from __future__ import annotations
import argparse
import logging
from pathlib import Path
from typing import List

import cv2 as cv
import numpy as np

# --------------------------------------------------------------------------- #
# Argument parsing                                                            #
# --------------------------------------------------------------------------- #
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Single‑camera checkerboard calibration")
    p.add_argument("folder", type=Path,
                   help="Directory containing calibration images (jpg, png).")
    p.add_argument("--rows", type=int, default=4,
                   help="Number of inner corners per checkerboard row (default: 4).")
    p.add_argument("--cols", type=int, default=6,
                   help="Number of inner corners per checkerboard column (default: 6).")
    p.add_argument("--square", type=float, default=1.0,
                   help="Square size in user units (e.g. millimetres). Only affects extrinsics.")
    p.add_argument("--out", type=Path, default=Path.cwd(),
                   help="Output directory for calibration files (default: current).")
    p.add_argument("--vis", action="store_true",
                   help="Visualise detections while processing.")
    return p.parse_args()

# --------------------------------------------------------------------------- #
# Utility functions                                                           #
# --------------------------------------------------------------------------- #
def detect_corners(gray: np.ndarray,
                   pattern_size: tuple[int, int]) -> tuple[bool, np.ndarray]:
    """Try OpenCV’s classic corner detector, fall back to the modern SB variant."""
    ok, corners = cv.findChessboardCorners(gray, pattern_size,
                                           flags=cv.CALIB_CB_ADAPTIVE_THRESH +
                                                 cv.CALIB_CB_NORMALIZE_IMAGE +
                                                 cv.CALIB_CB_FAST_CHECK)
    if not ok and hasattr(cv, "findChessboardCornersSB"):
        ok, corners = cv.findChessboardCornersSB(gray, pattern_size, None)
    return ok, corners

# --------------------------------------------------------------------------- #
# Main calibration routine                                                    #
# --------------------------------------------------------------------------- #
def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(levelname)s: %(message)s")

    if not args.folder.is_dir():
        logging.error("Image folder does not exist: %s", args.folder)
        return

    images: List[Path] = sorted(args.folder.glob("*.jpg")) + \
                         sorted(args.folder.glob("*.png"))
    if not images:
        logging.error("No images found in %s", args.folder)
        return

    pattern_size = (args.cols, args.rows)  # (columns, rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square  # scale to real‑world units

    objpoints, imgpoints = [], []

    for img_path in images:
        img = cv.imread(str(img_path))
        if img is None:
            logging.warning("Failed to read %s – skipping.", img_path.name)
            continue
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ok, corners = detect_corners(gray, pattern_size)
        if not ok:
            logging.warning("Corners not found in %s – skipping.", img_path.name)
            continue

        # Sub‑pixel refinement
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_sub = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners_sub)

        if args.vis:
            cv.drawChessboardCorners(img, pattern_size, corners_sub, True)
            cv.imshow("Detections", img)
            cv.waitKey(1)

    if args.vis:
        cv.destroyAllWindows()

    if len(objpoints) < 3:
        logging.error("Not enough valid images for calibration (need ≥ 3).")
        return

    # Calibration
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    logging.info("Calibration RMS error: %.4f", ret)

    # Reprojection error per image
    errors = []
    for i, (op, ip) in enumerate(zip(objpoints, imgpoints), 1):
        proj, _ = cv.projectPoints(op, rvecs[i-1], tvecs[i-1], mtx, dist)
        error = cv.norm(ip, proj, cv.NORM_L2) / len(proj)
        errors.append(error)
        logging.info("Image %02d rms error: %.4f", i, error)
    logging.info("Mean reprojection error: %.4f", np.mean(errors))

    # Write outputs
    args.out.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.out / "CameraMatrix.csv", mtx, delimiter=",")
    np.savetxt(args.out / "DistortionVector.csv", dist, delimiter=" ")
    fs = cv.FileStorage(str(args.out / "calibration.yaml"),
                        cv.FILE_STORAGE_WRITE)
    fs.write("camera_matrix", mtx)
    fs.write("dist_coeff", dist)
    fs.release()
    logging.info("Saved calibration to %s", args.out)

# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    main()


"""

# Calibrate with a 6×4 pattern, square size 25 mm, show detections live
python3 calib.py ~/Pictures/cam --rows 4 --cols 6 --square 25 --vis

# Non‑interactive, store results under ./output/
python3 calib.py ./left_images --rows 6 --cols 9 --square 15 --out ./output


"""