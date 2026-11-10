#!/usr/bin/env python3
"""Fit hard- and soft-iron magnetometer calibration from a recorded CSV."""

import argparse
import csv
import os

import numpy as np


def fit_calibration(input_path):
    """Return magnetometer offsets and a 3x3 correction matrix."""
    with open(input_path, newline="") as file:
        reader = csv.DictReader(file)
        required_fields = ("mag_x", "mag_y", "mag_z")
        if reader.fieldnames is None or any(
            field not in reader.fieldnames for field in required_fields
        ):
            raise ValueError(
                f"{input_path} must contain columns: "
                f"{', '.join(required_fields)}"
            )

        samples = []
        for row in reader:
            try:
                sample = [float(row[field]) for field in required_fields]
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Invalid magnetometer sample in {input_path}"
                ) from exc
            if np.isfinite(sample).all():
                samples.append(sample)

    data = np.asarray(samples, dtype=float)
    if len(data) < 30:
        raise ValueError("At least 30 finite magnetometer samples are required.")

    scale = float(np.median(np.linalg.norm(data, axis=1)))
    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError("Magnetometer samples have no usable magnitude.")

    # Normalize values to keep the ellipsoid least-squares fit well-conditioned.
    normalized = data / scale
    x, y, z = normalized.T
    design = np.column_stack(
        (x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, x, y, z)
    )
    coefficients, _, rank, _ = np.linalg.lstsq(
        design, np.ones(len(data)), rcond=None
    )
    if rank < 9:
        raise ValueError(
            "Magnetometer samples do not cover enough directions for a 3D fit."
        )

    a, b, c, d, e, f, g, h, i = coefficients
    quadratic = np.array(
        [[a, d, e], [d, b, f], [e, f, c]],
        dtype=float,
    )
    linear = np.array([g, h, i], dtype=float)

    eigenvalues, eigenvectors = np.linalg.eigh(quadratic)
    if np.any(eigenvalues <= 0.0):
        raise ValueError(
            "Could not fit a bounded ellipsoid. Rotate the sensor through "
            "all orientations and record a new dataset."
        )

    # Complete the quadratic form to recover the hard-iron center.
    center = -0.5 * np.linalg.solve(quadratic, linear)
    radius_squared = 1.0 + float(center @ quadratic @ center)
    if not np.isfinite(radius_squared) or radius_squared <= 0.0:
        raise ValueError("The fitted magnetometer ellipsoid is invalid.")

    # This symmetric matrix maps the fitted ellipsoid back to a sphere.
    matrix = (
        eigenvectors
        @ np.diag(np.sqrt(eigenvalues))
        @ eigenvectors.T
        * np.sqrt(radius_squared)
    )
    offsets = center * scale

    corrected = (matrix @ (normalized - center).T).T
    lengths = np.linalg.norm(corrected, axis=1)
    mean_length = float(np.mean(lengths))
    if mean_length <= 0.0 or float(np.std(lengths) / mean_length) > 0.15:
        raise ValueError(
            "The ellipsoid fit is poor. Record a slower, more complete "
            "rotation through all orientations."
        )

    return offsets, matrix


def write_calibration(output_path, offsets, matrix):
    """Write calibration values in the format expected by ImuCalLive."""
    fieldnames = ["offset_x", "offset_y", "offset_z"] + [
        f"matrix_{row}{column}"
        for row in range(3)
        for column in range(3)
    ]
    values = list(offsets) + [
        matrix[row, column]
        for row in range(3)
        for column in range(3)
    ]
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with open(output_path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(fieldnames)
        writer.writerow(values)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "recording",
        help="magnetometer CSV produced by cal_mag_record",
    )
    parser.add_argument(
        "--output",
        default=os.path.expanduser("~/imu_calibration/mag_calibration.csv"),
        help="output calibration CSV (default: ~/imu_calibration/mag_calibration.csv)",
    )
    args = parser.parse_args()

    offsets, matrix = fit_calibration(args.recording)
    write_calibration(args.output, offsets, matrix)
    print(f"Magnetometer calibration saved to {args.output}")


if __name__ == "__main__":
    main()
