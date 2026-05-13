#!/usr/bin/env python3

import math
import os
import yaml
import cv2
import numpy as np
import rospy


def load_ros_map(map_yaml):
    with open(map_yaml, "r") as f:
        info = yaml.safe_load(f)

    image_path = info["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml), image_path)

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError("Could not read map image: %s" % image_path)

    resolution = float(info["resolution"])
    origin = info["origin"]
    negate = int(info.get("negate", 0))
    occupied_thresh = float(info.get("occupied_thresh", 0.65))
    free_thresh = float(info.get("free_thresh", 0.196))

    if negate == 0:
        occ_prob = (255 - img.astype(np.float32)) / 255.0
    else:
        occ_prob = img.astype(np.float32) / 255.0

    occ = np.full(img.shape, -1, dtype=np.int16)
    occ[occ_prob > occupied_thresh] = 100
    occ[occ_prob < free_thresh] = 0

    return occ, resolution, origin


def pixel_to_world(px, py, image_height, resolution, origin):

    mx = px
    my = image_height - 1 - py

    wx = origin[0] + (mx + 0.5) * resolution
    wy = origin[1] + (my + 0.5) * resolution

    return float(wx), float(wy)


def visibility_score(safe_mask, px, py, max_range_cells, ray_count):
    
    h, w = safe_mask.shape
    score = 0

    for i in range(ray_count):
        theta = 2.0 * math.pi * i / ray_count
        dx = math.cos(theta)
        dy = math.sin(theta)

        for r in range(1, max_range_cells + 1):
            x = int(round(px + dx * r))
            y = int(round(py + dy * r))

            if x < 0 or x >= w or y < 0 or y >= h:
                break

            if not safe_mask[y, x]:
                break

            score += 1

    return score


def greedy_spread_select(candidates, min_spacing_m, max_hotspots):
    selected = []
    min_spacing_sq = min_spacing_m * min_spacing_m

    for c in candidates:
        too_close = False
        for s in selected:
            if (c["x"] - s["x"]) ** 2 + (c["y"] - s["y"]) ** 2 < min_spacing_sq:
                too_close = True
                break

        if not too_close:
            selected.append(c)

        if len(selected) >= max_hotspots:
            break

    return selected


def nearest_neighbor_order(points):
    if not points:
        return []

    remaining = points[:]
    ordered = [remaining.pop(0)]

    while remaining:
        last = ordered[-1]
        best_i = 0
        best_d = float("inf")

        for i, p in enumerate(remaining):
            d = (p["x"] - last["x"]) ** 2 + (p["y"] - last["y"]) ** 2
            if d < best_d:
                best_d = d
                best_i = i

        ordered.append(remaining.pop(best_i))

    return ordered


def main():
    rospy.init_node("generate_hotspots")

    map_yaml = rospy.get_param("~map_yaml")
    output_yaml = rospy.get_param("~output_yaml", "hotspots.yaml")

    clearance_m = rospy.get_param("~clearance_m", 0.20)
    sample_spacing_m = rospy.get_param("~sample_spacing_m", 0.50)
    hotspot_spacing_m = rospy.get_param("~hotspot_spacing_m", 1.0)
    max_hotspots = rospy.get_param("~max_hotspots", 20)
    visibility_range_m = rospy.get_param("~visibility_range_m", 4.0)
    ray_count = rospy.get_param("~ray_count", 32)

    occ, resolution, origin = load_ros_map(map_yaml)
    h, w = occ.shape

    free_mask = occ == 0

    safe_space = free_mask.astype(np.uint8)
    dist = cv2.distanceTransform(safe_space, cv2.DIST_L2, 5)

    clearance_cells = max(1, int(clearance_m / resolution))
    sample_step = max(1, int(sample_spacing_m / resolution))
    max_range_cells = max(1, int(visibility_range_m / resolution))

    safe_mask = free_mask & (dist >= clearance_cells)

    candidates = []

    for py in range(0, h, sample_step):
        for px in range(0, w, sample_step):
            if not safe_mask[py, px]:
                continue

            vis = visibility_score(safe_mask, px, py, max_range_cells, ray_count)
            clearance = float(dist[py, px] * resolution)

            # near_structure_bonus = max(0.0, 2.0 - clearance)

            score = float(vis) + 40.0

            wx, wy = pixel_to_world(px, py, h, resolution, origin)

            candidates.append({
                "x": wx,
                "y": wy,
                "yaw": 0.0,
                "score": score,
                "visibility": int(vis),
                "clearance_m": clearance,
            })

    candidates.sort(key=lambda c: c["score"], reverse=True)

    selected = greedy_spread_select(candidates, hotspot_spacing_m, max_hotspots)
    selected = nearest_neighbor_order(selected)

    for i, p in enumerate(selected):
        p["name"] = "hotspot_%02d" % (i + 1)

        if len(selected) > 1:
            q = selected[(i + 1) % len(selected)]
            p["yaw"] = float(math.atan2(q["y"] - p["y"], q["x"] - p["x"]))

    output = {
        "frame_id": "map",
        "hotspots": selected,
    }

    with open(output_yaml, "w") as f:
        yaml.safe_dump(output, f, default_flow_style=False)

    rospy.loginfo("Generated %d hotspots -> %s", len(selected), output_yaml)


if __name__ == "__main__":
    main()
