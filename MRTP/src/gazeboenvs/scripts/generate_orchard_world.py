#!/usr/bin/env python3
"""Generate gazeboenvs/worlds/orchard_nbv.sdf.

orchard_nbv.sdf says "DO NOT EDIT MANUALLY; regenerate from the script" --
this is that script. Trees sit on a grid: `--rows` tree rows spaced `--dy`
apart in Y (one aisle between each consecutive pair), `--trees-per-row`
trees spaced `--dx` apart in X within each row.

    ./generate_orchard_world.py --rows 8 --trees-per-row 18 \
        --out ../worlds/orchard_nbv.sdf

Ground tiles scale with `--rows`/`--trees-per-row` automatically. All trees
are always-loaded (no Ignition levels/performer gating): with a couple
hundred trees that costs nothing worth streaming for, and level gating tied
to fixed robot-spawn positions silently stops covering the far rows the
moment the farm grows or the fleet spawn layout changes -- exactly what
happened going from 3 to 8 rows here.

No blocking walls between rows either -- routing is now enforced at the
planning level instead (amiga_ros2_mripp_bridge's bridge_node sends
NavigateThroughPoses waypoints along the graph path, which only ever has
within-aisle and headland edges), so a physical wall against Nav2's own
free-space planner cutting through a row is no longer needed.
"""
from __future__ import annotations

import argparse
import math
import random

TILE_SIZE = 20.0  # ground_grass.dae, submesh "Plane", at <scale>10 10 10</scale>

# Mesh varieties available under meshes/trees_orchard/ (excludes the
# lowercase-named citrus_orange_3m_green.obj, whose casing doesn't match its
# own .mtl -- same one the original hand-authored file avoided).
VARIETIES = [
    "Citrus_orange_25m_green",
    "Citrus_orange_25m_no_mix",
    "Citrus_orange_35m_green4",
    "Citrus_orange_35m_green",
    "Citrus_orange_35m_no_green",
    "Citrus_orange_3m_green2",
    "Citrus_orange_3m_green3",
    "Citrus_orange_3m_mix",
    "Citrus_orange_3m_yellow",
]

TREE_TEMPLATE = """    <!-- Tree {n} (row {row} col {col}): variety={variety}, yaw={yaw_deg:.1f}° -->
    <model name="tree_{n:02d}">
      <static>true</static>
      <pose>{x:.1f} {y:.1f} 0.0 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="trunk_collision">
          <pose>0 0 1.25 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>1.2</radius>
              <length>2.5</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <!-- Rotate so OBJ Y-up becomes world Z-up -->
          <pose>0 0 0 1.5708 0 0</pose>
          <geometry>
            <mesh>
              <uri>package://gazeboenvs/meshes/trees_orchard/{variety}.obj</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
"""

GROUND_TILE_TEMPLATE = """        <visual name="ground_tile_{n}">
          <pose>{x:.1f} {y:.1f} 0 0 0 0</pose>
          <geometry>
            <mesh>
              <scale>10 10 10</scale>
              <uri>package://gazeboenvs/models/ground/meshes/ground_grass.dae</uri>
              <submesh><name>Plane</name></submesh>
            </mesh>
          </geometry>
        </visual>
"""


def build_world(
    rows: int, trees_per_row: int, dx: float, dy: float, x0: float, seed: int
) -> str:
    rng = random.Random(seed)

    xs = [x0 + j * dx for j in range(trees_per_row)]
    ys = [(i - (rows - 1) / 2) * dy for i in range(rows)]
    x_min, x_max = xs[0], xs[-1]
    y_min, y_max = ys[0], ys[-1]

    # ── trees ───────────────────────────────────────────────────────────
    trees = []
    n = 1
    for row, y in enumerate(ys):
        for col, x in enumerate(xs):
            variety = rng.choice(VARIETIES)
            yaw_deg = rng.uniform(-180.0, 180.0)
            trees.append(
                TREE_TEMPLATE.format(
                    n=n,
                    row=row + 1,
                    col=col + 1,
                    variety=variety,
                    yaw_deg=yaw_deg,
                    x=x,
                    y=y,
                    yaw=math.radians(yaw_deg),
                )
            )
            n += 1
    trees_xml = "\n".join(trees)

    # ── ground tiles: cover the tree footprint plus headland/robot-spawn
    # margin, on the same 20m grid the original hand-authored file used ──
    margin = 20.0
    gx0 = math.floor((x_min - margin) / TILE_SIZE) * TILE_SIZE + TILE_SIZE / 2
    gx1 = math.ceil((x_max + margin) / TILE_SIZE) * TILE_SIZE
    gy0 = math.floor((y_min - margin) / TILE_SIZE) * TILE_SIZE + TILE_SIZE / 2
    gy1 = math.ceil((y_max + margin) / TILE_SIZE) * TILE_SIZE
    tile_xs = [gx0 + k * TILE_SIZE for k in range(int((gx1 - gx0) / TILE_SIZE) + 1)]
    tile_ys = [gy0 + k * TILE_SIZE for k in range(int((gy1 - gy0) / TILE_SIZE) + 1)]
    ground_tiles = []
    t = 0
    for ty in tile_ys:
        for tx in tile_xs:
            ground_tiles.append(GROUND_TILE_TEMPLATE.format(n=t, x=tx, y=ty))
            t += 1
    ground_tiles_xml = "".join(ground_tiles)
    plane_size = max(tile_xs[-1] - tile_xs[0], tile_ys[-1] - tile_ys[0]) + TILE_SIZE

    return WORLD_TEMPLATE.format(
        n_trees=rows * trees_per_row,
        trees_per_row=trees_per_row,
        rows=rows,
        dx=dx,
        dy=dy,
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        plane_size=plane_size,
        ground_tiles=ground_tiles_xml,
        trees=trees_xml,
    )


WORLD_TEMPLATE = """<?xml version="1.0"?>
<!--
  orchard_nbv.sdf — {rows}x{trees_per_row} citrus orchard for NBV multi-tree experiments.
  Generated by generate_orchard_world.py. DO NOT EDIT MANUALLY;
  regenerate from the script to keep the tree-locations YAML in sync.

  Trees: {n_trees} ({trees_per_row} columns x {rows} rows)
  Spacing: {dx:.1f}m x {dy:.1f}m
  Footprint: x [{x_min:.1f}, {x_max:.1f}] y [{y_min:.1f}, {y_max:.1f}]
  GPS origin: (35.1234, 33.4567, 150.0)

  World name "orchard_nbv" — distinct from the existing "orchard"
  world (worlds/orchard.sdf) so both can coexist in the workspace.
-->
<sdf version="1.9">
  <world name="orchard_nbv">

    <!-- Physics ─────────────────────────────────────────────────── -->
    <physics name="fast" type="ode">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>

    <!-- Required plugins -->
    <plugin filename="libgz-sim-physics-system.so"
            name="gz::sim::systems::Physics"/>
    <plugin filename="libgz-sim-user-commands-system.so"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="libgz-sim-scene-broadcaster-system.so"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="libgz-sim-sensors-system.so"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="libgz-sim-imu-system.so"
            name="gz::sim::systems::Imu">
    </plugin>
    <plugin filename="libgz-sim-navsat-system.so"
            name="gz::sim::systems::NavSat">
    </plugin>

    <!-- Spherical coordinates for GPS conversion -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>37.3611</latitude_deg>
      <longitude_deg>-120.4322</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <gravity>0 0 -9.8</gravity>

    <!-- Lighting ──────────────────────────────────────────────────── -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.95 0.95 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.3 0.1 -0.9</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.4 0.45 0.5 1</diffuse>
      <direction>0.5 -0.2 -0.5</direction>
    </light>

    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.75 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Ground plane (sized to fit the orchard) ──────────────── -->
        <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{plane_size:.1f} {plane_size:.1f}</size>
            </plane>
          </geometry>
        </collision>
{ground_tiles}      </link>
    </model>

    <!-- Trees ────────────────────────────────────────────────────── -->
    <!-- All always-loaded, like the ground/lights above — no Ignition
         levels/performer gating (see module docstring for why). No
         blocking walls between rows either — see module docstring. -->
{trees}
  </world>
</sdf>
"""


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--rows", type=int, default=8, help="Tree rows, separated by (rows - 1) aisles"
    )
    ap.add_argument("--trees-per-row", type=int, default=18)
    ap.add_argument(
        "--dx", type=float, default=4.0, help="Tree spacing along a row (m)"
    )
    ap.add_argument(
        "--dy",
        type=float,
        default=9.0,
        help="Row spacing / aisle width (m). Canopy meshes run up to "
        "~1.85m radius and are randomly yawed, so this needs to clear "
        "2x that plus robot width regardless of orientation -- 6.0 was "
        "not enough (canopies could pinch the aisle shut); 9.0 gives "
        "~2.65m of guaranteed clearance from canopy edge to centerline "
        "in the worst case.",
    )
    ap.add_argument("--x0", type=float, default=5.0, help="X of the first tree column")
    ap.add_argument(
        "--seed", type=int, default=42, help="Selects each tree's mesh variety + yaw"
    )
    ap.add_argument("--out", default="../worlds/orchard_nbv.sdf")
    args = ap.parse_args()

    world = build_world(
        args.rows, args.trees_per_row, args.dx, args.dy, args.x0, args.seed
    )
    with open(args.out, "w") as f:
        f.write(world)
    print(
        f"wrote {args.out}: {args.rows}x{args.trees_per_row} = "
        f"{args.rows * args.trees_per_row} trees, "
        f"{args.rows - 1} aisles"
    )


if __name__ == "__main__":
    main()
