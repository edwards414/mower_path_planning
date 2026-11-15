//! The global costmap with its inflation layer (`costmap::GlobalLayers`), and
//! NavFn on it:
//!
//! * against the real nav2 `planner_server` started with
//!   `nav2_no_map_params.yaml` (tests/data/global_costmap.json, recorded by
//!   tests/oracle/global_costmap/scenarios.py): the master grid cell for
//!   cell, every plan, and the cells left changed after the plans;
//! * the outdoor failure of 2026-11-16 on the map of that run
//!   (tests/data/field_2026_11_16_free_space_inflated.json): the plan to a
//!   coverage start one cell from the edge of the configuration space.

mod common;

use common::{assert_close, f, load};
use mower_nav_core::costmap::{
    global_costmap, static_costmap, Costmap2D, GlobalLayers, InflationLayer, InflationLayerParams, OccupancyGrid, StaticLayerParams,
    FREE_SPACE, GLOBAL_ROBOT_RADIUS, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE,
};
use mower_nav_core::geometry::Pose2D;
use mower_nav_core::planner::{compute_path_to_pose, NavfnParams, NavfnPlanner, PlannerError};
use serde_json::Value;

/// A run-length fixture: cells 100 / 0, starting with 100.
fn fixture(name: &str) -> OccupancyGrid {
    let v = load(name);
    let mut data = Vec::new();
    let mut value = 100i8;
    for n in v["runs"].as_array().unwrap() {
        data.extend(std::iter::repeat(value).take(n.as_u64().unwrap() as usize));
        value = 100 - value;
    }
    let g = OccupancyGrid {
        width: v["width"].as_u64().unwrap() as u32,
        height: v["height"].as_u64().unwrap() as u32,
        resolution: f(&v["resolution"]),
        origin_x: f(&v["origin_x"]),
        origin_y: f(&v["origin_y"]),
        data,
    };
    assert_eq!(g.data.len(), (g.width * g.height) as usize, "{name}");
    g
}

fn grid(case: &Value) -> OccupancyGrid {
    if let Some(name) = case.get("fixture") {
        return fixture(name.as_str().unwrap());
    }
    let v = &case["grid"];
    OccupancyGrid {
        width: v["width"].as_u64().unwrap() as u32,
        height: v["height"].as_u64().unwrap() as u32,
        // MapMetaData.resolution is a float32
        resolution: f(&v["resolution"]) as f32 as f64,
        origin_x: f(&v["origin_x"]),
        origin_y: f(&v["origin_y"]),
        data: v["data"].as_array().unwrap().iter().map(|x| x.as_i64().unwrap() as i8).collect(),
    }
}

fn pose(v: &Value) -> Pose2D {
    Pose2D::new(f(&v[0]), f(&v[1]), f(&v[2]))
}

fn plan(planner: &mut NavfnPlanner, costmap: &mut Costmap2D, start: &Pose2D, goal: &Pose2D) -> Result<Vec<Pose2D>, PlannerError> {
    compute_path_to_pose(planner, costmap, start, goal, &mut || false)
}

/// The master grid, the plans and what the plans left in the master grid,
/// against nav2's.
#[test]
fn global_costmap_and_plans_match_nav2() {
    let d = load("global_costmap.json");
    let cases = d["cases"].as_array().unwrap();
    assert_eq!(cases.len(), 3);
    let mut inflated_cells = 0;
    let mut planned = 0;
    // one costmap for the whole recording, as in the node
    let mut layers = GlobalLayers::mower();
    let mut first = true;
    for case in cases {
        let name = case["name"].as_str().unwrap();
        let g = grid(case);
        let layer = layers.static_layer(&g);
        let mut master = layer.clone();
        // the update that takes the map repaints; so does the one after it,
        // unless this is the first map (see `GlobalLayers`); then none does
        layers.new_map();
        let repaints: Vec<bool> = (0..4).map(|_| layers.update(&layer, &mut master)).collect();
        assert_eq!(repaints, [true, !first, false, false], "{name}");
        first = false;

        let theirs = &case["costmap"];
        assert_eq!(master.size_x() as u64, theirs["size_x"].as_u64().unwrap(), "{name}: size_x");
        assert_eq!(master.size_y() as u64, theirs["size_y"].as_u64().unwrap(), "{name}: size_y");
        assert_eq!(master.resolution(), f(&theirs["resolution"]), "{name}: resolution");
        assert_close(master.origin_x(), f(&theirs["origin_x"]), &format!("{name}: origin_x"));
        assert_close(master.origin_y(), f(&theirs["origin_y"]), &format!("{name}: origin_y"));
        let expected: Vec<u8> = theirs["data"].as_array().unwrap().iter().map(|x| x.as_u64().unwrap() as u8).collect();
        let diff = master.data().iter().zip(&expected).filter(|(a, b)| a != b).count();
        assert_eq!(diff, 0, "{name}: {diff} cells differ");
        inflated_cells += master.data().iter().zip(layer.data()).filter(|(a, b)| a != b).count();

        let initial = master.clone();
        let mut planner = NavfnPlanner::new(NavfnParams::mower(), &master);
        for (k, p) in case["plans"].as_array().unwrap().iter().enumerate() {
            let what = format!("{name} plan {k}");
            let result = plan(&mut planner, &mut master, &pose(&p["start"]), &pose(&p["goal"]));
            let code = result.as_ref().err().map_or(0, |e| e.compute_path_to_pose_code().unwrap() as u64);
            assert_eq!(code, p["error_code"].as_u64().unwrap(), "{what}: error code ({result:?})");
            assert_eq!(if result.is_ok() { "succeeded" } else { "aborted" }, p["status"].as_str().unwrap(), "{what}");
            let path = result.unwrap_or_default();
            let expected = p["poses"].as_array().unwrap();
            assert_eq!(path.len(), expected.len(), "{what}: poses");
            for (i, (ours, e)) in path.iter().zip(expected).enumerate() {
                assert_close(ours.x, f(&e[0]), &format!("{what} pose {i} x"));
                assert_close(ours.y, f(&e[1]), &format!("{what} pose {i} y"));
                let yaw = 2.0 * f(&e[2]).atan2(f(&e[3]));
                assert_close(ours.yaw, yaw, &format!("{what} pose {i} yaw"));
            }
            planned += !path.is_empty() as usize;
        }

        // clearRobotCell's holes, still there three updates later: an
        // update with no new map does not repaint the master grid
        assert!((0..3).all(|_| !layers.update(&layer, &mut master)), "{name}");
        let ours: Vec<(u64, u64)> =
            master.data().iter().zip(initial.data()).enumerate().filter(|(_, (a, b))| a != b).map(|(i, (a, _))| (i as u64, *a as u64)).collect();
        let theirs: Vec<(u64, u64)> =
            case["changed_after_plans"].as_array().unwrap().iter().map(|c| (c[0].as_u64().unwrap(), c[1].as_u64().unwrap())).collect();
        assert_eq!(ours, theirs, "{name}: cells changed after the plans");
    }
    assert!(inflated_cells > 10_000, "only {inflated_cells} cells inflated");
    assert!(planned >= 12, "only {planned} plans");
}

// --- the outdoor failure of 2026-11-16 --------------------------------------
//
// With a static layer only, NavFn's plan followed the edge of the free cells
// cell for cell; a few centimetres of tracking error put the robot in an
// occupied cell, and from there no plan exists ("Failed to create plan with
// tolerance of: 0.000000"). The map is the run's; the goal is the coverage
// start it navigated to, one cell from the edge like every coverage start.

const FIELD: &str = "field_2026_11_16_free_space_inflated.json";
/// The coverage start the run navigated to: cell (37, 225).
const FIELD_GOAL: (f64, f64) = (-11.48, -3.31);
/// The free cell farthest from the edge.
const FIELD_INTERIOR: (f64, f64) = (-7.26, -1.95);
/// A free cell on the far side of the zone, next to the edge: the way to the
/// goal from here goes round the zone's concave boundary.
const FIELD_FAR_SIDE: (f64, f64) = (-1.526, -6.062);
/// The cost one cell from an occupied one: 252 * exp(-4 * (0.05 - 0.0294)).
const NEXT_TO_EDGE: u8 = 232;

fn at(p: (f64, f64)) -> Pose2D {
    Pose2D::new(p.0, p.1, 0.0)
}

fn static_only(g: &OccupancyGrid) -> Costmap2D {
    static_costmap(g, &StaticLayerParams::new(true))
}

fn field_plan(map: &mut Costmap2D, start: &Pose2D, goal: &Pose2D) -> Result<Vec<Pose2D>, PlannerError> {
    let mut planner = NavfnPlanner::new(NavfnParams::mower(), map);
    plan(&mut planner, map, start, goal)
}

/// Metres from a point of a plan to the centre of the nearest occupied cell.
/// NavFn's points are in cell units from the origin (its `mapToWorld` has no
/// half cell), so that is the frame the distance is taken in.
fn clearance(g: &OccupancyGrid, p: &Pose2D) -> f64 {
    let (mx, my) = ((p.x - g.origin_x) / g.resolution, (p.y - g.origin_y) / g.resolution);
    let reach = 60i64;
    let mut best = f64::MAX;
    for cy in (my as i64 - reach).max(0)..(my as i64 + reach).min(g.height as i64) {
        for cx in (mx as i64 - reach).max(0)..(mx as i64 + reach).min(g.width as i64) {
            if g.data[cy as usize * g.width as usize + cx as usize] != 0 {
                best = best.min((mx - cx as f64).hypot(my - cy as f64));
            }
        }
    }
    best * g.resolution
}

/// The clearances of the poses more than `off` metres from both ends.
fn clearances_away_from_the_ends(g: &OccupancyGrid, path: &[Pose2D], off: f64) -> Vec<f64> {
    let (first, last) = (path[0], path[path.len() - 1]);
    let far = |p: &&Pose2D| (p.x - first.x).hypot(p.y - first.y) > off && (p.x - last.x).hypot(p.y - last.y) > off;
    path.iter().filter(far).map(|p| clearance(g, p)).collect()
}

fn min(v: &[f64]) -> f64 {
    v.iter().copied().fold(f64::MAX, f64::min)
}

fn occupied_neighbours(g: &OccupancyGrid, x: u32, y: u32) -> usize {
    let cell = |x: i64, y: i64| x < 0 || y < 0 || x >= g.width as i64 || y >= g.height as i64 || g.data[y as usize * g.width as usize + x as usize] != 0;
    let (x, y) = (x as i64, y as i64);
    [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)].into_iter().filter(|&(a, b)| cell(a, b)).count()
}

/// The layer is a cost gradient: it makes no cell an obstacle to NavFn (253
/// or more), and the free cells next to the edge all cost 232.
#[test]
fn field_inflation_adds_no_obstacle() {
    let g = fixture(FIELD);
    let (layer, master) = (static_only(&g), global_costmap(&g));
    let (mut free, mut edge, mut raised) = (0, 0, 0);
    for y in 0..g.height {
        for x in 0..g.width {
            let (before, after) = (layer.cost(x, y), master.cost(x, y));
            if before == LETHAL_OBSTACLE {
                assert_eq!(after, LETHAL_OBSTACLE);
                continue;
            }
            assert_eq!(before, FREE_SPACE);
            assert!(after < INSCRIBED_INFLATED_OBSTACLE, "({x}, {y}) costs {after}");
            free += 1;
            raised += (after != FREE_SPACE) as usize;
            if occupied_neighbours(&g, x, y) > 0 {
                assert_eq!(after, NEXT_TO_EDGE, "({x}, {y})");
                edge += 1;
            }
        }
    }
    // 28006 free cells: 787 of them next to the edge, 7932 within 0.5 m of it
    assert_eq!((free, edge, raised), (28006, 787, 7932));
}

/// From the middle of the zone to the coverage start: the plan is found, ends
/// on the goal, and more than 0.5 m from the goal it stays 0.4 m from the
/// edge, which the plan on the static layer alone did not.
#[test]
fn field_plan_to_the_coverage_start() {
    let g = fixture(FIELD);
    let (start, goal) = (at(FIELD_INTERIOR), at(FIELD_GOAL));

    let mut master = global_costmap(&g);
    let (gx, gy) = master.world_to_map(goal.x, goal.y).unwrap();
    assert_eq!((gx, gy), (37, 225));
    assert_eq!(occupied_neighbours(&g, gx, gy), 1, "the goal is one cell from the edge");
    assert_eq!(master.cost(gx, gy), NEXT_TO_EDGE);
    let inflated = field_plan(&mut master, &start, &goal).expect("a plan on the inflated costmap");
    assert_eq!(*inflated.last().unwrap(), goal);
    let last_cells = clearance(&g, &inflated[inflated.len() - 2]);
    assert!(last_cells < 0.15, "the plan does reach the edge at its goal: {last_cells}");

    let fixed = static_only(&g);
    let plain = field_plan(&mut fixed.clone(), &start, &goal).expect("a plan on the static layer");

    let with = min(&clearances_away_from_the_ends(&g, &inflated, 0.5));
    let without = min(&clearances_away_from_the_ends(&g, &plain, 0.5));
    assert!(with > 0.40, "inflated: {with}");
    assert!(without < 0.30, "static only: {without}");
    // this start is 2.5 m from any edge: the detour costs 5 cm
    assert!(length(&inflated) < length(&plain) + 0.10, "{} vs {}", length(&inflated), length(&plain));
}

/// The failure itself, on a way round the zone's boundary: on the static
/// layer alone the plan runs in the cells next to the edge (0.05 m) for
/// metres; inflated, it stays 0.4 m off wherever it is not arriving.
#[test]
fn field_plan_along_the_boundary_keeps_off_the_edge() {
    let g = fixture(FIELD);
    let (start, goal) = (at(FIELD_FAR_SIDE), at(FIELD_GOAL));

    let plain = field_plan(&mut static_only(&g), &start, &goal).expect("a plan on the static layer");
    let without = clearances_away_from_the_ends(&g, &plain, 1.0);
    assert!((min(&without) - 0.05).abs() < 1e-6, "static only: {}", min(&without));
    let on_the_edge = without.iter().filter(|c| **c < 0.12).count();
    assert!(on_the_edge >= 80, "static only: {on_the_edge} poses next to the edge"); // 2 m of path and more

    let inflated = field_plan(&mut global_costmap(&g), &start, &goal).expect("a plan on the inflated costmap");
    assert_eq!(*inflated.last().unwrap(), goal);
    let with = clearances_away_from_the_ends(&g, &inflated, 0.5);
    assert!(min(&with) > 0.40, "inflated: {}", min(&with));
    assert!(min(&clearances_away_from_the_ends(&g, &inflated, 1.0)) > 0.55);
    // and it is no detour: 10.7 m against 10.9 m along the edge
    assert!(length(&inflated) < length(&plain) * 1.05, "{} vs {}", length(&inflated), length(&plain));
}

/// Coverage paths start and end one cell from the edge, so those cells must
/// stay reachable: a corner cell with two occupied neighbours, and one in 25
/// of all the cells next to the edge, from the middle and back.
#[test]
fn field_edge_cells_stay_reachable() {
    let g = fixture(FIELD);
    let fresh = global_costmap(&g);
    let interior = at(FIELD_INTERIOR);
    // NavFn rounds to the cell, so aim a quarter into it
    let centre = |x: u32, y: u32| Pose2D::new(g.origin_x + (x as f64 + 0.25) * g.resolution, g.origin_y + (y as f64 + 0.25) * g.resolution, 0.0);

    assert_eq!(occupied_neighbours(&g, 36, 225), 2);
    let corner = centre(36, 225);
    let path = field_plan(&mut fresh.clone(), &interior, &corner).expect("a plan to the corner cell");
    assert_eq!(*path.last().unwrap(), corner);

    let edge: Vec<(u32, u32)> = (0..g.height)
        .flat_map(|y| (0..g.width).map(move |x| (x, y)))
        .filter(|&(x, y)| fresh.cost(x, y) != LETHAL_OBSTACLE && occupied_neighbours(&g, x, y) > 0)
        .collect();
    let mut planner = NavfnPlanner::new(NavfnParams::mower(), &fresh);
    let mut reached = 0;
    for &(x, y) in edge.iter().step_by(25) {
        let cell = centre(x, y);
        let mut map = fresh.clone();
        let there = plan(&mut planner, &mut map, &interior, &cell);
        assert!(there.is_ok(), "to ({x}, {y}): {there:?}");
        let back = plan(&mut planner, &mut map, &cell, &interior);
        assert!(back.is_ok(), "from ({x}, {y}): {back:?}");
        reached += 1;
    }
    assert_eq!(reached, 32);
}

/// What the layer does not do: once the robot is in an occupied cell (where
/// the run's robot stood when the map was taken), there is still no plan.
/// The gradient keeps the plan away from that; it is not a recovery.
#[test]
fn field_robot_in_an_occupied_cell_still_has_no_plan() {
    let g = fixture(FIELD);
    let stuck = Pose2D::new(-11.1225, -0.7535, -0.4756);
    for mut map in [static_only(&g), global_costmap(&g)] {
        let (x, y) = map.world_to_map(stuck.x, stuck.y).unwrap();
        assert_eq!(map.cost(x, y), LETHAL_OBSTACLE);
        assert_eq!(field_plan(&mut map, &stuck, &at(FIELD_GOAL)), Err(PlannerError::NoValidPathCouldBeFound));
    }
}

fn length(path: &[Pose2D]) -> f64 {
    path.windows(2).map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y)).sum()
}

/// The yaml still says what `GlobalLayers::mower()` says (the recording was
/// made from it), and the local costmap is as it was.
#[test]
fn mower_costmaps_match_the_yaml() {
    let path = format!("{}/../../../mower_nav2/config/nav2_no_map_params.yaml", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let y: serde_yaml::Value = serde_yaml::from_str(&text).unwrap_or_else(|e| panic!("{path}: {e}"));
    let names = |v: &serde_yaml::Value| v.as_sequence().unwrap().iter().map(|n| n.as_str().unwrap().to_string()).collect::<Vec<_>>();

    let global = &y["global_costmap"]["global_costmap"]["ros__parameters"];
    assert_eq!(names(&global["plugins"]), ["static_layer", "inflation_layer"]);
    assert_eq!(global["static_layer"]["plugin"].as_str(), Some("nav2_costmap_2d::StaticLayer"));
    assert_eq!(global["static_layer"]["map_topic"].as_str(), Some("/map_grid_global"));
    assert_eq!(global["track_unknown_space"].as_bool(), Some(true));
    assert_eq!(global["robot_radius"].as_f64(), Some(GLOBAL_ROBOT_RADIUS));
    let layer = &global["inflation_layer"];
    assert_eq!(layer["plugin"].as_str(), Some("nav2_costmap_2d::InflationLayer"));
    let p = InflationLayerParams::mower_global();
    assert_eq!(layer["inflation_radius"].as_f64(), Some(p.inflation_radius));
    assert_eq!(layer["cost_scaling_factor"].as_f64(), Some(p.cost_scaling_factor));
    // left at upstream's defaults, as are the footprint and its padding
    let d = InflationLayerParams::default();
    assert_eq!((p.enabled, p.inflate_unknown, p.inflate_around_unknown), (d.enabled, d.inflate_unknown, d.inflate_around_unknown));
    for key in ["enabled", "inflate_unknown", "inflate_around_unknown"] {
        assert!(layer.get(key).is_none(), "inflation_layer.{key}");
    }
    for key in ["footprint", "footprint_padding", "filters"] {
        assert!(global.get(key).is_none(), "global_costmap.{key}");
    }
    assert_eq!(*InflationLayer::mower_global().params(), p);
    // the inscribed band must not reach the next cell
    assert!(InflationLayer::mower_global().inscribed_radius() < global["resolution"].as_f64().unwrap());

    let local = &y["local_costmap"]["local_costmap"]["ros__parameters"];
    assert_eq!(names(&local["plugins"]), ["static_layer"]);
    assert_eq!(local["robot_radius"].as_f64(), Some(0.5));
    assert!(local.get("inflation_layer").is_none());
}
