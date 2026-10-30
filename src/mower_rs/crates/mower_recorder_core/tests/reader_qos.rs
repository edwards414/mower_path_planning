//! The reader QoS rule (`recording::reader_qos`) against what rosbag2 did in
//! the E5-0 runs: each fixture's `qos.json` (every topic's publishers and
//! subscriptions with their full QoS, 20 s into the run, as rmw_cyclonedds
//! reported them) holds the `mower_*_recorder` subscriptions; the rule,
//! given the publishers and qos_overrides.yaml, must give the same QoS.
//! The offers are taken twice: from qos.json's publishers, and from the
//! bag's channel (the writers rosbag2 knew when it created the topic).

mod bagfile;

use std::collections::BTreeMap;

use bagfile::*;
use mower_mcap::{Qos, INFINITE};
use mower_recorder_core::profile;
use mower_recorder_core::recording::{duration_from_ns, reader_qos, QosOverrides};
use serde_json::Value;

/// A profile from qos.json (rmw's report).
fn qos_of(v: &Value) -> Qos {
    let s = |k: &str| v[k].as_str().unwrap().to_string();
    let ns = |k: &str| duration_from_ns(v[k].as_u64().unwrap());
    Qos {
        history: s("history"),
        depth: v["depth"].as_u64().unwrap(),
        reliability: s("reliability"),
        durability: s("durability"),
        deadline: ns("deadline_ns"),
        lifespan: ns("lifespan_ns"),
        liveliness: s("liveliness"),
        liveliness_lease_duration: ns("liveliness_lease_duration_ns"),
        avoid_ros_namespace_conventions: v["avoid_ros_namespace_conventions"].as_bool().unwrap(),
    }
}

/// What Cyclone DDS reports for what rclcpp left to the rmw.
fn as_rmw_reports(mut q: Qos) -> Qos {
    let d = |t: (u64, u64)| if t == (0, 0) { INFINITE } else { t };
    q.deadline = d(q.deadline);
    q.lifespan = d(q.lifespan);
    q.liveliness_lease_duration = d(q.liveliness_lease_duration);
    if q.liveliness == "system_default" {
        q.liveliness = "automatic".into();
    }
    q
}

#[test]
fn the_recorders_subscribed_as_the_rule_says() {
    let overrides = QosOverrides::load(&config_dir().join("qos_overrides.yaml")).unwrap();
    assert_eq!(overrides.topics().count(), 9);
    let runs = [
        ("F1", "MW-E5FIX_20261030T081851", config_dir().join("record_topics.yaml")),
        ("F2", "MW-E5FIX_20261030T080903", golden().join("F2/record_topics_split.yaml")),
        ("F2S", "MW-E5FIX_20261030T081247", golden().join("F2S/record_topics_split.yaml")),
        ("F3", "MW-E5FIX_20261030T080906", config_dir().join("data_collection.yaml")),
    ];
    let (mut checked, mut overridden, mut adapted_from_channel, mut unpublished) = (0, 0, 0, 0);
    for (f, run, profile_path) in runs {
        let qos: Value = serde_json::from_slice(&unzst(&golden().join(f).join("qos.json.zst"))).unwrap();
        let topics = qos["topics"].as_object().unwrap();
        let cfg = profile::load_config(Some(&profile_path), &mut Vec::new());
        let specs = profile::recorder_specs(&cfg, &golden().join(f).join(run), None, "t").unwrap();
        for spec in &specs {
            let node = format!("/{}", spec.node_name);
            // the channels rosbag2 created (file 0's summary)
            let bag = read(&unzst(&spec.out_dir.join(format!("{}_0.mcap.zst", spec.out_dir.file_name().unwrap().to_str().unwrap()))));
            let channels: BTreeMap<&str, &Channel> = bag.channels.values().map(|c| (c.topic.as_str(), c)).collect();
            for name in &spec.topics {
                let subs: Vec<&Value> = topics
                    .get(name.as_str())
                    .map(|t| t["subscriptions"].as_array().unwrap().iter().filter(|s| s["node"] == node.as_str()).collect())
                    .unwrap_or_default();
                if profile::is_hidden(name) {
                    assert!(subs.is_empty(), "{f} {node} subscribed to the hidden {name}");
                    assert!(!spec.recorded.contains(name));
                    continue;
                }
                let pubs: Vec<Qos> = topics.get(name.as_str()).map(|t| t["publishers"].as_array().unwrap().iter().map(|p| qos_of(&p["qos"])).collect()).unwrap_or_default();
                if pubs.is_empty() {
                    // rosbag2 subscribes to a name only once it has a writer
                    assert!(subs.is_empty() && !channels.contains_key(name.as_str()), "{f} {node} {name}");
                    unpublished += 1;
                    continue;
                }
                assert_eq!(subs.len(), 1, "{f} {node} {name}");
                let want = qos_of(&subs[0]["qos"]);
                let from_now = as_rmw_reports(reader_qos(name, &pubs, &overrides));
                assert_eq!(from_now, want, "{f} {node} {name}: from qos.json's publishers");
                if let Some(ch) = channels.get(name.as_str()) {
                    let offers = Qos::parse_list(ch.meta("offered_qos_profiles")).unwrap();
                    assert_eq!(as_rmw_reports(reader_qos(name, &offers, &overrides)), want, "{f} {node} {name}: from the channel");
                    adapted_from_channel += 1;
                }
                if overrides.get(name).is_some() {
                    overridden += 1;
                    // the writers alone (depth 1) would give keep_all
                    let adapted = as_rmw_reports(reader_qos(name, &pubs, &QosOverrides::empty()));
                    assert_ne!(adapted, want, "{f} {name}: the override matters");
                }
                checked += 1;
            }
        }
    }
    eprintln!("{checked} subscriptions checked ({overridden} by an override, {adapted_from_channel} against the channel too), {unpublished} unpublished names not subscribed");
    assert!(checked >= 90 && overridden >= 25 && unpublished >= 4, "{checked} {overridden} {unpublished}");
}
