//! Cross-check actual CAD export probes with complete indexed continuous material.
use super::*;

#[test]
#[ignore = "reads CAD-export probes and emits whole-roll material evidence"]
fn cad_export_material_probes_match_continuous_indexed_sweeps() {
    let input = std::env::var_os("SOLVENT_CAD_PROBES_INPUT").expect("set input probe path");
    let output = std::env::var_os("SOLVENT_CAD_MATERIAL_OUTPUT").expect("set output evidence path");
    let text = std::fs::read_to_string(input).unwrap();
    let pair = Pair::read([24,48],2.);
    let options = Options {value_tolerance:0.0002,max_evaluations:100000};
    let mut cases = vec![];
    let mut failures = vec![];
    for member in 0..2 {
        let mut material = Member::read(&pair,member);
        let frame = pair.local_frame(member).inverse();
        let mut rows = vec![];
        for line in text.lines() {
            let columns: Vec<_> = line.split_whitespace().collect();
            assert_eq!(columns.len(),6);
            let selected: usize = columns[0].parse().unwrap();
            assert!(selected < 2);
            if selected != member { continue; }
            let label = columns[1];
            assert!(label.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-'));
            let inside = match columns[2] { "1" => true,"0" => false,_ => panic!("invalid material sign") };
            let local = std::array::from_fn(|k| columns[k+3].parse::<f64>().unwrap());
            let p = frame.point(local);
            let started = std::time::Instant::now();
            let found = material.bounds(p.map(|v| I::point(v).unwrap()),options);
            let bounds = found.value.bounds();
            let converged = found.sweeps.iter().all(|s| s.status == Status::Converged);
            let matches = if inside { bounds[1] < 0. } else { bounds[0] > 0. };
            eprintln!("CAD member {member} {label}: expected {inside}, {bounds:?}, converged {converged}, {:?}",started.elapsed());
            if matches && converged {
                rows.push(found.row(p,label,inside));
            } else {
                failures.push(format!("{{\"member\":{member},\"label\":\"{label}\",\"expected_inside\":{inside},\"position_mm\":{p:?},\"material\":{bounds:?},\"converged\":{converged}}}"));
            }
        }
        assert!(!rows.is_empty() || !failures.is_empty(),"no probes supplied");
        cases.push(format!("{{\"definition\":{},\"points\":[{}]}}",material.definition,rows.join(",")));
    }
    std::fs::write(output,format!("{{\"schema\":5,\"units\":{{\"length\":\"mm\",\"angle\":\"rad\"}},\"source_error\":\"not certified\",\"value_tolerance_mm\":{},\"cad_failures\":[{}],\"cases\":[{}]}}\n",
        options.value_tolerance,failures.join(","),cases.join(","))).unwrap();
    assert!(failures.is_empty(),"CAD and continuous material disagree: {}",failures.join(","));
}
