//! `\u` escapes in JSON strings: a surrogate pair is one character (issue #118).
use gcs_core::json::{parse, Json};

fn text(s: &str) -> String {
    match parse(s).unwrap() {
        Json::Str(t) => t,
        other => panic!("not a string: {other:?}"),
    }
}

#[test]
fn a_surrogate_pair_is_one_character() {
    assert_eq!(text(r#""😀""#), "😀");
    assert_eq!(text(r#""a😀b""#), "a😀b");
    assert_eq!(text(r#""é€""#), "é€");
}

#[test]
fn a_lone_surrogate_is_a_replacement() {
    assert_eq!(text(r#""\ud83d""#), "\u{fffd}");
    assert_eq!(text(r#""\ude00""#), "\u{fffd}");
    assert_eq!(text(r#""\ud83dx""#), "\u{fffd}x");
    assert_eq!(text(r#""\ud83dA""#), "\u{fffd}A");
}

#[test]
fn a_short_escape_is_an_error() {
    assert!(parse(r#""\u12""#).is_err());
}
