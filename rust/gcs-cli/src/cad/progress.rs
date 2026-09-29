//! What an export says while it works: prose on stderr, and the stage keys a harness reads.
#![cfg_attr(not(feature="occt"),allow(dead_code))]
pub use gcs_core::solid::export::Stage;

/// Progress on stderr: a member takes minutes, and the JSON report owns stdout. Each line
/// carries the time since the first, so a whole export reads as one timeline.
static START: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
pub fn stage(message: &str) {
    if held(|lines| lines.push(Said::Line(message.into()))) { return; }
    let at = START.get_or_init(std::time::Instant::now).elapsed().as_secs_f64();
    eprintln!("solventc: [{at:7.1} s] {message}");
}

/// Start the timeline now, if nothing has: an export's lines count from its start.
pub fn start() { START.get_or_init(std::time::Instant::now); }

/// What a task run beside others said, kept to be said once they are done.
enum Said { Line(String),Trace(String) }

std::thread_local! { static HELD: std::cell::RefCell<Option<Vec<Said>>> = const { std::cell::RefCell::new(None) }; }

/// Keep what is said on this thread, if it is holding it.
fn held(keep: impl FnOnce(&mut Vec<Said>)) -> bool {
    HELD.with(|h| h.borrow_mut().as_mut().map(keep).is_some())
}

/// Run `there` on a thread of its own beside `here` on this one, and say what each said, `there`'s
/// first, once both are done — `here`'s only if `there` has not `failed`, as running them one
/// after the other would. Both results.
pub fn beside<A: Send,B>(there: impl FnOnce() -> A+Send,here: impl FnOnce() -> B,failed: impl Fn(&A) -> bool) -> (A,B) {
    let ((a,said),(b,mine)) = both(there,here);
    let quiet = failed(&a);
    speak(said);
    if !quiet { speak(mine); }
    (a,b)
}

/// The same, `here` running first as it were: what it said said first, and `there`'s only if
/// `here` has not `failed`. Both results, `here`'s first.
pub fn under<A,B: Send>(here: impl FnOnce() -> A,there: impl FnOnce() -> B+Send,failed: impl Fn(&A) -> bool) -> (A,B) {
    let ((b,said),(a,mine)) = both(there,here);
    let quiet = failed(&a);
    speak(mine);
    if !quiet { speak(said); }
    (a,b)
}

/// `there` on a thread of its own and `here` on this one, what each says held: each result with it.
fn both<A: Send,B>(there: impl FnOnce() -> A+Send,here: impl FnOnce() -> B) -> ((A,Vec<Said>),(B,Vec<Said>)) {
    std::thread::scope(|scope| {
        let running = scope.spawn(move || {
            HELD.with(|h| *h.borrow_mut() = Some(Vec::new()));
            let result = there();
            (result,HELD.with(|h| h.borrow_mut().take().unwrap_or_default()))
        });
        let before = HELD.with(|h| h.borrow_mut().replace(Vec::new()));
        let b = here();
        let mine = HELD.with(|h| std::mem::replace(&mut *h.borrow_mut(),before).unwrap_or_default());
        (running.join().unwrap_or_else(|e| std::panic::resume_unwind(e)),(b,mine))
    })
}

/// Say again what was held.
fn speak(said: Vec<Said>) { for s in said { match s { Said::Line(m) => stage(&m), Said::Trace(k) => trace(&k) } } }

/// Run `tasks` side by side, each on a thread of its own, and say what each said, in the tasks'
/// order, once all are done — up to and including the first whose result `failed`, as running them
/// one after another would have. The results, in order.
pub fn side_by_side<'a,T: Send>(tasks: Vec<Box<dyn FnOnce() -> T+Send+'a>>,failed: impl Fn(&T) -> bool) -> Vec<T> {
    let done: Vec<(T,Vec<Said>)> = std::thread::scope(|scope| {
        let running: Vec<_> = tasks.into_iter().map(|task| scope.spawn(move || {
            HELD.with(|h| *h.borrow_mut() = Some(Vec::new()));
            let result = task();
            (result,HELD.with(|h| h.borrow_mut().take().unwrap_or_default()))
        })).collect();
        running.into_iter().map(|t| t.join().unwrap_or_else(|e| std::panic::resume_unwind(e))).collect()
    });
    let mut results = Vec::new();
    let mut speaking = true;
    for (result,said) in done {
        if speaking {
            speak(said);
            speaking = !failed(&result);
        }
        results.push(result);
    }
    results
}

/// A stage completed, for a harness: with `SOLVENT_STAGE_TRACE` naming a file, one line
/// `key<TAB>seconds` is appended to it, the keys `Stage::key`'s in the order `Stage::ORDER`
/// completes them. Independent of the prose lines, which may change.
pub fn mark(stage: Stage) { trace(stage.key()); }

/// A stage refused, for the same harness: the line `refused:key<TAB>seconds`.
pub fn refused(stage: Stage) { trace(&format!("refused:{}",stage.key())); }

fn trace(key: &str) {
    if held(|lines| lines.push(Said::Trace(key.into()))) { return; }
    let Ok(path) = std::env::var("SOLVENT_STAGE_TRACE") else { return };
    let at = START.get_or_init(std::time::Instant::now).elapsed().as_secs_f64();
    use std::io::Write;
    if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
        let _ = writeln!(file,"{key}\t{at:.3}");
    }
}
