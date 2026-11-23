//! mechbot: the buckeye-apex Mech Royale bot (mechfight/1 over stdin/stdout).
#![allow(dead_code)]

mod bot;
mod proto;
mod world;

use proto::{Action, Hello, Obs};
use std::io::{BufRead, BufReader, Write};
use std::panic::{self, AssertUnwindSafe};
use std::rc::Rc;
use std::sync::mpsc::{self, Receiver, TryRecvError};

enum Agent {
    Blind,
    Bot(Box<bot::Bot>),
}

/// The fallback when the arena cannot be loaded: walk to the next circle, shoot the nearest mech.
fn blind_act(o: &Obs) -> Action {
    let mut a = Action::new(o.tick);
    let me = &o.self_;
    let (gx, gy) = (o.zone.next_center[0], o.zone.next_center[1]);
    let yaw = (gy - me.pos[1]).atan2(gx - me.pos[0]);
    if (gx - me.pos[0]).hypot(gy - me.pos[1]) > 10.0 {
        a.mv = [yaw.cos(), yaw.sin()];
    }
    a.aim = [yaw, 0.0];
    a.has_aim = true;
    a.boost = !me.in_zone;
    let mut best = -1.0;
    for m in &o.mechs {
        if m.kind != "mech" {
            continue;
        }
        let d = (m.pos[0] - me.pos[0]).hypot(m.pos[1] - me.pos[1]);
        if best < 0.0 || d < best {
            best = d;
            let tf = d / 252.0;
            a.aim = [
                (m.pos[1] + m.vel[1] * tf - me.pos[1]).atan2(m.pos[0] + m.vel[0] * tf - me.pos[0]),
                (m.pos[2] + 4.5 - me.pos[2] - 7.0).atan2(d.max(1.0)),
            ];
            a.fire = [true, true];
        }
    }
    if o.offer.is_some() {
        a.pick = 0;
    }
    a
}

fn load_world(dirs: &[&str]) -> Option<world::World> {
    for d in dirs {
        if d.is_empty() {
            continue;
        }
        match world::World::load(d) {
            Ok(w) => return Some(w),
            Err(e) => eprintln!("world load from {:?} failed: {}", d, e),
        }
    }
    None
}

/// Drops ".0" style zero fractions outside strings (for integer fields written as floats).
fn strip_zero_fractions(inp: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(inp.len());
    let (mut in_str, mut esc) = (false, false);
    let mut i = 0;
    while i < inp.len() {
        let c = inp[i];
        if in_str {
            out.push(c);
            if esc {
                esc = false;
            } else if c == b'\\' {
                esc = true;
            } else if c == b'"' {
                in_str = false;
            }
            i += 1;
            continue;
        }
        if c == b'"' {
            in_str = true;
            out.push(c);
            i += 1;
            continue;
        }
        if c == b'.' && i > 0 && inp[i - 1].is_ascii_digit() {
            let mut j = i + 1;
            while j < inp.len() && inp[j] == b'0' {
                j += 1;
            }
            if j > i + 1 && j < inp.len() && matches!(inp[j], b',' | b']' | b'}' | b' ' | b'\n') {
                i = j;
                continue;
            }
        }
        out.push(c);
        i += 1;
    }
    out
}

fn sniff_tick(line: &[u8]) -> Option<i64> {
    let pat = b"\"tick\":";
    let k = line.windows(pat.len()).position(|w| w == pat)?;
    let (mut n, mut seen) = (0i64, false);
    for &c in &line[k + pat.len()..] {
        if c == b' ' && !seen {
            continue;
        }
        if !c.is_ascii_digit() {
            break;
        }
        n = n * 10 + (c - b'0') as i64;
        seen = true;
    }
    if seen {
        Some(n)
    } else {
        None
    }
}

fn parse_with_retry<T: serde::de::DeserializeOwned>(line: &[u8]) -> Result<T, String> {
    match serde_json::from_slice::<T>(line) {
        Ok(v) => Ok(v),
        Err(e) => serde_json::from_slice::<T>(&strip_zero_fractions(line)).map_err(|_| e.to_string()),
    }
}

fn build_bot(h: Hello) -> Agent {
    let dir = h.map.dir.clone();
    let w = match load_world(&[&dir, "/arena", "arena"]) {
        Some(w) => Rc::new(w),
        None => return Agent::Blind,
    };
    let mech = &h.config.mech;
    let nav = Rc::new(world::Nav::new(w.clone(), *mech.get("step_up").unwrap_or(&1.2), *mech.get("height").unwrap_or(&9.0)));
    let mut p = bot::Params::default_params();
    if let Ok(v) = std::env::var("BOT_VARIANT") {
        if !v.is_empty() && !bot::variants::apply(&v, &mut p) {
            eprintln!("unknown BOT_VARIANT {:?}", v);
        }
    }
    if let Ok(raw) = std::env::var("BOT_PARAMS") {
        if !raw.is_empty() {
            if let Err(e) = p.apply_json(&raw) {
                eprintln!("bad BOT_PARAMS: {}", e);
            }
        }
    }
    Agent::Bot(Box::new(bot::Bot::new(h, w, nav, p)))
}

struct Io {
    out: std::io::BufWriter<std::io::Stdout>,
    buf: String,
}

impl Io {
    fn send(&mut self, act: &Action) {
        self.buf.clear();
        act.append_json(&mut self.buf);
        self.buf.push('\n');
        let _ = self.out.write_all(self.buf.as_bytes());
        let _ = self.out.flush();
    }
}

enum Handled {
    Obs(Vec<u8>),
    Done,
    Nothing,
}

fn handle(line: Vec<u8>, ag: &mut Option<Agent>, io: &mut Io) -> Handled {
    let head = &line[..line.len().min(40)];
    if head.windows(5).any(|w| w == b"\"obs\"") {
        return Handled::Obs(line);
    }
    #[derive(serde::Deserialize)]
    struct Head {
        #[serde(rename = "type", default)]
        typ: String,
    }
    let head: Head = match serde_json::from_slice(&line) {
        Ok(h) => h,
        Err(_) => return Handled::Nothing,
    };
    match head.typ.as_str() {
        "obs" => Handled::Obs(line),
        "end" => Handled::Done,
        "hello" => {
            let h: Hello = parse_with_retry(&line).unwrap_or_else(|e| {
                eprintln!("bad hello: {}", e);
                Hello::default()
            });
            let built = panic::catch_unwind(AssertUnwindSafe(|| build_bot(h)));
            let agent = built.unwrap_or_else(|_| {
                eprintln!("init panic: falling back to the blind agent");
                Agent::Blind
            });
            let name = match &agent {
                Agent::Bot(b) => b.p.name.clone(),
                Agent::Blind => "apex-go".to_string(),
            };
            *ag = Some(agent);
            let _ = writeln!(io.out, "{{\"type\":\"ready\",\"name\":{}}}", serde_json::to_string(&name).unwrap());
            let _ = io.out.flush();
            Handled::Nothing
        }
        _ => Handled::Nothing,
    }
}

fn run(rx: Receiver<Vec<u8>>, no_drain: bool) {
    let mut io = Io { out: std::io::BufWriter::with_capacity(1 << 16, std::io::stdout()), buf: String::with_capacity(512) };
    let mut ag: Option<Agent> = None;
    let mut last = Action::new(0);
    let mut bad_obs = 0u64;
    while let Ok(line) = rx.recv() {
        let mut latest = match handle(line, &mut ag, &mut io) {
            Handled::Done => return,
            Handled::Obs(l) => Some(l),
            Handled::Nothing => None,
        };
        // newest-observation rule: answer only the freshest of whatever has queued up
        if !no_drain {
            loop {
                match rx.try_recv() {
                    Ok(more) => match handle(more, &mut ag, &mut io) {
                        Handled::Done => return,
                        Handled::Obs(l) => latest = Some(l),
                        Handled::Nothing => {}
                    },
                    Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => break,
                }
            }
        }
        let latest = match latest {
            Some(l) => l,
            None => continue,
        };
        let o: Obs = match parse_with_retry(&latest) {
            Ok(o) => o,
            Err(e) => {
                bad_obs += 1;
                if bad_obs <= 5 || bad_obs % 200 == 0 {
                    eprintln!("bad obs ({} so far): {}", bad_obs, e);
                }
                if let Some(tick) = sniff_tick(&latest) {
                    let mut act = Action::new(tick);
                    act.mv = last.mv;
                    act.boost = last.boost;
                    act.aim = last.aim;
                    act.has_aim = last.has_aim;
                    io.send(&act);
                }
                continue;
            }
        };
        let tick = o.tick;
        let agent = ag.get_or_insert(Agent::Blind);
        let mut act = match agent {
            Agent::Blind => blind_act(&o),
            Agent::Bot(b) => match panic::catch_unwind(AssertUnwindSafe(|| b.act(o))) {
                Ok(a) => a,
                Err(_) => {
                    eprintln!("panic at tick {}", tick);
                    let mut a = Action::new(tick);
                    a.mv = last.mv;
                    a.boost = last.boost;
                    a.aim = last.aim;
                    a.has_aim = last.has_aim;
                    a
                }
            },
        };
        act.tick = tick;
        last = act.clone();
        io.send(&act);
    }
}

fn main() {
    // keep panics short on stderr (the default hook prints a full message per panic)
    panic::set_hook(Box::new(|info| eprintln!("panic: {}", info)));
    let (tx, rx) = mpsc::channel::<Vec<u8>>();
    std::thread::spawn(move || {
        let mut rd = BufReader::with_capacity(1 << 20, std::io::stdin().lock());
        let mut line = Vec::with_capacity(1 << 16);
        loop {
            line.clear();
            match rd.read_until(b'\n', &mut line) {
                Ok(0) | Err(_) => return,
                Ok(_) => {
                    if line.iter().any(|c| !c.is_ascii_whitespace()) && tx.send(line.clone()).is_err() {
                        return;
                    }
                }
            }
        }
    });
    // MECHBOT_NO_DRAIN=1 answers every observation in order (deterministic replays for testing)
    let no_drain = std::env::var("MECHBOT_NO_DRAIN").map(|v| !v.is_empty()).unwrap_or(false);
    run(rx, no_drain);
}
