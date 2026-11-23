# buckeye-apex

A Rust bot for the Builders Buildathon Mech Royale challenge on the Ohio State campus.
I wrote buckeye-apex with Claude. During development, I used reinforcement learning algorithms
to develop the decision rules and tune the numerical thresholds used by the bot.

The bot combines a geometric world model, opponent memory, weapon-specific aiming, and a
rule-based policy. It runs locally as a static binary in a `FROM scratch` Docker image.

## The challenge

The objective is mean placement points: `16 - place` per match, from 0 to 15.
Same-tick deaths share a place.

| Stage | Opponents | Strategic consequence |
| --- | --- | --- |
| Practice | 96 matches against 15 house bots that share vision, never fight each other, and hunt the entrant | Waiting for opponents to eliminate each other is ineffective; growth and manageable fights matter. |
| Qualifying and finals | 16-mech free-for-alls on fresh seeds | Preserve health, avoid being surrounded, and exploit favorable fights. |

Practice varies the spawn and slot. A fixed slot is not a reliable signal of match format.
The [challenge page](https://www.joinbuilders.org/buildathon/tracks/builders) defines the competition
format; its current practice rules supersede the older 48-match description in the kit.

## Approach

The policy aims to improve expected placement through five connected decisions.

1. **Get established.** Find a useful second weapon and collect reachable cubes. The opening
   discounts loot exposed to known enemies (`LootHiddenBias = 0.8`), balancing growth against
   the risk of dying before the loadout is ready.
2. **Infer the opposition.** `hide.rs` starts with an opening stance, then uses deaths not
   attributable to the bot and observed distant laser activity as evidence of a free-for-all.
   Otherwise it uses the squad stance. This is a heuristic: absence of evidence does not prove
   a squad. The default leaves the optional slot-based rule disabled.
3. **Choose a fight with an exit.** `policy.rs` compares estimated damage output, incoming
   pressure, health, range, and cover. The squad stance can pursue favorable targets; the
   free-for-all stance uses cautious brawling and can decline marginal engagements.
4. **Execute the mechanics.** Combat uses projectile lead, mortar trajectories, laser
   charge/release timing, line-of-sight checks, and weapon matchup estimates. Movement evaluates
   candidate directions against incoming projectiles and nearby threats.
5. **Rotate and recover.** Navigation routes around terrain and buildings toward the next
   circle. When conditions permit, the bot breaks contact and avoids unnecessary fire to
   preserve its opportunity to regenerate.

The [protocol](https://www.joinbuilders.org/buildathon/tracks/builders/files/protocol.md?inline=1)
defines the mechanics. Live `hello.config`, `self.stats`, weapon stats, and zone observations
supply match values. Documented defaults are examples, rather than guaranteed settings.

### Fight selection is a heuristic

`KEngage` raises the minimum engagement advantage when more opponents remain, in stances
with avoidance enabled. Advantage estimates compare our time to kill a target with its time
to kill us. `CoverEdgeAdv = 2.2` controls another engagement gate when a laser edge is absent.

The threshold table was motivated by the simplified condition

```text
p > k / (2 + h(k - 1))
```

Here `k` is the number of rivals, `p` is the estimated chance of winning an isolated fight,
and `h` approximates the future value retained after winning. This assumes an equal-opponent
free-for-all and a simplified continuation value. It does not establish an optimal policy
against a coordinated squad, account for every third-party attack, or guarantee a placement gain.
Actual decisions also depend on the stance, threat assessment, and retreat rules.

### Endgame behavior follows the configured schedule

`policy.rs` computes the endgame reference time from the sum of configured zone phase
durations, bounded by the match duration. `EndgameS = 60` activates the late policy before
that time. The rulebook's approximate 415-second collapse and 480-second match limit are
defaults, not fixed assumptions about every match.

The late policy raises engagement thresholds, limits cube detours to nearby opportunities,
and tries to hold covered ground in the last circle before moving inward. With one rival
remaining, it compares projected health before deciding whether to force a fight.

Preserving health can help at collapse, but arrival HP alone does not guarantee the outcome:
position, ongoing attacks, and the actual zone still matter. If the match clock expires,
the protocol ranks survivors by HP. Endgame improvements need matches that actually reach
this situation.

## Implementation

| Path | Responsibility |
| --- | --- |
| `src/main.rs` | JSON-lines protocol loop, queued-observation draining, tick echoing, output flushing, panic handling, and fallback agent |
| `src/proto.rs` | Observation, configuration, and action types |
| `src/world/` | Terrain and solid geometry, visibility, navigation grid, A*, rooftops, and exposure |
| `src/bot/mod.rs` | Agent state, opponent tracks, damage history, and configuration access |
| `src/bot/hide.rs` | Lobby inference and stance selection |
| `src/bot/policy.rs` | Targets, engagement, retreat, loot, zone movement, and late policy |
| `src/bot/combat.rs`, `src/bot/lead.rs` | Aiming, fire decisions, and opponent motion estimates |
| `src/bot/movement.rs` | Movement selection and projectile avoidance |
| `src/bot/strategy.rs` | Zone assessment, equipment, loot, and upgrade priorities |
| `src/bot/params.rs`, `src/bot/variants.rs` | Default parameters and experimental variants |

Expensive shared navigation maps are prepared during initialization. The protocol loop normally
answers the newest queued observation. Initialization or map-loading failure selects a simpler
fallback; a decision panic preserves the previous movement and aim in a fresh action.

## Build and check

Requires Rust for a native build, or Docker for the submission image, plus Python 3 and the
[official kit](https://www.joinbuilders.org/buildathon/tracks/builders/files/mechfight-kit.tar.gz)
for the checker and map. The kit is separate from this repository.

From the repository root, set the path to the extracted kit:

```sh
KIT_DIR=/absolute/path/to/mechfight-kit
cargo build --release --locked
python3 "$KIT_DIR/check/check.py" -- ./target/release/mechbot
```

Build and check the submission image under the match limits:

```sh
docker build --platform linux/amd64 -t buckeye-apex .
python3 "$KIT_DIR/check/check.py" --map-dir /arena -- \
  docker run --rm -i --network none --cpus 1 --memory 512m \
  --read-only --tmpfs /tmp:rw,size=64m --user 65534:65534 \
  -v "$KIT_DIR/arena:/arena:ro" buckeye-apex
```

The runtime budget is 50 ms per observation on 1 CPU with 512 MB RAM. The image needs no
runtime network or writes outside `/tmp`. Logs go to stderr; stdout carries protocol JSON.
Submission builds target `linux/amd64`, must finish within 10 minutes, and the submitted
repository must fit within 20 MB.

The checker verifies protocol behavior and timing using recorded observations. Its observations
do not react to actions, so a pass does not measure strategy.

## Evaluation and next improvements

Evaluate squad and free-for-all play separately. Use paired seeds and matching spawn slots for
local comparisons, then use fresh seeds and varied slots to check whether gains hold up.
Prioritize opening survival, time to a useful loadout, cube collection, damage efficiency,
successful escapes, and placement. Choose changes by their placement effect in the relevant
format.

Earlier development experiments reported the following local self-play results against 15
copies of a comparison build:

| Experiment | Reported placement-point change | Paired seeds |
| --- | --- | --- |
| Opening: `LootHiddenBias = 0.8` | +0.46 ± 0.21 | 72 |
| Hunt range 130 / lightning pad 12 | +0.74 ± 0.62 | 19 |
| Fight threshold table | −0.05 ± 0.15 | 129 |
| Endgame policy | Collapse was not reached | 47 |

These are historical results carried over from development, not measurements rerun for this
README. Self-play does not reproduce the coordinated practice squad or the finalist field.
The threshold experiment did not establish a gain, and the endgame experiment did not test
its intended situation. Neither local scores nor small differences between practice submissions
establish an improvement without further evidence.

The next useful experiments are changes to opening exposure and equipment acquisition,
fight execution under multiple attackers, and late-circle behavior on configurations where
collapse is reached. Keep each comparison narrow enough to identify which decision changed.

## Configuration

- `BOT_PARAMS`: JSON parameter overrides, matched case-insensitively with optional underscores;
  for example `{"LootHiddenBias":0.8,"EndgameS":60,"KEngage":true}`.
- `BOT_VARIANT`: a named experiment from `src/bot/variants.rs`, applied before `BOT_PARAMS`.
- `MECH_TRACE=1`: decision tracing to stderr.
- `MECHBOT_NO_DRAIN=1`: process observations in order for deterministic replay diagnostics.
  Leave it unset for competition so queued observations are drained.
