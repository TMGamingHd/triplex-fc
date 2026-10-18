// The director: which camera, looking at which body, at each moment of a flight, the way a broadcast cuts between the ground, the tracking shot and the vehicle's own views.
//
// A rule picks the shot from the pose alone (where the vehicle is, whether a stage has just been let go, whether a stage is coming down to the tower), and a shot is held for a few seconds before another may replace it, so
// that the picture does not flicker between two cameras when a number crosses a threshold. The rules are a table of the flight's kinds of moment, in order:
//   on the pad and the first kilometres      the ground camera, which keeps the vehicle in the picture as it leaves
//   the climb to staging                      the tracking camera: a long lens by the pad
//   twelve seconds after a stage is let go    a chase view of the pair
//   the stage coming back                     the stage's chase view, then the ground camera for the last two kilometres, until it is held in the arms
//   the rest of the ship's flight             a chase view in the air, the Earth view in space
// Nothing here moves a camera: it returns {cam, focus}; the page applies it, and a person who takes the camera (a drag, a click on a camera button) takes the director off.

const HOLD = 6;                       // seconds a shot is kept before a different one may replace it

export class Director {
  constructor() { this.reset(); }
  reset() { this.cam = null; this.focus = 'main'; this.since = -1e9; this.stagedAt = -1e9; this.lastStg = null; this.heldAt = null; this.lastT = -1e9; }

  /** The shot for the moment `raw` (the pose with the other bodies in it). Returns {cam, focus, cut}: `cut` is true when it differs from the one before. */
  shot(raw) {
    const t = raw.t;
    if (t < this.lastT - 1) this.reset();                            // time went back (a seek): the flight starts again for the director
    this.lastT = t;
    const stg = raw.stg ?? 255;
    if (this.lastStg != null && stg < this.lastStg && (this.lastStg & ~stg) !== 0) this.stagedAt = t;
    this.lastStg = stg;
    const body = (raw.others || []).find((o) => o.name !== 'main' && o.pose);
    const b = body ? body.pose : null;
    const sinceStage = t - this.stagedAt;
    let want;
    if (raw.clamp || raw.alt < 3000 && raw.thr > 0 && sinceStage > 1e8) want = { cam: 'ground', focus: 'main' };
    else if (sinceStage < 12) want = { cam: 'chase', focus: 'main' };
    else if (b && !b.crash && !(b.lnd && t - (this.heldAt ?? t) > 10)) {
      const held = b.cgt === 1 || b.lnd === 1;
      if (held && this.heldAt == null) this.heldAt = t;
      if (b.alt < 2500 || held) want = { cam: 'ground', focus: body.name };
      else { const ph = Math.floor((sinceStage - 12) / 16) % 3; want = ph === 1 ? { cam: 'chase', focus: body.name } : { cam: raw.alt > 100000 ? 'earth' : 'chase', focus: 'main' }; }
    } else if (sinceStage > 1e8) want = { cam: 'track', focus: 'main' };
    else want = { cam: raw.alt > 120000 ? 'earth' : 'chase', focus: 'main' };
    const same = want.cam === this.cam && want.focus === this.focus;
    if (same) return { cam: this.cam, focus: this.focus, cut: false };
    if (this.cam != null && t - this.since < HOLD && !(want.focus !== this.focus && want.cam === 'ground')) return { cam: this.cam, focus: this.focus, cut: false };
    const first = this.cam == null;
    this.cam = want.cam; this.focus = want.focus; this.since = t;
    return { cam: this.cam, focus: this.focus, cut: !first };
  }
}
