"""Plan a strip as separate W x H patches with the lift (boom) held still.

For each patch the boom (yaw, pitch, extension) is fixed and only the arm
(swivel + six arm joints) moves the nozzle in a horizontal raster with the
same 0.1 m row spacing and 0.12 m/s TCP speed as plan_strip.py. Between
patches the nozzle is considered off and all joints move in joint space to
the next patch.

Offline kinematics and box collision screening only (same models as
plan_strip.py). Run from the project folder:

    PYTHONPATH=src/lift_window_sim python3 -m lift_window_sim.plan_patches src/lift_window_sim
"""
from pathlib import Path
import argparse, csv, json, pickle, time
from multiprocessing import Pool
import numpy as np
from scipy.optimize import least_squares
from .model import NAMES, LOWER, UPPER, VMAX, environment_clearance
from .strip_checks import self_screen, boxes, sat_gap
from .plan_strip import fast_fk, target, valid, smooth_distance, write_csv, BASE_X, MOUNT, STAGES

ROW = 0.1          # raster row spacing (m), as plan_strip.py
SPEED = 0.12       # TCP speed along a row (m/s), as plan_strip.py
VFRAC = 0.85       # use at most 85 % of each joint velocity limit
ARM = np.arange(3, 9)   # swivel + arm joints 1-5 (arm_joint_6 locked, as plan_strip)
REG = 3e-3             # pull toward the previous solution (resolves the redundant DOF)
JUMP = 0.3              # max joint change (rad or m) between neighbouring IK points
# Candidate arm-mount placements relative to the patch centre: (dx, y, dz).
# y is the absolute world y of the arm mount (wall front y=1.97).
OFFSETS = [(0, .85, -.4), (0, .7, -.4), (0, .9, -.4), (0, .75, -.4),
           (0, .7, -.2), (0, .85, -.2), (-.25, .9, -.4), (-.25, .75, -.2),
           (-.2, .85, -.4), (.2, .85, -.4), (0, .9, -.6), (-.2, .7, -.4), (.2, .7, -.4)]


def solve_arm(p, r, seed):
    """IK with the boom locked at seed[:3]; same residual terms as plan_strip.solve."""
    def unpack(x):
        q = seed.copy(); q[ARM] = x; return q
    def res(x):
        q = unpack(x); t = fast_fk(q); bs = boxes(q)
        gaps = [sat_gap(bs[a], bs[b]) for a, b in [('arm3', 'arm5'), ('arm4', 'arm6'),
                ('arm_mount', 'arm2'), ('swivel_mount', 'arm2'), ('stage6', 'arm2')]]
        return np.r_[t[:3, 3]-p, .8*(t[:3, 0]-r[:, 0]), REG*(x-seed[ARM]),
                     *[4*max(0., .010-g) for g in gaps]]
    out = least_squares(res, np.clip(seed[ARM], LOWER[ARM]+1e-7, UPPER[ARM]-1e-7),
                        bounds=(LOWER[ARM]+1e-8, UPPER[ARM]-1e-8), max_nfev=200,
                        ftol=1e-10, xtol=1e-10, gtol=1e-10)
    q = unpack(out.x); t = fast_fk(q)
    pe = float(np.linalg.norm(t[:3, 3]-p))
    ae = float(np.arctan2(np.linalg.norm(np.cross(r[:, 0], t[:3, 0])), np.dot(r[:, 0], t[:3, 0])))
    return q, pe, ae


def boom_for(mount):
    v = mount-np.array([BASE_X, 0, 1.5]); L = np.linalg.norm(v)
    return np.array([np.arctan2(v[1], v[0]), -np.arcsin(v[2]/L), (L-MOUNT)/STAGES])


def rows_for(z0, H, last):
    """Row heights for one patch; the shared edge row belongs to the lower patch."""
    n = int(round(H/ROW))
    zs = [z0+k*ROW for k in range(n)]
    if last:
        zs.append(z0+H)
    return [round(z, 6) for z in zs]


class ArmPlan:
    """Like plan_strip.Plan, but only the arm moves (boom fixed)."""
    def __init__(self, q, x, z, t0=0.):
        self.q = q.copy(); self.t = t0; self.state = np.array([x, z, 0., 0.]); self.rows = []
        self.append(q, target(x, z)[0], self.state)

    def append(self, q, p, state):
        self.rows.append([self.t, *q, *p, *state])

    def move(self, end):
        end = np.asarray(end, float); start = self.state.copy()
        length = np.linalg.norm(end[:2]-start[:2])
        duration = max(length/SPEED+min(3, length/SPEED/2), 2.)
        steps = max(4, int(np.ceil(duration/.25))); dt = duration/steps
        prev_state = start
        for k in range(1, steps+1):
            u = k/steps
            s = 10*u**3-15*u**4+6*u**5 if length < .5 else smooth_distance(u*duration, length)/length
            state = start+(end-start)*s
            self.step(prev_state, state, dt)
            prev_state = state
        self.state = end

    def step(self, a, b, dt, depth=0):
        """IK at state b from the current q; bisect the step if a joint jumps.
        Near a wrist singularity joints legitimately move fast; small steps keep
        the solution on one branch and retime() later slows the motion down."""
        p, r = target(*b)
        q, pe, ae = solve_arm(p, r, self.q)
        if not valid(q, pe, ae):
            raise RuntimeError(f'arm-only IK failed at {b[:2]}: pe={pe:.4f} ae={ae:.4f}')
        if np.max(abs(q-self.q)) > JUMP/(2**depth):
            if depth >= 6:
                raise RuntimeError(f'IK jump at {b[:2]}')
            mid = (a+b)/2
            self.step(a, mid, dt/2, depth+1)
            self.step(mid, b, dt/2, depth+1)
            return
        self.t += dt; self.append(q, p, b); self.q = q


def coarse_ok(q, x0, z0, W, H, n=5):
    for i, z in enumerate(np.linspace(z0, z0+H, n)):
        xs = np.linspace(x0, x0+W, n)
        for x in (xs if i % 2 == 0 else xs[::-1]):
            p, r = target(x, z); qa, pe, ae = solve_arm(p, r, q)
            if not valid(qa, pe, ae):
                return False
            q = qa
    return True


def plan_patch(args):
    """Return (rows, info) for one patch, or raise if no boom placement works."""
    idx, x0, z0, W, H, last, reverse, q_ref = args
    q_ref = np.asarray(q_ref)
    zs = rows_for(z0, H, last)
    xa, xb = (x0+W, x0) if reverse else (x0, x0+W)
    tried = []
    for (dx, y, dz) in OFFSETS:
        for seed_id in range(4):
            q = q_ref.copy()
            q[:3] = boom_for(np.array([x0+W/2+dx, y, z0+H/2+dz]))
            if np.any(q[:3] <= LOWER[:3]) or np.any(q[:3] >= UPPER[:3]):
                break
            if seed_id:
                q[3:9] += np.random.default_rng(100*idx+seed_id).normal(0, .6, 6)
            p, r = target(x0+W/2, z0+H/2); qc, pe, ae = solve_arm(p, r, q)
            if not valid(qc, pe, ae) or not coarse_ok(qc, x0, z0, W, H):
                tried.append((dx, y, dz, seed_id, 'coarse')); continue
            try:
                # Walk continuously from the patch centre to the start corner
                # so the raster begins on the same IK branch as the centre.
                lead = ArmPlan(qc, x0+W/2, z0+H/2)
                lead.move([xa, zs[0], 0, 0])
                plan = ArmPlan(lead.q, xa, zs[0])
                ends = (xb, xa)
                for j, z in enumerate(zs):
                    plan.move([ends[j % 2], z, 0, 0])
                    if j < len(zs)-1:
                        plan.move([ends[j % 2], zs[j+1], 0, 0])
            except RuntimeError as exc:
                tried.append((dx, y, dz, seed_id, str(exc)[:60])); continue
            rows = np.array(plan.rows)
            rows[:, 1:4] = rows[0, 1:4]          # boom bit-identical within the patch
            try:
                rows = refine(retime(rows))
            except RuntimeError as exc:
                tried.append((dx, y, dz, seed_id, str(exc)[:60])); continue
            return rows, {'patch': idx, 'x_m': [x0, x0+W], 'z_m': [z0, z0+H], 'rows_z_m': zs,
                          'boom_yaw_pitch_ext': rows[0, 1:4].tolist(),
                          'mount_offset_dx_y_dz': [dx, y, dz], 'arm_seed': seed_id}
    raise RuntimeError(f'Patch {idx} x={x0} z={z0}: no boom placement found; tried {tried}')


def plan_patch_or_split(args):
    """Plan one patch; if the arm cannot cover it, split it into two halves in x."""
    idx, x0, z0, W, H, last, reverse, q_ref = args
    try:
        rows, info = plan_patch(args); info['split'] = False
        return [(rows, info)]
    except RuntimeError as exc:
        print(f'  patch {idx+1}: {exc}; splitting into two {W/2:g} x {H:g} m patches', flush=True)
    halves = [x0, x0+W/2] if not reverse else [x0+W/2, x0]
    out = []
    for h in halves:
        rows, info = plan_patch((idx, h, z0, W/2, H, last, reverse, q_ref))
        info['split'] = True; out.append((rows, info))
    return out


def joint_move(qa, qb, n_check=40):
    """Joint-space transfer (nozzle off) with quintic profile; collision screened."""
    dq = np.abs(qb-qa)
    duration = max(3., 1.875*float(np.max(dq/(VFRAC*VMAX))))   # quintic peak speed = 1.875*avg
    us = np.linspace(0, 1, n_check+1)[1:]
    out = []
    for u in us:
        s = 10*u**3-15*u**4+6*u**5; q = qa+(qb-qa)*s
        if environment_clearance(q) < .025 or self_screen(q, margin=.001):
            raise RuntimeError(f'transfer collision at u={u:.2f}')
        out.append((u*duration, q))
    return out, duration


def transfer_rows(t0, qa, qb, state):
    """Rows for a transfer; returns rows. Tries a via-pose (arm folded back) on collision."""
    try:
        pts, _ = joint_move(qa, qb)
        return [[t0+dt, *q, *fast_fk(q)[:3, 3], *state] for dt, q in pts]
    except RuntimeError:
        pass
    # Via pose: target boom with the arm configuration of the departure pose.
    via = qb.copy(); via[3:] = qa[3:]
    pts1, d1 = joint_move(qa, via); pts2, _ = joint_move(via, qb)
    return ([[t0+dt, *q, *fast_fk(q)[:3, 3], *state] for dt, q in pts1] +
            [[t0+d1+dt, *q, *fast_fk(q)[:3, 3], *state] for dt, q in pts2])


def retime(a):
    """Stretch each segment so no joint exceeds VFRAC*VMAX; never speeds up."""
    t = a[:, 0].copy(); dt = np.diff(t)
    need = np.max(np.abs(np.diff(a[:, 1:11], axis=0))/(VFRAC*VMAX), axis=1)
    dt = np.maximum(dt, need)
    a = a.copy(); a[:, 0] = np.r_[0, np.cumsum(dt)]
    return a


def refine(a, max_mid_error=.0025, passes=8):
    """Insert arm-only IK midpoints within one patch where linear joint
    interpolation strays from the TCP line (boom unchanged), then retime."""
    for _ in range(passes):
        out = [a[0]]; inserted = 0
        for left, right in zip(a[:-1], a[1:]):
            mid = (left+right)/2
            p, r = target(*mid[14:18]); t = fast_fk(mid[1:11])
            if np.linalg.norm(t[:3, 3]-p) > max_mid_error:
                q, pe, ae = solve_arm(p, r, mid[1:11])
                if not valid(q, pe, ae):
                    raise RuntimeError(f'cannot refine at t={mid[0]}')
                mid[1:11] = q; out.append(mid); inserted += 1
            out.append(right)
        a = np.array(out)
        if not inserted:
            return retime(a)
    raise RuntimeError('refinement did not converge')


def validate(a, wash, patch_id):
    """Sample linear joint interpolation every <=0.1 s, as plan_strip.validate_path."""
    maxpe = maxae = 0.; clear = 99.; n = 0; boom_move = 0.
    for i in range(len(a)-1):
        both = wash[i] and wash[i+1] and patch_id[i] == patch_id[i+1]
        if both:
            boom_move = max(boom_move, float(np.max(np.abs(a[i+1, 1:4]-a[i, 1:4]))))
        count = max(1, int(np.ceil((a[i+1, 0]-a[i, 0])/.1)))
        for f in np.linspace(0, 1, count+1):
            row = a[i]+f*(a[i+1]-a[i]); q = row[1:11]
            cl = environment_clearance(q); clear = min(clear, cl)
            if cl < .025 or self_screen(q, margin=.001):
                raise RuntimeError(f'interpolated collision at t={row[0]:.2f}')
            if both:
                p, r = target(*row[14:18]); t = fast_fk(q)
                maxpe = max(maxpe, np.linalg.norm(t[:3, 3]-p))
                maxae = max(maxae, np.arctan2(np.linalg.norm(np.cross(r[:, 0], t[:3, 0])), np.dot(r[:, 0], t[:3, 0])))
            n += 1
    v = np.max(np.abs(np.diff(a[:, 1:11], axis=0))/np.diff(a[:, 0])[:, None]/VMAX)
    if maxpe > .003 or maxae > .01:
        raise RuntimeError(f'wash interpolation error {maxpe:.4f} m, {np.rad2deg(maxae):.3f} deg')
    return dict(interpolation_samples=n, max_wash_tcp_error_m=float(maxpe),
                max_wash_orientation_error_deg=float(np.rad2deg(maxae)),
                min_environment_clearance_m=float(clear),
                max_boom_motion_while_washing=float(boom_move),
                max_joint_speed_fraction_of_vmax=float(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('package', type=Path)
    ap.add_argument('--width', type=float, default=1.0)
    ap.add_argument('--height', type=float, default=0.5)
    ap.add_argument('--z0', type=float, default=9.0, help='lower edge of the 1 m strip')
    ap.add_argument('--output', default='patches.csv')
    ap.add_argument('--jobs', type=int, default=2)
    args = ap.parse_args(); root = args.package; W, H = args.width, args.height
    t_start = time.time()
    with (root/'trajectories/demo.csv').open() as f:
        first = next(csv.DictReader(f))
    q_home = np.array([float(first[n]) for n in NAMES])   # same start pose as demo.csv
    nx, nz = int(round(10/W)), int(round(1/H))
    jobs = []; idx = 0
    for k in range(nz):                                   # serpentine over patches
        cols = range(nx) if k % 2 == 0 else reversed(range(nx))
        for c in cols:
            jobs.append((idx, c*W, args.z0+k*H, W, H, k == nz-1, False, q_home.tolist())); idx += 1
    cache = root/'trajectories'/(Path(args.output).stem+'_cache.pkl')
    if cache.exists():
        results = pickle.loads(cache.read_bytes())
        print(f'Using cached patch plans from {cache} (delete it to replan)', flush=True)
    else:
        print(f'Planning {len(jobs)} patches of {W} x {H} m with {args.jobs} processes', flush=True)
        with Pool(args.jobs) as pool:
            results = []
            for res in pool.imap(plan_patch_or_split, jobs):
                results.extend(res); print('patch', res[0][1]['patch']+1, '/', len(jobs), 'ok',
                                           '(split)' if len(res) > 1 else '',
                                           f'{time.time()-t_start:.0f}s', flush=True)
        cache.write_bytes(pickle.dumps(results))
    for k, (_, info) in enumerate(results):
        info['patch'] = k                                  # renumber after any splits
    # Stitch: home -> patch 0 -> transfer -> patch 1 ...
    allrows = []; wash = []; pid = []
    home_state = [0., args.z0, 0., 0.]
    allrows.append([0., *q_home, *fast_fk(q_home)[:3, 3], *home_state]); wash.append(False); pid.append(-1)
    t = 0.
    for rows, info in results:
        tr = transfer_rows(t, np.array(allrows[-1][1:11]), rows[0, 1:11], allrows[-1][14:18])
        for r_ in tr[:-1]:
            allrows.append(r_); wash.append(False); pid.append(-1)
        t = tr[-1][0]
        for r_ in rows:
            r_ = r_.copy(); r_[0] += t; allrows.append(r_.tolist()); wash.append(True); pid.append(info['patch'])
        t = allrows[-1][0]
    # Patches are refined and retimed individually; retime() only stretches.
    a2 = retime(np.array(allrows)); wash2 = np.array(wash); pid2 = np.array(pid)
    print('Validating interpolated path', flush=True)
    report = validate(a2, wash2, pid2)
    out = root/'trajectories'/args.output
    write_csv(out, a2)
    wash_time = float(sum(a2[i+1, 0]-a2[i, 0] for i in range(len(a2)-1)
                          if wash2[i] and wash2[i+1] and pid2[i] == pid2[i+1]))
    report.update({'kind': 'offline kinematic screening of patch plan; not a Gazebo result',
                   'patch_size_m': [W, H], 'strip_z_m': [args.z0, args.z0+1], 'patches': len(results),
                   'row_spacing_m': ROW, 'tcp_speed_m_s': SPEED,
                   'duration_s': float(a2[-1, 0]), 'washing_time_s': wash_time,
                   'transfer_time_s': float(a2[-1, 0])-wash_time,
                   'points': len(a2), 'trajectory_csv': str(out.name),
                   'patch_details': [info for _, info in results],
                   'planning_time_s': round(time.time()-t_start)})
    (root/'config'/(Path(args.output).stem+'_report.json')).write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'patch_details'}, indent=2))


if __name__ == '__main__':
    main()
