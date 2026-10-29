"""Record each servo's raw position at the URDF zero pose.

Zero pose (from the CAD): the arm hanging straight down, gripper closed.
Torque is turned off so the arm can be posed by hand; nothing is moved.
"""
import argparse
import sys

from arm_body_hardware import config
from arm_body_hardware.feetech import FeetechBus, TICKS_PER_REV


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', help='arm config yaml (default: the installed arm_a.yaml)')
    args = parser.parse_args()

    cfg = config.load_config(args.config)
    servos = cfg['servos']
    bus = FeetechBus(cfg['port'], cfg['baud'])

    missing = [j for j, s in servos.items() if not bus.ping(s['id'])]
    if missing:
        sys.exit(f'No answer from: {", ".join(missing)}. Is the arm powered?')

    input('Torque will be turned OFF on every servo. Support the arm, then press Enter... ')
    bus.set_torque([s['id'] for s in servos.values()], False)
    input('Pose the arm in the zero pose: hanging straight down, gripper closed.\n'
          'Hold it still and press Enter... ')

    zero_raw = {}
    for joint, s in servos.items():
        raw = bus.read_position(s['id'])
        if raw is None:
            sys.exit(f'{joint} (id {s["id"]}) did not answer; nothing saved.')
        zero_raw[joint] = raw
    bus.close()

    # Each joint's URDF range must fit in the servo's 0..4095 without wrapping
    limits = config.urdf_limits()
    ok = True
    print(f'\n{"joint":<22}{"zero raw":>9}{"raw at lower":>14}{"raw at upper":>14}')
    for joint, s in servos.items():
        lo, hi = limits[joint]
        r_lo = config.to_raw(lo, zero_raw[joint], s['sign'])
        r_hi = config.to_raw(hi, zero_raw[joint], s['sign'])
        fits = all(0 <= r < TICKS_PER_REV for r in (r_lo, r_hi))
        ok &= fits
        print(f'{joint:<22}{zero_raw[joint]:>9}{r_lo:>14.0f}{r_hi:>14.0f}'
              f'{"" if fits else "   <-- range wraps past 0/4095"}')

    config.save_calibration(cfg, zero_raw)
    print(f'\nSaved {cfg["calibration_file"]}')
    if not ok:
        print('WARNING: a joint above cannot reach its full URDF range without the servo '
              'wrapping. Re-seat that servo horn nearer the middle of its travel and calibrate '
              'again.')


if __name__ == '__main__':
    main()
