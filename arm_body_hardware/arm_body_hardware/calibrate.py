"""Record each servo's raw position at the URDF zero pose.

Zero pose (from the CAD): the arm hanging straight down, gripper closed.
Torque is turned off so the arm can be posed by hand; nothing is moved.

    ros2 run arm_body_hardware calibrate --arm arm_b                        # all of arm B
    ros2 run arm_body_hardware calibrate --arm arm_a --joints arm_a_joint6  # only joint6
"""
import argparse
import math
import sys

from arm_body_hardware import config
from arm_body_hardware.feetech import FeetechBus, TICKS_PER_REV


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', help='arms config yaml (default: the installed arms.yaml)')
    parser.add_argument('--arm', required=True, help='arm to calibrate, e.g. arm_a or arm_b')
    parser.add_argument('--joints', nargs='+',
                        help='calibrate only these joints and keep the saved zero of the others')
    args = parser.parse_args()

    cfg = config.load_config(args.config)
    if args.arm not in cfg['arms']:
        sys.exit(f'Unknown arm {args.arm}; the config has {", ".join(cfg["arms"])}.')
    arm = cfg['arms'][args.arm]
    servos = arm['servos']
    if args.joints:
        unknown = set(args.joints) - set(servos)
        if unknown:
            sys.exit(f'{", ".join(sorted(unknown))} have no servo on {args.arm}.')
        servos = {j: servos[j] for j in args.joints}
    bus = FeetechBus(arm['port'], cfg['baud'])

    missing = [j for j, s in servos.items() if not bus.ping(s['id'])]
    if missing:
        sys.exit(f'No answer from: {", ".join(missing)}. Is {args.arm} powered?')

    input('Torque will be turned OFF on these servos. Support the arm, then press Enter... ')
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

    # Each joint's URDF range must fit in the servo's 0..4095 without wrapping. A joint whose
    # URDF range is a full turn (wrist roll) can never fit: the bridge keeps it to what it reaches.
    limits = config.urdf_limits()
    ok = True
    print(f'\n{"joint":<22}{"zero raw":>9}{"raw at lower":>14}{"raw at upper":>14}')
    for joint, s in servos.items():
        lo, hi = limits[joint]
        r_lo = config.to_raw(lo, zero_raw[joint], s['sign'])
        r_hi = config.to_raw(hi, zero_raw[joint], s['sign'])
        note = ''
        if not all(0 <= r < TICKS_PER_REV for r in (r_lo, r_hi)):
            if hi - lo < 2 * math.pi - 0.01:
                ok = False
                note = '   <-- range wraps past 0/4095'
            else:
                reach = config.servo_limits(lo, hi, zero_raw[joint], s['sign'])
                note = f'   full turn: usable {reach[0]:.2f}..{reach[1]:.2f} rad'
        print(f'{joint:<22}{zero_raw[joint]:>9}{r_lo:>14.0f}{r_hi:>14.0f}{note}')

    config.save_calibration(arm, {**config.load_calibration(arm), **zero_raw})
    print(f'\nSaved {arm["calibration_file"]}')
    if not ok:
        print('WARNING: a joint above cannot reach its full URDF range without the servo '
              'wrapping. Re-seat that servo horn nearer the middle of its travel and calibrate '
              'again.')


if __name__ == '__main__':
    main()
