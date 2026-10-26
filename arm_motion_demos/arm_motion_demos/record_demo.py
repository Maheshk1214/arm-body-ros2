"""
Record the Gazebo demo camera (worlds/demo_recording.sdf) to an MP4 file.

    ros2 run arm_motion_demos record_demo demo.mp4 [--duration 120]

Reads the camera's Gazebo topic directly (same GZ_PARTITION / GZ_IP as the
simulation) and pipes the frames to ffmpeg. The camera publishes 30 frames
per second of sim time, so the video plays at the true tempo even when
Gazebo runs slower than real time. Stops after --duration seconds of video,
or on Ctrl-C, and finishes the file cleanly either way.
"""
import argparse
import queue
import subprocess
import sys
import time

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

FPS = 30


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('output', help='MP4 file to write')
    parser.add_argument('--topic', default='/demo_camera/image')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='seconds of video (sim time) to record; 0 = until Ctrl-C')
    args = parser.parse_args()

    frames = queue.Queue(maxsize=300)
    node = Node()
    if not node.subscribe(Image, args.topic, frames.put):
        sys.exit(f'Cannot subscribe to {args.topic}')
    print(f'Waiting for {args.topic} (is Gazebo running demo_recording.sdf?)', flush=True)
    try:
        first = frames.get(timeout=60.0)
    except queue.Empty:
        sys.exit(f'No frames on {args.topic} within 60 s')
    ffmpeg = subprocess.Popen(
        ['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
         '-s', f'{first.width}x{first.height}', '-r', str(FPS), '-i', '-',
         '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', args.output],
        stdin=subprocess.PIPE)
    count, limit = 0, int(args.duration * FPS) if args.duration > 0 else None
    print(f'Recording {first.width}x{first.height} to {args.output}; Ctrl-C to stop',
          flush=True)
    msg, last_report = first, time.monotonic()
    try:
        while limit is None or count < limit:
            ffmpeg.stdin.write(msg.data)
            count += 1
            if time.monotonic() - last_report > 10.0:
                print(f'  {count / FPS:.0f} s recorded', flush=True)
                last_report = time.monotonic()
            msg = frames.get(timeout=30.0)
    except KeyboardInterrupt:
        pass
    except queue.Empty:
        print('Camera stopped publishing', flush=True)
    finally:
        ffmpeg.stdin.close()
        ffmpeg.wait()
    print(f'Wrote {count} frames ({count / FPS:.1f} s) to {args.output}', flush=True)


if __name__ == '__main__':
    main()
