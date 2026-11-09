import subprocess
import time
import re
import sys
import os
import signal


def kill_all():
    subprocess.run(['pkill', '-9', '-x', 'gzserver'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-x', 'gzclient'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'tactical_recon'], stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def run_trial(cooperative, timeout_sec=140):
    kill_all()
    cmd = [
        'bash', '-c',
        f'source /opt/ros/humble/setup.bash && '
        f'source /home/fury/College/Robotics/uav_ugv_ws/install/setup.bash && '
        f'export GAZEBO_MODEL_DATABASE_URI="" && '
        f'ros2 launch tactical_recon sim_mission.launch.py '
        f'cooperative_mode:={"true" if cooperative else "false"} gui:=false'
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=os.setsid
    )

    report_pattern = re.compile(
        r'MISSION REPORT:\s+elapsed_s=([\d\.]+)\s+distance_m=([\d\.]+)\s+'
        r'mine_time_s=([\d\.]+)\s+mud_time_s=([\d\.]+)\s+road_time_s=([\d\.]+)'
    )

    result = None
    start_time = time.time()

    try:
        while True:
            if time.time() - start_time > timeout_sec:
                print(f"TIMEOUT after {timeout_sec}s")
                break
            line = proc.stdout.readline()
            if not line:
                if proc.poll() is not None:
                    break
                time.sleep(0.1)
                continue

            match = report_pattern.search(line)
            if match:
                result = {
                    'elapsed_s': float(match.group(1)),
                    'distance_m': float(match.group(2)),
                    'mine_time_s': float(match.group(3)),
                    'mud_time_s': float(match.group(4)),
                    'road_time_s': float(match.group(5)),
                }
                print(f"Report captured: {result}", flush=True)
                break
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        kill_all()

    return result


def main():
    modes = [('Cooperative', True), ('Baseline', False)]
    all_results = {}

    for mode_name, coop in modes:
        print(f"\n================ Running 3 trials for {mode_name} ================", flush=True)
        all_results[mode_name] = []
        for trial in range(1, 4):
            print(f"Starting {mode_name} Trial {trial}...", flush=True)
            res = run_trial(coop)
            if res is not None:
                all_results[mode_name].append(res)
                print(f"{mode_name} Trial {trial} SUCCESS: {res}", flush=True)
            else:
                print(f"{mode_name} Trial {trial} FAILED", flush=True)

    print("\n\n================ FINAL PHASE 6 BENCHMARK TABLE ================\n")
    print("| Mode | Run | Elapsed (s) | Distance (m) | Mine Time (s) | Mud Time (s) | Road Time (s) |")
    print("|---|---|---|---|---|---|---|")
    for mode_name in ['Cooperative', 'Baseline']:
        for i, res in enumerate(all_results.get(mode_name, []), 1):
            print(f"| {mode_name} | {i} | {res['elapsed_s']:.2f} | {res['distance_m']:.2f} | "
                  f"{res['mine_time_s']:.2f} | {res['mud_time_s']:.2f} | {res['road_time_s']:.2f} |")


if __name__ == '__main__':
    main()

