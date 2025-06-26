import subprocess
import threading

bash = """
        sudo ss -ltnp | grep -E ':(5560|5561|5562|5563|8889)\\b'
        sudo fuser -k 5560/tcp 5561/tcp 5562/tcp 5563/tcp 8889/tcp
        conda activate mppi
        cd /home/eren/GitHub/ControlSystem/Control
        rm -rf build
        mkdir -p build && cd build
        cmake -DBOOST_ROOT=$CONDA_PREFIX \
              -DCMAKE_CXX_COMPILER=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++ \
              -DCMAKE_CUDA_HOST_COMPILER=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc \
              -Dcasadi_DIR=$CONDA_PREFIX/lib/cmake/casadi \
              -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
              -DCMAKE_BUILD_TYPE=Debug ..
        make -j$(nproc)
        ./control_system
"""

def stream_output(name, proc):
    for line in proc.stdout:
        print(f"[{name}] {line}", end='')

def launch_terminal(name, cmd_str, cwd=None):
    p = subprocess.Popen(cmd_str,
                         shell=True,
                         executable="/bin/bash",
                         cwd=cwd,
                         stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT,
                         bufsize=1,
                         text=True)
    thread = threading.Thread(target=stream_output, args=(name, p), daemon=True)
    thread.start()
    return p, thread

def main():
    base = "/home/eren/GitHub/ControlSystem/Simulation/Control"

    t1_cmd = """
        conda activate mppi
        cd {base}
        rm -rf build
        mkdir -p build && cd build
        cmake -DBOOST_ROOT=$CONDA_PREFIX \
              -DCMAKE_CXX_COMPILER=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++ \
              -DCMAKE_CUDA_HOST_COMPILER=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc \
              -Dcasadi_DIR=$CONDA_PREFIX/lib/cmake/casadi \
              -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
              -DCMAKE_BUILD_TYPE=Release ..
        make -j$(nproc)
        cd {base}/build
        ./control_system
    """.format(base=base)

    t2_cmd = f"""
        conda activate mppi
        python {base}/comm.py
    """

    t3_cmd = f"""
        conda activate mppi
        python /home/eren/GitHub/ControlSystem/Simulation/Assets/Scripts/bridge.py
    """

    procs = []
    for name, cmd in [("build", t1_cmd), ("comm", t2_cmd), ("bridge", t3_cmd)]:
        p, thread = launch_terminal(name, cmd, cwd=base)
        procs.append((p, thread))

    # Wait for all to finish
    for p, thread in procs:
        p.wait()
        thread.join()

if __name__ == "__main__":
    main()
