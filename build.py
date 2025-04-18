import os
import subprocess

def main():

    system_build = """
        cd /home/eren/GitHub/ControlSystem
        rm -rf build
        mkdir -p build && cd build

        cmake -DBOOST_ROOT=$CONDA_PREFIX \
            -DCMAKE_CXX_COMPILER=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-c++ \
            -Dcasadi_DIR=$CONDA_PREFIX/lib/cmake/casadi \
            -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
            -DCMAKE_BUILD_TYPE=Release ..

        make -j4
        ./control_system
    """

    print("Running in:", os.getcwd())

    print("Checking Python version...")
    subprocess.run(system_build, shell=True, check=True)

if __name__ == "__main__":
    main()