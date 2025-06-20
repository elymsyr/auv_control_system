import os
import subprocess

def main():

    system_build = """
        cd /home/eren/GitHub/ControlSystem
        rm -rf build
        sudo ss -ltnp | grep -E ':(5560|5561|5562|5563|8889)\b'
        sudo fuser -k 5560/tcp 5561/tcp 5562/tcp 5563/tcp 8889/tcp
        mkdir -p build && cd build

        cmake -DBOOST_ROOT=$CONDA_PREFIX \
            -DCMAKE_CXX_COMPILER=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++ \
            -DCMAKE_CUDA_HOST_COMPILER=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc \
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
