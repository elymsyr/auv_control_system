FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# ─── Install system deps and GCC-11 ───────────────────────────────────────────
RUN apt-get update && apt-get install -y software-properties-common curl \
    && add-apt-repository ppa:ubuntu-toolchain-r/test -y \
    && apt-get update

# ─── Install Miniconda and CasADi ─────────────────────────────────────────────
RUN curl -sLo ~/miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    bash ~/miniconda.sh -b -p /opt/conda && rm ~/miniconda.sh

# Add Conda to PATH
ENV PATH="/opt/conda/bin:$PATH"

# Initialize Conda
RUN conda init bash

COPY environment.yml .
RUN conda env create -f environment.yml

# ─── Set working dir ──────────────────────────────────────────────────────────
WORKDIR /app
COPY . .

# ─── Build the CMake Project ────────────────────────────────────────────────
RUN conda run -n mp_test bash -c " \
    cd /app && \
    rm -rf build && \
    mkdir build && \
    cd build && \
    cmake -DBOOST_ROOT=\$CONDA_PREFIX \
      -DCMAKE_CXX_COMPILER=\$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++ \
      -DCMAKE_CUDA_HOST_COMPILER=\$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc \
      -DCMAKE_PREFIX_PATH=\$CONDA_PREFIX \
      -DCMAKE_BUILD_TYPE=Debug .. && \
    make -j\$(nproc) \
    "

# ─── Set default command ──────────────────────────────────────────────────────
WORKDIR /app/build
ENTRYPOINT ["./test"]
CMD []