FROM debian:latest
RUN apt-get update && apt-get -y --no-install-recommends install \
   python3-venv \
   python3-setuptools \
   python3-pip \
   python3-dev \
   git\
   make\
   g++\
   libeigen3-dev\
   libboost-dev\
   cmake && apt-get clean

ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
RUN pip install tqdm h5py wheel

# disable visuals as we are on docker
ENV PCL_BENCHMARK_NOVISUALIZATION=1

# The add will invalidate all subsequent steps if the latest_commit has changed. Nice!
ADD "https://api.github.com/repos/tum-bgd/pointcloudqueries/commits?per_page=1" latest_commit
RUN git clone --recurse-submodules https://github.com/xdrl1/pointcloudqueries.git
WORKDIR /pointcloudqueries
RUN python3 setup.py bdist_wheel
RUN pip install dist/*
