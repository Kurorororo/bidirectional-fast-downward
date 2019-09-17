FROM kurorororo/downward-lab-experiment:latest

WORKDIR /bidirectional-fast-downward
ADD ./build.py /bidirectional-fast-downward/build.py
ADD ./build_configs.py /bidirectional-fast-downward/build_configs.py
ADD ./fast-downward.py /bidirectional-fast-downward/fast-downward.py
ADD ./src /bidirectional-fast-downward/src
ADD ./driver /bidirectional-fast-downward/driver
RUN ./build.py