FROM kurorororo/downward-lab-experiment:latest

RUN apt install -y automake

RUN git clone https://github.com/ivmai/cudd /cudd
WORKDIR /cudd
RUN aclocal && automake --add-missing && autoconf
RUN ./configure --enable-obj --build=i686-pc-linux-gnu "CFLAGS=-m64 -Wall -Wextra -g -O3" "CXXFLAGS=-m64 -Wall -Wextra -std=c++0x -g -O3" "LDFLAGS=-m64"
RUN make -j4 check && make install

WORKDIR /bidirectional-fast-downward
ADD ./build.py /bidirectional-fast-downward/build.py
ADD ./build_configs.py /bidirectional-fast-downward/build_configs.py
ADD ./fast-downward.py /bidirectional-fast-downward/fast-downward.py
ADD ./src /bidirectional-fast-downward/src
ADD ./driver /bidirectional-fast-downward/driver
RUN ./build.py