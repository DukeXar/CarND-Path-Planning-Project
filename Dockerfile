FROM ubuntu:xenial

RUN apt-get update && \
  apt-get install -y libuv1-dev cmake make g++

RUN apt-get install -y git libssl-dev

RUN git clone https://github.com/uWebSockets/uWebSockets
RUN cd uWebSockets && git checkout e94b6e1 && mkdir build && cd build && cmake .. && make install
RUN ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so


RUN mkdir /src
WORKDIR /src

COPY . /src
RUN mkdir build && cd build && cmake .. && make
