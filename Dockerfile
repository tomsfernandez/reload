FROM ubuntu:18.04

RUN apt-get update && apt-get install -y build-essential \
                    cmake \ 
                    git \
                    libgtk2.0-dev \
                     pkg-config \ 
                     libavcodec-dev \ 
                     libavformat-dev \ 
                     libswscale-dev \
                     python-dev \ 
                     python-numpy \ 
                     libtbb2 \ 
                     libtbb-dev \ 
                     libjpeg-dev \ 
                     libpng-dev \ 
                     libtiff-dev \ 
                     libdc1394-22-dev \
                     wget
RUN cd ~ && mkdir libs && cd libs && \ 
            wget https://github.com/opencv/opencv/archive/3.2.0.tar.gz && \
            tar -xvzf 3.2.0.tar.gz && \ 
            rm 3.2.0.tar.gz && \ 
            cd opencv-3.2.0 && \ 
            mkdir build && \ 
            cd build && \ 
            cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. && \ 
            make install
RUN cd ~/libs && wget https://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz \ 
                && tar -xvzf 3.3.4.tar.gz \ 
                && rm 3.3.4.tar.gz \ 
                && mv eigen-eigen-5a0156e40feb eigen-3.3.4
RUN cd ~/libs && apt-get install libglew-dev
RUN cd ~/libs && git clone https://github.com/stevenlovegrove/Pangolin.git && \ 
            cd Pangolin && \ 
            mkdir build && \ 
            cd build && \ 
            cmake .. && \ 
            cmake --build .
RUN cd ~/libs \ cd ~/libs && wget https://dl.bintray.com/boostorg/release/1.67.0/source/boost_1_67_0.tar.gz \ 
                && tar -xvzf boost_1_67_0.tar.gz \ 
                && rm boost_1_67_0.tar.gz \ 
                && mv boost_1_67_0 boost-1.67.0 \ 
                && cd boost-1.67.0 \ 
                && mkdir build \ 
                && ./bootstrap.sh --prefix=build && ./b2 install
RUN cd ~ && git clone https://github.com/tomasfernandezm/OS1RelocationSystem.git && \
                mv OS1RelocationSystem os1 && \
                cd os1 && \ 
                cd Thirdparty && \
                cd DBoW2 && \
                mkdir build && \
                cd build && \
                cmake .. -DCMAKE_BUILD_TYPE=Release && \
                make -j3 && \
                cd ../../g2o && \
                mkdir build && \
                cd build && \
                cmake .. -DCMAKE_BUILD_TYPE=Release && \
                apt-get install libeigen3-dev && \
                make -j3 && \
                cd ../../../ && \
                mkdir build && \
                cd build && \
                cmake .. && \
                make -j2