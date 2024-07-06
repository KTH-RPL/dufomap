FROM ubuntu:focal
LABEL maintainer="Qingwen Zhang <https://kin-zhang.github.io/>"

# Just in case we need it
ENV DEBIAN_FRONTEND noninteractive

# install g++10 and tbb
RUN apt update && apt install -y wget git zsh tmux vim g++ curl unzip
RUN apt update && apt install -y gcc-10 g++-10 libtbb-dev liblz4-dev liblzf-dev pkg-config

# install CMAKE
RUN apt update && apt install -y gnupg gnupg2 software-properties-common
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main' && apt update && apt install -y cmake

# since we will output pcd file, don't want to root to lock it. normally 1000 is the first user in our desktop also
RUN useradd -ms /bin/bash -u 1000 kin
USER kin

# setup oh-my-zsh 
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended
RUN printf "y\ny\ny\n\n" | bash -c "$(curl -fsSL https://raw.githubusercontent.com/Kin-Zhang/Kin-Zhang/main/scripts/setup_ohmyzsh.sh)"

RUN mkdir -p /home/kin/workspace && mkdir -p /home/kin/data
RUN git clone --recurse-submodules -b main --single-branch https://github.com/KTH-RPL/dufomap /home/kin/workspace/dufomap
WORKDIR /home/kin/workspace/dufomap

# Run container: docker run -it --rm --name dufomap -v /home/kin/data:/home/kin/data zhangkin/dufomap /bin/zsh
# you can also run with root user in existing container: docker exec -it -u 0 dufomap /bin/zsh
