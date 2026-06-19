FROM ubuntu:24.04 AS mapf-base

WORKDIR /tmp

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update -q \
    && apt install -yq --no-install-recommends \
    zsh \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean

ENV SHELL /bin/zsh
SHELL ["/bin/zsh", "-c"]

# Install COMMON

RUN apt update -q \
    && apt install -yq --no-install-recommends \
    ca-certificates \
    curl \
    htop \
    git \
    gpg \
    neovim \
    ssh \
    software-properties-common \
    sudo \
    tar \
    tmux \
    wget \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean

RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" -x

# Install LLVM

ENV CLANG_VERSION=19
ENV CXX="clang++-${CLANG_VERSION}"
ENV CC="clang-${CLANG_VERSION}"
ENV BAZEL_CXXOPTS="-std=c++2c"

RUN echo "Build info:" \
    && echo "  CC=${CC}" \
    && echo "  CXX=${CXX}"

RUN wget https://apt.llvm.org/llvm.sh \
    && chmod +x llvm.sh \
    && sudo ./llvm.sh ${CLANG_VERSION} \
    && rm -rf /tmp/*

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
    clang-${CLANG_VERSION} \
    clang-format-${CLANG_VERSION} \
    clang-tidy-${CLANG_VERSION} \
    lldb-${CLANG_VERSION} \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN printf "export CC='${CC}'\n" >> /root/.zshrc \
    && printf "export CXX='${CXX}'\n" >> /root/.zshrc \
    && printf "export BAZEL_CXXOPTS='${BAZEL_CXXOPTS}'\n" >> /root/.zshrc \
    && ln -sf ${CC} /usr/bin/clang \
    && ln -sf ${CXX} /usr/bin/clang++ \
    && ln -sf /usr/bin/clang-format-${CLANG_VERSION} /usr/bin/clang-format

# Install Bazel

ARG TARGETARCH
RUN wget https://github.com/bazelbuild/bazelisk/releases/download/v1.23.0/bazelisk-linux-${TARGETARCH} \
    && chmod +x bazelisk-linux-${TARGETARCH} \
    && mv bazelisk-linux-${TARGETARCH} /usr/bin/bazel

RUN wget https://github.com/bazelbuild/buildtools/releases/download/v7.3.1/buildifier-linux-${TARGETARCH} \
    && chmod +x buildifier-linux-${TARGETARCH} \
    && mv buildifier-linux-${TARGETARCH} /usr/bin/buildifier

# Python + matplotlib for the experiment plots (viz/plot_*.py).

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
    python3 \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

WORKDIR /mapf
ENTRYPOINT ["/bin/zsh", "-lc"]
CMD ["trap : TERM INT; sleep infinity & wait"]
