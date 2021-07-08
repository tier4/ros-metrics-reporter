FROM alpine:3.10

RUN apt-get update && apt-get install -y \
    lcov \
    git \
    python-is-python3 \
    python3-pip \
    hugo \
    clang-tidy-11 \
    gcc-multilib \
    python3-venv

RUN update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-11 9999
RUN pip3 install \
    jinja2 \
    bs4 \
    pandas \
    plotly

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
