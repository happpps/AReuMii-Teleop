FROM python:3.12-slim-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1 libgl1 libglib2.0-0 libx11-6 libxext6 libxrender1 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY . /app
RUN python -m pip install --no-cache-dir uv==0.12.21 \
    && uv sync --frozen --extra runtime --no-dev
ENV PATH="/app/.venv/bin:$PATH"

# CloudXR runs externally; select the host runtime/devices explicitly before live use.
ENV LEROBOT_CLOUDXR_SKIP_AUTOLAUNCH=1
ENTRYPOINT ["areumii-teleop"]
CMD ["--mode", "sim", "--print_config"]
