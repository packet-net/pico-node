FROM debian:bookworm-slim
RUN apt-get update -qq && apt-get install -y -qq libdbus-1-3 libasound2 && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/bpq
COPY linbpq64 /opt/bpq/linbpq
COPY bpq32.cfg /opt/bpq/bpq32.cfg
RUN chmod +x /opt/bpq/linbpq
CMD ["/opt/bpq/linbpq"]
