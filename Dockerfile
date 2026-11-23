FROM rust:1-bookworm AS build
RUN rustup target add x86_64-unknown-linux-musl
WORKDIR /src
COPY Cargo.toml Cargo.lock ./
COPY src ./src
RUN cargo build --release --locked --target x86_64-unknown-linux-musl \
 && cp target/x86_64-unknown-linux-musl/release/mechbot /mechbot

FROM scratch
COPY --from=build /mechbot /mechbot
USER 65534:65534
ENTRYPOINT ["/mechbot"]
