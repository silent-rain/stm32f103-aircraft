# Defmt 单元测试

## 安装依赖

- flip-link:

```shell
cargo install flip-link
```

- probe-run:

```shell
# make sure to install v0.2.0 or later
cargo install probe-run
```

- cargo-generate（可选）:

```shell
cargo install cargo-generate
```

## 运行测试

- 运行这些单元测试

```shell
cargo test --lib

cargo test --target thumbv7m-none-eabi --lib

cargo test --target thumbv7m-none-eabi --lib probe-run -- --chip STM32F103C8
```

## 设备调试

```shell
# 完整指令
cargo run --target thumbv7m-none-eabi probe-run -- --chip STM32F103C8 trace

# 简写, 配置的有 `.cargo/config.toml` 文件
cargo run


# 日志级别
DEFMT_LOG=trace cargo run

```

## 设置日志级别

```shell
export DEFMT_LOG=info

# or
DEFMT_LOG=trace cargo run
```

## 相关文档

- [defmt app-template](https://github.com/knurling-rs/app-template)
- [defmt](https://github.com/knurling-rs/defmt)
