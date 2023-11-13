# 环境搭建

## 系统环境

- 系统版本: Archlinux
- Rust 版本: 1.73.0
- Cargo 版本: 1.73.0
- Rustup 版本: 1.75.0-nightly

## 安装工具链

工具链简介：

- thumbv6m-none-eabi，适用于 Cortex-M0 和 Cortex-M1 处理器
- thumbv7m-none-eabi，适用于 Cortex-M3 处理器
- thumbv7em-none-eabi，适用于 Cortex-M4 和 Cortex-M 处理器
- thumbv7em-none-eabihf，适用于 Cortex-M4F 和 Cortex-M7F 处理器
- thumbv8m.main-none-eabi，适用于 Cortex-M33 和 Cortex-M35P 处理器
- thumbv8m.main-none-eabihf，适用于 Cortex-M33F 和 Cortex-M35PF 处理器

```shell
rustup target add thumbv7m-none-eabi
```

## 安装 cargo-binutils

```shell
rustup component add llvm-tools-preview

# 安装指定版本
# cargo install cargo-binutils --vers 0.3.3
# 安装最新版本
cargo install cargo-binutils

cargo size --version
```

## 安装 ARM GCC 编译环境

```shell
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib
```

## 安装 ARM GDB

```shell
sudo pacman -S arm-none-eabi-gdb
```

## 终端串口工具

```shell
sudo pacman -S minicom
```

## 图形化串口工具（可选）

```shell
# 推荐
yay -S aur/coolterm-bin

# 备用
yay -S aur/cutecom
```

## Openocd 调试器

```shell
sudo pacman -S openocd
```

## 检查 ELF 头

```shell
# Displays the section headers
cargo readobj --bin stm32f103-uav --release -- -s

# Displays the symbol table
cargo readobj --bin stm32f103-uav --release -- -t
```

## 检查二进制项的 linker section 的大小

```shell
# 查看所有的 section, 调试信息
cargo size --bin stm32f103-uav --release -- -A

# 仅查看烧录的大小
cargo size --bin stm32f103-uav --release -- -B
```
