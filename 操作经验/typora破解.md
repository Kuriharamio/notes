```bat
mio@Kurihara-Mio:~$ git clone https://github.com/hazukieq/Yporaject.git
找不到命令 “git”，但可以通过以下软件包安装它：
sudo apt install git
mio@Kurihara-Mio:~$ sudo apt install git
[sudo] mio 的密码： 
正在读取软件包列表... 完成
正在分析软件包的依赖关系树... 完成
正在读取状态信息... 完成                 
将会同时安装下列软件：
  git-man liberror-perl
建议安装：
  git-daemon-run | git-daemon-sysvinit git-doc git-email git-gui gitk gitweb
  git-cvs git-mediawiki git-svn
下列【新】软件包将被安装：
  git git-man liberror-perl
升级了 0 个软件包，新安装了 3 个软件包，要卸载 0 个软件包，有 2 个软件包未被升级。
需要下载 4,147 kB 的归档。
解压缩后会消耗 21.0 MB 的额外空间。
您希望继续执行吗？ [Y/n] y
获取:1 https://mirrors.cloud.tencent.com/ubuntu jammy/main amd64 liberror-perl all 0.17029-1 [26.5 kB]
获取:2 https://mirrors.cloud.tencent.com/ubuntu jammy-updates/main amd64 git-man all 1:2.34.1-1ubuntu1.10 [954 kB]
获取:3 https://mirrors.cloud.tencent.com/ubuntu jammy-updates/main amd64 git amd64 1:2.34.1-1ubuntu1.10 [3,166 kB]
已下载 4,147 kB，耗时 1秒 (4,059 kB/s)
正在选中未选择的软件包 liberror-perl。
(正在读取数据库 ... 系统当前共安装有 226073 个文件和目录。)
准备解压 .../liberror-perl_0.17029-1_all.deb  ...
正在解压 liberror-perl (0.17029-1) ...
正在选中未选择的软件包 git-man。
准备解压 .../git-man_1%3a2.34.1-1ubuntu1.10_all.deb  ...
正在解压 git-man (1:2.34.1-1ubuntu1.10) ...
正在选中未选择的软件包 git。
准备解压 .../git_1%3a2.34.1-1ubuntu1.10_amd64.deb  ...
正在解压 git (1:2.34.1-1ubuntu1.10) ...
正在设置 liberror-perl (0.17029-1) ...
正在设置 git-man (1:2.34.1-1ubuntu1.10) ...
正在设置 git (1:2.34.1-1ubuntu1.10) ...
正在处理用于 man-db (2.10.2-1) 的触发器 ...
mio@Kurihara-Mio:~$ git clone https://github.com/hazukieq/Yporaject.git
正克隆到 'Yporaject'...
remote: Enumerating objects: 224, done.
remote: Counting objects: 100% (224/224), done.
remote: Compressing objects: 100% (173/173), done.
remote: Total 224 (delta 78), reused 128 (delta 27), pack-reused 0
接收对象中: 100% (224/224), 2.54 MiB | 2.49 MiB/s, 完成.
处理 delta 中: 100% (78/78), 完成.
mio@Kurihara-Mio:~$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
找不到命令 “curl”，但可以通过以下软件包安装它：
sudo snap install curl  # version 8.1.2, or
sudo apt  install curl  # version 7.81.0-1ubuntu1.14
输入 “snap info curl” 以查看更多版本。
mio@Kurihara-Mio:~$ sudo apt install curl  # version 8.1.2
正在读取软件包列表... 完成
正在分析软件包的依赖关系树... 完成
正在读取状态信息... 完成                 
下列【新】软件包将被安装：
  curl
升级了 0 个软件包，新安装了 1 个软件包，要卸载 0 个软件包，有 2 个软件包未被升级。
需要下载 194 kB 的归档。
解压缩后会消耗 454 kB 的额外空间。
获取:1 https://mirrors.cloud.tencent.com/ubuntu jammy-updates/main amd64 curl amd64 7.81.0-1ubuntu1.14 [194 kB]
已下载 194 kB，耗时 0秒 (847 kB/s)
正在选中未选择的软件包 curl。
(正在读取数据库 ... 系统当前共安装有 227058 个文件和目录。)
准备解压 .../curl_7.81.0-1ubuntu1.14_amd64.deb  ...
正在解压 curl (7.81.0-1ubuntu1.14) ...
正在设置 curl (7.81.0-1ubuntu1.14) ...
正在处理用于 man-db (2.10.2-1) 的触发器 ...
mio@Kurihara-Mio:~$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
info: downloading installer

Welcome to Rust!

This will download and install the official compiler for the Rust
programming language, and its package manager, Cargo.

Rustup metadata and toolchains will be installed into the Rustup
home directory, located at:

  /home/mio/.rustup

This can be modified with the RUSTUP_HOME environment variable.

The Cargo home directory is located at:

  /home/mio/.cargo

This can be modified with the CARGO_HOME environment variable.

The cargo, rustc, rustup and other commands will be added to
Cargo's bin directory, located at:

  /home/mio/.cargo/bin

This path will then be added to your PATH environment variable by
modifying the profile files located at:

  /home/mio/.profile
  /home/mio/.bashrc

You can uninstall at any time with rustup self uninstall and
these changes will be reverted.

Current installation options:


   default host triple: x86_64-unknown-linux-gnu
     default toolchain: stable (default)
               profile: default
  modify PATH variable: yes

1) Proceed with installation (default)
2) Customize installation
3) Cancel installation
>1

info: profile set to 'default'
info: default host triple is x86_64-unknown-linux-gnu
info: syncing channel updates for 'stable-x86_64-unknown-linux-gnu'
info: latest update on 2023-10-05, rust version 1.73.0 (cc66ad468 2023-10-03)
info: downloading component 'cargo'
  7.8 MiB /   7.8 MiB (100 %) 320.5 KiB/s in 24s ETA:  0s
info: downloading component 'clippy'
  2.5 MiB /   2.5 MiB (100 %) 383.7 KiB/s in  7s ETA:  0s
info: downloading component 'rust-docs'
 13.8 MiB /  13.8 MiB (100 %) 326.4 KiB/s in 35s ETA:  0s
info: downloading component 'rust-std'
 24.7 MiB /  24.7 MiB (100 %) 237.3 KiB/s in  1m 46s ETA:  0s
info: downloading component 'rustc'
 61.6 MiB /  61.6 MiB (100 %) 400.5 KiB/s in  3m 39s ETA:  0s    
info: downloading component 'rustfmt'
  2.4 MiB /   2.4 MiB (100 %) 272.8 KiB/s in  8s ETA:  0s
info: installing component 'cargo'
info: installing component 'clippy'
info: installing component 'rust-docs'
info: installing component 'rust-std'
 24.7 MiB /  24.7 MiB (100 %)  23.5 MiB/s in  1s ETA:  0s
info: installing component 'rustc'
 61.6 MiB /  61.6 MiB (100 %)  26.6 MiB/s in  2s ETA:  0s
info: installing component 'rustfmt'
info: default toolchain set to 'stable-x86_64-unknown-linux-gnu'

  stable-x86_64-unknown-linux-gnu installed - rustc 1.73.0 (cc66ad468 2023-10-03)


Rust is installed now. Great!

To get started you may need to restart your current shell.
This would reload your PATH environment variable to include
Cargo's bin directory ($HOME/.cargo/bin).

To configure your current shell, run:
source "$HOME/.cargo/env"
mio@Kurihara-Mio:~$ cargo -v
找不到命令 “cargo”，但可以通过以下软件包安装它：
sudo snap install rustup  # version 1.24.3, or
sudo apt  install cargo   # version 0.67.1+ds0ubuntu0.libgit2-0ubuntu0.22.04.2
输入 “snap info rustup” 以查看更多版本。
mio@Kurihara-Mio:~$ cargo -v
找不到命令 “cargo”，但可以通过以下软件包安装它：
sudo snap install rustup  # version 1.24.3, or
sudo apt  install cargo   # version 0.67.1+ds0ubuntu0.libgit2-0ubuntu0.22.04.2
输入 “snap info rustup” 以查看更多版本。
mio@Kurihara-Mio:~$ source "$HOME/.cargo/env"
mio@Kurihara-Mio:~$ cargo -v
Rust's package manager

Usage: cargo [+toolchain] [OPTIONS] [COMMAND]
       cargo [+toolchain] [OPTIONS] -Zscript <MANIFEST_RS> [ARGS]...

Options:
  -V, --version             Print version info and exit
      --list                List installed commands
      --explain <CODE>      Run `rustc --explain CODE`
  -v, --verbose...          Use verbose output (-vv very verbose/build.rs
                            output)
  -q, --quiet               Do not print cargo log messages
      --color <WHEN>        Coloring: auto, always, never
  -C <DIRECTORY>            Change to DIRECTORY before doing anything
                            (nightly-only)
      --frozen              Require Cargo.lock and cache are up to date
      --locked              Require Cargo.lock is up to date
      --offline             Run without accessing the network
      --config <KEY=VALUE>  Override a configuration value
  -Z <FLAG>                 Unstable (nightly-only) flags to Cargo, see 'cargo
                            -Z help' for details
  -h, --help                Print help

Some common cargo commands are (see all commands with --list):
    build, b    Compile the current package
    check, c    Analyze the current package and report errors, but don't build object files
    clean       Remove the target directory
    doc, d      Build this package's and its dependencies' documentation
    new         Create a new cargo package
    init        Create a new cargo package in an existing directory
    add         Add dependencies to a manifest file
    remove      Remove dependencies from a manifest file
    run, r      Run a binary or example of the local package
    test, t     Run the tests
    bench       Run the benchmarks
    update      Update dependencies listed in Cargo.lock
    search      Search registry for crates
    publish     Package and upload this package to the registry
    install     Install a Rust binary. Default location is $HOME/.cargo/bin
    uninstall   Uninstall a Rust binary

See 'cargo help <command>' for more information on a specific command.
mio@Kurihara-Mio:~$ cargo -v
Rust's package manager

Usage: cargo [+toolchain] [OPTIONS] [COMMAND]
       cargo [+toolchain] [OPTIONS] -Zscript <MANIFEST_RS> [ARGS]...

Options:
  -V, --version             Print version info and exit
      --list                List installed commands
      --explain <CODE>      Run `rustc --explain CODE`
  -v, --verbose...          Use verbose output (-vv very verbose/build.rs
                            output)
  -q, --quiet               Do not print cargo log messages
      --color <WHEN>        Coloring: auto, always, never
  -C <DIRECTORY>            Change to DIRECTORY before doing anything
                            (nightly-only)
      --frozen              Require Cargo.lock and cache are up to date
      --locked              Require Cargo.lock is up to date
      --offline             Run without accessing the network
      --config <KEY=VALUE>  Override a configuration value
  -Z <FLAG>                 Unstable (nightly-only) flags to Cargo, see 'cargo
                            -Z help' for details
  -h, --help                Print help

Some common cargo commands are (see all commands with --list):
    build, b    Compile the current package
    check, c    Analyze the current package and report errors, but don't build object files
    clean       Remove the target directory
    doc, d      Build this package's and its dependencies' documentation
    new         Create a new cargo package
    init        Create a new cargo package in an existing directory
    add         Add dependencies to a manifest file
    remove      Remove dependencies from a manifest file
    run, r      Run a binary or example of the local package
    test, t     Run the tests
    bench       Run the benchmarks
    update      Update dependencies listed in Cargo.lock
    search      Search registry for crates
    publish     Package and upload this package to the registry
    install     Install a Rust binary. Default location is $HOME/.cargo/bin
    uninstall   Uninstall a Rust binary

See 'cargo help <command>' for more information on a specific command.
mio@Kurihara-Mio:~$ 
mio@Kurihara-Mio:~$ cargo -V
cargo 1.73.0 (9c4383fb5 2023-08-26)
mio@Kurihara-Mio:~$ cd Yporaject
mio@Kurihara-Mio:~/Yporaject$ cargo build
    Updating crates.io index
    Updating git repository `https://github.com/Zerthox/rasar.git`
error: failed to get `rasar` as a dependency of package `node_inject v0.1.0 (/home/mio/Yporaject)`

Caused by:
  failed to load source for dependency `rasar`

Caused by:
  Unable to update https://github.com/Zerthox/rasar.git#a384c941

Caused by:
  failed to clone into: /home/mio/.cargo/git/db/rasar-7eee17e47f6f6cf6

Caused by:
  network failure seems to have happened
  if a proxy or similar is necessary `net.git-fetch-with-cli` may help here
  https://doc.rust-lang.org/cargo/reference/config.html#netgit-fetch-with-cli

Caused by:
  SSL error: unknown error; class=Ssl (16)
mio@Kurihara-Mio:~/Yporaject$ ls target/debug
ls: 无法访问 'target/debug': 没有那个文件或目录
mio@Kurihara-Mio:~/Yporaject$ cargo run
output: 
no node_modules.asar found
move me to the root of your typora installation(the same directory as executable of electron)
    Updating crates.io index
    Updating git repository `https://github.com/Zerthox/rasar.git`
error: failed to get `rasar` as a dependency of package `node_inject v0.1.0 (/home/mio/Yporaject)`

Caused by:
  failed to load source for dependency `rasar`

Caused by:
  Unable to update https://github.com/Zerthox/rasar.git#a384c941

Caused by:
  failed to fetch into: /home/mio/.cargo/git/db/rasar-7eee17e47f6f6cf6

Caused by:
  network failure seems to have happened
  if a proxy or similar is necessary `net.git-fetch-with-cli` may help here
  https://doc.rust-lang.org/cargo/reference/config.html#netgit-fetch-with-cli

Caused by:
  SSL error: received early EOF; class=Ssl (16); code=Eof (-20)
找不到命令 “output:”，您的意思是：
  “output” 命令来自 Debian 软件包 yagiuda (1.19-10)
尝试 sudo apt install <deb name>
no：未找到命令
bash: 未预期的记号 "(" 附近有语法错误
mio@Kurihara-Mio:~/Yporaject$ cd Yporaject
bash: cd: Yporaject: 没有那个文件或目录
mio@Kurihara-Mio:~/Yporaject$ cd Yporaject
bash: cd: Yporaject: 没有那个文件或目录
mio@Kurihara-Mio:~/Yporaject$ cargo -V
cargo 1.73.0 (9c4383fb5 2023-08-26)
mio@Kurihara-Mio:~/Yporaject$ cd Yporaject
bash: cd: Yporaject: 没有那个文件或目录
mio@Kurihara-Mio:~/Yporaject$ cargo build
    Updating crates.io index
    Updating git repository `https://github.com/Zerthox/rasar.git`
  Downloaded random-string v1.0.0
  Downloaded ansi_term v0.12.1
  Downloaded bitflags v1.3.2
  Downloaded ryu v1.0.11
  Downloaded glob v0.3.0
  Downloaded unicode-width v0.1.10
  Downloaded textwrap v0.11.0
  Downloaded vec_map v0.8.2
  Downloaded atty v0.2.14
  Downloaded itoa v1.0.4
  Downloaded strsim v0.8.0
  Downloaded fastrand v1.8.0
  Downloaded serde v1.0.147
  Downloaded serde_json v1.0.87
  Downloaded clap v2.34.0
  Downloaded libc v0.2.137
  Downloaded 16 crates (1.2 MB) in 47.11s
   Compiling libc v0.2.137
   Compiling serde v1.0.147
   Compiling unicode-width v0.1.10
   Compiling serde_json v1.0.87
   Compiling vec_map v0.8.2
   Compiling itoa v1.0.4
   Compiling ryu v1.0.11
   Compiling ansi_term v0.12.1
   Compiling strsim v0.8.0
   Compiling bitflags v1.3.2
   Compiling fastrand v1.8.0
   Compiling glob v0.3.0
   Compiling textwrap v0.11.0
   Compiling random-string v1.0.0
   Compiling atty v0.2.14
   Compiling clap v2.34.0
   Compiling rasar v0.1.6 (https://github.com/Zerthox/rasar.git#a384c941)
   Compiling node_inject v0.1.0 (/home/mio/Yporaject)
    Finished dev [unoptimized + debuginfo] target(s) in 51.40s
mio@Kurihara-Mio:~/Yporaject$ ls target/debug
build  deps  examples  incremental  node_inject  node_inject.d
mio@Kurihara-Mio:~/Yporaject$ cargo run
output: 
no node_modules.asar found
move me to the root of your typora installation(the same directory as executable of electron)
    Finished dev [unoptimized + debuginfo] target(s) in 0.01s
     Running `target/debug/node_inject`
no node_modules.asar found
move me to the root of your typora installation(the same directory as executable of electron)
找不到命令 “output:”，您的意思是：
  “output” 命令来自 Debian 软件包 yagiuda (1.19-10)
尝试 sudo apt install <deb name>
no：未找到命令
bash: 未预期的记号 "(" 附近有语法错误
mio@Kurihara-Mio:~/Yporaject$ ls target/debug
build  deps  examples  incremental  node_inject  node_inject.d
mio@Kurihara-Mio:~/Yporaject$ cur=`pwd`
mio@Kurihara-Mio:~/Yporaject$ sudo cp target/debug/node_inject /usr/share/typora
[sudo] mio 的密码： 
mio@Kurihara-Mio:~/Yporaject$ cd /usr/share/typora
mio@Kurihara-Mio:/usr/share/typora$ sudo chmod +x node_inject
mio@Kurihara-Mio:/usr/share/typora$ sudo ./node_inject
extracting node_modules.asar
adding hook.js
applying patch
packing node_modules.asar
done!
mio@Kurihara-Mio:/usr/share/typora$ cd $cur
mio@Kurihara-Mio:~/Yporaject$ cd license-gen
mio@Kurihara-Mio:~/Yporaject/license-gen$ cargo build
   Compiling fastrand v1.8.0
   Compiling random-string v1.0.0
   Compiling license-gen v0.1.0 (/home/mio/Yporaject/license-gen)
    Finished dev [unoptimized + debuginfo] target(s) in 0.21s
mio@Kurihara-Mio:~/Yporaject/license-gen$ cargo run
    Finished dev [unoptimized + debuginfo] target(s) in 0.00s
     Running `target/debug/license-gen`
License for you: DZZM7Q-GECK6G-5F29HH-332JPD
mio@Kurihara-Mio:~/Yporaject/license-gen$ 

```

