Simple solver for the Flexible Jobshop with Transportation Resources.
It extends the flexible jobshop with the allocation of transport requirements to "robots".

### Usage

The scheduler is written in Rust so in order to install it you should have a working [rust installation](https://www.rust-lang.org/tools/install).
To compile it you should run:
```shell
cargo build --release --bin scheduler
```
This will produce an executable binary `target/release/scheduler` (target being at the root of this repository).

```shell
./scheduler instances/flexible/hu/rdata/la02.fjs --layout instances/layouts/layout10.txt   --robots 6 --timeout 20
```

In general `./scheduler` can be replaced with `cargo run --release --` (things may be very slow without `--release`)

### Data

- `instances/flexible` -> standard flexible jobshop instances (very few very small, but one can look for `mt06.fjs`)
- `instances/layouts` -> example layout files that specify the transit times between any pair of machines.




### Options

Run with `--help` to get a list of command line options.


### Reference


- Arthur Bit-Monnot. *Enhancing Hybrid CP-SAT Search for Disjunctive Scheduling* -- ECAI 2023 [(link)](https://hal.science/hal-04174800)
- Arthur Bit-Monnot. *Revisiting Optional Variables in Lazy Clause Generation Solvers for Flexible Scheduling* -- CP 2026 [(link)](https://doi.org/10.4230/LIPIcs.CP.2026.7)
