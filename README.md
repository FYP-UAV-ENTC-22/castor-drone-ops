# drone-ops

Codebase and operational tooling for the Raspberry Pi 5 drone companion
computers (drone1, drone2, drone3, ...), plus the infrastructure setup needed
to run them.

## Layout

- `companion/` — companion-computer code that runs on each drone's Pi.
  (Not yet populated — scaffold placeholder.)
- `setup/network-gateway/` — turns the Ubuntu base station into an
  internet gateway for the drones' FFT long-range wifi network (which has no
  internet of its own), so a Pi can reach the internet while you're SSHed
  into it for development. See that folder's `README.md` for the full
  writeup, scripts, and rollback instructions.

## Adding a new setup/ops piece

Each operational concern (network gateway, deployment, monitoring, etc.)
gets its own subfolder under `setup/` with its own README and scripts,
following the pattern in `setup/network-gateway/`.
