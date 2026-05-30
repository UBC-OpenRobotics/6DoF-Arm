# Vendored Interbotix Subset

This directory contains the vendored subset of Interbotix packages required by the RX-150 workflows in this repository.

Its purpose is to make the RX-150 simulation, control, and MoveIt paths buildable from a fresh clone of `bcr_arm` without requiring a separate `~/interbotix_ws` underlay.

Guidelines:

- Keep project-specific logic in `bcr_arm_rx150`, not in this subtree.
- Treat this directory as third-party/vendor code unless a local patch is intentionally required.
- If a vendored package must be patched, document the reason in the commit or README so the change can be tracked against upstream behavior.
