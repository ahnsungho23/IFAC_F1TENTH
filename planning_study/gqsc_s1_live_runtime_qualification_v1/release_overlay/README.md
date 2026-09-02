# Frozen Release overlay

Generated build/install/log directories are intentionally ignored. They were produced from source
commit `dc33b875a938fdb7730d35fe61de43ee863b04c0` using the exact colcon invocation now retained in
`tools/build_release_overlay.zsh`. The normal workspace `build/`, `install/`, and `log/` trees are
not modified by that command.

The authoritative provenance, compile flags, hashes, and bootstrap order are recorded in
`../binary_provenance.md` and `../execution_protocol.md`.
