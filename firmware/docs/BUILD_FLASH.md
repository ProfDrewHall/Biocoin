# Build and Flash Notes

## Tooling

- PlatformIO (CLI or VS Code extension)
- nRF52840 target board

## Typical Commands

```bash
pio run
pio run -t upload
```

## Environment Assumptions

- Repo-local board variant is selected in `platformio.ini`.
- Build currently targets GNU++11 toolchain defaults for Adafruit nRF52 Arduino framework.

## Troubleshooting

- If command-line build tools are missing in shell, use PlatformIO from VS Code.
- If BLE/device name behavior seems stale after updates, verify the device rebooted after configuration writes.

## Rollback (Known-Good Firmware)

If validation fails after flashing a new build, roll back to a known-good tag/commit.

1. Find the known-good version:

```bash
git tag --list
git log --oneline --decorate --max-count=30
```

2. Checkout the known-good revision (example with tag):

```bash
git checkout v1.1.0
```

3. Rebuild and reflash:

```bash
pio run
pio run -t upload
```

4. Confirm basic health checks:
- BLE advertises and connects.
- Status characteristic reports expected transitions.
- Start/stop works for at least one technique.

5. Return to working branch after rollback testing:

```bash
git checkout main
```
