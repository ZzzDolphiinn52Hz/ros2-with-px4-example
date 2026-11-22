# Raspberry Pi SSH and Git Synchronization

## Rule

The shared Git remote is the synchronization authority. Do not synchronize the
source tree with `rsync`, `scp`, editor upload, force-push, or hard reset. Normal
flow is:

```text
development machine -> commit -> push origin -> Pi fetch/pull --ff-only
```

If code is intentionally changed on the Pi, use a separate Pi branch, commit,
and push it to `origin`; review and merge it on the development machine.

## Connection Variables

Set these locally for the actual Pi account and checkout path; do not commit
machine-specific values or private keys:

```bash
export UAV_PI_SSH='user@raspberry-pi-host'
export UAV_PI_WORKSPACE='/home/user/ros2_ws'
```

## 1. SSH and Repository Preflight

```bash
ssh -o BatchMode=yes -o ConnectTimeout=10 "$UAV_PI_SSH" \
  "test -d '$UAV_PI_WORKSPACE/.git' && \
   hostname && uname -m && \
   git -C '$UAV_PI_WORKSPACE' status --short --branch && \
   git -C '$UAV_PI_WORKSPACE' remote -v"
```

The gate fails if SSH needs unresolved credentials, the path is not a Git
checkout, or the command exits non-zero. Inspect both sides before syncing:

```bash
git status --short --branch
ssh "$UAV_PI_SSH" \
  "git -C '$UAV_PI_WORKSPACE' status --short --branch"
```

Do not pull over uncommitted Pi changes. Preserve them on a branch or stop and
request a decision.

## 2. Push the Completed Branch

Run on the development machine after review and verification:

```bash
branch="$(git branch --show-current)"
git push -u origin "$branch"
```

Do not report synchronization complete if push fails. Never add `--force` to
work around a rejected push; fetch and reconcile the branch history instead.

## 3. Fast-Forward the Raspberry Pi

```bash
branch="$(git branch --show-current)"
ssh "$UAV_PI_SSH" \
  "cd '$UAV_PI_WORKSPACE' && \
   git fetch origin && \
   git switch '$branch' && \
   git pull --ff-only origin '$branch' && \
   git submodule update --init --recursive && \
   ./uavcup context"
```

If the Pi branch has diverged, `--ff-only` must fail. Do not reset or overwrite
it automatically; inspect the commits and reconcile them on a reviewable branch.

## 4. Prove Both Checkouts Match

```bash
local_commit="$(git rev-parse HEAD)"
pi_commit="$(ssh "$UAV_PI_SSH" \
  "git -C '$UAV_PI_WORKSPACE' rev-parse HEAD")"
test "$local_commit" = "$pi_commit"

local_px4="$(git -C src/px4_msgs rev-parse HEAD)"
pi_px4="$(ssh "$UAV_PI_SSH" \
  "git -C '$UAV_PI_WORKSPACE/src/px4_msgs' rev-parse HEAD")"
test "$local_px4" = "$pi_px4"
```

Record both SHA values in the active feature `verification.md` and
`docs/PROJECT_STATUS.md`.

## 5. Verify on the Pi

Run the checks appropriate to the feature:

```bash
ssh -t "$UAV_PI_SSH" \
  "cd '$UAV_PI_WORKSPACE' && ./uavcup doctor"
```

After the workspace is built, run software tests before hardware profiles:

```bash
ssh -t "$UAV_PI_SSH" \
  "cd '$UAV_PI_WORKSPACE' && ./uavcup unit-test"
```

Camera, UART, bench, and flight checks remain separate. A matching commit and
passing unit tests do not authorize flight.

## Changes Made Directly on the Pi

Prefer editing on the development machine. If a Pi-only diagnostic change must
be retained, do not leave it as an untracked handoff:

```bash
git switch -c pi/<short-feature-name>
git add <reviewed-paths>
git commit -m "Describe Pi-side diagnostic change"
git push -u origin HEAD
```

Then fetch and review that branch on the development machine. Never have both
machines independently modify the same branch without synchronizing first.
