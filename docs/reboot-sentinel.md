# Reboot Sentinel: A Cross-Language Guard

One of the more subtle but elegant pieces of wisdom in Psyched is its **reboot
sentinel system** — a lightweight mechanism that prevents expensive provisioning
operations from running when the system needs a reboot.

## The Problem

During host provisioning, certain operations trigger kernel updates, driver
installations, or system-level changes that require a restart before they take
full effect. If module setup proceeds immediately after these changes, you risk:

- Installing packages against the wrong kernel headers
- Running processes that can't access newly loaded drivers
- Building ROS nodes that reference stale system libraries
- Wasting time on operations that will fail or behave incorrectly

## The Solution

Psyched uses a **sentinel file** written at a specific boot time. When module
setup begins, it checks whether the current boot is _newer_ than the timestamp
recorded in that sentinel. If not, setup halts with a clear error message asking
the user to reboot first.

### How It Works

1. **During host provisioning**, if a reboot-requiring operation occurs (like a
   kernel update), a sentinel file is written containing the current boot time
   (read from `/proc/stat`'s `btime` field).

2. **Before module setup**, `ensureRebootCompleted()` reads the sentinel and
   compares its timestamp against the current boot time:
   - If no sentinel exists, setup proceeds normally.
   - If the boot time is _older than or equal to_ the sentinel, setup throws
     `RebootRequiredError`.
   - If the boot time is _newer_, the sentinel is automatically removed and
     setup continues.

3. **After a successful reboot**, the boot time increases, the guard passes, and
   provisioning resumes where it left off.

## Cross-Language Consistency

What makes this particularly elegant is that the logic is implemented
**identically** in both TypeScript (for `psh`) and Bash (for bootstrap scripts):

| Aspect               | TypeScript (`tools/psh/lib/reboot_guard.ts`)                                                        | Bash (`tools/bootstrap/reboot_helpers.sh`)            |
| -------------------- | --------------------------------------------------------------------------------------------------- | ----------------------------------------------------- |
| **Sentinel path**    | `$XDG_STATE_HOME/psyched/reboot-required`<br>↓ `~/.local/state/...`<br>↓ `.psyched/reboot-required` | Same fallback chain                                   |
| **Boot time source** | Parse `/proc/stat` for `btime` field                                                                | `awk '/^btime/ {print $2}' /proc/stat`                |
| **Override env var** | `PSYCHED_REBOOT_SENTINEL`                                                                           | `PSYCHED_REBOOT_SENTINEL`                             |
| **Test hooks**       | `readBootTime` callback parameter                                                                   | `PSYCHED_FAKE_BOOT_TIME` environment variable         |
| **Auto-cleanup**     | Removes sentinel after detecting fresh boot                                                         | `rm -f "${sentinel_path}"` after detecting fresh boot |

Both languages share the same conceptual model, file locations, and error
conditions, ensuring that whether you're running `psh` commands or raw bootstrap
scripts, the system behaves consistently.

## Usage Examples

### TypeScript (psh)

```typescript
import {
  ensureRebootCompleted,
  RebootRequiredError,
} from "./lib/reboot_guard.ts";

export async function setupModules(modules: string[]): Promise<void> {
  ensureRebootCompleted(); // Throws if reboot is pending
  // ... proceed with apt, pip, colcon build, etc.
}
```

### Bash (bootstrap scripts)

```bash
source "$(dirname "${BASH_SOURCE[0]}")/tools/bootstrap/reboot_helpers.sh"

if ! psyched_bootstrap::require_reboot_if_pending; then
    echo "ERROR: System reboot required. Please restart and rerun this script."
    exit 1
fi

# ... continue with provisioning
```

### Writing the Sentinel

The sentinel is typically written by host provisioning tasks that know they've
triggered a reboot requirement:

```bash
# After installing new kernel headers or drivers:
psyched_bootstrap::write_reboot_sentinel
```

Or explicitly with a custom path/time:

```bash
psyched_bootstrap::write_reboot_sentinel "/custom/path" "$(date +%s)"
```

### Testing

Both implementations support overrides for deterministic testing:

```bash
# Bash
export PSYCHED_FAKE_BOOT_TIME=300
export PSYCHED_REBOOT_SENTINEL="/tmp/test-sentinel"
```

```typescript
// TypeScript
ensureRebootCompleted({
  sentinelPath: "/tmp/test-sentinel",
  readBootTime: () => 300,
});
```

## Why This Matters

This pattern demonstrates several principles worth emulating in systems that
provision themselves:

- **Defensive provisioning**: Assume humans will forget to reboot. Make the
  system remind them.
- **Cross-language discipline**: When logic must exist in multiple languages,
  mirror it exactly. Same paths, same logic, same environment variables.
- **Testable infrastructure**: Provide escape hatches for tests without
  compromising production behavior.
- **XDG compliance**: Respect user expectations about where state lives
  (`$XDG_STATE_HOME` first).
- **Auto-cleanup**: Once a condition is satisfied, remove evidence of it so
  future runs aren't confused.

The reboot sentinel is invisible when everything works correctly, but prevents
hours of debugging when a user forgets to restart after a kernel update. It's a
small file with a big responsibility — and it performs that duty in both
scripting languages, consistently and quietly.

## Implementation Details

### Sentinel File Location Priority

1. `$PSYCHED_REBOOT_SENTINEL` (if set)
2. `$XDG_STATE_HOME/psyched/reboot-required`
3. `$HOME/.local/state/psyched/reboot-required`
4. `$PWD/.psyched/reboot-required` (workspace-local fallback)

### Boot Time Detection

The system boot time is read from `/proc/stat`:

```
btime 1732133456
```

This is a Unix timestamp representing when the system booted. It's monotonically
increasing across reboots (modulo clock adjustments), making it ideal for
detecting fresh boots.

### Error Messages

When a reboot is required, users see:

```
ERROR: RebootRequiredError: Reboot required before running module setup. 
Please reboot the system and rerun this command.
```

Clear, actionable, and non-technical enough for operators who aren't familiar
with the internals.

---

## See Also

- `tools/psh/lib/reboot_guard.ts` — TypeScript implementation
- `tools/bootstrap/reboot_helpers.sh` — Bash implementation
- `tests/reboot_sentinel_test.sh` — Shell test suite
- `tools/psh/lib/reboot_guard_test.ts` — Deno test suite
