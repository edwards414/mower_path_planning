Debug a failing GitHub Actions workflow run and propose a fix.

## Arguments
`$ARGUMENTS` — optional run ID or workflow name. If omitted, fetch the most recent failed run.

## Steps

1. **Find the run to debug**
   - If `$ARGUMENTS` is a numeric run ID: `gh run view $ARGUMENTS --log-failed`
   - If `$ARGUMENTS` is a workflow name/file: `gh run list --workflow "$ARGUMENTS" --status failure --limit 5`, then pick the most recent
   - If `$ARGUMENTS` is empty: `gh run list --status failure --limit 5` and pick the most recent

2. **Fetch structured info first**
   ```
   gh run view <RUN_ID> --json conclusion,status,workflowName,headBranch,createdAt,jobs
   ```

3. **Fetch failure logs** (log output is large — pipe through grep to filter signal from noise)
   ```
   gh run view <RUN_ID> --log-failed
   ```
   Look for:
   - Lines containing `ERROR`, `error`, `Error`, `failed`, `FAILED`
   - Docker build errors: `RUN`, `COPY`, `apt-get`, layer hash lines
   - `##[error]` annotations injected by the Actions runner
   - Exit codes (`exited with code N`)
   - Matrix job name that failed

4. **Root-cause analysis** — categorise the failure:
   | Category | Signals |
   |---|---|
   | Docker build failure | `failed to solve`, `executor failed`, `process exited with code` |
   | apt-get / package not found | `E: Unable to locate package`, `404 Not Found` |
   | COPY / missing file | `failed to read dockerfile`, `no such file or directory` |
   | Multi-arch (QEMU) timeout | `qemu: uncaught target signal`, `signal: killed` on arm64 |
   | Workflow syntax | `yaml: line`, `unexpected symbol` |
   | GHCR auth | `unauthorized`, `denied`, `403` |
   | Cache miss / stale | `cache not found`, unexpected layer rebuild |

5. **Propose fix** — show the exact file and line to change. If the fix touches a Dockerfile or workflow YAML, show a unified diff. Do NOT apply changes until the user confirms.

6. **Offer to apply** — ask "Apply this fix? (y/n)" before editing any file.

## Notes
- Always read the failing step's full log, not just the last line — the root cause is usually several lines before the final `Error`.
- For Docker multi-stage builds, identify which stage failed (base / builder / dev).
- `gh` CLI must be authenticated (`gh auth status`); if not, remind the user to run `gh auth login`.
