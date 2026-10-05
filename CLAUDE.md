# CLAUDE.md

Guidance for Claude Code working in this repository.

## What this project is

Coupled aerodynamic–structural analysis built on **OpenMDAO**. The repo is currently a
greenfield scaffold: `README.md`, `requirements.txt`, and a venv are in place, but no
source code has been written yet. Conventions below are the intended setup — when adding
the first modules, establish structure deliberately rather than assuming one already exists.

## Environment

Python **3.12** in a local venv at `.venv/` (Windows layout — `Scripts/`, not `bin/`).

```powershell
.\.venv\Scripts\Activate.ps1        # PowerShell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

The venv is created but **empty** — only `pip` is installed. Install `requirements.txt`
before running anything.

From the Bash tool, invoke the interpreter directly rather than activating:
`./.venv/Scripts/python.exe script.py`

## Dependencies

- `openmdao[all]` — the MDAO framework; `[all]` pulls in visualization and optional solvers
- `numpy`
- `matplotlib`

## Known issue: `.gitignore.txt`

The ignore file is named `.gitignore.txt`, so **git does not read it**. `.venv/` currently
shows up as untracked in `git status` as a result. Renaming it to `.gitignore` fixes this.

## OpenMDAO notes

The ignore rules anticipate two OpenMDAO artifacts that should stay out of version control:

- `*_out/` — output directories OpenMDAO creates per-run
- `*.sql` — case recorder databases

When adding recorders or reports, keep their output under paths matching these patterns.
